import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SRC = Path(__file__).resolve().parent / "src"
sys.path.insert(0, str(SRC))


class LocksetAnalysisIntegrationTest(unittest.TestCase):
    def setUp(self):
        if not shutil.which("cppcheck"):
            self.skipTest("cppcheck is required")
        self.temp = tempfile.TemporaryDirectory()
        self.project = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def write(self, relative, text):
        path = self.project / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def analyze(self):
        from lockset_analysis import analyze_project

        output = self.project / "lockset.json"
        analyze_project(str(self.project), str(output), cppcheck="cppcheck")
        return json.loads(output.read_text(encoding="utf-8"))["accesses"]

    def test_cross_file_calls_cfg_raii_and_missing_headers(self):
        self.write(
            "include/state.hpp",
            """
#pragma once
#include <mutex>
#include <rclcpp/rclcpp.hpp>
extern std::mutex mutex_;
extern int data_;
void helper();
void conditional(bool take_lock);
void guarded();
void looped(bool again);
void scoped_release();
""",
        )
        self.write(
            "src/callback.cpp",
            """
#include "state.hpp"
std::mutex mutex_;
int data_ = 0;
void callback() {
  int local = 0;
  local++;
  mutex_.lock();
  helper();
  mutex_.unlock();
}
int main() { callback(); guarded(); scoped_release(); conditional(false); looped(false); }
""",
        )
        self.write(
            "src/helper.cc",
            """
#include "state.hpp"
void helper() { data_++; }
void conditional(bool take_lock) {
  if (take_lock) mutex_.lock();
  data_ = 1;
  if (take_lock) mutex_.unlock();
}
void guarded() {
  std::lock_guard<std::mutex> guard(mutex_);
  data_ += 2;
}
void looped(bool again) {
  while (again) { mutex_.lock(); again = false; }
  data_ = 2;
}
void scoped_release() {
  { std::lock_guard<std::mutex> guard(mutex_); data_ = 3; }
  data_ = 4;
}
""",
        )

        accesses = self.analyze()
        by_function = {}
        for access in accesses:
            by_function.setdefault(access["function"], []).append(access)

        helper = by_function["helper"]
        self.assertEqual(1, len(helper))
        self.assertEqual("src/helper.cc", helper[0]["location"]["file"])
        self.assertEqual("READ_WRITE", helper[0]["access_kind"])
        self.assertEqual(["mutex_"], helper[0]["may_locks"])
        self.assertEqual(["mutex_"], helper[0]["must_locks"])

        conditional = by_function["conditional"]
        self.assertEqual(1, len(conditional))
        self.assertEqual("WRITE", conditional[0]["access_kind"])
        self.assertEqual(["mutex_"], conditional[0]["may_locks"])
        self.assertEqual([], conditional[0]["must_locks"])

        guarded = by_function["guarded"]
        self.assertEqual(1, len(guarded))
        self.assertEqual("READ_WRITE", guarded[0]["access_kind"])
        self.assertEqual(["mutex_"], guarded[0]["may_locks"])
        self.assertEqual(["mutex_"], guarded[0]["must_locks"])

        looped = by_function["looped"]
        self.assertEqual(["mutex_"], looped[0]["may_locks"])
        self.assertEqual([], looped[0]["must_locks"])

        scoped = sorted(by_function["scoped_release"], key=lambda item: item["location"]["line"])
        self.assertEqual(["mutex_"], scoped[0]["must_locks"])
        self.assertEqual([], scoped[1]["may_locks"])

        self.assertFalse(any(a["variable"] == "local" for a in accesses))
        self.assertTrue(all(Path(a["location"]["file"]).suffix in {".cpp", ".cc", ".hpp"} for a in accesses))

    def test_all_supported_raii_and_shared_lock_operations(self):
        self.write(
            "locks.cpp",
            """
#include <mutex>
#include <shared_mutex>
class Foo {
public:
  void shared_read() {
    std::shared_lock<std::shared_mutex> guard(shared_mutex_);
    int copy = value_;
  }
  void deferred_write() {
    std::unique_lock<std::mutex> guard(mutex_, std::defer_lock);
    guard.lock();
    value_ = 1;
    guard.unlock();
  }
  void two_locks() {
    std::scoped_lock guard(mutex_, other_mutex_);
    value_ += 2;
  }
  void manual_shared() {
    shared_mutex_.lock_shared();
    value_--;
    shared_mutex_.unlock_shared();
  }
  void pointer_write() { this->value_ = 5; }
private:
  std::mutex mutex_;
  std::mutex other_mutex_;
  std::shared_mutex shared_mutex_;
  int value_ = 0;
};
int main() { Foo foo; foo.shared_read(); foo.deferred_write(); foo.two_locks(); foo.manual_shared(); foo.pointer_write(); }
""",
        )

        accesses = self.analyze()
        by_function = {access["function"]: access for access in accesses}
        self.assertTrue(all(access["location"]["file"] == "locks.cpp" for access in accesses))

        self.assertEqual(["Foo::shared_mutex_"], by_function["Foo::shared_read"]["must_locks"])
        self.assertEqual(["Foo::mutex_"], by_function["Foo::deferred_write"]["must_locks"])
        self.assertEqual(
            ["Foo::mutex_", "Foo::other_mutex_"],
            by_function["Foo::two_locks"]["must_locks"],
        )
        self.assertEqual(["Foo::shared_mutex_"], by_function["Foo::manual_shared"]["must_locks"])
        self.assertEqual("WRITE", by_function["Foo::pointer_write"]["access_kind"])
        self.assertTrue(all(access["variable"] == "Foo::value_" for access in accesses))

    def test_return_state_flows_back_to_caller(self):
        self.write(
            "return.cpp",
            """
#include <mutex>
std::mutex mutex_;
int data_;
void acquire_then_return() {
  mutex_.lock();
  return;
  data_ = 99;
}
void caller() {
  acquire_then_return();
  data_ = 7;
  mutex_.unlock();
}
int main() { caller(); }
""",
        )

        accesses = self.analyze()
        caller = [access for access in accesses if access["function"] == "caller"]
        self.assertEqual(1, len(caller))
        self.assertEqual(["mutex_"], caller[0]["must_locks"])
        self.assertFalse(any(access["location"]["line"] == 8 for access in accesses))

    def test_roscallbax_cli_lockset_mode_has_no_callback_checker_dependency(self):
        self.write("main.cpp", "int shared; int main() { shared = 1; }\n")
        output = self.project / "result.json"
        command = [
            sys.executable,
            str(SRC / "roscbexec.py"),
            "--lockset",
            "--lockset-output",
            str(output),
            str(self.project),
        ]
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
        self.assertEqual(0, completed.returncode, completed.stderr)
        self.assertTrue(output.exists())

    def test_scc_for_raii_virtual_and_parameter_aliases(self):
        self.write(
            "corners.cpp",
            """
#include <mutex>
std::mutex mutex_;
int data_;
int other_;
void cycle_b();
void cycle_a() { cycle_b(); data_ = 1; }
void cycle_b() { if (other_) cycle_a(); }
void unrelated_root() { other_ = 0; }
void for_order() {
  mutex_.lock();
  for (int i = 0; i < 1; ++i, mutex_.unlock()) { data_ = 2; }
}
void conditional_raii(bool release) {
  std::unique_lock<std::mutex> guard(mutex_);
  if (release) { guard.unlock(); }
}
void after_conditional_raii() { conditional_raii(false); other_ = 3; }
void acquire(std::mutex & lock) { lock.lock(); }
void release(std::mutex & lock) { lock.unlock(); }
void parameter_locking() { acquire(mutex_); data_ = 4; release(mutex_); other_ = 5; }
int unlock_and_return() { mutex_.unlock(); return 7; }
void rhs_call_order() { mutex_.lock(); data_ = unlock_and_return(); }
void lock_after_argument_read(int &) { mutex_.lock(); }
void argument_order() { lock_after_argument_read(data_); mutex_.unlock(); }
struct Base { virtual void go() {} };
struct Derived : Base { void go() override { mutex_.lock(); } };
void virtual_call(Base & base) { base.go(); data_ = 6; }
int main() {
  unrelated_root(); after_conditional_raii(); parameter_locking();
  rhs_call_order(); argument_order(); Derived derived; virtual_call(derived); for_order();
}
""",
        )

        accesses = self.analyze()
        by_function = {}
        for access in accesses:
            by_function.setdefault(access["function"], []).append(access)

        self.assertIn("cycle_a", by_function)
        for_write = next(a for a in by_function["for_order"] if a["variable"] == "data_")
        # The bounded loop is not value-evaluated; its first iteration is locked,
        # while the abstract back-edge may execute again after the increment unlocks.
        self.assertEqual(["mutex_"], for_write["may_locks"])
        self.assertEqual([], for_write["must_locks"])
        after_raii = next(a for a in by_function["after_conditional_raii"] if a["variable"] == "other_")
        self.assertEqual([], after_raii["may_locks"])
        parameter = sorted(by_function["parameter_locking"], key=lambda a: a["location"]["line"])
        data_write = next(a for a in parameter if a["variable"] == "data_")
        other_write = next(a for a in parameter if a["variable"] == "other_")
        self.assertEqual(["mutex_"], data_write["must_locks"], data_write)
        self.assertEqual([], other_write["may_locks"])
        rhs_write = next(a for a in by_function["rhs_call_order"] if a["variable"] == "data_")
        self.assertEqual([], rhs_write["may_locks"])
        argument_read = next(a for a in by_function["argument_order"] if a["variable"] == "data_")
        self.assertEqual("READ", argument_read["access_kind"])
        self.assertEqual([], argument_read["may_locks"])
        virtual = next(a for a in by_function["virtual_call"] if a["variable"] == "data_")
        self.assertEqual(["mutex_"], virtual["may_locks"])
        self.assertEqual([], virtual["must_locks"])

    def test_recursive_lock_depth_survives_nested_raii_and_calls(self):
        self.write("nested.cpp", """
#include <mutex>
std::recursive_mutex mutex_;
int data_;
void inner() {
  std::lock_guard<std::recursive_mutex> inner_guard(mutex_);
  data_ = 1;
}
void outer() {
  std::lock_guard<std::recursive_mutex> outer_guard(mutex_);
  inner();
  data_ = 2;
  { std::lock_guard<std::recursive_mutex> second_guard(mutex_); data_ = 3; }
  data_ = 4;
}
int main() { outer(); data_ = 5; }
""")
        accesses = self.analyze()
        outer = [a for a in accesses if a["function"] == "outer" and a["variable"] == "data_"]
        self.assertEqual(3, len(outer))
        self.assertTrue(all(a["must_locks"] == ["mutex_"] for a in outer))
        main = next(a for a in accesses if a["function"] == "main" and a["variable"] == "data_")
        self.assertEqual([], main["may_locks"])

    def test_async_worker_starts_without_parent_lock_and_catch_return_keeps_normal_path(self):
        self.write("async.cpp", """
#include <mutex>
#include <future>
std::recursive_mutex mutex_;
int data_;
void worker() {
  try { data_ = 1; }
  catch (...) { return; }
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  data_ = 2;
}
void start() {
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  auto future = std::async(std::launch::async, []() { worker(); });
  data_ = 3;
}
int main() { start(); }
""")
        accesses = self.analyze()
        worker = sorted(
            (a for a in accesses if a["function"] == "worker" and a["variable"] == "data_"),
            key=lambda a: a["location"]["line"],
        )
        self.assertEqual(2, len(worker))
        self.assertEqual([], worker[0]["may_locks"])
        self.assertEqual(["mutex_"], worker[1]["must_locks"])
        start = next(a for a in accesses if a["function"] == "start" and a["variable"] == "data_")
        self.assertEqual(["mutex_"], start["must_locks"])

    def test_value_parameters_are_local_and_member_mutators_are_writes(self):
        self.write("access.cpp", """
#include <memory>
struct State {
  std::shared_ptr<int> member_;
  void inspect(std::shared_ptr<int> copy) { auto x = copy.get(); }
  void change() { member_.reset(); }
  void run() { inspect(member_); change(); }
};
int main() { State state; state.run(); }
""")
        accesses = self.analyze()
        self.assertFalse(any(a["function"] == "State::inspect" for a in accesses))
        mutation = next(a for a in accesses if a["function"] == "State::change")
        self.assertEqual("State::member_", mutation["variable"])
        self.assertEqual("READ_WRITE", mutation["access_kind"])

    def test_immediately_invoked_lambda_uses_caller_lock_state(self):
        self.write("lambda.cpp", """
#include <mutex>
std::mutex mutex_;
int data_;
void direct() {
  mutex_.lock();
  []() { data_ = 1; }();
  mutex_.unlock();
}
int main() { direct(); }
""")
        accesses = self.analyze()
        access = next(a for a in accesses if a["variable"] == "data_")
        self.assertEqual(["mutex_"], access["must_locks"])

    def test_loop_break_and_continue_stop_following_statements(self):
        self.write("jumps.cpp", """
#include <mutex>
std::mutex mutex_;
int data_;
void via_break(bool again) {
  while (again) { mutex_.lock(); break; mutex_.unlock(); }
  data_ = 1;
}
void via_continue(bool again) {
  mutex_.lock();
  while (again) { mutex_.unlock(); continue; mutex_.lock(); }
  data_ = 2;
  mutex_.unlock();
}
int main() { via_break(false); via_continue(false); }
""")
        accesses = self.analyze()
        via_break = next(a for a in accesses if a["function"] == "via_break")
        via_continue = next(a for a in accesses if a["function"] == "via_continue")
        self.assertEqual(["mutex_"], via_break["may_locks"])
        self.assertEqual([], via_break["must_locks"])
        self.assertEqual(["mutex_"], via_continue["may_locks"])
        self.assertEqual([], via_continue["must_locks"])

    def test_deadlock_master_regression(self):
        from lockset_analysis import LocksetAnalyzer

        project = Path(__file__).resolve().parents[2] / "case" / "deadlock-master"
        accesses = LocksetAnalyzer(str(project)).run()["accesses"]
        header = [a for a in accesses if a["location"]["file"] == "src/simple_action_server.hpp"]

        def at(function, variable, line):
            return next(a for a in header if a["function"].endswith("::" + function)
                        and a["variable"].endswith("::" + variable)
                        and a["location"]["line"] == line)

        for line, variable in [(114, "pending_handle_"), (115, "preempt_requested_"),
                               (121, "preempt_requested_"), (124, "current_handle_")]:
            self.assertTrue(at("handle_accepted", variable, line)["must_locks"], line)
        for line, variable in [(289, "pending_handle_"), (290, "preempt_requested_")]:
            self.assertTrue(at("terminate_all", variable, line)["must_locks"], line)
        self.assertEqual([], at("work", "stop_execution_", 134)["may_locks"])
        self.assertEqual([], at("work", "execute_callback_", 137)["may_locks"])
        self.assertTrue(at("work", "stop_execution_", 149)["must_locks"])
        self.assertFalse(any(a["variable"].endswith("::feedback") for a in header))
        self.assertFalse(any(a["variable"].endswith("::result") for a in header))
        self.assertEqual("READ_WRITE", at("accept_pending_goal", "pending_handle_", 245)["access_kind"])
        self.assertEqual("READ_WRITE", at("succeeded_current", "current_handle_", 310)["access_kind"])
        self.assertEqual("WRITE", at("handle_accepted", "execution_future_", 128)["access_kind"])
        constructor_write = next(a for a in header if a["location"]["line"] == 67
                                 and a["variable"].endswith("::action_server_"))
        self.assertEqual("WRITE", constructor_write["access_kind"])
        self.assertFalse(any(a["location"]["line"] == 62 and a["variable"] in
                             {"node", "action_name"} for a in header))

    def test_library_method_evidence_and_uncertain_writes(self):
        self.write("methods.cpp", """
#include <string>
#include <future>
struct External { void inspect() const; void change(); };
struct State {
  std::string name_;
  std::future<int> future_;
  External external_;
  void run() {
    name_.c_str();
    future_.valid();
    external_.change();
  }
};
int main() { State state; state.run(); }
""")
        rows = {a["variable"].split("::")[-1]: a for a in self.analyze()}
        self.assertEqual("READ", rows["name_"]["access_kind"])
        self.assertFalse(rows["name_"].get("may_write", False))
        self.assertEqual("READ", rows["future_"]["access_kind"])
        self.assertTrue(rows["future_"]["may_write"])
        self.assertEqual("READ", rows["external_"]["access_kind"])
        self.assertTrue(rows["external_"]["may_write"])

    def test_external_call_assignment_records_lhs_write(self):
        self.write("external.cpp", """
#include <future>
int external();
int shared_;
struct State {
  std::future<int> future_;
  void run() {
    shared_ = external();
    future_ = std::async(std::launch::async, [] { return 1; });
  }
};
int main() { State state; state.run(); }
""")
        rows = {a["variable"].split("::")[-1]: a for a in self.analyze()}
        self.assertEqual("WRITE", rows["shared_"]["access_kind"])
        self.assertEqual("WRITE", rows["future_"]["access_kind"])
        self.assertFalse(rows["future_"].get("may_write", False))

    def test_constructor_member_initializers_are_writes(self):
        self.write("initializer.cpp", """
struct State {
  int value_;
  State(int x) : value_(x) {}
};
int main() { State state(1); }
""")
        rows = [a for a in self.analyze() if a["variable"] == "State::value_"]
        self.assertEqual(1, len(rows))
        self.assertEqual("WRITE", rows[0]["access_kind"])
        self.assertEqual(4, rows[0]["location"]["line"])

    def test_reference_capture_escaping_through_callback_argument(self):
        self.write("capture.cpp", """
void register_callback(void (*callback)());
template<class F> void consume(F);
int main() {
  int shared = 0;
  int local = 0;
  auto callback = [&shared]() { shared = 1; };
  consume(callback);
  [&local]() { local = 2; }();
  if (shared) { local++; }
}
""")
        accesses = self.analyze()
        self.assertEqual({5, 7, 10}, {a["location"]["line"] for a in accesses
                                     if a["variable"] == "shared"})
        self.assertFalse(any(a["variable"] == "local" for a in accesses))

    def test_deadlock_master_captured_callback_locals_are_included(self):
        from lockset_analysis import LocksetAnalyzer

        project = Path(__file__).resolve().parents[2] / "case" / "deadlock-master"
        accesses = LocksetAnalyzer(str(project)).run()["accesses"]
        rows = [a for a in accesses if a["location"]["file"] == "src/deadlock.cpp"]
        self.assertTrue(any(a["variable"] == "goal_result_available"
                            and a["location"]["line"] == 28
                            and a["access_kind"] == "WRITE" for a in rows))
        self.assertTrue(any(a["variable"] == "goal_result_available"
                            and a["location"]["line"] == 50
                            and a["access_kind"] == "READ" for a in rows))

    def test_directly_passed_reference_capture_is_shared(self):
        self.write("direct_capture.cpp", """
template<class F> void consume(F);
int main() {
  int shared = 0;
  consume([&shared]() { shared = 1; });
  if (shared) {}
}
""")
        accesses = self.analyze()
        self.assertTrue(any(a["variable"] == "shared" and a["location"]["line"] == 5
                            for a in accesses))

    def test_implicit_reference_capture_that_escapes_is_shared(self):
        self.write("implicit_capture.cpp", """
template<class F> void consume(F);
int main() {
  int shared = 0;
  consume([&]() { shared = 1; });
  if (shared) {}
}
""")
        accesses = self.analyze()
        self.assertTrue(any(a["variable"] == "shared" and a["location"]["line"] == 5
                            for a in accesses))

    def test_constructor_initializer_reads_another_member(self):
        self.write("initializer_read.cpp", """
struct State {
  int first_;
  int second_;
  State() : first_(1), second_(first_) {}
};
int main() { State state; }
""")
        rows = [a for a in self.analyze() if a["location"]["line"] == 5]
        self.assertTrue(any(a["variable"] == "State::first_" and
                            a["access_kind"] == "READ_WRITE" for a in rows))
        self.assertTrue(any(a["variable"] == "State::second_" and
                            a["access_kind"] == "WRITE" for a in rows))

    def test_constructor_callback_initializer_captures_reference(self):
        self.write("initializer_callback.cpp", """
#include <functional>
struct State {
  std::function<void()> callback_;
  State(int & shared) : callback_([&shared]() { shared++; }) {}
};
int main() { int data = 0; State state(data); }
""")
        rows = self.analyze()
        self.assertTrue(any(a["variable"] == "shared" and a["access_kind"] == "READ_WRITE"
                            and a["location"]["line"] == 5 for a in rows))

    def test_returned_reference_capture_is_shared(self):
        self.write("return_capture.cpp", """
auto make_callback(int & shared) {
  return [&shared]() { shared++; };
}
int main() {
  int data = 0;
  auto callback = make_callback(data);
  if (data) {}
}
""")
        rows = self.analyze()
        self.assertTrue(any(a["variable"] == "data" and a["access_kind"] == "READ_WRITE"
                            and a["location"]["line"] == 3 for a in rows))
        self.assertTrue(any(a["variable"] == "data" and a["access_kind"] == "READ"
                            and a["location"]["line"] == 8 for a in rows))

    def test_returned_lambda_does_not_inherit_returning_functions_lock(self):
        self.write("returned_lambda_lock.cpp", """
#include <mutex>
std::mutex mutex_;
int data_;
auto make_callback() {
  std::lock_guard<std::mutex> guard(mutex_);
  return []() { data_ = 1; };
}
int main() { auto callback = make_callback(); }
""")
        row = next(a for a in self.analyze() if a["variable"] == "data_")
        self.assertEqual([], row["may_locks"])

    def test_original_function_graph_excludes_function_references(self):
        self.write("function_references.cpp", """
using Callback = void (*)();
Callback registered_callback = nullptr;
void callback() {}
void register_callback(Callback cb) { registered_callback = cb; }
void setup() {
  Callback local = &callback;
  register_callback(callback);
}
void direct() { callback(); }
void run_event_loop() { registered_callback(); }
int main() { setup(); direct(); run_event_loop(); }
""")
        from functions_manager import FunctionsManager
        from lockset_analysis import LocksetAnalyzer

        analyzer = LocksetAnalyzer(str(self.project))
        analyzer.run()
        manager = FunctionsManager()
        functions = manager.find_functions(analyzer.functions[0].config)
        manager.build_function_graph(functions)
        edges = {
            item["function"].name: {target.name for target in item["function_graph_edges"]}
            for item in functions.values()
        }
        self.assertEqual({"register_callback"}, edges["setup"])
        self.assertEqual({"callback"}, edges["direct"])
        self.assertEqual(set(), edges["run_event_loop"])
        self.assertEqual({"setup", "direct", "run_event_loop"}, edges["main"])

    def test_original_function_graph_keeps_member_and_qualified_calls(self):
        self.write("qualified_calls.cpp", """
namespace N { void work() {} }
struct Worker {
  void ping() {}
  void invoke() { ping(); }
};
void use(Worker & worker) { worker.ping(); N::work(); }
""")
        from functions_manager import FunctionsManager
        from lockset_analysis import LocksetAnalyzer

        analyzer = LocksetAnalyzer(str(self.project))
        analyzer.run()
        manager = FunctionsManager()
        functions = manager.find_functions(analyzer.functions[0].config)
        manager.build_function_graph(functions)
        edges = {
            item["function"].name: {target.name for target in item["function_graph_edges"]}
            for item in functions.values()
        }
        self.assertEqual({"ping"}, edges["invoke"])
        self.assertEqual({"ping", "work"}, edges["use"])

    def test_function_pointer_registration_propagates_callsite_locks(self):
        self.write("later_function.cpp", """
#include <mutex>
std::mutex registration_mutex;
std::mutex invocation_mutex;
std::mutex inner_mutex;
int x;
int y;
using Callback = void (*)();
Callback registered_callback = nullptr;
void callback() {
  x++;
  std::lock_guard<std::mutex> inner(inner_mutex);
  y++;
}
void register_callback(Callback cb) { registered_callback = cb; }
void setup() {
  std::lock_guard<std::mutex> registration(registration_mutex);
  register_callback(callback);
}
void run_event_loop() {
  std::lock_guard<std::mutex> invocation(invocation_mutex);
  if (registered_callback != nullptr) registered_callback();
}
int main() { setup(); run_event_loop(); }
""")
        accesses = self.analyze()
        x = next(item for item in accesses if item["function"] == "callback"
                 and item["variable"] == "x")
        y = next(item for item in accesses if item["function"] == "callback"
                 and item["variable"] == "y")
        self.assertEqual(["invocation_mutex"], x["must_locks"])
        self.assertEqual(["inner_mutex", "invocation_mutex"], y["must_locks"])
        self.assertNotIn("registration_mutex", x["may_locks"])
        self.assertNotIn("registration_mutex", y["may_locks"])

    def test_function_pointer_candidates_flow_through_value_parameter(self):
        self.write("choice.cpp", """
#include <mutex>
using Callback = void (*)();
std::mutex guard;
int first_data;
int second_data;
void first() { first_data++; }
void second() { second_data++; }
void invoke(Callback cb) { cb(); }
int main(int argc, char **) {
  Callback selected = nullptr;
  if (argc) { selected = first; }
  else { selected = second; }
  std::lock_guard<std::mutex> lock(guard);
  invoke(selected);
}
""")
        accesses = self.analyze()
        for function in ("first", "second"):
            row = next(item for item in accesses if item["function"] == function
                       and item["variable"] == function + "_data")
            self.assertEqual(["guard"], row["must_locks"])

    def test_function_pointer_target_flows_across_files(self):
        self.write("state.hpp", """
#pragma once
using Callback = void (*)();
extern Callback registered_callback;
extern int data;
void callback();
void register_callback(Callback cb);
void run_event_loop();
""")
        self.write("register.cpp", """
#include "state.hpp"
Callback registered_callback = nullptr;
void register_callback(Callback cb) { registered_callback = cb; }
""")
        self.write("run.cpp", """
#include "state.hpp"
#include <mutex>
std::mutex mutex_;
int data;
void callback() { data++; }
void run_event_loop() {
  std::lock_guard<std::mutex> guard(mutex_);
  registered_callback();
}
""")
        self.write("main.cpp", """
#include "state.hpp"
int main() { register_callback(callback); run_event_loop(); }
""")
        row = next(item for item in self.analyze() if item["function"] == "callback"
                   and item["variable"] == "data")
        self.assertEqual(["mutex_"], row["must_locks"])

    def test_resolved_function_pointer_value_does_not_include_unrelated_namesake(self):
        self.write("qualified_target.cpp", """
#include <mutex>
int a_data;
int b_data;
namespace A {
void cb() { a_data++; }
}
namespace B {
void cb() { b_data++; }
}
using Callback = void (*)();
std::mutex mutex_;
int main() {
  Callback fp = A::cb;
  std::lock_guard<std::mutex> guard(mutex_);
  fp();
}
""")
        accesses = self.analyze()
        a = next(item for item in accesses if item["function"] == "A::cb"
                 and item["variable"] == "a_data")
        b = next(item for item in accesses if item["function"] == "B::cb"
                 and item["variable"] == "b_data")
        self.assertEqual(["mutex_"], a["must_locks"])
        self.assertEqual([], b["may_locks"])

    def test_brace_initialized_function_pointer_is_called_under_current_lock(self):
        self.write("direct_init.cpp", """
#include <mutex>
using Callback = void (*)();
std::mutex mutex_;
int data;
void cb() { data++; }
void with_braces() {
  Callback fp{cb};
  std::lock_guard<std::mutex> guard(mutex_);
  fp();
}
int main() { with_braces(); }
""")
        row = next(item for item in self.analyze() if item["function"] == "cb"
                   and item["variable"] == "data")
        self.assertEqual(["mutex_"], row["must_locks"])

    def test_global_function_pointer_initializer_is_available_at_entry(self):
        self.write("global_init.cpp", """
#include <mutex>
using Callback = void (*)();
std::mutex mutex_;
int data;
void cb() { data++; }
Callback fp = cb;
int main() {
  std::lock_guard<std::mutex> guard(mutex_);
  fp();
}
""")
        row = next(item for item in self.analyze() if item["function"] == "cb"
                   and item["variable"] == "data")
        self.assertEqual(["mutex_"], row["must_locks"])


if __name__ == "__main__":
    unittest.main()

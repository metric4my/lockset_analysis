#!/usr/bin/env python3
"""Small, Cppcheck-backed interprocedural lockset analysis.

This intentionally models only the control flow needed by locksets.  It uses
Cppcheck's tolerant parser/symbol database, so it remains useful when a ROS
workspace cannot be compiled and its third-party headers are unavailable.
"""

from __future__ import print_function

import argparse
import json
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict

import cppcheckdata


SOURCE_EXTENSIONS = (".cpp", ".cc", ".cxx", ".hpp", ".hh", ".h")
LOCK_METHODS = {
    "lock": "acquire",
    "try_lock": "acquire",
    "lock_shared": "acquire",
    "unlock": "release",
    "unlock_shared": "release",
}
RAII_TYPES = ("lock_guard", "unique_lock", "scoped_lock", "shared_lock")
CONTROL_WORDS = {"if", "for", "while", "switch", "return", "sizeof", "catch"}
# These are type-qualified effect summaries, not name-only call patterns.
# Cppcheck's std.cfg does not encode receiver mutation for these methods.
KNOWN_MUTATING_METHODS = {
    "std::shared_ptr": {"reset", "swap"},
    "std::unique_ptr": {"reset", "release", "swap"},
    "std::weak_ptr": {"reset", "swap"},
    "std::optional": {"reset", "emplace", "swap"},
}


class LockState(object):
    __slots__ = ("depths", "targets", "reachable")
    # The top value denotes "at least this many acquisitions".  It keeps
    # loops and recursive calls finite without losing an outer acquisition
    # when a nested guard releases the same recursive mutex.
    DEPTH_TOP = 4

    def __init__(self, depths=None, reachable=True, targets=None):
        self.depths = dict(depths or {})
        self.targets = {key: set(values) for key, values in (targets or {}).items()}
        self.reachable = reachable

    @property
    def may(self):
        return {name for name, (_, maximum) in self.depths.items() if maximum > 0}

    @property
    def must(self):
        return {name for name, (minimum, _) in self.depths.items() if minimum > 0}

    def copy(self):
        return LockState(self.depths, self.reachable, self.targets)

    def acquire(self, names):
        for name in names:
            minimum, maximum = self.depths.get(name, (0, 0))
            self.depths[name] = (
                min(self.DEPTH_TOP, minimum + 1),
                min(self.DEPTH_TOP, maximum + 1),
            )

    def release(self, names):
        for name in names:
            minimum, maximum = self.depths.get(name, (0, 0))
            minimum = max(0, minimum - 1)
            maximum = self.DEPTH_TOP if maximum == self.DEPTH_TOP else max(0, maximum - 1)
            if maximum:
                self.depths[name] = (minimum, maximum)
            else:
                self.depths.pop(name, None)

    def key(self):
        targets = tuple(sorted((key, tuple(sorted(values)))
                               for key, values in self.targets.items() if values))
        return (tuple(sorted(self.depths.items())), targets, self.reachable)

    @staticmethod
    def merge(states):
        states = [state for state in states if state and state.reachable]
        if not states:
            return LockState(reachable=False)
        names = set().union(*(state.depths.keys() for state in states))
        depths = {}
        for name in names:
            minimum = min(state.depths.get(name, (0, 0))[0] for state in states)
            maximum = max(state.depths.get(name, (0, 0))[1] for state in states)
            if maximum:
                depths[name] = (minimum, maximum)
        targets = defaultdict(set)
        for state in states:
            for key, values in state.targets.items():
                targets[key].update(values)
        return LockState(depths, targets=targets)


class FunctionModel(object):
    def __init__(self, analyzer, config, scope, dump_file):
        self.analyzer = analyzer
        self.config = config
        self.scope = scope
        self.function = scope.function
        self.dump_file = dump_file
        self.tokens = config.tokenlist
        self.index = {token.Id: pos for pos, token in enumerate(self.tokens)}
        self.start = self.index[scope.bodyStart.Id]
        self.end = self.index[scope.bodyEnd.Id]
        self.name = qualified_function_name(self.function)
        self.short_name = self.function.name or scope.className
        self.file = normalize_source_path(scope.bodyStart.file, analyzer.shadow_root)
        self.line = scope.bodyStart.linenr
        self.initializers = []
        self.initializer_colon = None
        definition = self.function.tokenDef
        parameters = definition.next if definition else None
        if parameters and parameters.str == "(" and parameters.link:
            after_parameters = self.index.get(parameters.link.Id, self.start) + 1
            colon = next((pos for pos in range(after_parameters, self.start)
                          if self.tokens[pos].str == ":"), None)
            if colon is not None:
                self.initializer_colon = colon
                for pos in range(colon + 1, self.start):
                    item = self.tokens[pos]
                    if (item.variable and item.variable.access in ("Public", "Protected", "Private")
                            and item.next and item.next.str in ("(", "{")):
                        self.initializers.append(item)
        self.deferred_assignments = defaultdict(list)
        for assignment in self.tokens[self.start + 1:self.end]:
            if not assignment.isAssignmentOp or not assignment.astOperand1:
                continue
            closes = analyzer._subtree_call_closes(assignment.astOperand2)
            if closes:
                last = max(closes, key=lambda item: self.index.get(item.Id, -1))
                self.deferred_assignments[last.Id].extend(
                    ast_variable_tokens(assignment.astOperand1)
                )

    @property
    def identity(self):
        return (self.name, self.file, self.line)


def enclosing_named_scope(scope):
    names = []
    current = scope
    while current:
        if current.type in ("Class", "Struct", "Namespace") and current.className:
            names.append(current.className)
        current = current.nestedIn
    return list(reversed(names))


def qualified_function_name(function):
    if not function:
        return "<unknown>"
    names = enclosing_named_scope(function.tokenDef.scope if function.tokenDef else None)
    name = function.name or (function.tokenDef.str if function.tokenDef else "<unknown>")
    if not names or name.startswith("::") or "::" in name:
        return name.strip(":")
    return "::".join(names + [name])


def normalize_source_path(path, shadow_root):
    if not path:
        return ""
    portable = path.replace("\\", "/")
    shadow_name = os.path.basename(shadow_root.rstrip("\\/"))
    marker = shadow_name + "/"
    marker_at = portable.lower().find(marker.lower())
    if marker_at >= 0:
        return portable[marker_at + len(marker):]
    path = os.path.abspath(os.path.join(shadow_root, path)) if not os.path.isabs(path) else path
    try:
        return os.path.relpath(path, shadow_root).replace("\\", "/")
    except ValueError:
        return path.replace("\\", "/")


def variable_type(variable):
    if not variable or not variable.typeStartToken:
        return ""
    parts = []
    token = variable.typeStartToken
    while token:
        parts.append(token.str)
        if token is variable.typeEndToken:
            break
        token = token.next
    return "".join(parts)


def variable_name(variable):
    if not variable or not variable.nameToken:
        return "<unknown>"
    name = variable.nameToken.str
    names = enclosing_named_scope(variable.scope)
    if names and not variable.isLocal and not variable.isArgument and variable.access not in ("Local", "Argument"):
        return "::".join(names + [name])
    return name


def ast_variable_tokens(token):
    found = []
    seen = set()

    def visit(current):
        if not current or current.Id in seen:
            return
        seen.add(current.Id)
        if current.variable:
            found.append(current)
        visit(current.astOperand1)
        visit(current.astOperand2)

    visit(token)
    return found


def ast_contains(root, sought):
    pending = [root]
    seen = set()
    while pending:
        current = pending.pop()
        if not current or current.Id in seen:
            continue
        if current is sought:
            return True
        seen.add(current.Id)
        pending.extend((current.astOperand1, current.astOperand2))
    return False


class LocksetAnalyzer(object):
    def __init__(self, project_root, cppcheck="cppcheck"):
        self.project_root = os.path.abspath(project_root)
        self.cppcheck = cppcheck
        self.shadow_root = None
        self.functions = []
        self.by_identity = {}
        self.by_short_name = defaultdict(list)
        self.by_qualified_name = defaultdict(list)
        self.accesses = {}
        self.active_calls = set()
        self.memo = {}
        self.reached_functions = set()
        self.dump_parse_errors = []
        self.source_lines = {}
        self.readonly_library_methods = self._load_readonly_library_methods()
        self.escaped_variables = set()
        self.initial_targets = {}

    def _load_readonly_library_methods(self):
        executable = shutil.which(self.cppcheck) or self.cppcheck
        install_dir = os.path.dirname(os.path.abspath(executable))
        candidates = (
            os.path.join(install_dir, "cfg", "std.cfg"),
            os.path.join(install_dir, "..", "share", "cppcheck", "cfg", "std.cfg"),
            os.path.join(os.sep, "usr", "share", "cppcheck", "cfg", "std.cfg"),
        )
        for path in candidates:
            try:
                root = ET.parse(path).getroot()
            except (OSError, ET.ParseError):
                continue
            names = set()
            for function in root.findall("function"):
                if function.find("const") is not None or function.find("pure") is not None:
                    names.update(name.strip() for name in function.get("name", "").split(","))
            return names
        return set()

    def run(self):
        with tempfile.TemporaryDirectory(prefix="roscallbax-lockset-") as shadow:
            self.shadow_root = shadow
            self._copy_sources(shadow)
            dumps = self._run_cppcheck(shadow)
            self._load_functions(dumps)
            self._index_functions()
            self._load_initial_function_targets()
            self._find_escaped_reference_captures()
            self._propagate_escaped_arguments()
            self._analyze_roots()
        return {"accesses": self._sorted_accesses()}

    def _copy_sources(self, destination):
        if os.path.isfile(self.project_root):
            files = [self.project_root]
            common = os.path.dirname(self.project_root)
        else:
            files = []
            common = self.project_root
            for root, dirs, names in os.walk(self.project_root):
                dirs[:] = [d for d in dirs if d not in {".git", "build", "install", "log"}]
                for name in names:
                    if name.lower().endswith(SOURCE_EXTENSIONS):
                        files.append(os.path.join(root, name))
        for source in files:
            relative = os.path.relpath(source, common)
            target = os.path.join(destination, relative)
            os.makedirs(os.path.dirname(target), exist_ok=True)
            shutil.copy2(source, target)

    def _run_cppcheck(self, shadow):
        include_dirs = {shadow}
        for root, dirs, names in os.walk(shadow):
            if any(name.lower().endswith((".h", ".hpp", ".hh")) for name in names):
                include_dirs.add(root)
        command = [
            self.cppcheck,
            "--dump",
            "--force",
            "--max-configs=1",
            "--language=c++",
            "--std=c++17",
            "--suppress=missingInclude",
            "--suppress=missingIncludeSystem",
        ]
        for include_dir in sorted(include_dirs):
            command.extend(["-I", include_dir])
        command.append(shadow)
        try:
            completed = subprocess.run(
                command, cwd=shadow, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                universal_newlines=True, check=False,
            )
        except OSError as error:
            raise RuntimeError("cannot execute Cppcheck: %s" % error)
        dumps = []
        for root, _, names in os.walk(shadow):
            dumps.extend(os.path.join(root, name) for name in names if name.endswith(".dump"))
        if not dumps:
            detail = (completed.stderr or completed.stdout).strip()
            raise RuntimeError("Cppcheck produced no dump files%s" % (": " + detail if detail else ""))
        return sorted(dumps)

    def _load_functions(self, dumps):
        identities = set()
        parsed_configurations = 0
        for dump_file in dumps:
            try:
                data = cppcheckdata.parsedump(dump_file)
            except Exception as error:
                self.dump_parse_errors.append((dump_file, str(error)))
                continue
            for config in data.configurations:
                parsed_configurations += 1
                for scope in config.scopes:
                    if scope.type != "Function" or not scope.function or not scope.bodyStart or not scope.bodyEnd:
                        continue
                    model = FunctionModel(self, config, scope, dump_file)
                    if model.identity not in identities:
                        identities.add(model.identity)
                        self.functions.append(model)
        if parsed_configurations == 0:
            details = "; ".join("%s: %s" % item for item in self.dump_parse_errors[:3])
            raise RuntimeError("Cppcheck dumps could not be parsed%s" % (": " + details if details else ""))

    def _index_functions(self):
        for model in self.functions:
            self.by_identity[model.identity] = model
            self.by_short_name[model.short_name].append(model)
            self.by_qualified_name[model.name].append(model)

    def _load_initial_function_targets(self):
        # Cppcheck does not put namespace-scope initializers in a function body.
        # Evaluate their function values once, before seeding any entry point.
        configurations = {}
        for model in self.functions:
            key = id(model.config)
            group = configurations.setdefault(key, {"model": model, "bodies": []})
            group["bodies"].append((model.start, model.end))
        initial = LockState()
        for group in configurations.values():
            model = group["model"]
            for pos, token in enumerate(model.tokens):
                if any(start <= pos <= end for start, end in group["bodies"]):
                    continue
                if (token.variable and token.variable.isGlobal and
                        token.variable.nameToken is token):
                    self._initialize_braced_function_target(model, pos, initial,
                                                             len(model.tokens))
                if (token.str == "=" and token.isAssignmentOp and token.astOperand1 and
                        token.astOperand1.variable and token.astOperand1.variable.isGlobal):
                    self._assign_function_target(model, token, initial)
        self.initial_targets = initial.targets

    def _find_escaped_reference_captures(self):
        for model in self.functions:
            tokens = model.tokens
            scan_start = model.initializer_colon if model.initializer_colon is not None else model.start
            calls = [token for token in tokens[scan_start + 1:model.end]
                     if token.str == "(" and token.astOperand2]
            for pos in range(scan_start + 1, model.end):
                capture = tokens[pos]
                if capture.str != "[" or not capture.astOperand2:
                    continue
                # An immediately invoked lambda does not escape merely by
                # capturing a local reference.
                parent = capture.astParent
                if parent and parent.str == "(" and parent.astOperand1 is capture:
                    continue
                referenced = [item.variable for item in ast_variable_tokens(capture.astOperand2)
                              if item.variable and item.astParent and item.astParent.str == "&"]
                pending = [capture.astOperand2]
                implicit_reference = False
                seen = set()
                while pending:
                    item = pending.pop()
                    if not item or item.Id in seen:
                        continue
                    seen.add(item.Id)
                    if item.str == "&" and not item.astOperand1 and not item.astOperand2:
                        implicit_reference = True
                    pending.extend((item.astOperand1, item.astOperand2))
                if implicit_reference:
                    body = next((item for item in tokens[pos + 1:model.end]
                                 if item.str == "{" and self._is_lambda_body(item)), None)
                    body_end = model.index.get(body.link.Id) if body and body.link else None
                    if body_end is not None:
                        referenced.extend(item.variable for item in tokens[model.index[body.Id] + 1:body_end]
                                          if item.variable and item.variable.nameToken
                                          and item.variable.access in ("Local", "Argument")
                                          and model.index.get(item.variable.nameToken.Id, pos) < pos)
                if not referenced:
                    continue
                escaped = False
                current = parent
                while current:
                    if current.str == "return":
                        escaped = True
                        break
                    if current.str == "(" and ast_contains(current.astOperand2, capture):
                        escaped = True
                        break
                    current = current.astParent
                escaped = escaped or any(
                    model.index[member.Id] < pos < model.index.get(member.next.link.Id, -1)
                    for member in model.initializers
                    if member.next and member.next.link
                )
                assignment = self._assignment_ancestor(capture)
                if assignment and assignment.astOperand1:
                    carriers = [item.variable for item in ast_variable_tokens(assignment.astOperand1)
                                if item.variable]
                    escaped = escaped or any(variable.isGlobal or variable.isStatic or
                                             variable.access in ("Public", "Protected", "Private")
                                             for variable in carriers)
                    carrier_ids = {variable.Id for variable in carriers}
                    escaped = escaped or any(
                        model.index[token.Id] > pos and
                        any(item.variable and item.variable.Id in carrier_ids
                            for item in ast_variable_tokens(token.astOperand2))
                        for token in calls
                    )
                if escaped:
                    self.escaped_variables.update(
                        (id(model.config), variable.Id) for variable in referenced
                    )

    def _propagate_escaped_arguments(self):
        changed = True
        while changed:
            changed = False
            for model in self.functions:
                start = model.initializer_colon if model.initializer_colon is not None else model.start
                for token in model.tokens[start + 1:model.end]:
                    callee = self._callee_token(token)
                    if not callee:
                        continue
                    actuals = cppcheckdata.getArguments(callee) or []
                    for candidate in self._call_candidates(model, token):
                        formals = [candidate.function.argument[key]
                                   for key in sorted(candidate.function.argument)]
                        for formal, actual in zip(formals, actuals):
                            if (id(candidate.config), formal.Id) not in self.escaped_variables:
                                continue
                            for item in ast_variable_tokens(actual):
                                identity = (id(model.config), item.variable.Id)
                                if identity not in self.escaped_variables:
                                    self.escaped_variables.add(identity)
                                    changed = True

    def _analyze_roots(self):
        called = set()
        referenced = set()
        for model in self.functions:
            for pos in range(model.start + 1, model.end):
                token = model.tokens[pos]
                called.update(candidate.identity for candidate in self._call_candidates(model, token))
                if token.function:
                    referenced.update(candidate.identity for candidate in self._function_models_for_symbol(token))
        roots = [model for model in self.functions if model.identity not in called]
        if not roots:
            roots = list(self.functions)
        # Analyze entry points before address-taken functions.  A callback
        # reached through a function pointer should not also be seeded with
        # an artificial empty-lock context.
        roots.sort(key=lambda model: (model.short_name != "main",
                                      model.identity in referenced, model.file, model.line))
        for root in roots:
            if root.identity not in self.reached_functions:
                self._analyze_function(root, LockState(targets=self.initial_targets))
        # A mutually-recursive component can have no zero-in-degree function.
        # Seed only components that were not already reached from a real root.
        for model in self.functions:
            if model.identity not in self.reached_functions:
                self._analyze_function(model, LockState(targets=self.initial_targets))

    def _analyze_function(self, model, entry, entry_aliases=None):
        aliases_key = tuple(sorted(
            (name, tuple(sorted(values))) for name, values in (entry_aliases or {}).items()
        ))
        memo_key = (model.identity, entry.key(), aliases_key)
        self.reached_functions.add(model.identity)
        if memo_key in self.memo:
            cached = self.memo[memo_key]
            return cached.copy()
        if memo_key in self.active_calls:
            return entry.copy()
        self.active_calls.add(memo_key)
        aliases = {name: set(values) for name, values in (entry_aliases or {}).items()}
        return_states = []
        if model.initializer_colon is not None:
            entry = self._analyze_block(model, model.initializer_colon, model.start,
                                        entry.copy(), aliases, return_states, [])
        result = self._analyze_block(
            model, model.start, model.end, entry.copy(), aliases, return_states, []
        )
        result = LockState.merge(([result] if result.reachable else []) + return_states)
        self.active_calls.remove(memo_key)
        self.memo[memo_key] = result.copy()
        return result

    def _analyze_block(self, model, start, end, state, aliases, return_states, active_raii,
                       loop_context=None):
        if not state.reachable:
            return state
        tokens = model.tokens
        local_aliases = dict(aliases)
        raii_marker = len(active_raii)
        pos = start + 1
        while pos < end and state.reachable:
            token = tokens[pos]
            if token.str == "try":
                try_body = self._brace_after(model, pos, end)
                if try_body:
                    before_try = state.copy()
                    normal = self._analyze_block(
                        model, try_body[0], try_body[1], state.copy(), local_aliases,
                        return_states, list(active_raii), loop_context,
                    )
                    next_pos = try_body[1] + 1
                    alternatives = [normal]
                    while next_pos < end and tokens[next_pos].str == "catch":
                        catch_body = self._brace_after(model, next_pos, end)
                        if not catch_body:
                            break
                        # An exception may leave the try body at different
                        # points.  Entry and normal exit bound the lock state
                        # without treating a catch-only return as unconditional.
                        catch_entry = LockState.merge([before_try, normal])
                        alternatives.append(self._analyze_block(
                            model, catch_body[0], catch_body[1], catch_entry,
                            local_aliases, return_states, list(active_raii), loop_context,
                        ))
                        next_pos = catch_body[1] + 1
                    state = LockState.merge(alternatives)
                    pos = next_pos
                    continue
            if token.str == "if":
                parsed = self._control_body(model, pos, end)
                if parsed:
                    condition_start, condition_end, body_start, body_end = parsed
                    condition_state = self._analyze_linear(model, condition_start, condition_end, state, local_aliases, active_raii)
                    then_state = self._analyze_block(
                        model, body_start, body_end, condition_state.copy(), local_aliases,
                        return_states, list(active_raii), loop_context,
                    )
                    next_pos = body_end + 1
                    else_state = condition_state.copy()
                    if next_pos < end and tokens[next_pos].str == "else":
                        else_body = self._brace_after(model, next_pos, end)
                        if else_body:
                            else_state = self._analyze_block(
                                model, else_body[0], else_body[1], condition_state.copy(),
                                local_aliases, return_states, list(active_raii), loop_context,
                            )
                            next_pos = else_body[1] + 1
                    state = LockState.merge([then_state, else_state])
                    pos = next_pos
                    continue
            if token.str in ("for", "while"):
                parsed = self._control_body(model, pos, end)
                if parsed:
                    condition_start, condition_end, body_start, body_end = parsed
                    increment_start = condition_end
                    if token.str == "for":
                        semicolons = [
                            index for index in range(condition_start, condition_end)
                            if tokens[index].str == ";"
                        ]
                        if len(semicolons) >= 2:
                            state = self._analyze_linear(
                                model, condition_start, semicolons[0], state,
                                local_aliases, active_raii,
                            )
                            entry = self._analyze_linear(
                                model, semicolons[0] + 1, semicolons[1], state,
                                local_aliases, active_raii,
                            )
                            increment_start = semicolons[1] + 1
                        else:
                            entry = self._analyze_linear(
                                model, condition_start, condition_end, state,
                                local_aliases, active_raii,
                            )
                    else:
                        entry = self._analyze_linear(
                            model, condition_start, condition_end, state,
                            local_aliases, active_raii,
                        )
                    loop_in = entry.copy()
                    break_states = []
                    for _ in range(4):
                        inner_loop = {
                            "base": len(active_raii), "break": [], "continue": [],
                        }
                        body_out = self._analyze_block(
                            model, body_start, body_end, loop_in.copy(), local_aliases,
                            return_states, list(active_raii), inner_loop,
                        )
                        break_states.extend(inner_loop["break"])
                        back_edges = ([body_out] if body_out.reachable else []) + inner_loop["continue"]
                        if token.str == "for" and increment_start < condition_end:
                            back_edges = [
                                self._analyze_linear(
                                    model, increment_start, condition_end, edge,
                                    local_aliases, list(active_raii),
                                )
                                for edge in back_edges
                            ]
                        widened = LockState.merge([entry] + back_edges)
                        if widened.key() == loop_in.key():
                            break
                        loop_in = widened
                    state = LockState.merge([loop_in] + break_states)
                    pos = body_end + 1
                    continue
            if token.str in ("break", "continue") and loop_context is not None:
                jumped = state.copy()
                for names in reversed(active_raii[loop_context["base"]:]):
                    jumped.release(names)
                loop_context[token.str].append(jumped)
                state.reachable = False
                break
            if token.str == "return":
                semi = self._statement_end(model, pos + 1, end)
                state = self._analyze_block(model, pos, semi, state, local_aliases,
                                            return_states, list(active_raii), loop_context)
                returned = state.copy()
                for names in reversed(active_raii):
                    returned.release(names)
                return_states.append(returned)
                state.reachable = False
                break
            if token.str == "{" and token.link and token.Id != tokens[start].Id:
                nested_end = model.index.get(token.link.Id)
                if nested_end is not None and nested_end <= end:
                    if self._is_lambda_body(token):
                        lambda_returns = []
                        immediate = self._lambda_is_immediately_invoked(token)
                        lambda_out = self._analyze_block(
                            model, pos, nested_end,
                            state.copy() if immediate else LockState(),
                            local_aliases, lambda_returns, [],
                        )
                        if immediate:
                            state = LockState.merge(
                                ([lambda_out] if lambda_out.reachable else []) + lambda_returns
                            )
                    else:
                        state = self._analyze_block(
                            model, pos, nested_end, state, local_aliases, return_states,
                            active_raii, loop_context,
                        )
                    pos = nested_end + 1
                    continue
            state = self._process_token(model, pos, state, local_aliases, active_raii)
            pos += 1
        if state.reachable:
            for names in reversed(active_raii[raii_marker:]):
                state.release(names)
        del active_raii[raii_marker:]
        return state

    def _analyze_linear(self, model, start, end, state, aliases, raii_locks):
        for pos in range(start, min(end, len(model.tokens))):
            state = self._process_token(model, pos, state, aliases, raii_locks)
        return state

    def _process_token(self, model, pos, state, aliases, raii_locks):
        token = model.tokens[pos]
        if not state.reachable:
            return state

        if token in model.initializers:
            variable = token.variable
            if (not self._is_lock_variable(variable)
                    and not any(item in variable_type(variable) for item in RAII_TYPES)):
                self._record_access(model, token, variable_name(variable), "WRITE", state)
            return state

        raii = self._raii_declaration(model, pos)
        if raii:
            alias, names, acquire_now = raii
            if names:
                aliases[alias] = set(names)
                if acquire_now:
                    state.acquire(names)
                    raii_locks.append(set(names))

        operation = self._lock_operation(token, aliases)
        if operation:
            kind, names, managed = operation
            if kind == "acquire":
                state.acquire(names)
                if managed:
                    raii_locks.append(set(names))
            else:
                state.release(names)
                if managed:
                    for index in range(len(raii_locks) - 1, -1, -1):
                        if raii_locks[index] == set(names):
                            del raii_locks[index]
                            break

        if token.str == "=" and token.isAssignmentOp:
            self._assign_function_target(model, token, state)
        if token.variable and token.variable.nameToken is token:
            self._initialize_braced_function_target(model, pos, state, model.end)

        access = self._memory_access(model, token, aliases)
        if access:
            names, kind, may_write = access
            for name in names:
                self._record_access(model, token, name, kind, state, may_write)

        candidates = self._call_candidates(model, token, state)
        if candidates:
            callee = self._callee_token(token)
            exits = [
                self._return_from_call(
                    state,
                    self._analyze_function(
                        candidate, self._call_entry(candidate, callee, model, state),
                        self._parameter_aliases(candidate, callee, aliases, model),
                    ),
                )
                for candidate in candidates
            ]
            state = LockState.merge(exits)
        if token.str == ")":
            self._record_deferred_assignment(model, token, state, aliases)
        return state

    def _control_body(self, model, pos, end):
        tokens = model.tokens
        open_paren = self._find_until(tokens, pos + 1, end, "(")
        if open_paren >= end or not tokens[open_paren].link:
            return None
        close_paren = model.index.get(tokens[open_paren].link.Id)
        if close_paren is None:
            return None
        body = self._brace_after(model, close_paren, end)
        if not body:
            return None
        return open_paren + 1, close_paren, body[0], body[1]

    def _brace_after(self, model, pos, end):
        tokens = model.tokens
        brace = self._find_until(tokens, pos + 1, end, "{")
        if brace >= end or not tokens[brace].link:
            return None
        close = model.index.get(tokens[brace].link.Id)
        return (brace, close) if close is not None else None

    @staticmethod
    def _is_lambda_body(body_start):
        signature = body_start.astParent
        capture = signature if signature and signature.str == "[" else (
            signature.astParent if signature else None
        )
        return bool(capture and capture.str == "[")

    @staticmethod
    def _lambda_is_immediately_invoked(body_start):
        signature = body_start.astParent
        capture = signature if signature and signature.str == "[" else (
            signature.astParent if signature else None
        )
        call = capture.astParent if capture else None
        return bool(capture and capture.str == "[" and call and call.str == "("
                    and call.astOperand1 is capture)

    @staticmethod
    def _find_until(tokens, start, end, spelling):
        pos = start
        while pos < end and tokens[pos].str != spelling:
            pos += 1
        return pos

    @staticmethod
    def _statement_end(model, start, end):
        pos = start
        while pos < end:
            token = model.tokens[pos]
            if token.str == ";":
                return pos
            if token.str in ("(", "[", "{") and token.link:
                close = model.index.get(token.link.Id)
                if close is not None and pos < close < end:
                    pos = close + 1
                    continue
            pos += 1
        return end

    def _raii_declaration(self, model, pos):
        token = model.tokens[pos]
        variable = token.variable
        if not variable or variable.nameToken is not token:
            return None
        type_name = variable_type(variable)
        if not any(lock_type in type_name for lock_type in RAII_TYPES):
            return None
        semi = self._find_until(model.tokens, pos + 1, model.end, ";")
        names = set()
        for item in model.tokens[pos + 1:semi]:
            if item.variable and self._is_lock_variable(item.variable):
                names.add(self._lock_name(item.variable))
        deferred = "defer_lock" in [item.str for item in model.tokens[pos:semi]]
        return token.str, names, not deferred

    def _lock_operation(self, token, aliases):
        if token.str not in LOCK_METHODS or not token.astParent or token.astParent.str not in (".", "::"):
            return None
        if token.astParent.astOperand2 is not token:
            return None
        receiver = token.astParent.astOperand1
        receiver_tokens = ast_variable_tokens(receiver)
        if not receiver_tokens:
            return None
        variable = receiver_tokens[-1].variable
        receiver_name = self._lock_name(variable)
        names = aliases.get(receiver_name) or aliases.get(receiver_tokens[-1].str)
        managed = bool(names) and any(item in variable_type(variable) for item in RAII_TYPES)
        if not names and self._is_lock_variable(variable):
            names = {receiver_name}
        if not names:
            return None
        return LOCK_METHODS[token.str], set(names), managed

    @staticmethod
    def _is_lock_variable(variable):
        type_name = variable_type(variable)
        if any(item in type_name for item in RAII_TYPES):
            return False
        return "mutex" in type_name or "spinlock" in type_name or "shared_mutex" in type_name

    @staticmethod
    def _lock_name(variable):
        if variable and variable.isLocal and not variable.isArgument:
            scope = variable.scope
            while scope and scope.type != "Function":
                scope = scope.nestedIn
            if scope and scope.function:
                return qualified_function_name(scope.function) + "::" + variable.nameToken.str
        return variable_name(variable)

    @staticmethod
    def _assignment_ancestor(token):
        current = token
        while current:
            if current.isAssignmentOp:
                return current
            current = current.astParent
        return None

    @staticmethod
    def _subtree_call_closes(token):
        closes = []
        seen = set()

        def visit(current):
            if not current or current.Id in seen:
                return
            seen.add(current.Id)
            if current.isName and current.next and current.next.str == "(" and current.next.link:
                closes.append(current.next.link)
            visit(current.astOperand1)
            visit(current.astOperand2)

        visit(token)
        return closes

    def _original_member_operator(self, token):
        # Cppcheck normalizes both '.' and '->' to '.' in its AST.  Its source
        # position still points to the original operator, which lets us tell
        # whether a call mutates the member object or only its pointee.
        relative = normalize_source_path(token.file, self.shadow_root)
        path = os.path.join(self.shadow_root, relative)
        if path not in self.source_lines:
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as stream:
                    self.source_lines[path] = stream.read().splitlines()
            except OSError:
                self.source_lines[path] = []
        lines = self.source_lines[path]
        if 0 < token.linenr <= len(lines):
            source = lines[token.linenr - 1]
            offset = token.column - 1
            if source[offset:offset + 2] == "->":
                return "->"
            if source[offset:offset + 1] == ".":
                return "."
        return None

    @staticmethod
    def _resolved_const_method(method):
        function = method.function
        definition = function.tokenDef if function else None
        open_paren = definition.next if definition else None
        if open_paren and open_paren.str == "(" and open_paren.link:
            after = open_paren.link.next
            return bool(after and after.str == "const")
        return None

    def _member_method_effect(self, token):
        current = token.astParent
        while current:
            method = current.astOperand2 if current.str == "." else None
            if (method and method.isName and method.next and method.next.str == "("
                    and token in ast_variable_tokens(current.astOperand1)):
                if self._original_member_operator(current) != ".":
                    return None
                owner_type = variable_type(token.variable).split("<", 1)[0].strip()
                if method.str in KNOWN_MUTATING_METHODS.get(owner_type, ()):
                    return "READ_WRITE", False
                if self._resolved_const_method(method) is True:
                    return "READ", False
                qualified = owner_type + "::" + method.str
                if qualified in self.readonly_library_methods:
                    return "READ", False
                return "READ", True
            current = current.astParent
        return None

    def _memory_access(self, model, token, aliases, defer_calls=True):
        variable = token.variable
        if not variable or variable.nameToken is token:
            return None
        if self._is_lock_variable(variable) or any(item in variable_type(variable) for item in RAII_TYPES):
            return None
        alias_names = aliases.get(variable_name(variable)) or aliases.get(token.str)
        shared = bool(alias_names) or self._shared_variable(variable, model.config)
        if not shared:
            return None
        kind = "READ"
        parent = token.astParent
        while parent:
            if parent.str in ("++", "--"):
                kind = "READ_WRITE"
                break
            if parent.isAssignmentOp and parent.astOperand1:
                lhs_ids = {item.Id for item in ast_variable_tokens(parent.astOperand1)}
                if token.Id in lhs_ids:
                    kind = "WRITE" if parent.str == "=" else "READ_WRITE"
                break
            parent = parent.astParent
        may_write = False
        if kind == "READ" and not variable.isConst:
            effect = self._member_method_effect(token)
            if effect:
                kind, may_write = effect
        assignment = self._assignment_ancestor(token)
        if defer_calls and assignment and assignment.astOperand1:
            lhs_ids = {item.Id for item in ast_variable_tokens(assignment.astOperand1)}
            if token.Id in lhs_ids and self._subtree_call_closes(assignment.astOperand2):
                return None
        return set(alias_names or (variable_name(variable),)), kind, may_write

    def _record_deferred_assignment(self, model, close_token, state, aliases):
        for lhs_token in model.deferred_assignments.get(close_token.Id, ()):
            access = self._memory_access(model, lhs_token, aliases, defer_calls=False)
            if access:
                names, kind, may_write = access
                for name in names:
                    self._record_access(model, lhs_token, name, kind, state, may_write)

    def _record_access(self, model, token, variable, kind, state, may_write=False):
        file_name = normalize_source_path(token.file, self.shadow_root) or model.file
        key = (model.name, variable, file_name, token.linenr)
        existing = self.accesses.get(key)
        if existing is None:
            self.accesses[key] = {
                "function": model.name,
                "variable": variable,
                "access_kind": kind,
                "location": {"file": file_name, "line": token.linenr},
                "may_locks": set(state.may),
                "must_locks": set(state.must),
                "may_write": may_write,
            }
            return
        if existing["access_kind"] != kind:
            existing["access_kind"] = "READ_WRITE"
        existing["may_locks"].update(state.may)
        existing["must_locks"].intersection_update(state.must)
        existing["may_write"] = existing["may_write"] or may_write

    @staticmethod
    def _callee_token(token):
        if not token or token.str != ")" or not token.link:
            return None
        callee = token.link.previous
        return callee if callee and callee.isName and callee.next is token.link else None

    def _shared_variable(self, variable, config):
        return bool(variable and (
            (id(config), variable.Id) in self.escaped_variables
            or variable.isGlobal or variable.isStatic
            or (not variable.isLocal and not variable.isArgument
                and variable.access in ("Public", "Protected", "Private"))
        ))

    def _parameter_aliases(self, candidate, callee, caller_aliases, caller_model):
        if not callee:
            return {}
        actuals = cppcheckdata.getArguments(callee) or []
        formals = [candidate.function.argument[key] for key in sorted(candidate.function.argument)]
        result = {}
        for formal, actual in zip(formals, actuals):
            if not formal.nameToken:
                continue
            # Passing an object by value copies it into the callee.  Only a
            # reference parameter denotes the caller's variable itself.
            if not (formal.isReference or "&" in variable_type(formal)):
                continue
            names = set()
            for actual_token in ast_variable_tokens(actual):
                actual_variable = actual_token.variable
                actual_name = variable_name(actual_variable)
                mapped = caller_aliases.get(actual_name) or caller_aliases.get(actual_token.str)
                if mapped:
                    names.update(mapped)
                elif ((id(candidate.config), formal.Id) in self.escaped_variables
                      or self._shared_variable(actual_variable, caller_model.config)
                      or self._is_lock_variable(actual_variable)):
                    names.add(actual_name)
            if names:
                result[formal.nameToken.str] = names
                result[variable_name(formal)] = names
        return result

    @staticmethod
    def _function_target_key(model, variable):
        if (variable.isGlobal or
                (not variable.isLocal and not variable.isArgument
                 and variable.access in ("Public", "Protected", "Private"))):
            return ("shared", variable_name(variable))
        return ("local", model.identity, variable.Id)

    def _function_models_for_symbol(self, token, resolved_only=False):
        qualified = qualified_function_name(token.function) if token.function else None
        candidates = list(self.by_qualified_name.get(qualified, ())) if qualified else []
        if not resolved_only or not candidates:
            candidates.extend(self.by_short_name.get(token.str, ()))
        unique = {}
        for candidate in candidates:
            unique[candidate.identity] = candidate
        return list(unique.values())

    def _function_value_targets(self, model, token, state):
        if not token:
            return set()
        if token.function:
            return {candidate.identity for candidate in self._function_models_for_symbol(
                token, resolved_only=True
            )}
        if token.variable:
            return set(state.targets.get(self._function_target_key(model, token.variable), ()))
        if token.str in ("&", "*", "::", "."):
            child = token.astOperand2 if token.str in ("::", ".") else (
                token.astOperand1 or token.astOperand2
            )
            return self._function_value_targets(model, child, state)
        if token.str == "?" and token.astOperand2 and token.astOperand2.str == ":":
            branch = token.astOperand2
            return (self._function_value_targets(model, branch.astOperand1, state) |
                    self._function_value_targets(model, branch.astOperand2, state))
        return set()

    def _assign_function_target(self, model, assignment, state):
        left = assignment.astOperand1
        if left and left.str in (".", "::"):
            left = left.astOperand2
        if not left or not left.variable:
            return
        key = self._function_target_key(model, left.variable)
        targets = self._function_value_targets(model, assignment.astOperand2, state)
        if targets:
            state.targets[key] = targets
        else:
            state.targets.pop(key, None)

    def _initialize_braced_function_target(self, model, pos, state, limit):
        variable = model.tokens[pos].variable
        if not variable or not variable.isPointer:
            return
        end = self._statement_end(model, pos + 1, limit)
        for index in range(pos + 1, end):
            opener = model.tokens[index]
            if opener.str == "=":
                return  # Assignment initialization is handled by its AST.
            if opener.str != "{" or not opener.link:
                continue
            close = model.index.get(opener.link.Id)
            if close is None or close > end:
                return
            targets = set()
            for value in model.tokens[index + 1:close]:
                if value.function:
                    targets.update(self._function_value_targets(model, value, state))
                elif value.variable:
                    targets.update(self._function_value_targets(model, value, state))
            if targets:
                state.targets[self._function_target_key(model, variable)] = targets
            return

    def _call_entry(self, candidate, callee, caller_model, caller_state):
        shared = {key: values for key, values in caller_state.targets.items()
                  if key[0] == "shared"}
        entry = LockState(caller_state.depths, caller_state.reachable, shared)
        actuals = cppcheckdata.getArguments(callee) or []
        formals = [candidate.function.argument[key]
                   for key in sorted(candidate.function.argument)]
        for formal, actual in zip(formals, actuals):
            targets = self._function_value_targets(caller_model, actual, caller_state)
            if targets:
                entry.targets[self._function_target_key(candidate, formal)] = targets
        return entry

    @staticmethod
    def _return_from_call(caller_state, callee_exit):
        result = caller_state.copy()
        result.depths = dict(callee_exit.depths)
        result.reachable = callee_exit.reachable
        for key in list(result.targets):
            if key[0] == "shared":
                del result.targets[key]
        result.targets.update({key: set(values) for key, values in callee_exit.targets.items()
                               if key[0] == "shared"})
        return result

    def _call_candidates(self, model, token, state=None):
        callee = self._callee_token(token)
        if not callee:
            return []
        if callee.str in CONTROL_WORDS or callee.str in LOCK_METHODS:
            return []
        if callee.variable and any(item in variable_type(callee.variable) for item in RAII_TYPES):
            return []
        if callee.variable:
            targets = (state.targets.get(self._function_target_key(model, callee.variable), ())
                       if state else ())
            return [self.by_identity[target] for target in sorted(targets)
                    if target in self.by_identity]
        # Keep same-name alternatives even when Cppcheck resolved one concrete
        # declaration.  This preserves conservative virtual/ambiguous targets.
        return self._function_models_for_symbol(callee)

    def _sorted_accesses(self):
        result = []
        for access in self.accesses.values():
            item = dict(access)
            if not item["may_write"]:
                del item["may_write"]
            item["may_locks"] = sorted(access["may_locks"])
            item["must_locks"] = sorted(access["must_locks"])
            result.append(item)
        return sorted(result, key=lambda item: (
            item["location"]["file"], item["location"]["line"], item["function"], item["variable"]
        ))


def analyze_project(project, output_path=None, cppcheck="cppcheck"):
    result = LocksetAnalyzer(project, cppcheck=cppcheck).run()
    default_dir = os.path.abspath(project) if os.path.isdir(project) else os.path.dirname(os.path.abspath(project))
    output_path = output_path or os.path.join(default_dir, "lockset.json")
    with open(output_path, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="Cppcheck-backed lightweight C++ lockset analysis")
    parser.add_argument("project", help="C++ project source directory or source file")
    parser.add_argument("-o", "--output", help="output path (default: PROJECT/lockset.json)")
    parser.add_argument("--cppcheck", default="cppcheck", help="Cppcheck executable")
    args = parser.parse_args(argv)
    analyze_project(args.project, args.output, args.cppcheck)


if __name__ == "__main__":
    main()

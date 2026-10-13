# Lightweight C++ Lockset Analysis

一个基于 Cppcheck AST、作用域和符号信息的轻量化 C++ 静态锁集分析工具。输入单个 C++ 源文件或整个源码目录，输出一份 `lockset.json`。工具聚焦锁集，不执行 ROS callback composition / executor 检查，也不是 race detector；目标项目不必成功编译。

本项目复用 ROSCallBaX 的 Cppcheck 前端及部分符号处理代码；Cppcheck dump 读取模块来自 Cppcheck 项目。许可与版权说明见 [LICENSE.txt](LICENSE.txt)。

## 环境

- Python 3（使用标准库运行分析器）
- [Cppcheck](https://cppcheck.sourceforge.io/) 命令行程序；`cppcheck` 应在 `PATH` 中，或通过 `--cppcheck` 指定路径
- 运行附带的完整测试时另需 `networkx`：`python -m pip install networkx`

无需配置 CMake、ROS 2 或目标项目的编译数据库。缺失第三方头文件时会尽量继续分析，但解析精度可能下降。

## 使用

分析整个项目目录（递归读取 `.cpp`、`.cc`、`.cxx`、`.hpp`、`.hh`、`.h`）：

```powershell
python .\src\lockset_analysis.py "D:\path\to\cpp_project"
```

默认在输入目录生成一份 `lockset.json`，其中汇总所有已分析文件的访问点。也可分析单个文件，或指定输出位置：

```powershell
python .\src\lockset_analysis.py "D:\path\to\sample.cpp" -o "D:\path\to\result.json"
python .\src\lockset_analysis.py "D:\path\to\cpp_project" --cppcheck "C:\Program Files\Cppcheck\cppcheck.exe"
```

Linux/macOS 使用相同的 Python 命令，换成相应路径即可。仓库里的最小示例可这样运行：

```powershell
python .\src\lockset_analysis.py .\examples
```

分析器在临时目录运行 Cppcheck，不改动输入源码。缺失 include 会被容忍；但如果 Cppcheck 无法解析某段语法，该部分结果仍可能缺失。

## 输出含义

```json
{
  "accesses": [
    {
      "function": "Counter::increment",
      "variable": "Counter::value_",
      "access_kind": "READ_WRITE",
      "location": {"file": "counter.cpp", "line": 7},
      "may_locks": ["Counter::mutex_"],
      "must_locks": ["Counter::mutex_"]
    }
  ]
}
```

`access_kind` 为 `READ`、`WRITE` 或 `READ_WRITE`。`may_locks` 表示已分析路径中可能持有的锁（并集），`must_locks` 表示每条已分析路径都持有的锁（交集）。同一源码访问点经多条路径到达时只输出一条合并结果。`may_write: true` 是某些未完全解析的成员方法可能修改接收对象的保守标记。

当 lambda 被传给**实现不可见**的调用时，工具不知道它何时被执行：此时调用点持有的锁只保留在 `may_locks`，`must_locks` 不会据此增加，并用可选字段 `"lock_context_unknown": true` 标记这种不确定性。例如 `condition_variable::wait(guard, pred)` 的谓词可能出现该标记；它**不表示**工具已经建模了 `wait` 的标准库语义。明确的 `std::async` / `std::thread` / `std::jthread` 启动与普通未知调用分开处理。

## 分析范围与限制

- 识别 `lock()`、`unlock()`、`try_lock()`、`lock_shared()`、`unlock_shared()`，以及 `std::lock_guard`、`std::unique_lock`、`std::scoped_lock`、`std::shared_lock` 的常见 RAII 用法。
- 使用轻量控制流处理顺序语句、条件、循环、返回、函数调用和作用域释放；支持跨文件函数解析和有限的函数指针目标传播。
- 优先输出类成员、全局/静态变量及明显逃逸的共享变量，过滤普通局部变量。
- 不模拟完整 C++ 编译器或标准库。`try_lock()` 的成功/失败、复杂模板与宏、异常、别名、动态派发、异步执行、锁深度等情形可能产生保守或不完整结果。
- `lockset.json` 是访问点锁集摘要，不是数据竞争、死锁或线程安全结论。

## 测试

```powershell
python -m unittest test_lockset_analysis -q
```

两个集成测试需要原开发工作区里的 `deadlock-master` 案例，另一个测试需要原 ROSCallBaX 命令行入口；独立克隆本仓库时会跳过这三个可选测试。

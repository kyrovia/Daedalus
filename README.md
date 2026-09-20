# Daedalus

固定基座机械臂的力矩控制库。Pinocchio 提供动力学，控制器算出关节力矩，`ControlPipeline` 再做软件限幅（参考裁剪、力矩幅值/斜率限制）。

这不是认证安全功能。限幅只约束软件指令；错误的 `SafetyLimits` 仍然会发出错误力矩。

当前只支持 `nq == nv` 的固定基座模型。

## 依赖

- C++17
- Eigen3
- Pinocchio
- Catch2 3（仅测试）

CMake 会优先使用 `DAEDALUS_DEPENDENCY_PREFIX`，否则使用 `CONDA_PREFIX`。

## 构建

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

示例：

```bash
./build/daedalus_control_step tests/fixtures/two_link.urdf
```

下游工程：

```cmake
find_package(daedalus REQUIRED)
target_link_libraries(your_target PRIVATE daedalus::control)
```

## 用法

周期内走 `ControlPipeline::step`。成功时 `result.command` 指向管线内部缓冲，下一拍 `step()` 或成功的 `reset()` 会使其失效，用之前先拷贝力矩。

失败时检查 `result.status`，不要只看指针是否为空。默认 `kNoCommand`；`kHoldLast` 会带上非 ok 状态重放上一拍指令，不会自动把已经顶限位的力矩反过来。

提供的控制器：

- `GravityCompensator`
- `JointImpedanceController`
- `ComputedTorqueController`
- `CartesianImpedanceController`
- `OperationalSpaceController`

关节参考由管线裁剪。笛卡尔参考只做合法性检查，任务误差限幅在控制器配置里（`limit_error` / `error_clip`）。笛卡尔阻抗是对当前任务速度阻尼，没有期望速度前馈。

若 OSC 跑在管线里，把 `OperationalSpaceConfig::joint_limit_lower/upper` 设成和 `SafetyLimits` 同一组数，否则关节排斥力可能进不去软件限位盒。

完整调用见 `examples/control_step.cpp`。

# Changelog

版本遵循 semver：对外契约是 `pub` 接口（`.mbti`），内部实现随时可改。
每条变更后面的短哈希指向本仓库的对应提交。

## [1.0.0] - 2026-10-23

首个稳定版本。64 项测试在 `moon test` 与 `moon test --target js` 上都全绿，
另有 `node tools/check-playground.mjs` 守着浏览器侧的 wasm 构建。

### Added

- **关节**（`3504622`）：`PrismaticJoint`（轴锁定 + 行程限位 + 电动）、
  `AngleLimitJoint`（上下限 + 暖启动）、`MotorJoint`（目标角速度 + 最大力矩）；
  距离关节支持 `with_softness(frequency, damping_ratio)` 软约束；
  `World` 侧补 `add_prismatic_joint` / `add_angle_limit_joint_at` /
  `add_motor_joint` / `add_soft_distance_joint`。
- **查询**（`bb530c0`）：`World::ray_cast`（最近命中，返回 point/normal/distance）、
  `query_aabb`、`query_point`；`ray.mbt` 提供 ray-circle 与 ray-polygon。
- **示例与建场 DSL**（`64df76f`）：`examples/bounce`、`examples/pyramid`、
  `examples/ragdoll`；`World::builder()` 链式建场；`World::snapshot_json()`
  输出可回放的 JSON；192 个 `pub fn` 补齐 doc comment。
- **基准**（`fc079ac`）：`bench/` 三场景（stack 101 体 / crowd 501 体 /
  chain 22 体，各 600 步），打印 ms/step、接触数与 `state_hash`。
- **交互演示**（`2c6d0c0`）：`playground/` 以 `foreign_library` 编译到 wasm-gc，
  导出 24 个纯数值 `pg_*` 函数；`docs/playground/` 用 canvas 渲染 6 个场景，
  支持拖拽、冲击波、暂停/单步/重置、重力与迭代滑块；`tools/build-playground.{ps1,sh}`
  重建产物，`tools/check-playground.mjs` 做无浏览器冒烟。
- **部署**（`a97e8bc`、`622dbda`）：`.github/workflows/pages.yml` 部署 `docs/`；
  Pages 未启用时只打 notice，不制造红叉。

### Changed

- 宽相从「每步重建数组 + 整表插入排序」改为增量 sort-and-sweep（跨帧复用排序
  数组与 pair 缓冲区），持久接触的查找从「每条 pair 扫全部 arbiter」改为按低位
  body 下标分桶（`fc079ac`）。方块塔 0.62 → 0.40 ms/step，状态摘要不变。
- 位置修正加 `MAX_CORRECTION` 上限与 `allowing_penetration` 留缝，深穿透不再弹飞
  （`655b2c9`）。
- 全仓 `Int` → `Int32`，适配 MoonBit 0.1.20261021（`78bcbd2`）。
- 模块名改为 `weopqrst/moonphys`，与 mooncakes 账号一致（`ca2e1e7`）。

### Fixed

- **睡眠唤醒策略**（`64df76f`）：原先「邻居醒着就唤醒我」会让堆里第一个入睡者
  每步被惊醒，整堆永远睡不下去；改成「邻居速度超过阈值才唤醒」后，5 层金字塔
  2 秒内 15/15 入睡，方块塔耗时同步下降。新增回归测试覆盖「整堆入睡 + 落地唤醒」。
- 退化多边形（共线、重复点、极小边）在构造时被剔除，不进窄相（`655b2c9`）。
- `render` 去掉弃用的 `StringBuilder::new` 调用（`c1f2dc0`）。
- 黑盒测试里的包内符号补前缀（`941d204`）。
- `Vec2` 运算符 impl 显式提升，消除 `implicit_impl_as_method` 警告
  （`2ad67a3`、`5155ff8`）。
- CI 在 check / fmt 失败时把诊断输出成可读注解（`7f499c5`、`a6d5758`）。

## [0.1.0] - 2026-10-23

首个公开版本（mooncakes）。包含 `Vec2`/`Transform`、圆与凸多边形、SAT + 参考面
裁剪、顺序冲量 + 暖启动、库仑摩擦锥、恢复系数、Baumgarte 偏置、sort-and-sweep
宽相、睡眠、距离/铰链关节、不可变快照、SVG 渲染与三场景演示页。

之后的射线查询、Prismatic/AngleLimit/Motor 关节、软约束、`bench`、`examples`、
`snapshot_json`、`World::builder` 与 wasm playground 都不在这个版本里。

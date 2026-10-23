# 架构

写给想读源码或想改引擎的人：这里解释**每个文件为什么存在**、一次 `step` 里发生了什么、
以及几个关键决策的取舍。API 用法见 [README](../README.md)。

## 模块图

```
                    ┌────────────────────────────────────────────┐
   用户代码 ───────▶│ world.mbt   World：调度 + 宽相 + 睡眠 + 查询 │
                    └───────┬───────────────────┬────────────────┘
                            │                   │
          ┌─────────────────┼───────────────────┼──────────────────┐
          ▼                 ▼                   ▼                  ▼
     math.mbt          shape.mbt            body.mbt        joint.mbt / joint_extra.mbt
     Vec2/Transform    圆/凸多边形+质量属性   速度/力/冲量/阻尼   距离/铰链/滑轨/角限位/马达
          │                 │                   │                  │
          └────────┬────────┴─────────┬─────────┘                  │
                   ▼                  ▼                            │
              collide.mbt        arbiter.mbt                       │
              SAT + 参考面裁剪    持久流形 + 顺序冲量 + 摩擦锥      │
                   │                  │                            │
                   └──────────┬───────┴────────────────────────────┘
                              ▼
                    ray.mbt（射线/区域查询）
                    snapshot_json.mbt（快照 JSON）
                    builder.mbt（链式建场）
                              │
              ┌───────────────┴────────────────┐
              ▼                                ▼
     render/ ──▶ cmd/main ──▶ docs/index.html   playground/ ──▶ docs/playground
     SVG 逐帧回放                                wasm-gc + canvas 交互
```

## 一次 `World::step(dt)` 里发生什么

`world.mbt:440` 起，顺序固定（固定顺序本身是确定性的一部分）：

1. **同步 AABB** —— 每个 body 按当前位姿重算包围盒（增量宽相要用）。
2. **宽相** —— `compute_pairs()`：复用上一帧按 x 排好序的数组做近序插入排序，
   sweep 出候选对；persistent contact 用按低位 body 下标建的桶查找。
3. **窄相** —— 每对跑 `collide()`，拿到 0–2 个接触点；命中已有 arbiter 就按
   feature id 迁移上一帧的冲量（暖启动），否则新建。
4. **唤醒** —— 若一方在动、另一方睡着且都是动态体，把睡着的叫醒。
   这里的「在动」用**速度阈值**，不是「醒着」——见下文。
5. **积分外力** —— 重力 + 外力、阻尼、限速；然后清空累积力（外力只对一步有效）。
6. **预备** —— 每个 arbiter / joint 用自己的有效质量算 bias 与初始冲量。
7. **迭代求解** —— `velocity_iterations` 次，依次求解 arbiter、再关节；
   只改速度，不改位置。
8. **积分位置** —— 位置与角度按新的速度推进。
9. **关节位置修正** —— 硬限位穿出的残余在这里投影回可行域。
10. **睡眠** —— 线/角速度都低于阈值持续 `TIME_TO_SLEEP = 0.5 s` 就睡；一有速度就清零计时。
11. **再同步 AABB**，`time += dt`。

## 每个文件的存在理由

| 文件 | 负责 | 为什么单独放 |
| --- | --- | --- |
| `math.mbt` / `math_ops.mbt` | `Vec2`、`Transform`、运算符与 `Show` | 值类型 + 内联运算符，避免求解器里出现临时对象；`Show` 只给调试用 |
| `shape.mbt` | 圆 / 凸多边形的几何、AABB、质量与转动惯量 | 形状是纯数据，构造时就把质心与惯量算好，`Body` 只持有指针 |
| `body.mbt` | 质量、速度、力/冲量、阻尼、睡眠标记、局部↔世界变换 | 所有「状态」集中在这里，求解器只读写字段，不做业务判断 |
| `collide.mbt` | SAT + 参考面/入射面裁剪，输出接触点 | 碰撞是纯函数（shape, transform, shape, transform → 点），好测 |
| `arbiter.mbt` | 持久接触流形、冲量累积、摩擦锥、暖启动、位置偏置 | 「跨帧记住冲量」是堆叠稳定的关键，必须与一次性碰撞分开 |
| `world.mbt` | 调度、宽相、睡眠、查询入口、关节与 body 的增删 | 唯一的可变全局；顺序改了就会影响确定性 |
| `joint.mbt` | 距离关节（含软弹簧）、铰链关节 | Box2D-Lite 风格的经典约束，先落地最常用的一对 |
| `joint_extra.mbt` | Prismatic（轴锁 + 行程限位 + 电动）、AngleLimit、Motor | 后加的关节单独成文件，避免把首个关节实现撑成巨型函数 |
| `ray.mbt` | `ray_cast`、`query_aabb`、`query_point` | 查询复用 shape 的几何与宽相判断，不碰求解器 |
| `snapshot_json.mbt` | 把快照序列化成单行 JSON | 消费方（前端、日志、回放）不该被迫理解 `Body` 内部字段 |
| `builder.mbt` | `World::builder()` 链式建场 | 只是语法糖，构建完与手写 `add_body` 逐位相同（有测试对拍） |
| `render/` | SVG 帧与整页文档 | 渲染只吃快照，不进物理循环 |
| `cmd/main/` | 生成 `docs/index.html`（3 场景 × 240 帧） | 演示是「求解器结果的录像」，浏览器只回放 |
| `playground/` | 导出 24 个 `pg_*` 函数给浏览器 | 包类型是 `foreign_library`，否则 `#export_name` 进不了 wasm 导出表 |
| `bench/` | 三个基准场景 | 性能数字要能一条命令复现，且带 `state_hash` 防「变快是因为算错了」 |

## 几个关键决策

**顺序冲量法，而不是 LCP / 投影高斯-赛德尔。**
每条约束只存一个累积冲量，迭代 10 次就能让堆叠站住；代码量小一个量级，
性能也够（101 体的 10×10 塔 0.40 ms/step）。代价是需要暖启动与位置偏置来补
「只改速度」的不足，这两块都在 `arbiter.mbt` 里显式写出来。

**SAT 加参考面偏置。**
`relative_tol = 0.98`、`absolute_tol = 0.001`：当两个轴的分离量几乎相等时，
宁可继续用上一帧的参考面，也不要在两帧之间来回翻面——翻面会让接触点的
feature id 对不上，暖启动失效，堆叠开始抖。

**位置修正有上限。**
`MAX_CORRECTION = 0.2` + `allowing_penetration = 0.01`：深穿透时一次性拉回会
给物体一个巨大的速度（看起来像被弹飞），所以每步最多修 0.2 m，并且允许
1 cm 的稳态穿透。

**增量宽相 + 接触分桶。**
物体在帧间基本是「近序」的，所以复用上一帧的排序数组做插入排序接近 O(n)。
持久接触的查找最初是「每条候选对扫全部 arbiter」，体数一多就是 O(n²)；
改成按低位 body 下标分桶后，方块塔 0.62 → 0.50 ms/step，摘要不变。

**睡眠的唤醒条件用速度阈值。**
原实现是「邻居 arbiter 存在且邻居醒着就唤醒我」——结果堆里第一个入睡者每步
被邻居叫醒，整堆永远睡不下去。改成「邻居**真的在移动**（速度超过睡眠阈值）
才唤醒」后，5 层金字塔 2 秒内 15/15 入睡，方块塔耗时再降一档。
这是 M4 里唯一一个会改变轨迹的行为修复，回归测试钉在 `world_test.mbt`。

**关节统一用有效质量 + 软约束（γ/β）。**
硬约束在 20 节链条上会积累抖动，`with_softness(frequency, damping_ratio)`
把约束变成弹簧-阻尼形式，代价是有微小柔度，收益是长链不发散。

**确定性是设计出来的。**
不引入随机数；宽相排序的所有 tie 按下标打破；接触点顺序由裁剪顺序决定；
关节按加入顺序求解；`Vec2` 是值类型，没有哈希遍历。于是「同样构造 + 同样步长
= 逐位相同的轨迹」，`state_hash()` 把每个 body 的位置/旋转/线速度/角速度按原始
bit 折成 FNV-1a 摘要，测试直接钉死某个场景 300 步后的数值。

**wasm 的 ABI 只走数值。**
`playground/engine.mbt` 的 24 个导出全是 `Int32` / `Double`：字符串与内存视图
一律不过边界，前端不需要胶水，`WebAssembly.instantiateStreaming` 之后直接调用。
渲染需要的多边形顶点也拆成 `pg_vertex_count` + `pg_vx/pg_vy` 两次数值调用。

## 怎么验证这些说法

| 说法 | 命令 / 文件 |
| --- | --- |
| 测试全绿（双 target） | `moon test`、`moon test --target js`（64 项） |
| 跨 target 逐位一致 | `moon run bench --target js` 与 `--target wasm-gc` 的 `hash=` 行；CI 直接 diff |
| 堆叠稳定、整堆入睡 | `examples/pyramid`（2 秒 15/15 入睡）、`world_test.mbt` |
| builder 与手写逐位相同 | `builder_test.mbt`（状态摘要对拍） |
| 快照结构完整 | `builder_test.mbt` 校验 `snapshot_json()` 条目数 |
| wasm 导出没退化 | `node tools/check-playground.mjs` |
| 交互页真的能跑 | `docs/playground/?selftest=1` 的 `document.title` |
| 性能数字 | `moon run bench`（README 表） |

## 边界：有意不做

3D、凹多边形分解、软体/布料、流体、多线程、内置编辑器、与具体游戏引擎绑定。
`moonphys` 的目标是「一晚上能读完、改得动」的 2D 刚体库，不是通用物理中间件。

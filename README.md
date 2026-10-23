# moonphys

**纯 MoonBit 实现的确定性 2D 刚体物理引擎** — 用 SAT 做碰撞检测，用顺序冲量法求解接触，支持摩擦、恢复系数、关节、睡眠，并能把每一帧渲染成 SVG。

一个 2D 物理引擎是游戏引擎、物理教学和机器人仿真里最基础的一块拼图，MoonBit 生态此前没有：`mooncakes` 上的 `chipmunk`、`box2d`、`verlet`、`aabb`、`collision` 等关键词全部为空。`moonphys` 填的是这个空位，而不是又一个业务工具。

```moonbit
let world = @moonphys.World::new()

// 静态地面
ignore(
  world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(10.0, 0.1)).at(0.0, -0.1),
  ),
)

// 一个自由下落的方块
ignore(world.add_body(@moonphys.Body::box(0.5, 0.5).at(0.0, 4.0)))

// 固定步长推进 2 秒
for _ in 0..<120 {
  world.step(1.0 / 60.0)
}

println(world.body(1).position().y()) // 0.5：稳稳落在有厚度的地面上
```

## 特性

| 模块 | 内容 |
| --- | --- |
| 数学基座 | `Vec2`、`Transform`、向量运算符重载、`Show` 实现 |
| 形状 | 圆、任意**凸**多边形（逆时针），自动计算质心与转动惯量（平行轴定理） |
| 宽相 | sort-and-sweep：按 AABB 下界插入排序后扫描，稀疏场景近似线性；tie 按 body 下标打破 |
| 窄相 | SAT + reference/incident edge 裁剪（圆/圆、多边形/圆、多边形/多边形），返回多点接触 |
| 求解器 | 顺序冲量法 + 持久接触（Arbiter 按 feature id 迁移冲量做**暖启动**）+ Baumgarte 位置偏置 |
| 摩擦 | 库仑锥，切向冲量钳制在 ±μ·Pn |
| 恢复系数 | 只在法向相对速度超过阈值时生效，避免静止抖动 |
| 关节 | `DistanceJoint`（定长）、`RevolveJoint`（铰链），用 2×2 有效质量矩阵求解 |
| 睡眠 | 线/角速度低于阈值持续 0.5s 后休眠，接触对的动态体互相唤醒 |
| 确定性 | 无随机、无哈希迭代、固定步长 + 稳定排序 → 两次运行逐位一致（有测试） |
| 渲染 | `render` 包把世界快照输出为 SVG，带网格与配色，可逐帧拼成动画 |

## 安装

```bash
moon add mihujiang/moonphys
```

在需要使用的包的 `moon.pkg` 中：

```
import {
  "mihujiang/moonphys" @moonphys,
}
```

## 演示

仓库自带一个把物理结果渲染成动画的生成器，输出**不依赖任何外部资源**的单文件 HTML（每帧一个内联 SVG，浏览器只负责回放）：

```bash
moon run cmd/main --target js > docs/index.html
```

生成的页面包含三个场景，各 240 帧、4 秒循环：

1. **Pyramid stack** — 6 层 21 个方块的金字塔，左上方斜面滚球（摩擦），三颗弹球砸下来。
2. **Five link chain · double pendulum** — 5 节链条与双摆，铰链约束在重力下的混沌运动。
3. **Plank bridge** — 13 块木板用距离关节连成的桥，两个静态桥墩，中间砸下三颗球。

在线预览：GitHub Pages 打开 `docs/index.html` 即可（仓库 Settings → Pages → Source 选 `main` + `/docs`）。

## API 速览

```moonbit
// 世界
let world = @moonphys.World::new()                 // 重力默认 (0, -9.81)
world.set_gravity(@moonphys.Vec2::new(0.0, -3.0))  // 低重力
world.set_velocity_iterations(10)                  // 迭代次数，越大越硬
world.set_sleep_enabled(true)
world.step(1.0 / 60.0)                             // 固定步长
world.run(120, 1.0 / 60.0)                         // 等价于循环 step
world.time()                                       // 累计仿真时间
world.arbiter_count()                              // 当前接触对
world.snapshot()                                   // 渲染/测试用的不可变快照

// 刚体
let body = @moonphys.Body::box(0.5, 0.5)           // 动态方块
  .at(0.0, 3.0)                                    // 质心位置
  .with_rotation(0.3)                              // 弧度
  .with_velocity(1.0, 0.0)
  .with_angular_velocity(-0.5)
  .with_friction(0.6)
  .with_restitution(0.2)                           // 弹性
  .with_density(1.0)                               // 质量 = 密度 × 面积
  .with_damping(0.05, 0.05)                        // 线/角阻尼
  .with_allow_sleep(true)
  .with_user_data(3)                               // 渲染配色用的自定义标记
ignore(world.add_body(body))

// 形状
@moonphys.Shape::box(0.5, 0.5)                     // 半宽半高
@moonphys.Shape::circle(0.4)
@moonphys.Shape::convex(vertices)                  // 逆时针凸多边形

// 力与冲量
body.apply_force(@moonphys.Vec2::new(0.0, 10.0))   // 本步有效
body.apply_impulse(@moonphys.Vec2::new(2.0, 0.0), body.position())
body.point_velocity(body.position())

// 查询
body.mass(); body.inertia(); body.aabb(); body.contains_point(p)
body.local_to_world(p); body.world_to_local(p)

// 关节
world.add_distance_joint(index_a, index_b)                     // 保持当前中心距
world.add_revolve_joint(index_a, index_b, @moonphys.Vec2::new(0.0, 5.0)) // 世界锚点
world.add_revolve_joint_local(a, b, local_a, local_b)          // 各自的局部锚点
```

## 实现要点

**碰撞检测**：多边形用 SAT 求最大分离轴；分离轴落在 A 上时以 A 为参考面，否则交换 A/B（`relative_tol = 0.98`、`absolute_tol = 0.001` 做偏置，避免两帧之间反复翻转）。确定参考面后找入射面，把入射线段裁剪到参考面的侧平面，保留 `separation < 0` 的点，最多两点。

**求解顺序**（`World::step`）：

1. 同步 AABB → 宽相配对
2. 窄相生成接触点；命中已有 Arbiter 则按 feature id 迁移旧冲量（暖启动），否则新建
3. 积分重力、外力、阻尼，清除累积力
4. 每个 Arbiter / 关节计算有效质量与偏置
5. 迭代 `velocity_iterations` 次求解速度约束
6. 积分位置 → 更新睡眠状态 → 同步 AABB

**为什么是顺序冲量法**：它只需要存储每条约束的累计冲量，迭代几次就能让堆叠稳定，代码量比 LCP/投影高斯-赛德尔求解器小一个量级，适合"看得懂、改得动"的库。10 次迭代下 6 层方块塔在 4 秒内不塌、不沉陷（有测试盯着）。

**确定性**：不引入随机数；宽相排序的所有 tie 都按下标打破；接触点顺序由裁剪顺序决定。因此同样的构造 + 同样的步长 = 逐位相同的轨迹，`world_test.mbt` 里有逐位比较的回归测试。

## 测试

```bash
moon test
```

覆盖：向量/变换数学、质量属性、AABB、圆与多边形的 7 类碰撞情形（含法向翻转）、自由落体解析解对照、落地静止、堆叠稳定性、恢复系数、睡眠、宽相剪枝、冲量、关节长度与锚点误差、演示场景数值健全性、SVG 输出结构。

## 移植来源与许可

`moonphys` 是原创的 MoonBit 实现，算法与工程结构参考了以下 MIT 许可的公开实现，**未包含其任何源码**：

- **Box2D-Lite** — Erin Catto，https://github.com/erincatto/box2d-lite （MIT）：Arbiter/顺序冲量、裁剪式多边碰撞的经典教学实现
- **Box2D** — Erin Catto，https://github.com/erincatto/box2d （MIT）：宽相、睡眠、关节的工业级参考
- **Planck.js** — Ali Shakiba，https://github.com/shakiba/planck.js （MIT）：JavaScript 移植版，本文的 API 风格参考

本项目以 **Apache-2.0** 发布，见 [LICENSE](LICENSE)。

## 边界与后续

当前明确不做（欢迎 PR）：连续碰撞检测（CCD）、凹多边形分解、关节角度限位与马达、传感器、软约束/布料、多线程求解。已规划的方向：CCD、更多关节类型、与 MoonBit GUI/游戏框架对接的适配层、性能基准（对照 Box2D-Lite 的基准场景）。

## 开发方式

本项目在 AI 编程助手（CodeBuddy / DeepSeek）的协助下完成：数学基底、SAT 裁剪、顺序冲量与关节求解的算法出处均列在上面的"移植来源"一节，代码是逐模块重写而不是复制；每一处物理行为都有对应的测试（38 项）与可复现的数值断言，因此任何一段实现都能被读、被改、被证伪。若你对某段策略有疑问（例如为什么偏置系数是 0.2、为什么恢复系数有速度阈值），可以直接在 issue 里问，代码里的注释和测试会一起回答。

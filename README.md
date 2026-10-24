# moonphys

[![CI](https://github.com/mihujiang/moonphys/actions/workflows/ci.yml/badge.svg)](https://github.com/mihujiang/moonphys/actions/workflows/ci.yml)
[![mooncakes](https://img.shields.io/badge/mooncakes-weopqrst%2Fmoonphys-7dd3fc)](https://mooncakes.io/docs/weopqrst/moonphys)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

**纯 MoonBit 实现的确定性 2D 刚体物理引擎** — 用 SAT 做碰撞检测，用顺序冲量法求解接触，支持摩擦、恢复系数、关节、睡眠，并能把每一帧渲染成 SVG。

文档：[五分钟上手](#五分钟上手) · [API 速览](#api-速览) · [架构与取舍](docs/ARCHITECTURE.md) · [CHANGELOG](CHANGELOG.md)

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
| 宽相 | **增量** sort-and-sweep：复用上一帧的排序数组，只对近序数组做插入排序（每步近乎 O(n)）；pair 缓冲区与持久接触桶都跨帧复用，tie 按 body 下标打破 |
| 窄相 | SAT + reference/incident edge 裁剪（圆/圆、多边形/圆、多边形/多边形），返回多点接触 |
| 求解器 | 顺序冲量法 + 持久接触（Arbiter 按 feature id 迁移冲量做**暖启动**）+ Baumgarte 位置偏置 |
| 摩擦 | 库仑锥，切向冲量钳制在 ±μ·Pn |
| 恢复系数 | 只在法向相对速度超过阈值时生效，避免静止抖动 |
| 关节 | `DistanceJoint`（定长/软弹簧）、`RevolveJoint`（铰链）、`PrismaticJoint`（滑轨+行程限位+电动）、`AngleLimitJoint`（角限位）、`MotorJoint`（角马达），统一用有效质量矩阵求解 |
| 限位 | 硬限位按纯速度约束求解（不反弹），残余穿出由积分后的位置投影消除 |
| 睡眠 | 线/角速度低于阈值持续 0.5s 后休眠，接触对的动态体互相唤醒 |
| 确定性 | 无随机、无哈希迭代、固定步长 + 稳定排序 → 两次运行逐位一致（有测试） |
| 渲染 | `render` 包把世界快照输出为 SVG，带网格与配色，可逐帧拼成动画 |
| 交互演示 | `playground/` 编译成 wasm-gc，导出 24 个纯数值 `pg_*` 函数，浏览器里拖拽刚体、点击施力 |

## 安装

```bash
moon add weopqrst/moonphys
```

在需要使用的包的 `moon.pkg` 中：

```
import {
  "weopqrst/moonphys" @moonphys,
}
```

## 五分钟上手

```moonbit
// main.mbt
fn main {
  let world = @moonphys.World::builder()
    .ground(20.0)                        // 静态地面，上表面 y = 0
    .ball(0.4, 0.0, 6.0)                 // 半径 0.4，位于 (0, 6)
    .box(0.5, 0.5, 1.5, 8.0)             // 半宽 0.5、半高 0.5
    .body(@moonphys.Body::ball(0.3).at(-2.0, 9.0).with_restitution(0.8))
    .build()
  for _ in 0..<240 {
    world.step(1.0 / 60.0)
  }
  println(world.snapshot_json())          // 直接喂给前端/回放
}
```

```bash
moon run examples/bounce      # 20 行：弹球落地，逐步打印位置与速度
moon run examples/pyramid     # 金字塔随动能衰减整体入睡
moon run examples/ragdoll     # 四段布娃娃，铰链连成的链条
```

`World::builder()` 只是语法糖，链式写完的场景和逐个 `add_body` 得到的世界逐位相同（`builder_test.mbt` 用状态摘要对拍）。需要关节时仍用 `add_body` 拿回下标再连：

```moonbit
let world = @moonphys.World::new()
let a = world.add_body(@moonphys.Body::box(0.2, 0.05).at(0.0, 5.0))
let b = world.add_body(@moonphys.Body::box(0.2, 0.05).at(0.4, 5.0))
world.add_revolve_joint(a, b, @moonphys.Vec2::new(0.2, 5.0))
```

## 演示

仓库自带一个把物理结果渲染成动画的生成器，输出**不依赖任何外部资源**的单文件 HTML（每帧一个内联 SVG，浏览器只负责回放）：

```bash
moon run cmd/main --target js > docs/index.html
```

生成的页面约 2.9 MB，包含三个场景，各 240 帧、4 秒循环（`docs/index.html` 已按 UTF-8 提交；Windows PowerShell 5.1 的 `>` 会写成 UTF-16，改用 `| Out-File -Encoding utf8` 或 git bash）：

1. **Pyramid stack** — 6 层 21 个方块的金字塔，左上方斜面滚球（摩擦），三颗弹球砸下来。
2. **Five link chain · double pendulum** — 5 节链条与双摆，铰链约束在重力下的混沌运动。
3. **Linear actuator · angular motor · soft rope** — 电动滑轨推箱、角马达带动的旋转臂与软弹簧绳、重力压在角限位上的跷跷板。

在线预览：<https://mihujiang.github.io/moonphys/>。`docs/` 由 `.github/workflows/pages.yml` 在每次推送后部署；首次需要在仓库 Settings → Pages 把 Source 选成 **GitHub Actions**（工作流自己建不了站点，没有 admin 权限），之后无需再动。

### 可交互 playground（wasm-gc）

`docs/playground/` 是可以直接上手玩的版本：真正的 `moonphys` 编译成 wasm-gc 后在浏览器里跑，canvas 逐帧渲染，鼠标和滑块直接改物理状态。

在线打开：`https://mihujiang.github.io/moonphys/playground/`。本地跑需要一个 HTTP 服务器（`file:` 协议下浏览器不允许取 `.wasm`）：

```bash
python -m http.server 8123        # 仓库根目录
# 浏览器打开 http://127.0.0.1:8123/docs/playground/
```

操作：

| 输入 | 效果 |
| --- | --- |
| 按住刚体拖动 | 用临界阻尼弹簧把刚体拉向光标（`pg_grab_*`），拖动中照样参与碰撞 |
| 单击空白处 | 半径 3 m 的衰减冲量（`pg_blast`） |
| 滚轮 | 缩放视图 |
| 空格 / `R` / 数字键 | 暂停继续 / 重置当前场景 / 切换到第 n 个场景 |
| 底部滑块 | 重力大小、速度迭代次数 |

内置 6 个场景：五层金字塔、九层塔 + 双球、200 圆密集堆、四节链 + 距离关节单摆、马达/滑轨/角限位同场、多形状落斜面。HUD 显示 FPS、刚体数、接触数与已入睡数（睡眠中的刚体半透明绘制）。

引擎侧只有 `playground/engine.mbt`：`moon.pkg` 声明 `pkgtype(kind: "foreign_library")`，再给函数加 `#export_name("pg_...")`，ABI 全用 `Int32` / `Double`，字符串和内存视图一律不过边界——所以前端不需要任何胶水代码，`WebAssembly.instantiateStreaming` 之后直接调用导出函数。

改完引擎后重新生成产物（`docs/playground/engine.wasm` 已提交，Pages 才能免工具链直接服务）：

```bash
powershell -File tools/build-playground.ps1   # 或 bash tools/build-playground.sh
node tools/check-playground.mjs               # 校验 24 个导出 + 6 场景各 180 步
```

页面自带 `?selftest=1` 开关：加载 wasm 后跑完 6 个场景各 180 步、各来一次冲击，再走一遍完整帧循环，把 FPS/刚体/接触/入睡数写进 `document.title`；无头浏览器用 `--dump-dom` 就能抓这行结果，例如 `selftest ok scenes=6 bodies=253 contacts=24 asleep=15 hud=10/16/24/15`。

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
world.snapshot_json()                              // 同上，序列化成 JSON
world.state_hash()                                 // 状态指纹，用于确定性回归

// 链式建场景（等价于逐个 add_body）
@moonphys.World::builder()
  .gravity(0.0, -9.81).iterations(10).sleep_enabled(true)
  .ground(20.0).ball(0.4, 0.0, 4.0).box(0.5, 0.5, 1.0, 6.0)
  .body(some_preconfigured_body)
  .build()

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
world.add_prismatic_joint(a, b, anchor, axis)                  // 滑轨，可 .with_limits/.with_motor
world.add_angle_limit_joint_at(a, b, anchor, -0.6, 0.6)        // 绕锚点的角限位
world.add_motor_joint(a, b, 2.0, 200.0)                        // 目标角速度, 最大力矩
world.add_soft_distance_joint(a, b, 4.0, 1.0)                  // 频率 Hz, 阻尼比
```

## 实现要点

**碰撞检测**：多边形用 SAT 求最大分离轴；分离轴落在 A 上时以 A 为参考面，否则交换 A/B（`relative_tol = 0.98`、`absolute_tol = 0.001` 做偏置，避免两帧之间反复翻转）。确定参考面后找入射面，把入射线段裁剪到参考面的侧平面，保留 `separation < 0` 的点，最多两点。

**求解顺序**（`World::step`）：

1. 同步 AABB → 宽相配对（复用上一帧的排序数组与 pair 缓冲区）
2. 窄相生成接触点；命中已有 Arbiter 则按 feature id 迁移旧冲量（暖启动），否则新建
3. 积分重力、外力、阻尼，清除累积力
4. 每个 Arbiter / 关节计算有效质量与偏置
5. 迭代 `velocity_iterations` 次求解速度约束
6. 积分位置 → 更新睡眠状态 → 同步 AABB

**为什么是顺序冲量法**：它只需要存储每条约束的累计冲量，迭代几次就能让堆叠稳定，代码量比 LCP/投影高斯-赛德尔求解器小一个量级，适合"看得懂、改得动"的库。10 次迭代下 6 层方块塔在 4 秒内不塌、不沉陷（有测试盯着）。

**确定性**：不引入随机数；宽相排序的所有 tie 都按下标打破；接触点顺序由裁剪顺序决定；关节按加入顺序求解。因此同样的构造 + 同样的步长 = 逐位相同的轨迹。

`World::state_hash()` 把每个 body 的位置、旋转、线速度、角速度按原始 bit 折进一个 FNV-1a 摘要，`world_test.mbt` 把某个固定场景 300 步后的摘要钉成常量——`moon test` 与 `moon test --target js` 跑的是同一条断言，任一后端出现 1 ulp 的差异都会让测试失败。CI 里还有一条更直接的对拍：同一组 bench 场景在 `wasm-gc` 与 `js` 上各跑 600 步，比对摘要行。

## 性能

```bash
moon run bench --target js        # 或 --target wasm-gc
```

三个场景各跑 600 步固定步长（`dt = 1/60`），输出每步毫秒数、body 数、接触数与状态摘要：

| 场景 | bodies | 接触数 | js ms/step | wasm-gc ms/step |
| --- | --- | --- | --- | --- |
| stack：10×10 方块塔 | 101 | 143 | 0.40 | 0.49 |
| crowd：500 个相切圆密集堆 | 501 | 883 | 1.13 | 1.21 |
| chain：20 节刚性链 | 22 | 17 | 0.07 | 0.09 |

（Windows 11 / Intel Core Ultra 9 275HX / Node 24；同机复跑 stack 在 0.39–0.45 之间波动，关注量级即可。两个 target 的 `hash=` 行完全相同，说明轨迹逐位一致。）

做过的两次改动：

1. **宽相**：原先每步重建数组并整表插入排序，改为跨帧复用排序结果 + 近序插入排序，并把持久接触的查找从「每条 pair 扫全部 arbiter」改成按低位 body 下标分桶。方块塔从 0.62 → 0.50 ms/step，摘要不变（说明只是变快，没有改变轨迹）。
2. **唤醒策略**：原先「邻居醒着就唤醒我」会让堆里的第一个入睡者每步被邻居惊醒，整堆永远睡不下去；改成邻居必须真的在移动（速度超过睡眠阈值）才唤醒后，静置的堆会整体入睡、不再参与求解，方块塔因此降到 0.40 ms/step。

## 测试

```bash
moon test                # 64 项
moon test --target js    # 同一套断言在 JS 后端上再跑一遍
node tools/check-playground.mjs    # playground 的 wasm：24 个导出 + 6 场景各 180 步
powershell -File tools/acceptance.ps1   # 模拟验收：克隆仓库 + 从 mooncakes 装包，各跑一遍
```

覆盖：向量/变换数学、质量属性、AABB、圆与多边形的 7 类碰撞情形（含法向翻转）、自由落体解析解对照、落地静止、堆叠稳定性、恢复系数、睡眠、整堆入睡与落地唤醒、宽相剪枝、增量宽相与暴力枚举逐步对拍、宽相能接纳后加入的 body、冲量、射线与 AABB/点查询、17° 斜面静摩擦不起滑、动量守恒、关节长度与锚点误差、滑轨只沿轴运动与行程限位、角马达达到目标角速度、角限位夹住重力负载、软关节收敛、20 节链条不发散、builder 与手写场景逐位一致、快照 JSON 覆盖每个 body、状态摘要可复现且钉死常量、演示场景数值健全性、SVG 输出结构。

## 开发方式：AI 参与，但每步都要有证据

这个项目是人与 AI 结对做的，先把分工和验收规则写清楚，避免「看起来能用」：

- **人定目标与边界**：里程碑、范围（2D、凸多边形、不做软体/流体）、性能与正确性的口径。
- **AI 写实现**：引擎、测试、文档、CI、SVG 回放与 wasm playground。
- **验收只看命令**：每个里程碑结束必须同时通过 `moon fmt --check`、`moon check --deny-warn`、`moon test`、`moon test --target js`、`moon package`；写进 README 的数字必须先在本机跑出来。

针对几类容易「自说自话」的地方，具体做法是：

1. **正确性用解析解对拍**：自由落体对照 `½gt²`，斜面滑落对照 `g(sinθ − μcosθ)`（误差 < 5%），等质量对撞对照动量守恒（误差 < 1e-9），而不是「看着像对的」。
2. **确定性用指纹盯**：`state_hash()` 把原始 bit 折进 FNV-1a，测试钉死 300 步后的常量；CI 再让 bench 在 wasm-gc 与 js 上各跑一遍并 diff `hash=` 行。
3. **性能必须同时看摘要**：优化后如果变快但摘要变了，说明轨迹被改动了，按 bug 处理——M3 的宽相优化就是靠这条确认「只是变快」。
4. **演示要能被机器检查**：`docs/playground/?selftest=1` 把自检结果写进 `document.title`，无头浏览器 `--dump-dom` 抓一行就能判定；`tools/check-playground.mjs` 在 CI 里核对 24 个导出与 6 个场景。
5. **人读 diff**：AI 的总结不作为证据，改动逐条过 diff，数字与结论都可以用上面的命令复现（清单见[架构文档](docs/ARCHITECTURE.md#怎么验证这些说法)）。

AI 在这个项目里踩过、并已修掉的真实问题（都补了回归测试或 CI 步骤）：把 wasm 包声明成 `executable` 导致 `#export_name` 进不了导出表（`[4219]`）；睡眠唤醒条件写成「邻居醒着就唤醒」让整堆永远睡不着；位置修正没有上限导致深穿透被弹飞；M6 重新生成演示页时发现旧文件是 PowerShell 重定向写出的 UTF-16，已改回 UTF-8。

一句话概括这套流程：**规格 → 实现 → 数值对拍 → 跨 target 摘要 → 人读 diff**，任何一环没有输出就不算完成。

## 移植来源与许可

`moonphys` 是原创的 MoonBit 实现，算法与工程结构参考了以下 MIT 许可的公开实现，**未包含其任何源码**：

- **Box2D-Lite** — Erin Catto，https://github.com/erincatto/box2d-lite （MIT）：Arbiter/顺序冲量、裁剪式多边碰撞的经典教学实现
- **Box2D** — Erin Catto，https://github.com/erincatto/box2d （MIT）：宽相、睡眠、关节的工业级参考
- **Planck.js** — Ali Shakiba，https://github.com/shakiba/planck.js （MIT）：JavaScript 移植版，本文的 API 风格参考

本项目以 **Apache-2.0** 发布，见 [LICENSE](LICENSE)。

# 验收材料（v0.2.0）

提交表单要的三样东西，以及每条硬指标对应的**可复现证据**。所有数字都来自本机实跑，
命令写在旁边，换台机器重跑即可复核（机器不同，ms/step 会变，量级不变）。

## 三件套

| 项 | 地址 |
| --- | --- |
| 仓库 | <https://github.com/mihujiang/moonphys> |
| mooncakes | <https://mooncakes.io/docs/weopqrst/moonphys> |
| 演示（Pages） | <https://mihujiang.github.io/moonphys/> · 交互版 <https://mihujiang.github.io/moonphys/playground/> |

> Pages 已于 2026-10-24 启用（Source = GitHub Actions），两个地址均实测 200：
> 根页 2,929,022 字节 / 720 帧 / 3 场景，`/playground/` 的 `?selftest=1` 返回
> `scenes=6 bodies=253 contacts=24 asleep=15`，`engine.wasm` 的
> `Content-Type` 为 `application/wasm`。之后每次 `docs/` 变动由
> `.github/workflows/pages.yml` 自动部署；fork 后首次仍需 owner 在 Settings → Pages
> 选一次 GitHub Actions（工作流拿不到建站权限）。

## 六条硬指标

| 要求 | 证据 |
| --- | --- |
| 1. 以 MoonBit 为主实现 | 仓库内 `.mbt` 全覆盖，`moon.pkg` 无第三方依赖；`moon tree` 只有标准库 |
| 2. 仓库公开 + 持续提交 | 20+ 次语义化提交（`git log --oneline`），CI 每次推送全绿（Actions 徽章） |
| 3. README 能跑通、示例与测试可复现 | README 的「五分钟上手」片段可原样编译运行；`examples/{bounce,pyramid,ragdoll}` 与 `moon test` 已实跑；见下方「复现步骤」 |
| 4. 本期新增、非旧项目包装 | 首次提交 `b72ccf3` 为脚手架，此后所有代码、测试、文档均为本期新写 |
| 5. 开源合规、注明移植来源 | `LICENSE` 为 Apache-2.0；README「移植来源与许可」列出 Box2D-Lite / Box2D / Planck.js（均为 MIT），并声明未包含其源码；`CHANGELOG.md` 记录每个版本的能力边界 |
| 6. AI 可解释 | 见下方「AI 参与与验证流程」；`docs/ARCHITECTURE.md` 逐文件解释「为什么这样写」与关键取舍；每条结论都挂可执行命令 |

## AI 参与与验证流程

项目是人与 AI 结对完成的，分工和验收规则写在这里，避免「看起来能用」：

- **人定目标与边界**：里程碑、范围（2D、凸多边形、不做软体/流体）、性能与正确性的口径。
- **AI 写实现**：引擎、测试、文档、CI、SVG 回放与 wasm playground。
- **验收只看命令**：每个里程碑结束必须同时通过 `moon fmt --check`、`moon check --deny-warn`、`moon test`、`moon test --target js`、`moon package`；写进文档的数字必须先在本机跑出来。

针对几类容易「自说自话」的地方，具体做法是：

1. **正确性用解析解对拍**：自由落体对照 `½gt²`，斜面滑落对照 `g(sinθ − μcosθ)`（误差 < 5%），等质量对撞对照动量守恒（误差 < 1e-9），而不是「看着像对的」。
2. **确定性用指纹盯**：`state_hash()` 把原始 bit 折进 FNV-1a，测试钉死 300 步后的常量；CI 再让 bench 在 wasm-gc 与 js 上各跑一遍并 diff `hash=` 行。
3. **性能必须同时看摘要**：优化后如果变快但摘要变了，说明轨迹被改动了，按 bug 处理——宽相优化就是靠这条确认「只是变快」。
4. **演示要能被机器检查**：`docs/playground/?selftest=1` 把自检结果写进 `document.title`，无头浏览器 `--dump-dom` 抓一行就能判定；`tools/check-playground.mjs` 在 CI 里核对 24 个导出与 6 个场景。
5. **人读 diff**：AI 的总结不作为证据，改动逐条过 diff，数字与结论都可以用上面的命令复现（清单见[架构文档](docs/ARCHITECTURE.md#怎么验证这些说法)）。

AI 在这个项目里踩过、并已修掉的真实问题（都补了回归测试或 CI 步骤）：把 wasm 包声明成 `executable` 导致 `#export_name` 进不了导出表（`[4219]`）；睡眠唤醒条件写成「邻居醒着就唤醒」让整堆永远睡不着；位置修正没有上限导致深穿透被弹飞；重新生成演示页时发现旧文件是 PowerShell 重定向写出的 UTF-16，已改回 UTF-8。

一句话概括：**规格 → 实现 → 数值对拍 → 跨 target 摘要 → 人读 diff**，任何一环没有输出就不算完成。

## 数字

| 项 | 值 | 怎么得到 |
| --- | --- | --- |
| 版本 | 0.2.0（mooncakes 目前只接受 `0.x.y`，详见 CHANGELOG） | `moon.mod` |
| 测试 | 64 项，双 target 全绿 | `moon test` / `moon test --target js` |
| wasm 导出 | 24 个 `pg_*`，6 个场景 | `node tools/check-playground.mjs` |
| 性能 stack（101 体） | 0.40 / 0.49 ms/step（js / wasm-gc） | `moon run bench --target js`、`--target wasm-gc` |
| 性能 crowd（501 体） | 1.13 / 1.21 ms/step | 同上 |
| 性能 chain（22 体） | 0.07 / 0.09 ms/step | 同上 |
| 确定性 | 两个 target 的 `hash=` 行逐位相同 | 同上，或 CI 的 cross-target diff |
| 演示页 | `docs/index.html` 2.9 MB，3 场景 × 240 帧内联 SVG | `moon run cmd/main --target js` |

## 复现步骤

一条命令版（两条验收路径都跑：克隆仓库 + 从 mooncakes 装包）：

```powershell
powershell -File tools/acceptance.ps1
```

手动版：

```bash
moon check --deny-warn
moon fmt --check
moon test
moon test --target js
moon run examples/bounce
moon run examples/pyramid
moon run examples/ragdoll
moon run bench --target js
node tools/check-playground.mjs
python -m http.server 8123     # 然后打开 /docs/playground/
```

浏览器侧自检：打开 `docs/playground/?selftest=1`，页面标题会变成
`selftest ok scenes=6 bodies=253 contacts=24 asleep=15 hud=…`；无头浏览器可以直接抓：

```bash
msedge --headless=new --virtual-time-budget=6000 --dump-dom \
  "http://127.0.0.1:8123/docs/playground/?selftest=1" | grep "<title>"
```

## 本轮验收记录（2026-10-24）

`tools/acceptance.ps1` 的 14 项检查全部通过：

- 路径 A（从 GitHub 克隆后复跑）：`fmt --check`、`check --deny-warn`、`moon test`、
  `moon test --target js`、三个 examples、按 README 构造的 `bench`（两个 target 的
  `hash=` 逐位相同）、wasm 构建 + playground 冒烟（提交产物与新构建各一次）、
  `engine.wasm` 版本与 `moon.mod` 一致、`docs/index.html` 为 UTF-8 且 720 帧、`moon package`。
- 路径 B（从 mooncakes 安装 0.2.0）：独立模块里 5 个测试（README 片段 / 落球静止 /
  射线与区域查询 / 五类关节不发散 / 状态摘要可复现）在 wasm-gc 与 js 上各 5/5。

本轮发现并修复：提交的 `docs/playground/engine.wasm` 内嵌版本仍是 `0.1.0`
（改版本号后忘记重建，功能无影响但产物与源码不同步），已用 0.2.0 重建；
「产物版本必须等于 `moon.mod`」也写进了验收脚本，避免再漏。

## 已知限制（写在明处）

- **wasm ABI 只有数值**：字符串与内存视图不过边界，所以场景名、说明文字留在 JS 侧；
  这是有意为之，换来零胶水。
- **`file:` 协议打不开 playground**：浏览器不允许 `fetch` 本地 `.wasm`，页面会给出明确提示。
- **6 层以上金字塔有残余抖动**（0.1–0.5 m/s，不入睡）：需要块求解器或位置求解器，
  已记在 `PLAN` 的后续项里，不在 0.2.0 承诺范围内。
- **fork 后首次部署仍需 owner 点一次开关**：工作流自建站点会被 GitHub 拒绝
  （`Resource not accessible by integration`），这是权限边界，不是代码缺陷；
  本仓库已完成这一步。

# 验收材料（v0.2.0）

提交表单要的三样东西，以及每条硬指标对应的**可复现证据**。所有数字都来自本机实跑，
命令写在旁边，换台机器重跑即可复核（机器不同，ms/step 会变，量级不变）。

## 三件套

| 项 | 地址 |
| --- | --- |
| 仓库 | <https://github.com/mihujiang/moonphys> |
| mooncakes | <https://mooncakes.io/docs/weopqrst/moonphys> |
| 演示（Pages） | <https://mihujiang.github.io/moonphys/> · 交互版 <https://mihujiang.github.io/moonphys/playground/> |

> Pages 需要仓库 owner 在 Settings → Pages → Source 里选一次 **GitHub Actions**
> （工作流拿不到建站权限）。之后每次推送由 `.github/workflows/pages.yml` 自动部署。

## 六条硬指标

| 要求 | 证据 |
| --- | --- |
| 1. 以 MoonBit 为主实现 | 仓库内 `.mbt` 全覆盖，`moon.pkg` 无第三方依赖；`moon tree` 只有标准库 |
| 2. 仓库公开 + 持续提交 | 20+ 次语义化提交（`git log --oneline`），CI 每次推送全绿（Actions 徽章） |
| 3. README 能跑通、示例与测试可复现 | README 的「五分钟上手」片段可原样编译运行；`examples/{bounce,pyramid,ragdoll}` 与 `moon test` 已实跑；见下方「复现步骤」 |
| 4. 本期新增、非旧项目包装 | 首次提交 `b72ccf3` 为脚手架，此后所有代码、测试、文档均为本期新写 |
| 5. 开源合规、注明移植来源 | `LICENSE` 为 Apache-2.0；README「移植来源与许可」列出 Box2D-Lite / Box2D / Planck.js（均为 MIT），并声明未包含其源码；`CHANGELOG.md` 记录每个版本的能力边界 |
| 6. AI 可解释 | README「开发方式：AI 参与，但每步都要有证据」写明分工与验收流程；`docs/ARCHITECTURE.md` 逐文件解释「为什么这样写」与关键取舍；每条结论都挂可执行命令 |

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

## 已知限制（写在明处）

- **wasm ABI 只有数值**：字符串与内存视图不过边界，所以场景名、说明文字留在 JS 侧；
  这是有意为之，换来零胶水。
- **`file:` 协议打不开 playground**：浏览器不允许 `fetch` 本地 `.wasm`，页面会给出明确提示。
- **6 层以上金字塔有残余抖动**（0.1–0.5 m/s，不入睡）：需要块求解器或位置求解器，
  已记在 `PLAN` 的后续项里，不在 0.2.0 承诺范围内。
- **Pages 首次启用需要 owner 点一次开关**：工作流自建站点会被 GitHub 拒绝
  （`Resource not accessible by integration`），这是权限边界，不是代码缺陷。

# 分层与命名规则草案（BSP ⇄ Driver Contract）

日期：2026-05-25
作者：与会讨论草案（用于团队审阅）

目的：统一 `bsp` 与 `driver` 两层的职责、包含关系与命名约定，避免类型/接口重复定义与循环包含，便于渐进迁移。

一、总体原则

- BSP 是契约（contract）拥有者（上层面向 BSP），Driver 是契约的实现者（BSP 调用 driver）。
- 明确三类头文件角色：
  - *上层聚合头*：`bsp/bsp.h`（或 `bsp/api.h`）——供应用层/任务直接包含的稳定接口集合，只暴露 `esm_bsp_*` API。上层应只包含这个头，不直接包含具体子模块头。
  - *契约/实现头*：`bsp_require_drv_hal.h`（建议名）——定义 BSP 与 driver 之间共享的类型与 `esm_drv_*` 函数原型（供 driver 实现与 BSP 调用）。只被 `driver/*` 和 `bsp/*`（实现部分）包含，不给应用层包含。
  - *driver 私有头*：`driver/<module>/driver_<module>.h` —— driver 的内部实现/扩展 API（实现 `esm_drv_*`），可包含 `bsp_require_drv_hal.h`，但不包含 `bsp/bsp.h`。

二、命名约定

- BSP 对上层的函数：`esm_bsp_<module>_<action>()`（例如 `esm_bsp_pwm_set_duty()`）。
- Driver 实现：使用 `esm_drv_<module>_<action>()`（例如 `esm_drv_mcpwm_set_duty()`），签名由 `bsp_require_drv_hal.h` 声明。
- 配置与类型命名：将与硬件配置相关的类型放在契约头并用 `esm_bsp_<module>_cfg_t`（或 `esm_drv_<module>_cfg_t` 若仅 driver 内部）。优先使用 `esm_bsp_` 前缀表示这是 BSP 可见/契约的一部分。

三、包含规则（谁包含谁）

- 应用层/任务：只包含 `bsp/bsp.h`（聚合头）。
- BSP 实现文件（`.c`）：包含 `bsp/bsp.h`（上层聚合）、并包含 `bsp_require_drv_hal.h` 来调用 `esm_drv_*`。
- Driver 头/实现：包含 `bsp_require_drv_hal.h`（获得类型与函数声明），实现 `esm_drv_*`。
- 任何头文件都应尽量只包含声明所需的最小头（前向声明优先），以降低耦合。

四、文件/目录建议

- `main/bsp/bsp.h`（聚合）
  - 仅包含并重新导出子模块 BSP 头（或直接声明核心 `esm_bsp_*` API）。
- `main/bsp/bsp_require_drv_hal.h`（契约）
  - 放入所有 `esm_drv_*` 原型、回调类型、driver/硬件配置结构的声明。
- `main/driver/<module>/driver_<module>.h` 与 `.c`
  - 包含 `bsp_require_drv_hal.h` 并实现 `esm_drv_*`。
- `main/bsp/<module>/bsp_<module>.h` 与 `.c`
  - BSP 层实现，包含 `bsp/bsp.h`（或在实现文件包含 `bsp_require_drv_hal.h` 以调用 driver）。

五、迁移/重构顺序（逐模块）

1. 定义 contract：为模块创建或补齐 `bsp_require_drv_hal.h` 中的类型与 `esm_drv_*` 原型。
2. 修改 driver：让 driver 的头包含该 contract，并移除重复定义；实现 `esm_drv_*`。
3. 修改 BSP 实现：BSP `.c` 包含 contract（实现调用驱动的 `esm_drv_*`），BSP 头导出 `esm_bsp_*` 给上层。
4. 更新上层调用：确认上层仅通过 `bsp/bsp.h`（聚合头）调用 BSP API。
5. 编译与验证：每完成一步即做一次 fullClean + build，保证改动可编译。

六、具体示例（MCPWM、Encoder）

示例：`bsp_require_drv_hal.h`（节选）

```c
// bsp_require_drv_hal.h
#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef void (*esm_drv_mcpwm_period_cb_t)(void *user_ctx);

typedef struct {
    uint32_t timer_num;
    uint32_t freq_hz;
    int8_t phase_u_high_pin;
    int8_t phase_u_low_pin;
    int8_t phase_v_high_pin;
    int8_t phase_v_low_pin;
    int8_t phase_w_high_pin;
    int8_t phase_w_low_pin;
    uint32_t deadtime_ns;
} esm_bsp_pwm_cfg_t; // BSP-visible config

// driver API implemented by driver/mcpwm
esp_err_t esm_drv_mcpwm_init(const esm_bsp_pwm_cfg_t *cfg);
esp_err_t esm_drv_mcpwm_set_duty(uint8_t phase, float duty);
... 
```

Driver 头 `driver/mcpwm/driver_mcpwm.h` 应只包含 `bsp_require_drv_hal.h` 并实现 `esm_drv_*`。

BSP 层 `bsp/mcpwm/bsp_mcpwm.c` 调用 `esm_drv_*`；其头 `bsp/mcpwm/bsp_mcpwm.h` 或 `bsp/bsp.h` 向上层暴露 `esm_bsp_pwm_*`。

七、编译/验证步骤（每模块）

- 在开始改动前 `git branch feat/bsp-contract`。
- 第一步：提交 contract 头（无破坏性改动），运行 `fullClean` + build。
- 第二步：修改 driver 以包含 contract，编译修正直到通过。
- 第三步：修改 BSP 实现，确保接口签名不变（向上层兼容）。
- 第四步：更新上层包含（若必要），跑整套构建并在硬件上做 smoke test（例如 encoder read + pwm set）。

八、风险、回滚与测试策略

- 风险：若一次性重命名或移动头，会导致大量编译失败。遵循“先增后改、分步提交”的策略降低风险。
- 回滚：每个步骤提交一个原子 commit，若问题可回退到上一 commit。
- 测试：为关键模块（MCPWM/Encoder/CurrentSensor）编写小型 board-level smoke tests（示例任务），确保运行时行为正确。

九、下一步建议（优先级）

1. 在 `.doc/todo` 中把此草案存档并征求团队确认（已生成本文件）。
2. 以 `mcpwm` 为样板执行一次迁移（contract → driver → bsp → top），记录遇到的问题并调整草案。
3. 在迁移过程中，补充 `bsp/bsp.h` 的聚合导出，使上层调用统一化。

---

如需我把上面的草案直接落成仓库内的文档（已完成）、或把 `bsp_require_drv_hal.h` 的初始模板放进 `main/bsp` 并按 `mcpwm` 真正迁移一轮，请回复“开始迁移 mcpwm”。

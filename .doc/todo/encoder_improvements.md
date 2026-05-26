# Encoder 改进 TODO

日期：2026-05-25
记录人：自动记录（由 Copilot 生成）

目标：在保持当前阻塞读取稳定的前提下，逐步完善编码器 BSP/driver 的健壮性、性能与可维护性。当前优先级为让 FOC 控制尽快运行；非阻塞/性能优化放在后续迭代。

- [ ] 互斥（High）：在 `bsp/encoder` 的 SPI 访问处加入互斥保护（mutex 或 critical section），确保多任务/中断并发访问安全。
  - 说明：使用 FreeRTOS 的 `SemaphoreHandle_t` 或 ESP-IDF 的互斥 API，锁粒度为单次 SPI 事务。

- [ ] 读角度重试与超时（High）：`esm_bsp_encoder_read_angle_rad()` 增加重试次数、单次事务超时和明确返回错误码。
  - 说明：建议 3 次重试、每次 10–50 ms 超时，可根据硬件实际调整。

- [ ] 初始化自检（High）：`esm_bsp_encoder_init()` 在初始化时做一次读写自检（校验奇偶/响应），失败返回可区分错误码。

- [ ] API 文档化（Medium）：在 `bsp/encoder/bsp_encoder.h` 添加函数注释、返回值与使用示例，写入 README 或 docs 页面。

- [ ] 运行时测试任务（Medium）：添加 `task/encoder_test` 示例任务，周期性读取角度并打印/记录，用于板上验证和调试。

- [ ] 异步/回调接口（Low）：提供非阻塞读取 API 或基于任务的异步读（回调或消息队列），适配高频控制环或低延迟采样需求。

- [ ] SPI 驱动复用改进（Low）：在 `driver/spi` 增加对多个从设备管理（CS 管理）、重入安全或设备复位/重建策略。

- [ ] 性能与错误统计（Low）：采集读取成功/失败计数、平均耗时、最近错误码，便于现场问题定位。

备注：当前阻塞实现稳定，优先完成 `互斥`、`重试与超时`、`初始化自检` 三项以提升可靠性并保证 FOC 能正常运行；其余优化在 FOC 运行稳定后逐步推进。

----
历史：在代码重构期间已将 `spi_encoder` 层合并到 `bsp/encoder` 并改为直接使用 `driver/spi`。

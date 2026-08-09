# Stage 3 TODO - ESP32-S3 DVP capture

Tracked by GitHub Issue #1. This checklist defines the first hardware capture
version. Hardware-only acceptance remains unchecked until the board is flashed
and monitored.

## Geometry contract

- [x] Full transport: 324x324 RAW8.
  - Sensor-valid crop: x=2, y=2, 320x320.
- [x] QVGA transport: 324x244 RAW8.
  - Sensor-valid crop: x=2, y=0, 320x244.
  - Optional standard-QVGA crop: x=2, y=2, 320x240.
- [x] Current QQVGA transport: 162x122 RAW8.
  - Sensor-valid crop: x=1, y=0, 160x122.
  - Optional standard-QQVGA crop: x=1, y=1, 160x120.
- [x] Stage 3 captures and validates the complete 324x244 QVGA transport
  frame. It does not crop.

## Controller and memory

- [x] Configure ESP32-S3 DVP RX for RAW8, 8-bit data and 324x244.
- [x] Route D0-D7=GPIO9-16, PCLK=GPIO6, FVLD=GPIO7 and LVLD=GPIO8.
- [x] Keep MCLK on the existing HM01B0 LEDC path; Camera XCLK is disabled.
- [x] Use a conservative 64-byte DMA burst.
- [x] Disable the driver backup buffer.
- [x] Query the driver-aligned buffer length with
  `esp_cam_ctlr_get_frame_buffer_len()`.
- [x] Allocate two application-visible buffers in internal DMA-capable SRAM.
- [x] Record DMA-capable free heap and largest block before and after buffer
  allocation, both buffer addresses, and the aligned length.

## Buffer ownership

- [x] Create a two-entry free queue and two-entry ready queue.
- [x] Seed Buffer A and Buffer B into the free queue.
- [x] `on_get_new_trans`: non-blocking free-buffer acquisition only.
- [x] `on_trans_finished`: attach metadata and queue the completed frame only.
- [x] On ready-queue overflow, count and discard the completed frame by
  returning it to the free queue.
- [x] Treat free-buffer starvation as a fatal first-version timing failure.
- [x] Keep payload size, buffer capacity and received size separate.

## Validation task and logging

- [x] Run CRC32 over the 79056-byte QVGA payload only.
- [x] Validate Walking-1 structurally without assuming an undocumented phase:
  one-hot bytes, cyclic adjacent columns, and identical rows.
- [x] Track frame sequence, stable received size, CRC changes, measured FPS,
  queue errors, and last/maximum processing time.
- [x] Print configuration and memory information once at startup.
- [x] Print a small first-frame sample once; never print a complete frame.
- [x] Print a rate-limited statistics summary once per second.
- [x] Rate-limit error details and include the first mismatch coordinate.

## Lifecycle

- [x] Initialize the HM01B0 and leave it in standby.
- [x] Create buffers, queues, callbacks and the validation task.
- [x] Enable/start ESP Camera RX before calling `hm01b0_start()`.
- [x] Stop the sensor before stopping/disabling Camera RX.
- [x] Release queues, buffers, task and controller without leaks.

## Hardware acceptance

- [x] Build for ESP32-S3 with ESP-IDF 6.0.
- [x] Flash and monitor the physical board.
- [ ] Capture continuously for at least 60 seconds at approximately 30 FPS.
- [ ] Observe zero size errors, Walking-1 errors, buffer starvation and ready
  queue overflow.
- [ ] Confirm maximum processing time remains below the 33.3 ms frame period
  with useful margin.
- [ ] Confirm stop and restart do not trigger DMA errors or assertions.

## Explicitly deferred

- ST7789 output and RAW8-to-RGB565 conversion.
- Any crop, resize or display transformation.
- PSRAM, a third application buffer, and the driver backup buffer.
- Continuous full-frame serial output.
- Runtime Full/QVGA/QQVGA capture switching.

## First hardware run - 2026-08-09

- [x] HM01B0 model ID read correctly as `0x01B0`.
- [x] Software reset and QVGA/8-bit/Walking-1 setup completed in standby.
- [x] DVP controller accepted RAW8 324x244 with a 79056-byte frame length.
- [x] Both 79056-byte buffers were allocated from internal DMA-capable SRAM.
- [ ] Camera RX did not start because ESP-IDF 6.0 called
  `esp_cache_msync(M2C)` on non-cached internal SRAM and propagated the
  expected `ESP_ERR_NOT_SUPPORTED` result as a transaction failure.
- [x] Added a project-local linker wrapper which treats only the internal-RAM
  M2C `ESP_ERR_NOT_SUPPORTED` case as a successful no-op. The ESP-IDF
  installation remains unchanged; hardware re-test is pending.

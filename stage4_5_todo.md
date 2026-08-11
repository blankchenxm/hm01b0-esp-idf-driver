# Stage 4.5: architecture cleanup and startup preflight

Tracked by GitHub Issue #5 and Draft PR #6. This stage prepares the project for
real-image streaming without changing the validated QVGA DVP hardware path.

## Startup contract

```text
initialize sensor, Camera RX resources, snapshot and ST7789
  -> Walking-1: configure in standby
  -> start Camera RX
  -> start sensor streaming
  -> skip 5 frames, analyze and copy one 240x240 snapshot
  -> display static image for 3 seconds while diagnostics continue
  -> stop sensor, then stop Camera RX and summarize the preflight
  -> repeat the same sequence for Color-Bar
  -> disable test pattern in standby
  -> remain idle; Stage 5 will add real-image streaming
```

Test-pattern content status is a structural `PASS` or `WARNING`, not a strict
byte-exact verdict: the datasheet illustrates the patterns but does not define
their complete RAW8 output codes. Received length and Buffer/queue ownership
remain hard transport checks.

## Implemented in Stage 4.5

- [x] Keep LEDC as the HM01B0 MCLK source; Camera RX remains
  `external_xtal=true` with `xclk_io=GPIO_NUM_NC`.
- [x] Preserve QVGA RAW8 324x244, 8-bit DVP, 64-byte DMA burst, non-gated PCLK.
- [x] Preserve two application-visible internal DMA-capable SRAM buffers.
- [x] Keep the driver backup buffer disabled and do not introduce PSRAM.
- [x] Keep `hm01b0_capture` as an independent component.
- [x] Limit mandatory per-frame work to transport length, sequence,
  Buffer/queue ownership, FPS, and lightweight timing counters.
- [x] Move CRC, Walking-1, and Color-Bar analysis to
  `hm01b0_diagnostics.c` and expose a tagged Pattern report.
- [x] Split transport statistics, diagnostic reports, and snapshot results.
- [x] Add a public frame descriptor that separates data pointer, capacity,
  received size, sequence, and timestamp.
- [x] Move RAW8 crop/copy to the allocation-free
  `hm01b0_frame_crop_raw8()` helper.
- [x] Replace constructor-bound Snapshot fields with
  `hm01b0_capture_request_snapshot()`.
- [x] Remove Pattern and Snapshot policy from `hm01b0_capture_config_t`.
- [x] Rename Camera lifecycle to `hm01b0_capture_rx_start/stop()`.
- [x] Rename sensor lifecycle to `hm01b0_stream_start/stop()`.
- [x] Move board wiring to `board_config.h` and experiment choices to
  `app_config.h`; remove the large macro block from `main.c`.
- [x] Add `hm01b0_get_mode_info()` so transport/crop geometry is not duplicated
  in application code.
- [x] Move the SPI ST7789 driver from `main` into its own component and pass
  board pins through a configuration structure.
- [x] Implement the two-pattern startup preflight and leave the final sensor
  state as standby with test pattern OFF and Camera RX stopped.

## Deliberately retained

- LEDC-generated external MCLK, because the sensor requires MCLK before I2C
  probe/configuration and the sensor layer should not depend on Camera RX.
- Direct full-frame DMA into two application-visible buffers.
- Internal SRAM, no backup buffer, no third buffer, no PSRAM.
- Existing SPI2 ST7789 transfer and 512-byte RAW8-to-RGB565 conversion chunk.
- Structural/observational Pattern checks rather than undocumented exact-byte
  assertions.

## Deferred beyond Stage 4.5

- [ ] Real-image sensor output and continuous display streaming.
- [ ] Dedicated display task and frame drop/latest-frame policy.
- [ ] ST7789 throughput optimization; current full refresh is about 63 ms.
- [ ] Migration from the custom SPI ST7789 component to `esp_lcd`.
- [ ] PSRAM, third application Buffer, driver backup Buffer, or line buffering.
- [ ] Camera-controller-generated XCLK/MCLK.
- [ ] Gated-PCLK experiments or sampling-edge changes.
- [ ] Strict byte-exact Walking-1/Color-Bar validation.
- [ ] Runtime Full/QVGA/QQVGA capture switching.

## Owner validation required

- [ ] Build in the owner's ESP-IDF 6.0 environment.
- [ ] Flash and confirm Walking-1 is displayed for about three seconds.
- [ ] Confirm Color-Bar is then displayed for about three seconds.
- [ ] Confirm both preflights report a complete 79,056-byte frame and no
  `size_err`, `no_buffer`, `ready_overflow`, or `free_err`.
- [ ] Confirm final log reports sensor standby, Camera RX stopped, and test
  pattern OFF.
- [ ] Confirm stop/restart between the two preflights causes no DVP/GDMA error.

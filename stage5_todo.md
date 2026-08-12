# Stage 5: real QVGA streaming to ST7789

Tracked by GitHub Issue #7. Stage 5 keeps the validated 324x244 RAW8 DVP
transport and adds continuous real-image display after both startup preflights.

## Runtime contract

```text
initialize sensor and esp_lcd ST7789
  -> reserve one 115,200-byte internal DMA RGB565 workspace
  -> use its first 57,600 bytes as the Walking-1 RAW8 snapshot
  -> run and display Walking-1 preflight
  -> reuse the same first half for the Color-Bar RAW8 snapshot
  -> run and display Color-Bar preflight
  -> disable test pattern and diagnostics in standby
  -> switch the shared workspace to full 240x240 RGB565 use
  -> register the live frame consumer
  -> start Camera RX
  -> start HM01B0 streaming
  -> skip five real-image exposure warm-up frames
  -> continuously crop, convert, and submit frames to SPI DMA
```

## Implemented

- [x] Replace the custom ST7789 SPI transport with ESP-IDF `esp_lcd`.
- [x] Preserve the validated porch, gate, VCOM, power, frame-rate, gamma, and
  inversion settings on top of the standard ST7789 panel initialization.
- [x] Configure SPI2 at 40 MHz with a 115,200-byte maximum color transfer.
- [x] Reserve one 240x240 RGB565 internal DMA-capable Buffer before Camera
  Buffer A/B are allocated, avoiding late-allocation fragmentation.
- [x] Reuse the first half of that workspace as the 240x240 RAW8 preflight
  snapshot; no separate long-lived Snapshot allocation remains.
- [x] Keep static preflight drawing synchronous and wait for each `esp_lcd`
  transaction before reusing its small line Buffer.
- [x] Add a capture-task frame-consumer callback whose borrowed Camera Frame is
  valid only for the callback duration.
- [x] Keep ISR callbacks limited to Camera Buffer queue ownership.
- [x] Combine the `(42,2,240,240)` crop and RAW8-to-RGB565 conversion in one
  pass directly into the complete display Buffer.
- [x] Submit the full RGB565 frame asynchronously with
  `esp_lcd_panel_draw_bitmap()`.
- [x] Recycle the RGB Buffer only from `on_color_trans_done()`.
- [x] Drop a display frame immediately when the RGB Buffer is busy; never wait
  while holding a Camera Buffer.
- [x] Disable Pattern diagnostics for real-image streaming and retain only
  capture length, sequence, queue, Buffer, FPS, and timing checks.
- [x] Report display FPS, submitted/completed/dropped frames, conversion time,
  submission time, SPI DMA time, Buffer state, address, and Heap capacity.

## Deliberately retained

- Two complete 79,056-byte Camera Buffers in internal DMA-capable SRAM.
- No Camera backup Buffer, no third Camera Buffer, and no PSRAM.
- HM01B0 QVGA 324x244 RAW8 transport at approximately 30 FPS.
- Center crop `(42,2,240,240)` rather than scaling the full 320-pixel field of
  view to the 240-pixel display.
- One complete RGB565 display Buffer. A busy Buffer drops display work without
  reducing Camera transport ownership correctness.

## Owner validation completed

- [x] Build with the owner's ESP-IDF 6.0 environment.
- [x] Flash and confirm Walking-1 displays for about three seconds.
- [x] Confirm Color-Bar then displays for about three seconds.
- [x] Confirm the log switches the shared workspace from 57,600-byte RAW8 use
  to 115,200-byte RGB565 use.
- [x] Confirm Test Pattern is OFF before real-image streaming starts.
- [x] Confirm a live centered grayscale image continuously appears.
- [x] Confirm `received_size=79056`, `size_err=0`, `no_buffer=0`,
  `ready_overflow=0`, and `free_err=0`.
- [x] Record Camera FPS, display FPS, conversion time, submit time, DMA time,
  and `dropped_busy`.
- [x] Confirm continuous display without mixed-frame corruption; no display
  frames dropped during this validation run.

## Validated 30 FPS baseline

Owner hardware validation on 2026-08-11 established the following baseline:

- Walking-1 transport/content preflight: PASS, 178/178 valid frames.
- Color-Bar transport/content preflight: PASS, 176/176 valid frames.
- Real-image Camera transport: approximately 30.006 FPS.
- ST7789 display throughput: approximately 29.990 FPS.
- Sustained observation: more than 1,200 valid Camera frames.
- Display timing maxima: 7,623 us conversion, 20,130 us draw-call return,
  and 23,503 us from draw request to completion callback.
- Capture timing maxima: 28,206 us processing and 28,209 us Camera Buffer hold.
- Error counters remained zero: `size_err`, `no_buffer`, `ready_overflow`,
  `free_err`, `dropped_busy`, and `submit_err`.

The reported `dma` interval starts immediately before
`esp_lcd_panel_draw_bitmap()` and ends in the color-transfer completion
callback. It therefore includes the measured draw-call interval and must not be
added to `submit` when calculating end-to-end latency.

## Deferred

- [ ] 60 FPS sensor timing and higher SPI clock experiments.
- [ ] TE/vertical-blank synchronization and tearing control.
- [ ] Full 320-to-240 horizontal downscaling.
- [ ] A second complete RGB565 Buffer.
- [ ] PSRAM or strip-buffer fallback if hardware memory results require it.

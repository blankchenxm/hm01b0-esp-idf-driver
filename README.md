# hm01b0-esp-idf-driver

ESP-IDF driver under active development for the Himax HM01B0 monochrome image
sensor on ESP32-S3.

The repository currently contains the completed sensor-control foundation:
external MCLK generation, 16-bit-address/8-bit-data I2C access, model-ID probe,
register tables, operating modes, interface configuration, test patterns, and a
standby/streaming state machine. It also contains direct ESP32-S3 DVP capture
with two internal DMA frame buffers. Stage 4.5 separates transport, diagnostics,
frame operations, and display responsibilities and adds a two-pattern startup
preflight. Stage 5 adds real-image streaming through the ESP-IDF `esp_lcd`
ST7789 driver and one complete internal-DMA RGB565 display Buffer.

## Current status

| Stage | Status | Scope |
|---|---|---|
| 1. MCLK + I2C | Complete | 12 MHz MCLK, I2C register access, `MODEL_ID=0x01B0` probe |
| 2. Register tables + state machine | Complete | Common initialization, FULL/QVGA/QQVGA modes, 8/4/1-bit interfaces, test patterns, standby/start/stop |
| 3. DVP capture | Complete | ESP32-S3 LCD_CAM/GDMA, two internal DMA frame buffers, callbacks, dual CRC and selectable test-pattern observation |
| 4. Static ST7789 display | Complete | One post-warm-up 240 x 240 RAW8 crop, task notification, grayscale RGB565 SPI output |
| 4.5. Pre-streaming architecture | Complete | Component split, on-demand snapshot, tagged diagnostics, Walking-1 then Color-Bar startup preflight |
| 5. Real-image display | Implemented, build/hardware validation pending | `esp_lcd` ST7789, one 115,200-byte RGB565 DMA Buffer, non-blocking QVGA live display |

The current sample initializes the sensor, capture controller, two Camera DMA
buffers, one shared display workspace, and ST7789. It then runs Walking-1 and
Color-Bar preflights in sequence. Each preflight starts Camera RX before sensor
streaming, skips five frames, analyzes and snapshots one complete frame, and
displays the static crop for three seconds while capture diagnostics continue.
It then stops the sensor and Camera RX. The application finally disables the
sensor test pattern, switches the shared display workspace to RGB565 use, and
starts continuous real-image streaming. Display work uses a centered 240 x 240
crop and is dropped rather than blocking Camera Buffer recycling when SPI DMA is
still busy.

## Initial sensor configuration

| Setting | Value |
|---|---|
| Sensor mode | QVGA |
| Pixel output | RAW8 |
| Data interface | 8-bit parallel |
| Initial/final test pattern | Off; Walking-1 and Color-Bar are enabled only during startup preflight |
| Operation | Non-SYNC |
| PCLK | Non-gated, rising edge |
| MCLK | 12 MHz external clock |
| Sensor_Core | MCLK / 2 = 6 MHz |
| Sensor_Register | MCLK / 1 = 12 MHz |
| Frame timing | 376 PCK x 532 lines, approximately 30 FPS |
| I2C address | `0x24` (7-bit) |
| Expected model ID | `0x01B0` |

## Stage 3 first capture configuration

The first capture version deliberately receives the complete HM01B0 QVGA
transport frame and performs no crop:

| Setting | Value |
|---|---|
| DVP transport | 324 x 244 RAW8 |
| Payload | 79,056 bytes |
| Sensor-valid crop metadata | x=2, y=0, 320 x 244 (not applied) |
| Optional standard-QVGA crop | x=2, y=2, 320 x 240 (not applied) |
| Buffers | Two application-visible buffers |
| Memory | Internal DMA-capable SRAM |
| Buffer allocation | Driver-aligned length from `esp_cam_ctlr_get_frame_buffer_len()` |
| Backup buffer | Disabled |
| DMA burst | 64 bytes |
| Mandatory processing | Exact received length, sequence, queue/Buffer errors, FPS and timing |
| Optional diagnostics | Raw/active CRC32 and selectable four-row Walking-1 or Color-Bar observation |
| Warm-up | Skip five startup frames before content baselines |

The frame descriptor keeps the 79,056-byte payload size, aligned buffer
capacity, and per-transaction received size separate. Full geometry decisions
and the hardware checklist are recorded in [stage3_todo.md](stage3_todo.md).
The detailed Stage 3 flow and the combined Stages 1-3 call sequence are
recorded in [docs/execution-flow.md](docs/execution-flow.md).

## Stage 4 static display configuration

Stage 4 leaves the Stage 3 DVP transport and two DMA buffers unchanged. On the
first size-valid frame after warm-up, the frame task copies the centered crop
`x=42, y=2, 240 x 240` into a separate 57,600-byte internal-SRAM RAW8 snapshot.
It returns the Camera DMA buffer before notifying `app_main`, so the one-time
SPI display transfer cannot hold Buffer A or B.

The ST7789 uses SPI2 at 40 MHz through ESP-IDF `esp_lcd` and its asynchronous SPI
DMA completion callback. `st7789_display_draw_gray8()` converts the packed
RAW8 preflight snapshot to RGB565 lines and writes the screen once. Camera RX
and optional diagnostics continue during the static hold, without owning either
Camera Buffer; they are stopped before switching patterns. The detailed Stage 4 scope is
recorded in [stage4_todo.md](stage4_todo.md), and the architecture decisions are
recorded in [stage4_5_todo.md](stage4_5_todo.md). The live display lifecycle and
hardware checklist are recorded in [stage5_todo.md](stage5_todo.md).

The capture component exposes mutually exclusive `NONE`, `WALKING_1`, and
`COLOR_BAR` analysis modes. Color-Bar observation compares four representative
rows, counts horizontal and strong transitions, records the six nominal bar
center samples, and reports their vertical consistency without assuming exact
RAW8 values not specified by the datasheet. On the monochrome HM01B0-MWA, the
LCD shows these bars as grayscale regions rather than RGB colors. Analysis uses
the full 320-pixel active row, while the unchanged 240-pixel center crop trims
40 columns from each side, so the two outer bars appear narrower on the LCD.

### ESP-IDF 6.0 internal-SRAM compatibility

ESP-IDF 6.0's DVP controller unconditionally calls
`esp_cache_msync(..., ESP_CACHE_MSYNC_FLAG_DIR_M2C)` before starting a frame.
On ESP32-S3, internal SRAM is not data-cached, so the cache API returns
`ESP_ERR_NOT_SUPPORTED` even though the buffer is valid for GDMA and no cache
operation is needed. `hm01b0_cache_compat.c` provides a project-local linker
wrapper that converts only this internal-memory/no-cache result to `ESP_OK`.
PSRAM cache operations, cacheable-memory alignment failures, invalid
arguments, and every other result still use the original ESP-IDF
implementation. The installed ESP-IDF tree is not modified.

## Wiring used by the sample application

| HM01B0 signal | ESP32-S3 GPIO |
|---|---:|
| SCL | 1 |
| SDA | 2 |
| MCLK | 5 |
| PCLK | 6 |
| FVLD / VSYNC | 7 |
| LVLD / HREF | 8 |
| D0-D7 | 9-16 |

| ST7789 signal | ESP32-S3 GPIO |
|---|---:|
| SCL / SPI clock | 35 |
| SDA / MOSI | 36 |
| RES | 37 |
| DC | 38 |
| CS | 39 |

The GPIO mapping belongs to the sample board and is defined in
`main/board_config.h`; the reusable sensor component receives its control pins
through `hm01b0_config_t`.

## Project layout

```text
components/hm01b0/
  include/                 Public API and types
  private_include/         Private register, table, and device definitions
  hm01b0.c                 Device creation, MCLK, I2C, initialization lifecycle
  hm01b0_reg.c             Low-level register read/write/update/table operations
  hm01b0_modes.c           Common, mode, interface, and test-pattern tables
  hm01b0_mode_info.c       Full/QVGA/QQVGA transport and crop metadata
  hm01b0_sensor.c          Probe and sensor state-machine operations

components/hm01b0_capture/
  include/hm01b0_capture.h DVP lifecycle, transport stats, snapshot requests
  include/hm01b0_diagnostics.h  Tagged optional Pattern reports
  include/hm01b0_frame_ops.h    Generic RAW8 crop/copy operation
  hm01b0_capture.c         DVP, DMA buffers, queues, callbacks, frame task
  hm01b0_diagnostics.c     CRC, Walking-1 and Color-Bar observation
  hm01b0_frame_ops.c       Allocation-free RAW8 rectangle copy
  hm01b0_cache_compat.c    ESP-IDF 6.0 internal-SRAM DVP cache workaround

components/st7789_display/
  include/                 Handle-based esp_lcd display API and statistics
  st7789_display.c         ST7789 panel, RGB workspace, conversion and SPI DMA

main/
  board_config.h           Sample board pin mapping
  app_config.h             Stage 5 preflight and live-display policy
  main.c                   Run preflights, then start real-image streaming

examples/
  i2c_finder.c             Standalone I2C diagnostic source

docs/
  execution-flow.md        Detailed Stage 3 and combined Stages 1-3 call flow
```

## Public API

The component currently exposes:

```c
esp_err_t hm01b0_new(const hm01b0_config_t *config,
                     hm01b0_handle_t **out_handle);
esp_err_t hm01b0_delete(hm01b0_handle_t *dev);

esp_err_t hm01b0_reg_read(hm01b0_handle_t *dev,
                          uint16_t addr,
                          uint8_t *value);
esp_err_t hm01b0_reg_write(hm01b0_handle_t *dev,
                           uint16_t addr,
                           uint8_t value);
esp_err_t hm01b0_reg_update_bits(hm01b0_handle_t *dev,
                                 uint16_t addr,
                                 uint8_t mask,
                                 uint8_t value);
esp_err_t hm01b0_write_table(hm01b0_handle_t *dev,
                             const hm01b0_regval_t *table,
                             size_t count);

esp_err_t hm01b0_probe(hm01b0_handle_t *dev, uint16_t *model_id);
esp_err_t hm01b0_reset(hm01b0_handle_t *dev);
esp_err_t hm01b0_standby(hm01b0_handle_t *dev);
esp_err_t hm01b0_stream_start(hm01b0_handle_t *dev);
esp_err_t hm01b0_stream_stop(hm01b0_handle_t *dev);
esp_err_t hm01b0_get_mode_info(hm01b0_mode_t mode,
                               hm01b0_mode_info_t *info);
```

Mode, interface, test-pattern, state-query, and configuration APIs are declared
in `components/hm01b0/include/hm01b0.h`.

## Build

Prerequisites:

- ESP32-S3 target
- ESP-IDF 6.0
- HM01B0 hardware connected with compatible I/O voltage levels

From an initialized ESP-IDF terminal:

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Expected successful initialization includes an ID report equivalent to:

```text
MODEL_ID_H=0x01, MODEL_ID_L=0xB0, MODEL_ID=0x01B0
PASS: HM01B0 initialized in STANDBY
```

Stage 5 reports the 324 x 244 geometry, aligned Buffer A/B allocations,
transport health, tagged Pattern observations, snapshot copy/hold timing, and
one-time LCD transfer duration for each preflight. During real-image streaming
it reports Camera and display FPS, RGB conversion/submission/DMA timing, and
busy-Buffer display drops. It never prints a complete frame. Hardware acceptance
requires both static patterns and the continuous live image to display with zero
size or Camera queue/Buffer errors.

Board-specific `sdkconfig` files and local editor/tool paths are intentionally
not committed.

## Development workflow

`main` records completed and reviewed stage baselines. Each new stage should use:

1. A GitHub Issue with scope and hardware acceptance criteria.
2. A short-lived `codex/feat/issue-N-topic` branch.
3. Focused commits for coherent implementation steps.
4. A Draft pull request containing build and hardware-validation results.
5. Merge only after the stage acceptance criteria pass.

This keeps every stage inspectable through commits and pull requests and makes it
possible to return to any previously completed implementation.

## Documentation and attribution

The HM01B0 datasheet is not redistributed in this public repository. Obtain the
applicable documentation directly from the sensor vendor or an authorized
source.

Some sensor tuning values in `hm01b0_modes.c` are adapted from the OpenMV HM01B0
driver. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for attribution.

## License

No project-wide license has been selected yet. Until a license is added, the
repository is publicly viewable but no general permission to copy, modify, or
redistribute the original project code is granted. Third-party portions retain
their original notices and terms.

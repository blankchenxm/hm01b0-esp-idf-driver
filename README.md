# hm01b0-esp-idf-driver

ESP-IDF driver under active development for the Himax HM01B0 monochrome image
sensor on ESP32-S3.

The repository currently contains the completed sensor-control foundation:
external MCLK generation, 16-bit-address/8-bit-data I2C access, model-ID probe,
register tables, operating modes, interface configuration, test patterns, and a
standby/streaming state machine. Parallel DVP capture is the next development
stage.

## Current status

| Stage | Status | Scope |
|---|---|---|
| 1. MCLK + I2C | Complete | 12 MHz MCLK, I2C register access, `MODEL_ID=0x01B0` probe |
| 2. Register tables + state machine | Complete | Common initialization, FULL/QVGA/QQVGA modes, 8/4/1-bit interfaces, test patterns, standby/start/stop |
| 3. DVP capture | Planned | ESP32-S3 LCD_CAM/GDMA, DMA frame buffers, frame callbacks, RAW8 validation |
| 4. Display/application pipeline | Planned | Frame processing and optional LCD output |

The current sample application initializes the sensor and intentionally leaves
it in standby. It does not capture image data yet.

## Initial sensor configuration

| Setting | Value |
|---|---|
| Sensor mode | QVGA |
| Pixel output | RAW8 |
| Data interface | 8-bit parallel |
| Test pattern | Walking 1 |
| Operation | Non-SYNC |
| PCLK | Non-gated, rising edge |
| MCLK | 12 MHz external clock |
| Sensor_Core | MCLK / 2 = 6 MHz |
| Sensor_Register | MCLK / 1 = 12 MHz |
| Frame timing | 376 PCK x 532 lines, approximately 30 FPS |
| I2C address | `0x24` (7-bit) |
| Expected model ID | `0x01B0` |

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
  hm01b0_sensor.c          Probe and sensor state-machine operations

main/
  board_config.h           Sample board pin mapping
  main.c                   Stage 1+2 probe/initialization application

examples/
  i2c_finder.c             Standalone I2C diagnostic source
```

The ST7789 source files under `main/` are not part of the current build and are
reserved for a later application/display stage.

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
esp_err_t hm01b0_start(hm01b0_handle_t *dev);
esp_err_t hm01b0_stop(hm01b0_handle_t *dev);
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

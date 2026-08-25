# Stage 6: Full/QVGA/QQVGA common streaming flow

Tracked by GitHub Issue #11. Stage 6 extends the hardware-validated Stage 5
pipeline without duplicating its startup preflight, DVP ownership, or display
lifecycle.

## Mode geometry

The application obtains sensor geometry from `hm01b0_get_mode_info()` and then
centers the standard valid area relative to the 240x240 panel. An area larger
than the panel is center-cropped; an area smaller than the panel is kept at its
native size and centered.

| Mode | DVP transport | Standard valid area | Display source | LCD destination |
|---|---:|---:|---:|---:|
| Full | 324x324 | `(2,2 320x320)` | `(42,42 240x240)` | `(0,0)` |
| QVGA | 324x244 | `(2,2 320x240)` | `(42,2 240x240)` | `(0,0)` |
| QQVGA | 162x122 | `(1,1 160x120)` | `(1,1 160x120)` | `(40,60)` |

QQVGA is produced by applying 2x2 monochrome binning to the 324x244 QVGA
window. The DVP transport is therefore 162x122 rather than 162x162. The panel
is cleared to black once before preflight, and only the centered 160x120 region
is updated afterwards, preserving black borders without a scaling pass.

## Shared Buffer lifecycle

The display component still allocates exactly one complete 240x240 RGB565
internal-DMA workspace:

```text
240 x 240 x 2 = 115,200 bytes
```

Before live streaming, the same address is borrowed as packed RAW8 Snapshot
storage:

| Mode | RAW8 Snapshot | Borrowed bytes |
|---|---:|---:|
| Full | 240x240 | 57,600 |
| QVGA | 240x240 | 57,600 |
| QQVGA | 160x120 | 19,200 |

There is no separate long-lived Snapshot allocation. After both preflights,
`st7789_display_prepare_stream()` ends RAW8 use and makes the whole workspace
available for RGB565 conversion and SPI DMA. Full and QVGA submit 115,200 bytes
per displayed frame; QQVGA packs and submits only 38,400 bytes from the
beginning of the same workspace.

Camera A/B continue to contain complete uncropped DVP frames. Full places the
two large Camera buffers in DMA-capable PSRAM; QVGA and QQVGA keep their
validated internal-DMA allocation. The complete RGB565 display workspace stays
in internal DMA SRAM for every mode:

| Mode | Camera A/B memory | RAW8 payload per buffer | RGB565 memory |
|---|---|---:|---|
| Full | DMA-capable PSRAM | 104,976 bytes | Internal DMA SRAM |
| QVGA | Internal DMA SRAM | 79,056 bytes | Internal DMA SRAM |
| QQVGA | Internal DMA SRAM | 19,764 bytes | Internal DMA SRAM |

Actual allocation capacity is still reported by
`esp_cam_ctlr_get_frame_buffer_len()` and may include trailing DMA/cache
alignment padding. The allocation API verifies that each returned address is
in the requested memory pool. Full fails clearly if DMA-capable PSRAM is not
enabled or exposed through the capability allocator.

## Frame-rate policy

The driver uses the datasheet equation:

```text
FPS = Sensor_Core / (LINE_LENGTH_PCK * FRAME_LENGTH_LINES)
```

It keeps `LINE_LENGTH_PCK` at the mode minimum, calculates a frame length that
does not exceed the requested preset, enforces the minimum frame length, and
sets `MAX_INTEGRATION = FRAME_LENGTH_LINES - 2`.

| Mode | Minimum line | Minimum frame | Public presets |
|---|---:|---:|---|
| Full | 376 | 344 | 15, 20, 30, 45 FPS |
| QVGA | 376 | 260 | 15, 20, 30, 45, 60 FPS |
| QQVGA | 215 | 128 | 15, 20, 30, 45, 60, 120 FPS |

At a 6 MHz Sensor_Core, the minimum-timing mathematical ceilings are about
46.388, 61.375, and 218.023 FPS respectively. Stage 6 intentionally keeps the
existing conservative public maxima of 45, 60, and 120 FPS.

## Configuration

Only the mode and a compatible preset need to be selected in
`main/app_config.h`:

```c
/* Full example */
#define APP_HM01B0_MODE       HM01B0_SENSOR_MODE_FULL
#define APP_HM01B0_FRAME_RATE HM01B0_FRAME_RATE_30

/* QVGA validated default */
#define APP_HM01B0_MODE       HM01B0_SENSOR_MODE_QVGA
#define APP_HM01B0_FRAME_RATE HM01B0_FRAME_RATE_60

/* QQVGA example */
#define APP_HM01B0_MODE       HM01B0_SENSOR_MODE_QQVGA
#define APP_HM01B0_FRAME_RATE HM01B0_FRAME_RATE_60
```

The examples are alternatives, not definitions to enable simultaneously.
Crop coordinates, Snapshot length, Camera payload, and LCD destination are no
longer user-maintained macros.

## Implementation checklist

- [x] Add an application mode-profile layer.
- [x] Derive DVP allocation from mode transport geometry.
- [x] Derive Snapshot and live source rectangles from the standard valid area.
- [x] Generalize live RGB565 submission to a rectangular panel destination.
- [x] Clear the panel to black before QQVGA partial updates.
- [x] Reuse one RGB565 workspace for all preflight Snapshots.
- [x] Place Full Camera A/B in PSRAM while retaining the RGB565 workspace in
  internal DMA SRAM.
- [x] Keep QVGA and QQVGA Camera A/B in internal DMA SRAM.
- [x] Remove fixed QVGA crop macros and QVGA-only runtime logs.
- [x] Keep QVGA 60 FPS as the default configuration.

## Owner hardware validation

- [ ] Build and flash the unchanged QVGA 60 FPS default as a regression check.
- [ ] Confirm QVGA Pattern preflights and live display remain correct.
- [ ] Select Full 30 FPS and confirm both Camera Buffer addresses are reported
  as external while the RGB565 workspace is reported as internal DMA.
- [ ] Confirm Full uses 324x324 transport and a centered 240x240 image.
- [ ] If Full 30 FPS succeeds, repeat at Full 45 FPS.
- [ ] Select QQVGA 60 FPS and confirm 162x122 transport.
- [ ] Confirm the QQVGA 160x120 image is centered at `(40,60)` with stable black
  borders.
- [ ] If QQVGA 60 FPS succeeds, repeat at QQVGA 120 FPS.
- [ ] Confirm Walking-1 and Color-Bar preflights in every tested mode.
- [ ] Confirm `size_err`, `no_buffer`, `ready_overflow`, `free_err`,
  `dropped_busy`, and `submit_err` remain zero.
- [ ] Record Camera/display FPS and conversion/submission/DMA/Buffer-hold
  maxima for each mode/rate pair.

## Deferred

- [ ] Runtime mode switching without recreating Camera resources.
- [ ] QQVGA scaling to 240x180 or 240x240.
- [ ] A second RGB565 Buffer or a third Camera Buffer.
- [ ] QQVGA presets above 120 FPS.

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

## Shared workspace and Strip lifecycle

The display component allocates one 57,600-byte internal-DMA workspace:

```text
240 x 240 RAW8 = 57,600 bytes
```

Before live streaming, that address is borrowed as packed RAW8 Snapshot
storage:

| Mode | RAW8 Snapshot | Borrowed bytes |
|---|---:|---:|
| Full | 240x240 | 57,600 |
| QVGA | 240x240 | 57,600 |
| QQVGA | 160x120 | 19,200 |

There is no separate long-lived Snapshot allocation. After both preflights,
`st7789_display_prepare_stream()` ends RAW8 use and splits the same address:

```text
57,600-byte Workspace
|-- Strip A: 240 x 60 x 2 = 28,800 bytes
`-- Strip B: 240 x 60 x 2 = 28,800 bytes
```

Full and QVGA alternate four `240x60` Strips per displayed frame. QQVGA
alternates two `160x60` Strips; each uses 19,200 bytes of the corresponding
28,800-byte capacity. The SPI transaction queue depth is two. A completion
callback returns the matching Strip, and only completion of the final Strip
increments the completed-frame count. All source rows have been copied out
before the Camera Buffer is returned, while the last Strip may continue in SPI
DMA.

Runtime timing fields now have frame-level Strip semantics:

- `convert`: sum of CPU time spent converting every Strip in the frame; time
  waiting for SPI is excluded.
- `submit`: sum of all `esp_lcd_panel_draw_bitmap()` call durations, including
  any wait needed to recycle the preceding SPI transaction.
- `dma`: wall time from the first Strip submission to the final Strip DMA
  callback; it includes the conversion/submission overlap between those points.

Camera A/B continue to contain complete uncropped DVP frames:

| Mode | RAW8 payload per Camera Buffer | Two-Buffer payload |
|---|---:|---:|
| Full | 104,976 bytes | 209,952 bytes |
| QVGA | 79,056 bytes | 158,112 bytes |
| QQVGA | 19,764 bytes | 39,528 bytes |

Actual allocation capacity is still reported by
`esp_cam_ctlr_get_frame_buffer_len()` and may include driver requirements.
The Strip workspace replaces the Stage 5 complete 115,200-byte RGB565 Buffer.
For Full, the two RAW8 Camera payloads plus display workspace require 267,552
bytes before allocator alignment and controller/task overhead, instead of
325,152 bytes with a complete RGB565 frame.

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
- [x] Reuse one shared workspace for all preflight Snapshots and live Strips.
- [x] Remove fixed QVGA crop macros and QVGA-only runtime logs.
- [x] Keep the selected mode/rate policy centralized in `app_config.h`.
- [x] Replace the complete RGB565 frame with two alternating 240x60 Strips.
- [x] Submit four Strips for Full/QVGA and two Strips for QQVGA.
- [x] Track per-frame conversion, aggregate submission, and first-to-final
  Strip DMA timing.

## Owner hardware validation

- [ ] Select Full 30 FPS and record both Camera Buffer allocations and Heap
  diagnostics.
- [ ] Confirm Full uses 324x324 transport and a centered 240x240 image.
- [ ] If Full 30 FPS succeeds, repeat at Full 45 FPS.
- [ ] Select QVGA 60 FPS as a regression check.
- [ ] Confirm QVGA Pattern preflights and live display remain correct.
- [ ] Select QQVGA 60 FPS and confirm 162x122 transport.
- [ ] Confirm the QQVGA 160x120 image is centered at `(40,60)` with stable black
  borders.
- [ ] If QQVGA 60 FPS succeeds, repeat at QQVGA 120 FPS.
- [ ] Confirm Walking-1 and Color-Bar preflights in every tested mode.
- [ ] Confirm `size_err`, `no_buffer`, `ready_overflow`, `free_err`,
  `dropped_busy`, and `submit_err` remain zero.
- [ ] Confirm the display log reports a 57,600-byte workspace and two
  28,800-byte, 60-row Strips.
- [ ] Record Camera/display FPS and conversion/submission/DMA/Buffer-hold
  maxima for each mode/rate pair.

## Deferred

- [ ] Runtime mode switching without recreating Camera resources.
- [ ] QQVGA scaling to 240x180 or 240x240.
- [ ] A third Camera Buffer or PSRAM.
- [ ] QQVGA presets above 120 FPS.

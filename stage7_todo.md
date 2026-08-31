# Stage 7: HM01B0-ANA Bayer support

## Scope

Stage 7 adds HM01B0-ANA color-sensor support without changing the validated
Stage 6 DVP transport. ANA sends Bayer RAW8: every pixel contributes one byte,
not an RGB888 triplet. The Camera payload therefore remains `width * height`.

## Variant policy

| Variant | RAW8 interpretation | Modes | DPC |
|---|---|---|---|
| MWA | MONO8 | Full, QVGA, QQVGA | Mono option 1 (`0x1008=0x01`) |
| ANA | Bayer, selected by `PIXEL_ORDER` | Full, QVGA | Bayer option 1 (`0x1008=0x03`) |

Both packages report `MODEL_ID=0x01B0`, so the application must select the
physical variant through `APP_HM01B0_VARIANT`. ANA + QQVGA returns
`ESP_ERR_NOT_SUPPORTED` because the datasheet limits binning to monochrome.

The default `PIXEL_ORDER=0x02` is BGGR. All four reported orders are modeled:
GRBG8, RGGB8, BGGR8, and GBRG8. Conversion uses absolute sensor coordinates,
so an odd crop offset changes the local Bayer phase correctly rather than
silently treating the crop as BGGR again.

## Image and memory pipeline

```text
HM01B0 ANA Bayer RAW8
        |
        v
Camera Buffer A/B (one byte per pixel, unchanged Stage 6 policy)
        |
        | Capture Task: non-blocking crop + one-pixel halo copy
        v
Internal-SRAM RAW8 staging (242 x 242 = 58,564 bytes for 240 x 240 output)
        |
        | single-slot queue -> dedicated Image/Display Task
        v
optimized bilinear Demosaic + RGB565 packing
        |
        v
115,200-byte internal-DMA RGB565 Buffer
        |
        v
esp_lcd SPI DMA -> ST7789
```

There is no complete RGB888 frame. Each Bayer output pixel is reconstructed as
R/G/B and immediately packed to two-byte RGB565.

Stage 7 memory policy is:

| Variant / Mode | Camera A/B | RAW8 staging | RGB565 workspace |
|---|---|---|---|
| ANA Full | PSRAM | Internal SRAM | Internal DMA SRAM |
| ANA QVGA | PSRAM | Internal SRAM | Internal DMA SRAM |
| MWA Full | PSRAM | Not used | Internal DMA SRAM |
| MWA QVGA/QQVGA | Internal DMA SRAM | Not used | Internal DMA SRAM |

ANA additionally allocates one 58,564-byte internal-SRAM RAW8 staging image. It
is not a third Camera Buffer and GDMA never writes into it. Capture Task copies
only the LCD source crop plus its Bayer halo and returns Camera A/B before
Demosaic begins.
If that one staging image is still owned by Image/Display Task, the next display
frame is dropped rather than blocking capture or requesting a third Camera
Buffer.

The Image/Display task uses a faster interior-row Demosaic path with direct row
pointers, fixed two-/four-sample averages, and two-pixel loops specialized for
the row's Bayer phase. The one-pixel halo means all 240 x 240 output pixels can
use this path; the generic border-safe algorithm is retained for other callers.
Internal SRAM avoids repeated cached-PSRAM neighbor reads during Demosaic.

After submitting a frame, `esp_lcd_panel_draw_bitmap()` can return shortly
before SPI DMA releases the RGB565 workspace. Image/Display Task now waits up
to 20 ms for that completion rather than immediately discarding an already
staged frame. `rgb_busy` therefore represents an actual wait timeout.

The preflight snapshot continues borrowing 57,600 RAW8 bytes from the display
workspace. It is converted one row at a time into the existing 480-byte line
buffer, so Bayer preflight does not require another full frame.

## Diagnostics

- Walking-1 keeps the existing structural RAW8 observation.
- Color-Bar records the selected pixel format and compares horizontal Bayer
  samples at a two-pixel interval, avoiding direct comparison of neighboring
  pixels from different CFA color planes.
- CRC remains a transport/content-change observation, not an exact datasheet
  color-value oracle.
- Runtime statistics distinguish Camera/display rates: `pipeline_fps input`
  counts frames offered by Capture Task, `staged` counts successful PSRAM
  copies, `submitted` counts frames submitted to the LCD, while `display_fps`
  counts completed SPI-DMA frames. Copy/Demosaic times and staging/RGB-busy
  drops are reported separately.

## Owner hardware checklist

- [ ] Build with `APP_HM01B0_VARIANT=HM01B0_VARIANT_ANA_BAYER`.
- [ ] Confirm startup log reports the expected `PIXEL_ORDER` and Bayer format.
- [ ] Confirm `DPC_CTRL=0x03` readback.
- [ ] Run Full at a conservative frame rate and check both test patterns.
- [x] Confirm Walking-1 and Color-Bar preflight images display correctly.
- [ ] Confirm the live image has plausible color ordering (no red/blue swap).
- [ ] Record Bayer conversion time, Camera Buffer hold time, display FPS, and
      all drop/queue counters.
- [ ] Repeat with QVGA.
- [ ] Select ANA + QQVGA and confirm clean `ESP_ERR_NOT_SUPPORTED` rejection.
- [ ] Switch to MWA and repeat the previously validated modes for regression.

Advanced AWB, gamma, color-correction matrix tuning, and higher-quality
Demosaic remain follow-up work after live-pipeline hardware validation.

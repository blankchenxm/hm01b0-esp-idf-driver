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
        v
hm01b0_image bilinear Demosaic + RGB565 packing
        |
        v
115,200-byte internal-DMA RGB565 Buffer
        |
        v
esp_lcd SPI DMA -> ST7789
```

There is no complete RGB888 frame. Each Bayer output pixel is reconstructed as
R/G/B and immediately packed to two-byte RGB565.

Memory policy remains:

| Mode | Camera A/B | RGB565 workspace |
|---|---|---|
| Full | PSRAM | Internal DMA SRAM |
| QVGA | Internal DMA SRAM | Internal DMA SRAM |
| QQVGA (MWA only) | Internal DMA SRAM | Internal DMA SRAM |

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

## Owner hardware checklist

- [ ] Build with `APP_HM01B0_VARIANT=HM01B0_VARIANT_ANA_BAYER`.
- [ ] Confirm startup log reports the expected `PIXEL_ORDER` and Bayer format.
- [ ] Confirm `DPC_CTRL=0x03` readback.
- [ ] Run Full at a conservative frame rate and check both test patterns.
- [ ] Confirm the live image has plausible color ordering (no red/blue swap).
- [ ] Record Bayer conversion time, Camera Buffer hold time, display FPS, and
      all drop/queue counters.
- [ ] Repeat with QVGA.
- [ ] Select ANA + QQVGA and confirm clean `ESP_ERR_NOT_SUPPORTED` rejection.
- [ ] Switch to MWA and repeat the previously validated modes for regression.

Advanced AWB, gamma, color-correction matrix tuning, higher-quality Demosaic,
and ANA throughput optimization remain follow-up work after first hardware
validation.

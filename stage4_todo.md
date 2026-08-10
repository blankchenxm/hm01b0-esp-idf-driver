# Stage 4: one-shot HM01B0 frame on ST7789

Stage 4 adds the smallest display path needed to inspect the HM01B0 Walking-1
test pattern visually. It deliberately displays one static image; continuous
LCD refresh and display FPS optimization are outside this stage.

## Fixed first-version configuration

| Item | Value |
|---|---|
| Camera transport | QVGA RAW8, 324 x 244 bytes |
| Camera DMA buffers | Two complete internal-SRAM buffers, unchanged from Stage 3 |
| Display crop | `x=42`, `y=2`, `width=240`, `height=240` |
| Snapshot layout | Tightly packed RAW8, 240-byte stride |
| Snapshot size | 57,600 bytes |
| Snapshot memory | Internal 8-bit SRAM; it is not a Camera DMA buffer |
| LCD | ST7789, 240 x 240, SPI2 at 40 MHz |
| LCD output format | RAW8 converted to grayscale RGB565 during transmission |
| Refresh policy | One frame after the five-frame warm-up |

The horizontal crop is centered in the 324-byte DVP row:
`(324 - 240) / 2 = 42`. The vertical crop removes two rows from the top and two
from the bottom of the 244-row transport frame. Camera DMA still receives every
324 x 244 byte frame; the crop is applied only while making the one-shot
snapshot.

## Buffer ownership and execution flow

```text
HM01B0 DVP
  -> LCD_CAM Camera RX + GDMA
  -> Buffer A/B completed
  -> ready_queue
  -> hm01b0_capture_task()
       -> validate and analyze the frame
       -> on the first eligible frame, copy crop to snapshot buffer
       -> return Camera Buffer A/B to free_queue
       -> invoke the non-blocking snapshot-ready callback
  -> app_main is released by a task notification
  -> st7789_draw_gray_image() converts RAW8 chunks to RGB565
  -> SPI2 sends one complete 240 x 240 image
  -> ST7789 GRAM retains the static image
```

The callback is not an ISR. It runs in the capture frame task and only sends a
FreeRTOS task notification. It is invoked after the Camera buffer has been
returned, so the slow LCD transfer never owns or blocks Buffer A/B. The Camera
continues streaming and the Stage 3 analysis task continues running while the
static image remains on the LCD.

## Implementation checklist

- [x] Add optional one-shot snapshot configuration to `hm01b0_capture`.
- [x] Validate the snapshot crop and destination capacity.
- [x] Copy only the first size-valid frame after warm-up.
- [x] Return the Camera buffer before notifying the application.
- [x] Record snapshot-copy time and Camera-buffer hold time.
- [x] Allocate the 57,600-byte snapshot in internal SRAM.
- [x] Initialize the ST7789 before starting Camera streaming.
- [x] Add the existing SPI ST7789 source to the application build.
- [x] Convert the packed RAW8 snapshot to RGB565 and display it once.
- [ ] Build with the owner's ESP-IDF 6.0 environment.
- [ ] Flash and visually validate the Walking-1 image.

## Expected serial evidence

The hardware run should show:

- HM01B0 model ID `0x01B0` and standby initialization success.
- A 57,600-byte snapshot allocation and crop `(42,2 240x240)`.
- Two 79,056-byte Camera DMA buffers in internal SRAM.
- Camera RX started before HM01B0 enters streaming.
- One `snapshot ready` line containing source-frame sequence, copy time, and
  total Camera-buffer hold time.
- One `Stage 4 static frame displayed` line containing SPI display time.
- Continued Stage 3 statistics near 30 FPS with zero `no_buffer`,
  `ready_overflow`, and `free_err` counts.

## Hardware acceptance

1. The ST7789 shows one stable 240 x 240 grayscale test-pattern image.
2. The LCD image remains static; it is not rewritten every frame.
3. Camera capture continues after the image is shown.
4. No Camera-buffer starvation or ready-queue overflow is introduced.
5. Snapshot copy and total buffer-hold timings are present for later
   continuous-display design decisions.

If the pattern is mirrored, rotated, or color-inverted, adjust the ST7789
`MADCTL`/inversion settings separately; do not change the DVP sampling path
until the captured byte geometry has been distinguished from LCD orientation.

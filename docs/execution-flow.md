# HM01B0 阶段 1～3 完整执行顺序

本文记录当前示例程序的实际执行顺序。当前配置为：QVGA 原始传输
`324x244 RAW8`、8-bit DVP、外部 12 MHz MCLK、Walking-1、两块内部
DMA-capable 帧缓冲、不裁剪且不输出到显示屏。

当前实现包含两个相互独立的对象：

```text
hm01b0_handle_t
  传感器侧：MCLK、I2C、寄存器、模式配置、Standby/Streaming 状态

hm01b0_capture_handle_t
  ESP32-S3 侧：DVP Camera Controller、GDMA、帧缓冲、队列、回调、
  帧验证和统计
```

`hm01b0_start()` 只负责将 HM01B0 从 Standby 切换到 Streaming，不负责
LCD_CAM、GDMA、帧缓冲或队列。后者全部属于 `hm01b0_capture_*()`。

## 一、单独看阶段 3 的完整执行顺序

本节假设阶段 1、2 已经成功，HM01B0 已配置完成并保持在
`HM01B0_STATE_STANDBY`。

### 1. `app_main()` 准备 Capture 配置

`app_main()` 创建 `hm01b0_capture_config_t capture_config`：

```text
data_gpio[0..7] = GPIO9..GPIO16
pclk_gpio       = GPIO6
vsync_gpio      = GPIO7，对应 HM01B0 FVLD
de_gpio         = GPIO8，对应 HM01B0 LVLD/HREF
raw_width       = 324
raw_height      = 244
dma_burst_size  = 64
task_stack_size = 4096 bytes
task_priority   = 5
stats_period_ms = 1000 ms
Walking-1 观察分析 = 开启
```

这里只是在 `app_main()` 栈上准备配置值，还没有创建 Camera Controller，也
没有分配帧缓冲。

### 2. `hm01b0_capture_new()` 创建整个采集子系统

调用：

```c
hm01b0_capture_new(&capture_config, &s_capture);
```

内部依次执行：

1. `hm01b0_capture_validate_config()` 检查：
   - 配置指针不为空；
   - 原始宽高不为 0；
   - PCLK、VSYNC、DE GPIO 有效；
   - D0～D7 八个 GPIO 全部有效；
   - DMA burst 为 0，或者为 4～128 之间的 2 次幂；
   - FreeRTOS 任务优先级小于 `configMAX_PRIORITIES`。
2. `hm01b0_capture_validate_config()` 内部调用
   `hm01b0_capture_valid_dma_burst()` 判断 burst 是否合法。
3. `heap_caps_calloc()` 从内部、可按字节访问的 RAM 中分配
   `struct hm01b0_capture`。
4. 将应用提供的 `capture_config` 复制到 Capture Handle。
5. 如果任务栈、优先级或统计周期配置为 0，则使用组件内部默认值。当前
   `main.c` 已明确填写三者，所以不会触发默认值替换。
6. 创建 `esp_cam_ctlr_dvp_pin_config_t pin_config`，填入 D0～D7、PCLK、
   VSYNC 和 DE。`xclk_io = GPIO_NUM_NC`，因为 MCLK 由 HM01B0 组件中的
   LEDC 产生。
7. 创建 `esp_cam_ctlr_dvp_config_t dvp_config`：
   - Controller 0；
   - 324x244；
   - RAW8 输入和 RAW8 输出；
   - Camera 数据宽度 8 bit；
   - 不做 bit swap 和 byte swap；
   - 不使用 JPEG；
   - 禁用 Driver backup buffer；
   - `external_xtal = 1`，Camera Controller 不产生 MCLK；
   - GDMA burst 为 64 bytes。
8. 调用 `esp_cam_new_dvp_ctlr()` 创建 ESP-IDF DVP Controller。
9. `esp_cam_new_dvp_ctlr()` 在 ESP-IDF 内部继续完成：
   - 根据分辨率和 RAW8 计算、对齐帧长度；
   - 分配 DVP Controller 上下文；
   - 占用 Camera 外设和 GPIO 信号；
   - 创建 GDMA RX Channel；
   - 将 GDMA Channel 连接到 Camera 外设触发源；
   - 配置 burst 和内存访问能力；
   - 查询内部 SRAM/外部 PSRAM 的 DMA 对齐要求；
   - 分配 GDMA Descriptor 数组；
   - 注册 GDMA receive-EOF 中断；
   - 配置 DVP 输入 GPIO；
   - 初始化 Camera HAL 和 RAW8 数据路径；
   - 将 enable/start/stop/disable/alloc_buffer 等实现函数放入 Controller。
10. 调用 `esp_cam_ctlr_get_frame_buffer_len()`，取得 Driver 要求的缓冲容量。
    当前结果为 79056 bytes。
11. Capture Handle 分开保存三个长度：
    - `payload_size = 324 * 244 = 79056`；
    - `buffer_capacity`，由 Driver 返回；
    - `received_size`，每帧完成后由 Driver 单独填写。
12. 如果 `buffer_capacity < payload_size`，立即返回
    `ESP_ERR_INVALID_SIZE`。
13. 调用 `heap_caps_get_free_size()` 和
    `heap_caps_get_largest_free_block()`，记录分配帧缓冲前的内部
    DMA-capable 剩余内存及最大连续块。
14. 第一次调用 `esp_cam_ctlr_alloc_buffer()`，使用
    `MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA` 分配 Buffer A，并保存到
    `frames[0].data`。
15. 第二次调用 `esp_cam_ctlr_alloc_buffer()`，同样分配 Buffer B，保存到
    `frames[1].data`。
16. ESP-IDF 的 `esp_cam_ctlr_alloc_buffer()` 最终使用
    `heap_caps_aligned_calloc()`，按 GDMA 内部 SRAM 对齐要求申请并清零。
17. 再次记录内部 DMA-capable 剩余内存及最大连续块。
18. 调用 `xQueueCreateStatic()` 创建 `free_queue`。它最多保存两个
    `hm01b0_capture_frame_t *`，不保存图像字节。
19. 调用 `xQueueCreateStatic()` 创建 `ready_queue`，容量和元素类型相同。
20. 调用 `hm01b0_capture_reset_queues()`：
    - `xQueueReset(free_queue)`；
    - `xQueueReset(ready_queue)`；
    - 清零 `frames[0].received_size` 和 `frames[1].received_size`；
    - 将 `&frames[0]`、`&frames[1]` 放入 `free_queue`。
21. 此时 Buffer 所有权状态为：

    ```text
    free_queue  = [A, B]
    ready_queue = []
    DMA          = 尚未启动
    ```

22. 调用 `esp_cam_ctlr_register_event_callbacks()` 注册：
    - `hm01b0_capture_on_get_new_trans()`；
    - `hm01b0_capture_on_trans_finished()`；
    - Capture Handle 作为两个回调的 `user_data`。
23. 调用 `xTaskCreate()` 创建 `hm01b0_capture_task()`。
24. 帧处理任务初始化统计时间，然后阻塞等待 `ready_queue`；因为还没有
    完成帧，所以暂时不做 CRC 或 Walking-1 分析。
25. 调用 `hm01b0_capture_log_memory()`，打印：
    - 分配 A/B 前后的 DMA-capable 内存；
    - 最大连续块；
    - Buffer A/B 地址；
    - A/B 是否位于内部 SRAM；
    - 每块容量和 payload 长度。
26. `hm01b0_capture_new()` 将完整 Handle 写入 `s_capture`，返回
    `ESP_OK`。此时 Camera RX 仍未开始接收。

如果上述任意步骤失败，函数跳到 `fail`，调用
`hm01b0_capture_delete()`，清理由当前构造过程已经成功创建的资源。

### 3. `hm01b0_capture_start()` 先启动 ESP32-S3 接收端

调用：

```c
hm01b0_capture_start(s_capture);
```

内部依次执行：

1. 检查 Handle 不为空。
2. 检查 Controller 尚未启动，避免重复 start。
3. 检查当前没有正在处理的帧。
4. 再次调用 `hm01b0_capture_reset_queues()`，确保启动前状态为
   `free=[A,B]`、`ready=[]`。
5. 调用 `esp_cam_ctlr_enable()`：
   - ESP-IDF DVP 状态从 INIT 变为 ENABLED；
   - Camera 硬件/FIFO 被复位并启用；
   - 如果启用了电源管理，则获取对应 PM Lock。
6. 设置 `controller_enabled = true`。
7. 调用 `esp_cam_ctlr_start()`：
   - ESP-IDF DVP 状态从 ENABLED 变为 STARTED；
   - Driver 内部调用 `esp_cam_ctlr_dvp_start_trans()` 准备第一笔事务。
8. `esp_cam_ctlr_dvp_start_trans()` 调用我们的
   `hm01b0_capture_on_get_new_trans()`。
9. 第一次回调可能运行在普通任务上下文，因此执行：

   ```c
   xQueueReceive(free_queue, &frame, 0);
   ```

10. 回调从 `free_queue` 取出 Frame A，填写：

    ```c
    trans->buffer = frames[0].data;
    trans->buflen = buffer_capacity;
    ```

11. 此时状态为：

    ```text
    free_queue  = [B]
    DMA          = 正在准备使用 A
    ready_queue = []
    ```

12. ESP-IDF 在启动 DMA 前调用
    `esp_cache_msync(buffer, length, M2C)`。
13. 链接器将这个调用重定向到
    `__wrap_esp_cache_msync()`：
    - 先调用 `__real_esp_cache_msync()`；
    - 对可缓存内存和真实错误保留 ESP-IDF 原行为；
    - 只把“内部 SRAM + M2C + `ESP_ERR_NOT_SUPPORTED`”变成
      `ESP_OK`，因为内部 SRAM 不经过数据 Cache，无需同步。
14. ESP-IDF Reset GDMA Channel。
15. `esp_cam_ctlr_dvp_dma_start()` 重建 DMA Descriptor 链，使所有节点
    依次指向 Buffer A 的不同区段。
16. `gdma_start()` 从第一个 Descriptor 开始启动 DMA。
17. Driver 将 Buffer A 记录为当前事务 Buffer。
18. `hm01b0_capture_start()` 设置 `controller_started = true`，打印
    “等待 HM01B0 FVLD/PCLK”，然后返回。

这时 ESP32-S3 已经准备好，但 HM01B0 仍在 Standby，因此尚无图像输入。

### 4. `hm01b0_start()` 最后启动 HM01B0 输出

虽然 `hm01b0_start()` 属于阶段 2 的传感器状态机，但它是阶段 3 真正开始
接收前的最后一步：

1. `hm01b0_require_standby()` 检查传感器处于 Standby。
2. `hm01b0_reg_update_bits()` 更新 `MODE_SELECT`：
   - `hm01b0_reg_read()` 读取当前值；
   - 按 mask 保留无关 bit；
   - 如果值发生变化，`hm01b0_reg_write()` 写入 STREAMING。
3. 软件状态变为 `HM01B0_STATE_STREAMING`。
4. HM01B0 开始输出 FVLD、LVLD、PCLK 和 D0～D7。

硬件数据路径变为：

```text
HM01B0 D0-D7/PCLK/FVLD/LVLD
  -> GPIO Matrix
  -> LCD_CAM Camera RX
  -> Camera RX FIFO
  -> GDMA RX Channel
  -> GDMA Descriptor 链
  -> 当前内部 SRAM Frame Buffer
```

CPU 不逐像素复制数据。

### 5. 第一帧完成：切换 Buffer 并唤醒帧处理任务

第一帧采集中：

```text
free_queue  = [B]
DMA 正在写   = A
ready_queue = []
帧处理任务    = 阻塞等待 ready_queue
```

Buffer A 到达 DMA EOF 后，ESP-IDF 的 GDMA ISR 按以下真实顺序执行：

1. 保存“刚完成的是 Buffer A”。
2. 取得 GDMA 实际接收字节数。
3. 在上报 A 完成之前，先调用 Driver 内部
   `esp_cam_ctlr_dvp_start_trans()`，尽快准备下一帧。
4. `esp_cam_ctlr_dvp_start_trans()` 从 ISR 中调用
   `hm01b0_capture_on_get_new_trans()`。
5. 回调通过 `xPortInIsrContext()` 判断当前为 ISR，使用：

   ```c
   xQueueReceiveFromISR(free_queue, &frame, &task_woken);
   ```

6. 从 `free_queue` 取出 Frame B，将 `frames[1].data` 和容量交给 Driver。
7. Cache Wrapper 再次处理内部 SRAM 无需 Cache 同步的情况。
8. ESP-IDF Reset GDMA、将 Descriptor 链改为指向 B，并在 Buffer B 上启动
   下一帧 DMA。
9. 此时暂时为：

   ```text
   free_queue  = []
   DMA 正在写   = B
   A            = ISR 已知其完成，但尚未放进 ready_queue
   ```

10. ESP-IDF 填写 A 的 `trans.received_size`，调用
    `hm01b0_capture_on_trans_finished()`。
11. `hm01b0_capture_on_trans_finished()` 调用
    `hm01b0_capture_find_frame(handle, trans->buffer)`。
12. `hm01b0_capture_find_frame()` 比较：
    - `trans.buffer == frames[0].data`；
    - `trans.buffer == frames[1].data`。
    对 A 返回 `&frames[0]`。
13. `hm01b0_capture_on_trans_finished()`：
    - 保存 `frame->received_size`；
    - 增加 ISR 序号并写入 `frame->sequence`；
    - 增加 `isr_frames_received`；
    - 调用 `xQueueSendFromISR(ready_queue, &frame, &task_woken)`。
14. `ready_queue` 变为 `[A]`。
15. `hm01b0_capture_task()` 正阻塞等待该队列，所以 FreeRTOS 将它从
    Blocked 变为 Ready。
16. 如果该任务应该在 ISR 退出后立即抢占，`task_woken = pdTRUE`。
17. 回调把这个布尔值返回给 ESP-IDF，GDMA ISR 再返回给 FreeRTOS，允许
    ISR 退出后马上进行任务切换。

因此，“提醒帧处理任务”的机制就是：

```text
xQueueSendFromISR(ready_queue)
  -> FreeRTOS 发现有任务阻塞在该队列
  -> 唤醒 hm01b0_capture_task()
```

没有额外使用 Task Notification、Semaphore 或轮询标志。

### 6. `hm01b0_capture_task()` 处理并归还完成 Buffer

Frame A 被放入 `ready_queue` 后：

1. `xQueueReceive(ready_queue, &frame, 100 ms)` 返回 Frame A。
2. 设置 `processing_frame = true`。
3. 前 5 帧作为 warm-up，只检查传输长度，不建立内容基准。
4. 调用 `hm01b0_capture_process_frame(handle, frame)`。
5. `hm01b0_capture_process_frame()` 用 `esp_timer_get_time()` 记录开始时间。
6. 检查 `received_size` 是否严格等于 `payload_size`（79056 bytes）。
7. 第一帧通过后，将长度保存为 `baseline_received_size`。
8. 后续帧长度变化增加 `received_size_changes`，非 79056-byte 帧增加
   `size_errors`；长度正确即增加 `transport_valid_frames`。
9. warm-up 结束后，调用 `esp_crc32_le()` 计算完整 324x244 raw CRC，
   再逐行累计 x=2、width=320 的 320x244 active CRC。
10. 第一张 warm-up 后的帧建立 raw/active CRC 基准；active CRC 还会与
    前一帧比较，区分“始终不同于基准”和“相邻帧发生变化”。
11. 开启 Walking-1 观察时，调用 `hm01b0_analyze_walking_one()`：
    - 只分析 x=2、y=0、320x244，不复制或裁剪 DMA Buffer；
    - 比较后 243 行与第一行，统计相同行数和垂直 mismatch；
    - 统计第一行水平方向的数值跳变次数；
    - 统计 unique、zero、one-hot 和 other 数值数量；
    - 不再假定 datasheet 未定义的逐像素 one-hot 循环序列。
12. 需要输出长度错误时调用
    `hm01b0_capture_error_log_allowed()`，把错误日志限制为每秒最多一次。
13. CRC和 Walking-1 仅作为内容诊断，不会把长度正确的 DMA 帧判为无效。
14. 记录 `last_processing_time_us`，必要时更新
    `max_processing_time_us`。
15. 从 `hm01b0_capture_process_frame()` 返回。
16. 第一张分析帧先复制 raw 第一行和 active 顶部/中部/底部各
    32 bytes 到任务栈上的小快照。
17. 调用 `xQueueSend(free_queue, &frame, 0)`，将 A 归还空闲池。
18. 设置 `processing_frame = false`，再从小快照打印四组样本；只打印
    一次，不占用 DMA Buffer 输出串口，也不输出完整帧。
19. 达到统计周期时调用 `hm01b0_capture_log_stats()`：
    - 快照 ISR 计数；
    - 根据帧数增量和微秒时间计算 `fps_milli`；
    - 打印 transport、warm-up、长度和队列状态；
    - 打印 raw/active CRC 变化和 Walking-1 结构统计；
    - 打印处理时间和队列深度。
20. 任务重新阻塞等待 `ready_queue`。

归还 A 后，B 仍由 DMA 写入：

```text
free_queue  = [A]
DMA 正在写   = B
ready_queue = []
```

### 7. 后续帧持续交换 A/B

Buffer B 到达 EOF 时，ISR 必须先取得 A：

```text
on_get_new_trans(): A 从 free_queue -> DMA
on_trans_finished(): B 从“DMA 已完成” -> ready_queue
frame task: B 从 ready_queue -> 处理 -> free_queue
```

稳定循环为：

```text
DMA：  A -> B -> A -> B -> ...
任务：      A -> B -> A -> B -> ...
```

由于 Driver 会先请求下一块 Buffer，再上报上一块完成，帧处理任务必须在一个
帧周期内归还上一块 Buffer。当前禁用了 backup buffer；如果
`free_queue` 为空，`hm01b0_capture_on_get_new_trans()` 会增加
`no_free_buffer`，而 Driver 没有第三块 Buffer 可以继续。

如果 `ready_queue` 已满，`hm01b0_capture_on_trans_finished()` 会增加
`ready_queue_overflows`，并尝试把该完成帧直接退回 `free_queue`。该帧被
丢弃而不是处理，以免 Buffer 永久丢失。

### 8. 没有帧时的执行路径

如果 `ready_queue` 连续 100 ms 没有帧：

1. `xQueueReceive()` 超时。
2. 帧任务检查统计定时器。
3. 达到统计周期后仍调用 `hm01b0_capture_log_stats()`，因此 FVLD/PCLK
   缺失会表现为 `fps=0`，不会让任务永久沉默。
4. 任务再次进入队列等待。

### 9. 阶段 3 停止与删除顺序

推荐先停止传感器输出，再停止接收器：

```text
hm01b0_stop(sensor)
  -> hm01b0_standby()
  -> hm01b0_reg_update_bits(MODE_SELECT, STANDBY)

hm01b0_capture_stop(capture)
  -> esp_cam_ctlr_stop()
  -> esp_cam_ctlr_disable()
  -> 等待 processing_frame=false 且 ready_queue 为空
  -> hm01b0_capture_reset_queues()
```

`hm01b0_capture_stop()` 最多等待 500 ms。如果任务仍在使用 Buffer，则返回
`ESP_ERR_TIMEOUT`，不会在所有权不安全时强行 Reset 队列。

`hm01b0_capture_delete()` 依次执行：

1. 调用 `hm01b0_capture_stop()`。
2. 如果 Controller 仍然 Started/Enabled，则返回而不释放可能仍由 DMA
   持有的内存。
3. 设置 `task_should_exit = true`。
4. 最多等待 500 ms，让 `hm01b0_capture_task()` 自行退出循环并调用
   `vTaskDelete(NULL)`。
5. 如果任务没有在超时内退出，才强制 `vTaskDelete(task_handle)`。
6. `esp_cam_ctlr_del()` 删除 DVP Controller，并由 Driver 释放 GDMA Channel
   和 Descriptor。
7. `heap_caps_free()` 释放 Buffer A、Buffer B。
8. `vQueueDelete()` 删除 free/ready 队列。
9. 释放 Capture Handle。

`hm01b0_capture_get_stats()` 是公开的诊断函数，它复制统计结构并补充当前 ISR
计数。当前 `main.c` 没有调用它，因为内部帧任务已经定期输出统计。

## 二、阶段 1、2、3 串联后的完整执行顺序

本节从 ESP-IDF 进入 `app_main()` 开始，描述当前正常启动的完整路径。

### 1. ESP-IDF 进入 `app_main()`

1. ROM 和二级 Bootloader 启动芯片。
2. ESP-IDF 初始化时钟、Heap、驱动和 FreeRTOS。
3. ESP-IDF Main Task 调用 `app_main()`。
4. `app_main()` 打印控制引脚和 8-bit DVP GPIO 映射。
5. 创建 `hm01b0_config_t config`：
   - GPIO5 输出 12 MHz MCLK；
   - I2C0，SDA GPIO2，SCL GPIO1，100 kHz；
   - 开启内部 I2C 上拉；
   - 初始 QVGA；
   - 8-bit 接口；
   - Walking-1。

### 2. 阶段 1：`hm01b0_new()` 建立 MCLK 和 I2C/寄存器访问

调用：

```c
hm01b0_new(&config, &s_sensor);
```

内部顺序：

1. 验证配置和输出指针。
2. 先将 `*out_handle = NULL`，避免失败时暴露半初始化 Handle。
3. `calloc()` 分配 `struct hm01b0_dev`。
4. 初始化内部字段：
   - LEDC Low-speed；
   - Timer 0；
   - Channel 0；
   - 状态 UNINITIALIZED。
5. 调用内部 `hm01b0_start_mclk()`：
   - `ledc_timer_config()` 创建目标 12 MHz、1-bit duty timer；
   - `ledc_channel_config()` 将约 50% 占空比波形路由到 GPIO5；
   - 设置 `mclk_started = true`；
   - `ledc_get_freq()` 读取并打印实际 MCLK。
6. `esp_rom_delay_us(50)`，确保 HM01B0 在有 MCLK 的情况下完成 POR 等待。
7. 创建 `i2c_master_bus_config_t`，调用 `i2c_new_master_bus()`。
8. 创建地址为 `0x24`、100 kHz 的 `i2c_device_config_t`。
9. 调用 `i2c_master_bus_add_device()`，取得 HM01B0 I2C Device Handle。

此时阶段 1 的底层寄存器函数已经可用：

- `hm01b0_reg_read()`：构造 16-bit 寄存器地址高/低字节，调用
  `i2c_master_transmit_receive()`，完成写地址、Repeated START、读 1 byte。
- `hm01b0_reg_write()`：构造地址高、地址低、数据三个字节，调用
  `i2c_master_transmit()`。
- `hm01b0_reg_update_bits()`：先调用 `hm01b0_reg_read()`，计算
  `(current & ~mask) | (value & mask)`；只有值变化时才调用
  `hm01b0_reg_write()`。

### 3. 阶段 1 验收：`hm01b0_new()` 内第一次 Probe

`hm01b0_new()` 调用 `hm01b0_probe()`：

1. `hm01b0_reg_read(HM01B0_REG_MODEL_ID_H)` 读取 `0x0000`。
2. `hm01b0_reg_read(HM01B0_REG_MODEL_ID_L)` 读取 `0x0001`。
3. 组合 `(id_high << 8) | id_low`。
4. 将结果写入可选的 `model_id` 输出。
5. 与 `HM01B0_EXPECTED_MODEL_ID = 0x01B0` 比较。
6. 不匹配返回 `ESP_ERR_NOT_FOUND`；匹配返回 `ESP_OK`。

ID 未通过前，不执行模式寄存器表或 DMA 初始化。

### 4. 阶段 2：Software Reset 和 Standby

Probe 成功后，`hm01b0_new()` 调用：

1. `hm01b0_reset()`：
   - `hm01b0_reg_write(SW_RESET, SOFTWARE_RESET)`；
   - `esp_rom_delay_us(1000)` 等待恢复；
   - 软件状态设置为 STANDBY。
2. `hm01b0_standby()`：
   - `hm01b0_reg_update_bits(MODE_SELECT, mask, STANDBY)`；
   - 内部可能调用 `hm01b0_reg_read()` 和 `hm01b0_reg_write()`；
   - 软件状态保持 STANDBY。

显式 Standby 保证后续所有配置表满足“不在 Streaming 中配置”的要求。

### 5. 阶段 2：写入 Common Init 表

`hm01b0_new()` 调用：

```c
hm01b0_write_table(dev,
                   hm01b0_common_init,
                   hm01b0_common_init_count);
```

`hm01b0_write_table()` 的逐项路径：

1. 检查 Handle、表指针和 count。
2. 只有状态为 STANDBY 才允许写表。
3. 遍历每一个 `hm01b0_regval_t`。
4. `mask == 0xFF` 时调用 `hm01b0_reg_write()` 直接写。
5. 部分 mask 时调用 `hm01b0_reg_update_bits()`；后者继续调用
   `hm01b0_reg_read()`，必要时再调用 `hm01b0_reg_write()`。
6. `delay_ms > 0` 时用 `pdMS_TO_TICKS()` 转换，并调用至少一个 Tick 的
   `vTaskDelay()`。
7. 遇到第一个寄存器错误立即停止并返回；全部成功才返回 `ESP_OK`。

Common 表配置 BLC、坏点修复、统计、AE、积分/增益、Flicker 默认值、关闭
Motion Detection 及默认图像方向等公共参数。

### 6. 阶段 2：写入 QVGA 模式表

`hm01b0_new()` 调用
`hm01b0_set_mode(HM01B0_SENSOR_MODE_QVGA)`：

1. `hm01b0_require_standby()` 检查 Handle 和 Standby 状态。
2. `switch` 选择 `hm01b0_mode_qvga` 及其 count。
3. 调用 `hm01b0_write_table()`，每项继续走 direct write 或
   read-modify-write 路径。
4. 表中启用固定 QVGA Window、保持 RAW8、关闭 binning，并设置
   MAX_INTEGRATION、FRAME_LENGTH_LINES、LINE_LENGTH_PCK。
5. 设置 `dev->mode = HM01B0_SENSOR_MODE_QVGA`。

`hm01b0_set_mode()` 还包含 FULL 和 QQVGA 分支，分别选择
`hm01b0_mode_full`、`hm01b0_mode_qqvga`，当前启动不会走这两条分支。

### 7. 阶段 2：写入 8-bit 接口和时钟表

`hm01b0_new()` 调用
`hm01b0_set_interface(HM01B0_DATA_INTERFACE_8_BIT)`：

1. `hm01b0_require_standby()` 检查状态。
2. `switch` 选择 `hm01b0_interface_8bit`。
3. `hm01b0_write_table()` 写入全部接口表项。
4. 当前表配置：
   - 8-bit 数据接口；
   - Sensor_Core = MCLK / 2；
   - Sensor_Register = MCLK / 1；
   - 外部 MCLK 时钟源；
   - Non-SYNC；
   - Non-gated PCLK；
   - PCLK Rising Edge 约定。
5. 设置 `dev->interface = HM01B0_DATA_INTERFACE_8_BIT`。

4-bit/1-bit 的表和选择分支已存在，但当前阶段 3 接收端不使用。

### 8. 阶段 2：写入 Walking-1 测试图表

`hm01b0_new()` 调用
`hm01b0_set_test_pattern(HM01B0_TEST_PATTERN_WALKING_1)`：

1. `hm01b0_require_standby()` 检查状态。
2. `switch` 选择 `hm01b0_test_pattern_walking_1`。
3. `hm01b0_write_table()` 对 TEST_PATTERN_MODE 做 masked update。
4. `hm01b0_reg_read(0x0601)` 读回 TEST_PATTERN_MODE。
5. 对 enable/select mask 检查读回值为 `0x11`，否则初始化失败。
6. 设置 `dev->test_pattern = HM01B0_TEST_PATTERN_WALKING_1` 并打印读回值。

OFF 和 COLOR_BAR 表及其选择分支存在，但当前启动不调用。

`hm01b0_new()` 至此打印初始化完成，将 Device Handle 写入 `s_sensor`，
并以 STANDBY 状态返回。

### 9. `app_main()` 再次 Probe 和检查状态

`hm01b0_new()` 返回后，`app_main()` 有意再次验证：

1. 调用 `hm01b0_probe(s_sensor, &model_id)`。
2. `hm01b0_probe()` 再次通过 `hm01b0_reg_read()` 读取 `0x0000/0x0001`。
3. 调用 `hm01b0_get_state(s_sensor)`。它只返回软件状态，不访问 I2C。
4. 只有 ID 正确且状态为 STANDBY 才继续，并打印应用层 PASS。

第二次 Probe 对库初始化而言是重复的，但它作为应用层阶段验收点保留下来。

### 10. 阶段 3：创建接收端并按 Receiver-first 顺序启动

`app_main()` 接着执行本文件第一部分详述的阶段 3：

```text
构造 hm01b0_capture_config_t
  -> hm01b0_capture_new()
       -> hm01b0_capture_validate_config()
            -> hm01b0_capture_valid_dma_burst()
       -> esp_cam_new_dvp_ctlr()
       -> esp_cam_ctlr_get_frame_buffer_len()
       -> esp_cam_ctlr_alloc_buffer(A)
       -> esp_cam_ctlr_alloc_buffer(B)
       -> xQueueCreateStatic(free_queue)
       -> xQueueCreateStatic(ready_queue)
       -> hm01b0_capture_reset_queues()
       -> esp_cam_ctlr_register_event_callbacks()
       -> xTaskCreate(hm01b0_capture_task)
       -> hm01b0_capture_log_memory()

hm01b0_capture_start()
  -> hm01b0_capture_reset_queues()
  -> esp_cam_ctlr_enable()
  -> esp_cam_ctlr_start()
       -> Driver esp_cam_ctlr_dvp_start_trans()
       -> hm01b0_capture_on_get_new_trans()
       -> __wrap_esp_cache_msync()
            -> __real_esp_cache_msync()
       -> Driver Reset DMA / 配置 Descriptor / gdma_start()

hm01b0_start()
  -> hm01b0_require_standby()
  -> hm01b0_reg_update_bits(MODE_SELECT, STREAMING)
       -> hm01b0_reg_read()
       -> 值变化时 hm01b0_reg_write()
  -> 状态变为 STREAMING
```

顺序始终是先启动 ESP32 Camera RX，再启动 HM01B0 Streaming。

### 11. 连续 Streaming 时的三个执行上下文

#### 11.1 HM01B0 与 ESP32-S3 硬件

HM01B0 输出 DVP 信号；LCD_CAM 按 PCLK 采样 RAW8，Camera FIFO 向 GDMA
发请求；GDMA 根据 Descriptor 直接将数据写入当前 SRAM Buffer。

#### 11.2 GDMA EOF 中断上下文

每帧完成：

```text
ESP-IDF GDMA EOF ISR
  -> Driver 保存完成 Buffer 和 DMA 字节数
  -> Driver esp_cam_ctlr_dvp_start_trans() 准备下一帧
       -> hm01b0_capture_on_get_new_trans()
       -> __wrap_esp_cache_msync()
       -> 重配 DMA Descriptor 并 gdma_start()
  -> hm01b0_capture_on_trans_finished() 上报上一帧
       -> hm01b0_capture_find_frame()
       -> 保存 received_size / sequence / ISR 计数
       -> xQueueSendFromISR(ready_queue)
       -> 必要时请求 ISR 退出后任务切换
```

ISR 不计算整帧 CRC、不遍历 Walking-1、不刷新显示、不阻塞等待，也不输出
普通日志。

#### 11.3 帧处理任务上下文

```text
hm01b0_capture_task()
  -> xQueueReceive(ready_queue, timeout=100 ms)
  -> hm01b0_capture_process_frame()
       -> 严格长度/长度基准检查
       -> 前 5 帧 warm-up
       -> esp_crc32_le(raw 324x244)
       -> hm01b0_capture_active_crc(active 320x244)
       -> hm01b0_analyze_walking_one()
            -> hm01b0_is_one_hot()
       -> 长度出错时 hm01b0_capture_error_log_allowed()
       -> 更新传输/CRC/Walking-1/处理时间统计
  -> warm-up 后第一张分析帧调用 hm01b0_capture_log_first_analysis()
  -> xQueueSend(free_queue) 归还 Buffer
  -> 到周期时 hm01b0_capture_log_stats()
  -> 重复
```

#### 11.4 Main Task 上下文

启动成功后，`app_main()` 进入：

```c
while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000U));
}
```

Main Task 只保持应用存活，不处理图像；ISR 和帧处理任务独立运行。

### 12. `app_main()` 启动错误清理路径

当前代码包含以下失败路径：

1. `hm01b0_new()` 失败：
   - 构造函数内部在需要时调用 `hm01b0_delete()`；
   - `app_main()` 打印错误并返回。
2. 应用层 Probe/状态检查失败：
   - 调用 `hm01b0_delete()`；
   - 如果正在 Streaming，内部先 `hm01b0_stop()`；
   - 删除 I2C Device；
   - 删除 I2C Bus；
   - 调用内部 `hm01b0_stop_mclk()`；
   - 释放 Sensor Handle。
3. `hm01b0_capture_new()` 失败：
   - Capture 构造函数内部调用 `hm01b0_capture_delete()` 清理部分资源；
   - `app_main()` 调用 `hm01b0_delete()`。
4. `hm01b0_capture_start()` 失败：
   - 调用 `hm01b0_capture_delete()`；
   - 调用 `hm01b0_delete()`。
5. RX 已启动但 `hm01b0_start()` 失败：
   - 调用 `hm01b0_capture_stop()`；
   - 调用 `hm01b0_capture_delete()`；
   - 调用 `hm01b0_delete()`。

当前成功路径无限循环，因此不会主动执行正常关闭。未来需要退出 Streaming 时，
推荐调用：

```text
hm01b0_stop(sensor)
  -> hm01b0_capture_stop(capture)
  -> hm01b0_capture_delete(capture)
  -> hm01b0_delete(sensor)
```

## 三、当前全部项目函数与调用时机

### 1. Main 组件

| 函数 | 当前正常路径中的调用时机 |
|---|---|
| `app_main()` | ESP-IDF Main Task 进入应用后调用；负责串联 Sensor 初始化、Capture 初始化、Receiver-first Start，并在成功后保持应用存活 |

### 2. Sensor 组件

| 函数 | 当前正常路径中的调用时机 |
|---|---|
| `hm01b0_start_mclk()` | `hm01b0_new()` 内部调用 |
| `hm01b0_stop_mclk()` | 构造失败、Delete 或未来正常关闭 |
| `hm01b0_new()` | 应用启动 |
| `hm01b0_delete()` | 启动错误或未来正常关闭 |
| `hm01b0_reg_read()` | Probe、masked update、Start/Standby |
| `hm01b0_reg_write()` | Reset、寄存器表、masked update |
| `hm01b0_reg_update_bits()` | 部分 mask 表项和状态切换 |
| `hm01b0_write_table()` | Common、QVGA、8-bit、Walking-1 表 |
| `hm01b0_require_standby()` | Mode/Interface/Pattern 配置和 Start |
| `hm01b0_probe()` | `hm01b0_new()` 内一次，`app_main()` 内再一次 |
| `hm01b0_reset()` | `hm01b0_new()` 内 |
| `hm01b0_standby()` | 初始化和 Stop |
| `hm01b0_start()` | Camera RX 准备好以后 |
| `hm01b0_stop()` | 错误清理或未来正常关闭 |
| `hm01b0_set_mode()` | 初始化时走 QVGA 分支 |
| `hm01b0_set_interface()` | 初始化时走 8-bit 分支 |
| `hm01b0_set_test_pattern()` | 初始化时走 Walking-1 分支 |
| `hm01b0_get_state()` | `app_main()` 初始化后状态检查 |

### 3. Capture 组件

| 函数 | 当前正常路径中的调用时机 |
|---|---|
| `hm01b0_capture_valid_dma_burst()` | Capture 配置检查 |
| `hm01b0_capture_validate_config()` | Capture 构造开始 |
| `hm01b0_capture_reset_queues()` | 构造、Start、成功 Stop |
| `hm01b0_capture_log_memory()` | A/B 分配完成后执行一次 |
| `hm01b0_capture_new()` | 应用启动 |
| `hm01b0_capture_start()` | `hm01b0_start()` 之前 |
| `hm01b0_capture_on_get_new_trans()` | 第一笔 DMA 和每次帧切换 |
| `__wrap_esp_cache_msync()` | Driver 每笔事务启动前的 Cache 调用 |
| `__real_esp_cache_msync()` | 由 Wrapper 先调用原始实现 |
| `hm01b0_capture_on_trans_finished()` | 每帧 DMA 完成 |
| `hm01b0_capture_find_frame()` | Finished 回调内部 |
| `hm01b0_capture_task()` | 持续运行的 FreeRTOS Task |
| `hm01b0_capture_process_frame()` | 每个从 ready_queue 取出的帧 |
| `hm01b0_capture_active_crc()` | warm-up 后逐行计算 320x244 active CRC |
| `hm01b0_analyze_walking_one()` | warm-up 后观察有效区行结构和值分布 |
| `hm01b0_is_one_hot()` | Walking-1 值分类，不再作为整帧硬性判错条件 |
| `hm01b0_capture_take_analysis_sample()` | 归还 DMA Buffer 前复制四组 32-byte 小样本 |
| `hm01b0_capture_log_first_analysis()` | 仅第一张分析帧输出四组小样本 |
| `hm01b0_capture_error_log_allowed()` | 有详细错误需要打印时 |
| `hm01b0_capture_log_stats()` | 每个统计周期，包括无帧情况 |
| `hm01b0_capture_stop()` | 启动错误或未来正常关闭 |
| `hm01b0_capture_delete()` | 构造/启动错误或未来正常关闭 |
| `hm01b0_capture_get_stats()` | 公开 API，当前 `main.c` 未调用 |

`main/st7789_display.c` 当前没有加入 `main/CMakeLists.txt` 的 `SRCS`，因此
其中的显示函数不属于阶段 1～3 的实际执行路径。

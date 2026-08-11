# SCAU-RA-2026 水果采摘机器人

本工程基于 Renesas RA8P1 双核平台。CPU0 负责 OV5640 图像采集、NPU 水果识别、LVGL 屏幕显示和 IPC 发送；CPU1 负责接收检测结果，并为后续坐标标定、逆运动学与 CAN 电机控制提供数据。

使用串口助手时，请开启“发送新行”。

## 目标类别编号

| 类别编号 | 水果类型 |
| ---: | --- |
| 0 | 番茄 |
| 1 | 绿葡萄 |
| 2 | 紫色葡萄 |

类别编号由 CPU0 识别模块产生，并通过 IPC 原样发送给 CPU1。

## 系统流程

```text
CPU0 启动 CPU1
  → 初始化 IPC、OV5640 和 NPU
  → 等待自动曝光 1500 ms
  → 丢弃前 5 帧
  → 持续采集 320×240 RGB565 图像
  → 裁剪中央 240×240 区域并缩放为 256×256
  → 执行 NPU 推理、置信度过滤和 NMS
  → 更新 LVGL 检测状态
  → 将有效检测结果通过 IPC 发送给 CPU1
```

当前检测置信度阈值为 `0.8`，单帧最多保留 3 个结果。每个结果包含：

```c
typedef struct st_app_detection_result
{
    uint32_t class_id;
    int32_t  x;
    int32_t  y;
} app_detection_result_t;
```

`x`、`y` 是检测框在 320×240 摄像头画面中的中心像素坐标。置信度和边界框只在 CPU0 内部用于过滤与 NMS，不通过 IPC 发送。

## 屏幕交互

屏幕分辨率为 480×320，显示和触摸坐标均旋转 180°。

- 未检测到水果时，首页 `Detected` 区域显示 `None`，`START` 按钮被禁用，不能进入选择页。
- 同时检测到多种水果时，首页 `Detected` 区域会列出全部已检测水果类型，并启用 `START`。
- 点击 `START` 后进入选择页；该页只显示本轮实际检测到的水果，用户可点击对应的 `PICK`。
- 选择水果后进入详情页，显示用户所选水果的类型和检测坐标。
- 检测线程只发布数据；所有 LVGL 控件更新均由屏幕线程执行，避免跨线程操作 LVGL。

## CPU0 与 CPU1 的 IPC 协议

CPU0 和 CPU1 分别保存一份内容一致的协议头：

```text
vision_CPU0/src/ipc_detection_protocol.h
vision_CPU1/src/ipc_detection_protocol.h
```

协议常量：

```c
#define IPC_DETECTION_BEGIN             (0x44455442U) /* "DETB" */
#define IPC_DETECTION_END               (0x44455445U) /* "DETE" */
#define IPC_DETECTION_MAX_RESULTS       (3U)
#define IPC_DETECTION_WORDS_PER_RESULT  (3U)
```

每个 IPC 消息为一个 32 位字，完整数据包按以下顺序发送：

```text
IPC_DETECTION_BEGIN
result_count

result[0].class_id
result[0].x
result[0].y

result[1].class_id
result[1].x
result[1].y
...

IPC_DETECTION_END
```

最大数据包包含：

```text
1 + 1 + 3×3 + 1 = 12 个 32 位 IPC 消息
```

### CPU0 发送

发送模块位于：

```text
vision_CPU0/src/hardware/ipc_detection_tx.c
vision_CPU0/src/hardware/ipc_detection_tx.h
```

发送接口：

```c
bool ipc_detection_send_results(app_detection_result_t const * p_results,
                                uint32_t                         result_count);
```

IPC 整包发送总超时为 100 ms。FIFO 持续满时，发送函数会返回 `false`，Camera 线程不会永久阻塞。

超时可能发生在数据包中间，因此 CPU1 在任何接收状态下收到新的 `IPC_DETECTION_BEGIN` 时，都会丢弃未完成数据并重新同步。

### CPU1 接收

CPU1 的接收状态机位于 `vision_CPU1/src/Can_Thread_entry.c`，状态顺序为：

```text
WAIT_BEGIN
  → READ_COUNT
  → READ_CLASS
  → READ_X
  → READ_Y
  → WAIT_END
```

接收端行为：

1. 任意状态收到 `IPC_DETECTION_BEGIN` 都重新开始接收。
2. 检查 `result_count <= IPC_DETECTION_MAX_RESULTS`。
3. 按顺序接收每个目标的 `class_id、x、y`。
4. 只有收到正确的 `IPC_DETECTION_END` 后才发布完整结果。
5. 任务通过以下接口原子地取走最新一包数据：

```c
bool ipc_detection_take_results(ipc_detection_result_t * p_results,
                                uint32_t                   result_capacity,
                                uint32_t                 * p_result_count);
```

IPC 中断回调只负责解析、暂存和发布数据，不执行逆运动学、CAN 控制、延时或其他重任务。

## UART9 调试输出

UART9 当前用于摄像头和检测流程调试，常见输出包括：

- `DET class=... x=... y=...`
- `FRAME_OK NO_DET`
- `DET_ERR init`
- `DET_ERR run`
- `DET_ERR ipc_send`
- `DET_IPC_SENT count=...`

整帧图像发送函数仍保留在 `camera_stream.c`，但当前主流程不会调用它。

## 主要源码位置

| 功能 | 文件 |
| --- | --- |
| 摄像头采集与主循环 | `vision_CPU0/src/hardware/camera_stream.c` |
| NPU 检测与后处理 | `vision_CPU0/src/hardware/app_detection.c` |
| IPC 发送 | `vision_CPU0/src/hardware/ipc_detection_tx.c` |
| LVGL 页面逻辑 | `vision_CPU0/src/hardware/fruit_ui.c` |
| LCD 驱动 | `vision_CPU0/src/hardware/lcd_spi.c` |
| 触摸映射 | `vision_CPU0/src/hardware/lv_port_indev.c` |
| CPU1 IPC 接收 | `vision_CPU1/src/Can_Thread_entry.c` |
| CPU1 检测结果接口 | `vision_CPU1/src/ipc_detection_rx.h` |
| 机械臂运动学 | `vision_CPU1/src/hardware/jiesuan.c` |

## 当前注意事项

- CPU1 已能完整接收并保存检测结果，但当前 CAN 线程尚未消费 `ipc_detection_take_results()` 返回的数据，也不会自动驱动电机。
- 摄像头像素坐标必须经过标定转换为机械臂坐标后，才能送入逆运动学和 CAN 控制。
- `RunModel(true)` 使用同步 NPU 调用；若 NPU 中断丢失或硬件异常，Camera 线程仍可能停在推理内部。
- 摄像头或 NPU 初始化失败后，Camera 线程会进入低频空闲循环，当前不会自动重试初始化。
- CPU0 的 IPC 发送成功仅表示数据写入 FIFO，不代表 CPU1 后续任务已经消费该结果；当前协议尚未实现 ACK。

## 构建与联调建议

1. 在 e² studio 中同时导入 `vision`、`vision_CPU0` 和 `vision_CPU1` 工程。
2. 先构建 CPU0 和 CPU1，确认生成最新 ELF。
3. 使用固定测试包验证 CPU1 状态机，例如：

```text
count = 2
class 0, x=100, y=80
class 1, x=220, y=120
```

4. 分别测试 0、1、3 个结果以及非法的大于 3 个结果。
5. 测试不完整数据包后重新发送 `IPC_DETECTION_BEGIN`，确认 CPU1 能恢复同步。
6. 最后接入真实摄像头，核对屏幕水果类型、IPC 类别编号和坐标数据。

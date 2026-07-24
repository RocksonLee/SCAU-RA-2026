# CPU0 视觉识别与 IPC 联调记录

日期：2026-07-24

## 1. 当前目标

系统上电后由 CPU0 启动 CPU1，初始化 OV5640、NPU 和 IPC。摄像头等待自动曝光并丢弃前几帧后，CPU0 持续采集图像并进行目标检测。首次检测到有效目标时，CPU0 将这一帧内全部目标的类别及中心坐标通过 IPC 一次性发送给 CPU1，发送成功后不再推理和发送。

当前“首个有效识别帧”的定义是：

- 摄像头捕获成功；
- 模型推理流程完成；
- `result_count > 0`。

如果某一帧没有识别到目标，CPU0 会继续捕获和推理下一帧。

## 2. 今晚完成的修改

### 2.1 模型和 NPU

- 将模型生成代码加入 `vision_CPU0/src/models/`。
- 将 `src/models` 加入 CPU0 的 C Compiler Include 路径。
- 工程已包含 `rm_ethosu`、Ethos-U 驱动、CMSIS-NN 和模型源文件。
- CPU0 工程已成功完成过一次完整编译并生成 `Debug/vision_CPU0.elf`。

### 2.2 线程调整

- 将原 `Uart9_thread` 重命名为 `Camera_thread`。
- 入口文件改为 `vision_CPU0/src/Camera_thread_entry.c`。
- 生成线程改为 `vision_CPU0/ra_gen/Camera_thread.c`。
- `Camera_thread_entry()` 当前只执行：

```text
启动 CPU1
  → 打开 IPC0
  → 调用 camera_stream_task()
```

- 删除旧 K230 串口坐标接收及旧的 `IPC_COORDINATE_HEADER + X + Y` 发送逻辑。
- 删除旧 `drv_uart.c/.h`，避免 UART9 被两套回调和阻塞等待函数同时管理。

### 2.3 识别结果精简

修改：

- `vision_CPU0/src/hardware/app_detection.c`
- `vision_CPU0/src/hardware/app_detection.h`

对外结果只保留：

```c
typedef struct st_app_detection_result
{
    uint32_t class_id;
    int32_t  x;
    int32_t  y;
} app_detection_result_t;
```

其中 `x、y` 是检测框在 320×240 摄像头画面中的中心坐标。

置信度和边界框仍保留在 `app_detection.c` 内部，用于置信度过滤和 NMS，不通过 IPC 发送。

识别输出上限为 16 个目标：

```c
#define APP_DETECTION_MAX_RESULTS (16U)
```

### 2.4 摄像头识别流程

修改 `vision_CPU0/src/hardware/camera_stream.c` 后，当前执行流程为：

```text
打开 UART9 调试输出
  → 初始化 OV5640
  → 初始化 NPU
  → 输出摄像头寄存器诊断
  → 等待自动曝光 1500 ms
  → 丢弃 5 次捕获
  → 循环捕获图像
  → 图像预处理与模型推理
  → 没有目标则继续下一帧
  → 有目标则发送完整 IPC 数据包
  → 发送成功后进入低频空闲循环
```

图像预处理过程为：

```text
320×240 RGB565
  → 裁剪中央 240×240 区域
  → 缩放为 192×192
  → 转换为 CHW float RGB
  → 输入模型
```

### 2.5 精简 IPC 协议

CPU0 和 CPU1 已各自加入内容一致的：

```text
vision_CPU0/src/ipc_detection_protocol.h
vision_CPU1/src/ipc_detection_protocol.h
```

协议常量：

```c
#define IPC_DETECTION_BEGIN             (0x44455442U) /* "DETB" */
#define IPC_DETECTION_END               (0x44455445U) /* "DETE" */
#define IPC_DETECTION_MAX_RESULTS       (16U)
#define IPC_DETECTION_WORDS_PER_RESULT  (3U)
```

发送顺序：

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

最多发送：

```text
1 + 1 + 16×3 + 1 = 51 个 32 位 IPC 消息
```

### 2.6 CPU0 IPC 发送模块

新增：

```text
vision_CPU0/src/hardware/ipc_detection_tx.c
vision_CPU0/src/hardware/ipc_detection_tx.h
```

发送接口：

```c
bool ipc_detection_send_results(app_detection_result_t const * p_results,
                                uint32_t                         result_count);
```

完整发送到 `IPC_DETECTION_END` 后才返回 `true`。`camera_stream.c` 只有在返回 `true` 后才设置：

```c
detection_sent = true;
```

因此正常情况下 CPU0 只发送一批识别结果。

### 2.7 IPC 永久阻塞优化

IPC 整包发送增加 100 ms 总超时：

```c
#define IPC_DETECTION_SEND_TIMEOUT_MS (100U)
```

如果 CPU1 没有排空 FIFO，CPU0 最多等待约 100 ms，然后发送函数返回 `false`。Camera 线程随后继续采集并允许后续重试，不会永久停在 IPC FIFO 满循环中。

由于超时可能发生在数据包中间，CPU1 接收状态机必须满足：

> 无论当前处于什么接收状态，只要收到新的 `IPC_DETECTION_BEGIN`，立即丢弃上一包未完成数据并重新开始接收。

### 2.8 UART9 当前用途

UART9 不再接收 K230 坐标，目前只由 `camera_stream.c` 用作调试输出：

- OV5640 寄存器诊断；
- `DET class=... x=... y=...`；
- `DET_ERR init`；
- `DET_ERR run`；
- `DET_ERR ipc_send`；
- `DET_IPC_SENT count=...`。

整帧图像发送函数仍保留在 `camera_stream.c`，但当前主流程没有调用，不会通过 UART9 发送完整图片。

## 3. CPU0 最终检查结论

### 已确认

- `Camera_thread.c` 调用 `Camera_thread_entry()`，函数名称一致。
- `Camera_thread_entry()` 正确启动 CPU1、打开 IPC0 并调用 `camera_stream_task()`。
- 摄像头捕获有 1000 ms 超时，不会永久等待 CEU 帧结束。
- I2C 操作有 100 ms 超时。
- UART 调试发送有 1000 ms 超时。
- IPC 整包发送有 100 ms 超时。
- 检测候选上限为 64，输出和 IPC 上限均为 16。
- CPU0/CPU1 两份 IPC 协议头内容一致。
- Camera 线程栈大小为 4096 字节，模型大数组为全局数组，不直接占用线程栈。
- 新线程、识别、IPC 和模型源文件均已进入 Debug 构建清单。

### 当前仍存在的风险

#### NPU 可能永久等待

`RunModel(true)` 最终调用同步 `ethosu_invoke_v3()`。当前 Ethos-U 驱动的推理等待配置为无限等待。如果 NPU 中断丢失或硬件异常，Camera 线程可能停在模型推理内部。

#### 模型执行失败没有传递给上层

生成的 `model.c` 中 `RunModel()` 返回 `void`，并忽略 `sub_0001_invoke()` 和 `sub_0003_invoke()` 的返回值。因此 NPU 返回执行失败时，`app_detection_run_frame()` 仍可能解析无效或旧输出。

建议后续将 `RunModel()` 改成返回状态，任一 NPU 阶段失败时让 `app_detection_run_frame()` 返回 `false`。

#### 初始化失败后不会自动恢复

摄像头或 NPU 初始化失败后，Camera 线程会进入带 `vTaskDelay()` 的永久空闲循环。这不会锁死整个 CPU0，但视觉功能不会自动重试恢复。

#### 无目标时会持续推理

当前置信度阈值为 `0.75`。如果现场没有目标、阈值过高或图像颜色顺序不正确，CPU0 会持续捕获和推理。这属于当前设计，但外部看起来可能像没有响应。

#### 尚无 CPU1 ACK

CPU0 的“发送成功”表示所有数据已写入 IPC FIFO，不表示 CPU1 已完成解析和保存。需要严格确认接收成功时，可在后续增加 CPU1 ACK。

## 4. CPU1 当前状态

CPU1 当前 `vision_CPU1/src/Can_Thread_entry.c` 仍解析旧协议：

```text
0x434F4F52 + float X + float Y
```

它不认识新的：

```text
DETB + count + N组(class_id, x, y) + DETE
```

因此当前 CPU0 可以发送新协议，但 CPU1 只会从 FIFO 读走并忽略这些数据，不会写入目标数组。

## 5. CPU1 必须增加的内容

### 5.1 使用新协议头

在 CPU1 IPC 接收模块中加入：

```c
#include "ipc_detection_protocol.h"
```

删除旧的：

```c
#define IPC_COORDINATE_HEADER (0x434F4F52U)
```

### 5.2 建立结果数组

建议先写入暂存数组：

```c
static uint32_t s_class_id[IPC_DETECTION_MAX_RESULTS];
static int32_t  s_x[IPC_DETECTION_MAX_RESULTS];
static int32_t  s_y[IPC_DETECTION_MAX_RESULTS];
```

对后续选择逻辑公开一组已完成数据：

```c
uint32_t g_detection_class_id[IPC_DETECTION_MAX_RESULTS];
int32_t  g_detection_x[IPC_DETECTION_MAX_RESULTS];
int32_t  g_detection_y[IPC_DETECTION_MAX_RESULTS];

volatile uint32_t g_detection_count;
volatile bool     g_detection_data_ready;
```

### 5.3 替换 IPC 接收状态机

建议状态：

```c
typedef enum e_ipc_detection_rx_state
{
    IPC_RX_WAIT_BEGIN = 0,
    IPC_RX_READ_COUNT,
    IPC_RX_READ_CLASS,
    IPC_RX_READ_X,
    IPC_RX_READ_Y,
    IPC_RX_WAIT_END
} ipc_detection_rx_state_t;
```

接收规则：

1. 任意状态收到 `IPC_DETECTION_BEGIN` 都重新开始。
2. 读取 `result_count`。
3. 检查 `result_count <= IPC_DETECTION_MAX_RESULTS`。
4. 按顺序读取每个目标的 `class_id、x、y`。
5. 收满指定数量后等待 `IPC_DETECTION_END`。
6. 只有收到正确帧尾才发布结果。
7. 数据数量、顺序或帧尾异常时丢弃当前数据包并回到 `WAIT_BEGIN`。

### 5.4 完整接收后再发布

在 IPC 回调中先写暂存数组。收到正确帧尾后，再在临界区内复制或切换缓冲区，最后执行：

```c
g_detection_count      = received_count;
g_detection_data_ready = true;
```

必须最后设置 `g_detection_data_ready`，避免任务读取到只接收了一半的数据。

### 5.5 不要在 IPC 中断中执行重任务

`ipc0_callback()` 只负责：

- 状态机解析；
- 写入暂存数组；
- 设置完成标志。

不要在回调中执行：

- 逆运动学；
- CAN 电机控制；
- 延时；
- 动态内存分配；
- UART 阻塞输出。

### 5.6 调整 CAN 线程

当前 `Can_Thread_entry()` 收到旧坐标后会立即执行逆运动学并驱动电机。新需求是先保存所有识别结果供后续选择，因此应先关闭旧的自动控制路径。

后续任务应：

1. 检查 `g_detection_data_ready`。
2. 在临界区复制数量和数组快照。
3. 清除 ready 标志。
4. 根据 `class_id`、坐标或策略选择目标。
5. 将选中的目标中心坐标转换为机械臂坐标。
6. 再执行逆运动学和 CAN 控制。

### 5.7 可选 READY/ACK

为了进一步提高可靠性，可以增加：

- CPU1 打开 IPC 后发送 `CPU1_READY`；
- CPU0 等待 READY 后再允许发送检测结果；
- CPU1 完整保存一包后发送 ACK；
- CPU0 收到 ACK 后才真正结束 Camera 任务。

第一阶段可以先不加入 ACK，但必须完成新协议状态机和结果数组。

## 6. 推荐联调顺序

1. 再次在 e² studio 中完整 Build CPU0，确保最新的 IPC 100 ms 超时修改进入 ELF。
2. 暂时让 CPU0 使用固定测试结果发送一包：

```text
count = 2
class 0, x=100, y=80
class 1, x=220, y=120
```

3. 完成 CPU1 状态机并检查三个数组。
4. 测试不完整数据包后再次发送 `IPC_DETECTION_BEGIN`，确认 CPU1 能重新同步。
5. 测试 1 个、16 个和非法大于 16 个目标数量。
6. 验证 CPU1 只在收到 `IPC_DETECTION_END` 后设置 ready。
7. 最后连接真实摄像头和模型输出。
8. 确认类别 ID 与实际目标类别映射一致。

## 7. 本次最终验证说明

- CPU0 工程此前已在 e² studio 中完整编译成功。
- 最新的 `ipc_detection_tx.c` 已使用当前 LLVM ARM 编译器单独编译通过，只有 CMSIS 头文件原有的两个 signedness 警告。
- 命令行手工重新链接时缺少 e² studio 注入的 C/C++ 运行库搜索环境，因此未在命令行重新生成完整 ELF。
- 请在 e² studio 中再执行一次 Build Project，将最新 IPC 超时对象正式链接进最终固件。


/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "drv_uart.h"
#include <stdbool.h>
#include <string.h>

/**********************************************************************************************************************
 * Macro definitions	
 **********************************************************************************************************************/


/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/


/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/


/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static volatile int g_uart9_tx_complete = 0;
static volatile int g_uart9_rx_complete = 0;
// ================== 新增：用于存储K230坐标的全局变量 ==================
volatile float g_k230_pos_x = 0.0f;
volatile float g_k230_pos_y = 0.0f;
volatile bool  g_k230_data_ready = false; // 标志位：告诉LVGL有新数据了

// 串口接收缓冲区 (假设一帧数据不超过 64 字节)
#define RX9_BUF_SIZE 64
static char rx9_buffer[RX9_BUF_SIZE];
static uint16_t rx9_index = 0;
/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

fsp_err_t drv_uart_init(void)
{
    fsp_err_t err;

    /* 打开串口 */
    err = g_uart9.p_api->open(g_uart9.p_ctrl, g_uart9.p_cfg);
    if(FSP_SUCCESS != err) __BKPT();

    err = g_uart9.p_api->callbackSet(g_uart9.p_ctrl, uart9_callback, NULL, NULL);
    if(FSP_SUCCESS != err) __BKPT();

    return err;
}

fsp_err_t drv_uart_test(uint8_t *p_msg)
{
    fsp_err_t err;
    uint8_t msg_len = 0;
    char *p_temp_ptr = (char *)p_msg;

    /* 计算长度 */
    msg_len = ((uint8_t)(strlen((char *)p_temp_ptr)));

    /* 启动发送 */
    err = g_uart9.p_api->write(g_uart9.p_ctrl, p_msg, msg_len);
    /* 等待发送完毕 */
    drv_uart9_wait_for_tx();

    return err;
}

void drv_uart9_wait_for_tx(void)
{
    while (!g_uart9_tx_complete); // 阻塞等待
    g_uart9_tx_complete = 0;
}

void drv_uart9_wait_for_rx(void)
{
    while (!g_uart9_rx_complete); // 阻塞等待
    g_uart9_rx_complete = 0;
}

fsp_err_t drv_uart9_send(uint8_t *p_msg, uint32_t msg_len)
{
    fsp_err_t err;
    /* 启动发送 */
    err = g_uart9.p_api->write(g_uart9.p_ctrl, p_msg, msg_len);
    if(FSP_SUCCESS == err) {
        /* 等待发送完毕 */
        drv_uart9_wait_for_tx();
    }
    return err;
}
void uart9_callback(uart_callback_args_t * p_args)
{
	switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
        {
            g_uart9_tx_complete  = 1;
            break;
        }
        
        // 关键新增：单字节接收中断（收到一个字节就进一次）
        case UART_EVENT_RX_CHAR:
        {
            char c = (char)p_args->data;

            // 如果遇到换行符 '\n' 或回车符 '\r'，说明一帧数据接收完毕
            if (c == '\n' || c == '\r') 
            {
                rx9_buffer[rx9_index] = '\0'; // 字符串结尾添加结束符

                if (rx9_index > 0) 
                {
                    // 使用 sscanf 解析字符串 "X,Y" 格式，例如 "120.5,80.0"
                    float temp_x = 0.0f, temp_y = 0.0f;
                    if (sscanf(rx9_buffer, "%f,%f", &temp_x, &temp_y) == 2) 
                    {
                        // 解析成功，存入全局变量
                        g_k230_pos_x = temp_x;
                        g_k230_pos_y = temp_y;
                        g_k230_data_ready = true; // 举起标志位，通知 LVGL 刷新
                    }
                }
                rx9_index = 0; // 清零索引，准备接收下一帧
            }
            else
            {
                // 如果没遇到换行符，且缓冲区没满，就把字符存起来
                if (rx9_index < (RX9_BUF_SIZE - 1)) 
                {
                    rx9_buffer[rx9_index++] = c;
                } 
                else 
                {
                    rx9_index = 0; // 防止溢出，丢弃错误数据
                }
            }
            break;
        }
        
        case UART_EVENT_RX_COMPLETE:
        {
            g_uart9_rx_complete = 1;
            break;
        }
        default:
        {
            break;
        }
    }
}
/*printf输出重定向到串口*/
/* 将 drv_uart.c 中的这个函数名修改掉 */
int fputc(int ch, FILE *f)  // 修改函数名和参数列表
{
    ((void)f); // 防止编译器警告
    fsp_err_t err = FSP_SUCCESS;

    // 调用 API 发送单个字符
    err = g_uart9.p_api->write(g_uart9.p_ctrl, (uint8_t*)&ch, 1);

    if(FSP_SUCCESS != err) __BKPT();
    
    // 等待发送完成回调
    drv_uart9_wait_for_tx();

    return ch;
}


/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

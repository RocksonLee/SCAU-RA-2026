#include "canfd0.h"

#include "FreeRTOS.h"
#include "task.h"

#define CANFD_INTERFRAME_DELAY_MS    (5U)
#define CANFD_TX_TIMEOUT_MS          (20U)
#define CANFD_TX_POLL_MS             (1U)
#define CANFD_MOTOR_ADDRESS_MAX      (6U)
#define CANFD_COMMAND_COMPLETE       (0x9FU)
#define CANFD_CHECK_BYTE             (0x6BU)

#define uint8_t unsigned char
can_frame_t canfd0_rx_frame;
can_frame_t canfd0_tx_frame;
static volatile uint32_t g_claw_completion_sequence;

void CANFD0_Init(void)
{
	fsp_err_t err = R_CANFD_Open(&g_canfd0_ctrl,&g_canfd0_cfg);
	assert(err== FSP_SUCCESS);
}
/* CANFD Channel 0 Acceptance Filter List (AFL) rule array */
const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
		/* TODO: Add AFL Entries here (see Developer Assistance -> [thread] -> g_canfd0 -> AFL Entry) */ 
		{
			/* TODO: Edit the settings below to filter for specific messages. */

			.id =
			{
					/* Specify the ID, ID type and frame type to accept. */
					.id         = CANFD_FILTER_ID,
					.frame_type = CAN_FRAME_TYPE_DATA,
					.id_mode    = CAN_ID_MODE_EXTENDED
			},

			.mask =
			{
					/* These values mask which ID/mode bits to compare when filtering messages. */
					.mask_id         = MASK_ID,
					.mask_frame_type = 1,
					.mask_id_mode    = MASK_ID_MODE
			},

			.destination =
			{
					/* If DLC checking is enabled any messages shorter than the below setting will be rejected. */
					.minimum_dlc = CANFD_MINIMUM_DLC_3,

					/* Optionally specify a Receive Message Buffer (RX MB) to store accepted frames. RX MBs do not have an
					 * interrupt or overwrite protection and must be checked with R_CANFD_InfoGet and R_CANFD_Read. */
					.rx_buffer   = CANFD_RX_MB_0,

					/* Specify which FIFO(s) to send filtered messages to. Multiple FIFOs can be OR'd together. */
					.fifo_select_flags = (canfd_rx_fifo_t)CANFD_RX_FIFO_0
			}
		},
};

volatile bool canfd0_rx_complete_flag =false ;
volatile bool canfd0_tx_complete_flag =false ;
volatile bool canfd0_err_status_flag =false ;
volatile can_event_t canfd0_error_status =(can_event_t)0;

static void canfd0_delay_ms(uint32_t delay_ms)
{
	vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

/*
 * All commands share TX mailbox 0. A fixed delay cannot guarantee that the
 * previous request has left the mailbox. If arbitration or a retry keeps it
 * busy, R_CANFD_Write returns FSP_ERR_CAN_TRANSMIT_NOT_READY; the old
 * assert-only handling silently discarded that frame in release builds.
 *
 * Do not make command progress depend on the TX-complete callback. Some board
 * configurations can report that callback later than the former 20 ms wait,
 * even though the driver has already accepted the frame. Returning success as
 * soon as R_CANFD_Write accepts the frame preserves the motor protocol timing;
 * the next frame still waits here while mailbox 0 remains busy.
 */
static bool canfd0_send_current_frame(void)
{
	TickType_t const start_tick = xTaskGetTickCount();
	TickType_t const timeout_ticks = pdMS_TO_TICKS(CANFD_TX_TIMEOUT_MS);
	fsp_err_t err;

	do
	{
		err = R_CANFD_Write(&g_canfd0_ctrl,
		                    CAN_MAILBOX_NUMBER_0,
		                    &canfd0_tx_frame);
		if (FSP_SUCCESS == err)
		{
			return true;
		}

		if (FSP_ERR_CAN_TRANSMIT_NOT_READY != err)
		{
			return false;
		}

		vTaskDelay(pdMS_TO_TICKS(CANFD_TX_POLL_MS));
	} while ((xTaskGetTickCount() - start_tick) < timeout_ticks);

	return false;
}

void canfd0_callback(can_callback_args_t *p_args)
{
	switch(p_args->event)
	{
		case CAN_EVENT_RX_COMPLETE:	
		{
			can_frame_t const * p_frame = &(p_args->frame);
			uint32_t const motor_address = (p_frame->id >> 8U) & 0xFFU;

			canfd0_rx_complete_flag=true;
			memcpy(&canfd0_rx_frame, p_frame, sizeof(can_frame_t));

			if ((motor_address >= 1U) &&
			    (motor_address <= CANFD_MOTOR_ADDRESS_MAX))
			{
				/* The gripper manual uses C6 for the open/close command, while
				 * its Response description names F5 for the completion frame.
				 * Accept both documented variants and either CAN packet index. */
				if ((6U == motor_address) &&
				    (p_frame->data_length_code >= 3U) &&
				    ((0xC6U == p_frame->data[0]) || (0xF5U == p_frame->data[0])) &&
				    (CANFD_COMMAND_COMPLETE == p_frame->data[1]) &&
				    (CANFD_CHECK_BYTE == p_frame->data[2]))
				{
					g_claw_completion_sequence++;
				}
			}
			break;
		}
		case CAN_EVENT_TX_COMPLETE:
		{
			canfd0_tx_complete_flag=true;
			break;
		}
		case CAN_EVENT_ERR_WARNING:
	  case CAN_EVENT_ERR_PASSIVE:
		case CAN_EVENT_ERR_BUS_OFF:
		case CAN_EVENT_BUS_RECOVERY:
		case CAN_EVENT_MAILBOX_MESSAGE_LOST:
		case CAN_EVENT_ERR_BUS_LOCK:
	  case CAN_EVENT_ERR_CHANNEL:  
		case CAN_EVENT_TX_ABORTED:
		case CAN_EVENT_ERR_GLOBAL:
		case CAN_EVENT_TX_FIFO_EMPTY:
		case CAN_EVENT_FIFO_MESSAGE_LOST:
		{
			canfd0_err_status_flag=true;
			canfd0_error_status =(can_event_t)p_args->error;
			break;
		}
		default: break;
	}
}

uint32_t CANFD0_Claw_Completion_Snapshot(void)
{
	return g_claw_completion_sequence;
}

bool CANFD0_Wait_Claw_Complete(uint32_t completion_snapshot, uint32_t timeout_ms)
{
	TickType_t const start_tick = xTaskGetTickCount();

	while (completion_snapshot == g_claw_completion_sequence)
	{
		if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(timeout_ms))
		{
			return false;
		}

		vTaskDelay(pdMS_TO_TICKS(CANFD_TX_POLL_MS));
	}

	return true;
}
//这个是测试用的
uint32_t int32_abs(int32_t num) {
    if (num >= 0) {
        // 正数直接转换为无符号类型
        return (uint32_t)num;
    } else if (num == INT32_MIN) {
        // 单独处理最小值：-2147483648的绝对值是2147483648（刚好在uint32_t范围内）
        return (uint32_t)2147483648U;
    } else {
        // 普通负数先取反再转换
        return (uint32_t)-num;
    }
}
bool CANFD0_Operation_1(int32_t angle)
{
	uint8_t dir = (angle < 0) ? 0x01 : 0x00;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH1))*16;
	canfd0_tx_frame.id=CAN_ID_A;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.options=0;
	//梯形曲线加减速位置模式控制
		canfd0_tx_frame.data[0]=0xFD;
	//方向
		canfd0_tx_frame.data[1]=dir;
	//加速加速度0x04B0(600)
		canfd0_tx_frame.data[2]=0x02;
		canfd0_tx_frame.data[3]=0x58;
	//减速加速度
		canfd0_tx_frame.data[4]=0x02;
		canfd0_tx_frame.data[5]=0x58;
	//速度(10000)
		canfd0_tx_frame.data[6]=0x27;
		canfd0_tx_frame.data[7]=0x10;
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	
	//梯形曲线加减速位置模式控制packet包
	canfd0_tx_frame.id=CAN_ID_AP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.data[0]=0xFD;
	//角度
	canfd0_tx_frame.data[1]=(angle_1 >> 24) & 0xFF;
	canfd0_tx_frame.data[2]=(angle_1 >> 16) & 0xFF;
	canfd0_tx_frame.data[3]=(angle_1 >> 8) & 0xFF;
	canfd0_tx_frame.data[4]=angle_1 & 0xFF;
	
	canfd0_tx_frame.data[5]=0x01;
	canfd0_tx_frame.data[6]=0x01;
	canfd0_tx_frame.data[7]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}

bool CANFD0_Operation_2(int32_t angle)
{
	uint8_t dir = (angle < 0) ? 0x00 : 0x01;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH2))*16;
	canfd0_tx_frame.id=CAN_ID_B;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.options=0;
	//梯形曲线加减速位置模式控制
		canfd0_tx_frame.data[0]=0xFD;
	//方向
		canfd0_tx_frame.data[1]=dir;
	//加速加速度0x04B0(600)
		canfd0_tx_frame.data[2]=0x02;
		canfd0_tx_frame.data[3]=0x58;
	//减速加速度
		canfd0_tx_frame.data[4]=0x02;
		canfd0_tx_frame.data[5]=0x58;
	//速度(10000)，与第一轴一致
		canfd0_tx_frame.data[6]=0x27;
		canfd0_tx_frame.data[7]=0x10;
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	
	//梯形曲线加减速位置模式控制packet包
	canfd0_tx_frame.id=CAN_ID_BP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.data[0]=0xFD;
	//角度
	canfd0_tx_frame.data[1]=(angle_1 >> 24) & 0xFF;
	canfd0_tx_frame.data[2]=(angle_1 >> 16) & 0xFF;
	canfd0_tx_frame.data[3]=(angle_1 >> 8) & 0xFF;
	canfd0_tx_frame.data[4]=angle_1 & 0xFF;
	
	canfd0_tx_frame.data[5]=0x01;
	canfd0_tx_frame.data[6]=0x01;
	canfd0_tx_frame.data[7]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}

bool CANFD0_Operation_3(int32_t angle)
{
	uint8_t dir = (angle < 0) ? 0x00 : 0x01;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH3))*16;
	canfd0_tx_frame.id=CAN_ID_C;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.options=0;
	//梯形曲线加减速位置模式控制
		canfd0_tx_frame.data[0]=0xFD;
	//方向
		canfd0_tx_frame.data[1]=dir;
	//加速加速度0x04B0(600)
		canfd0_tx_frame.data[2]=0x02;
		canfd0_tx_frame.data[3]=0x58;
	//减速加速度
		canfd0_tx_frame.data[4]=0x02;
		canfd0_tx_frame.data[5]=0x58;
	//速度(10000)
		canfd0_tx_frame.data[6]=0x27;
		canfd0_tx_frame.data[7]=0x10;
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	
	//梯形曲线加减速位置模式控制packet包
	canfd0_tx_frame.id=CAN_ID_CP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.data[0]=0xFD;
	//角度
	canfd0_tx_frame.data[1]=(angle_1 >> 24) & 0xFF;
	canfd0_tx_frame.data[2]=(angle_1 >> 16) & 0xFF;
	canfd0_tx_frame.data[3]=(angle_1 >> 8) & 0xFF;
	canfd0_tx_frame.data[4]=angle_1 & 0xFF;

	canfd0_tx_frame.data[5]=0x01;
	canfd0_tx_frame.data[6]=0x01;
	canfd0_tx_frame.data[7]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}

bool CANFD0_Operation_4(int32_t angle)
{
	uint8_t dir = (angle < 0) ? 0x01 : 0x00;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH4))*16;
	canfd0_tx_frame.id=CAN_ID_D;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.options=0;
	//梯形曲线加减速位置模式控制
		canfd0_tx_frame.data[0]=0xFD;
	//方向
		canfd0_tx_frame.data[1]=dir;
	//加速加速度0x04B0(600)
		canfd0_tx_frame.data[2]=0x02;
		canfd0_tx_frame.data[3]=0x58;
	//减速加速度
		canfd0_tx_frame.data[4]=0x02;
		canfd0_tx_frame.data[5]=0x58;
	//速度(10000)
		canfd0_tx_frame.data[6]=0x27;
		canfd0_tx_frame.data[7]=0x10;
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	
	//梯形曲线加减速位置模式控制packet包
	canfd0_tx_frame.id=CAN_ID_DP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.data[0]=0xFD;
	//角度
	canfd0_tx_frame.data[1]=(angle_1 >> 24) & 0xFF;
	canfd0_tx_frame.data[2]=(angle_1 >> 16) & 0xFF;
	canfd0_tx_frame.data[3]=(angle_1 >> 8) & 0xFF;
	canfd0_tx_frame.data[4]=angle_1 & 0xFF;
	
	canfd0_tx_frame.data[5]=0x01;
	canfd0_tx_frame.data[6]=0x01;
	canfd0_tx_frame.data[7]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}


bool CANFD0_Operation_5(int32_t angle)
{
	uint8_t dir = (angle < 0) ? 0x01 : 0x00;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH5))*16;
	canfd0_tx_frame.id=CAN_ID_E;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.options=0;
	//梯形曲线加减速位置模式控制
		canfd0_tx_frame.data[0]=0xFD;
	//方向
		canfd0_tx_frame.data[1]=dir;
	//加速加速度0x04B0(600)
		canfd0_tx_frame.data[2]=0x02;
		canfd0_tx_frame.data[3]=0x58;
	//减速加速度
		canfd0_tx_frame.data[4]=0x02;
		canfd0_tx_frame.data[5]=0x58;
	//速度(10000)
		canfd0_tx_frame.data[6]=0x27;
		canfd0_tx_frame.data[7]=0x10;
	
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	
	canfd0_tx_frame.id=CAN_ID_EP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	//梯形曲线加减速位置模式控制packet包
	canfd0_tx_frame.data[0]=0xFD;
	//角度
	canfd0_tx_frame.data[1]=(angle_1 >> 24) & 0xFF;
	canfd0_tx_frame.data[2]=(angle_1 >> 16) & 0xFF;
	canfd0_tx_frame.data[3]=(angle_1 >> 8) & 0xFF;
	canfd0_tx_frame.data[4]=angle_1 & 0xFF;
	
	canfd0_tx_frame.data[5]=0x01;
	canfd0_tx_frame.data[6]=0x01;
	canfd0_tx_frame.data[7]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}

bool run(void)
{
	canfd0_tx_frame.id=CAN_ID_0;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_P;
	canfd0_tx_frame.options=0;
	canfd0_tx_frame.data[0]=0xFF;
	canfd0_tx_frame.data[1]=0x66;
	canfd0_tx_frame.data[2]=0x6B;
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}

//
bool Claw_Control(void)
{
	canfd0_tx_frame.id=CAN_ID_F;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.options=0;
	canfd0_tx_frame.data[0]=0xC6;
	//方向00闭合 01张开
	canfd0_tx_frame.data[1]=0x00;
	//加速度200
	canfd0_tx_frame.data[2]=0x00;
	canfd0_tx_frame.data[3]=0xC8;
	//速度1000 0x03 0xE8
	canfd0_tx_frame.data[4]=0x03;
	canfd0_tx_frame.data[5]=0xE8;
	//同步标志
	canfd0_tx_frame.data[6]=0x00;
	//力矩电流限制500 (0x01F4 mA)
	canfd0_tx_frame.data[7]=0x01;
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	
	canfd0_tx_frame.id=CAN_ID_FP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_P;
	canfd0_tx_frame.data[0]=0xC6;
	//接上面的电流限制
	canfd0_tx_frame.data[1]=0xF4;
	//校验码
	canfd0_tx_frame.data[2]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}
bool Claw_Open(void)
{
	canfd0_tx_frame.id=CAN_ID_F;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE;
	canfd0_tx_frame.options=0;
	canfd0_tx_frame.data[0]=0xC6;
	//方向00闭合 01张开
	canfd0_tx_frame.data[1]=0x01;
	//加速度200
	canfd0_tx_frame.data[2]=0x00;
	canfd0_tx_frame.data[3]=0xC8;
	//速度1000 0x03 0xE8
	canfd0_tx_frame.data[4]=0x03;
	canfd0_tx_frame.data[5]=0xE8;
	//同步标志
	canfd0_tx_frame.data[6]=0x00;
	//力矩电流限制1000
	canfd0_tx_frame.data[7]=0x03;
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	
	canfd0_tx_frame.id=CAN_ID_FP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_P;
	canfd0_tx_frame.data[0]=0xC6;
	//接上面的电流限制
	canfd0_tx_frame.data[1]=0xE8;
	//校验码
	canfd0_tx_frame.data[2]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}
//读取夹爪状态
bool Read_Claw(void)
{
	canfd0_tx_frame.id=CAN_ID_F;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_CLAW;
	canfd0_tx_frame.options=0;
	canfd0_tx_frame.data[0]=0x17;
	canfd0_tx_frame.data[1]=0x6B;
	
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	if (!canfd0_send_current_frame())
	{
		return false;
	}
	canfd0_delay_ms(CANFD_INTERFRAME_DELAY_MS);
	return true;
}


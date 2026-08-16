#include "canfd0.h" 
#define uint8_t unsigned char
can_frame_t canfd0_rx_frame;
can_frame_t canfd0_tx_frame;
volatile uint8_t canfd0_rx_cnt = 0;          // 已接收的返回帧计数
can_frame_t canfd0_rx_frames[MAX_MOTOR_CNT];  // 缓存每个电机的返回帧
//can_frame_t canfd0_rx_frames[MAX_MOTOR_CNT];  // 缓存每个电机的返回帧

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
					.minimum_dlc = CANFD_MINIMUM_DLC_4,

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

void canfd0_callback(can_callback_args_t *p_args)
{
	switch(p_args->event)
	{
		case CAN_EVENT_RX_COMPLETE:	
		{
			canfd0_rx_complete_flag=true;
//			memcpy(&canfd0_rx_frame,&(p_args->frame),sizeof(can_frame_t));
			if(canfd0_rx_cnt < MAX_MOTOR_CNT)
			{
					memcpy(&canfd0_rx_frames[canfd0_rx_cnt], &(p_args->frame), sizeof(can_frame_t));
					canfd0_rx_cnt++;  // 接收计数+1
				  //只有夹爪会返回两个包，第一个8字节
					//另一个5字节
					//电流在第一个电机
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
void CANFD0_Operation_1(int32_t angle) 
{
	uint8_t dir = (angle < 0) ? 0x01 : 0x00;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH1))*16;
	fsp_err_t err;
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
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	
	//梯形曲线加减速位置模式控制packet包
	uint32_t time_out =0xFFFF;
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
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	
}

void CANFD0_Operation_2(int32_t angle) 
{
	uint8_t dir = (angle < 0) ? 0x00 : 0x01;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH2))*16;
	fsp_err_t err;
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
	//速度(5000)
		canfd0_tx_frame.data[6]=0x13;
		canfd0_tx_frame.data[7]=0x88;
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	
	//梯形曲线加减速位置模式控制packet包
	uint32_t time_out =0xFFFF;
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
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	
}

void CANFD0_Operation_3(int32_t angle) 
{
	uint8_t dir = (angle < 0) ? 0x00 : 0x01;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH3))*16;
	fsp_err_t err;
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
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	
	//梯形曲线加减速位置模式控制packet包
	uint32_t time_out =0xFFFF;
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
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	
}

void CANFD0_Operation_4(int32_t angle) 
{
	uint8_t dir = (angle < 0) ? 0x01 : 0x00;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH4))*16;
	fsp_err_t err;
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
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	
	//梯形曲线加减速位置模式控制packet包
	uint32_t time_out =0xFFFF;
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
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	
}


void CANFD0_Operation_5(int32_t angle) 
{
	uint8_t dir = (angle < 0) ? 0x01 : 0x00;
	int32_t angle_1=(int32_t)(round(int32_abs(angle)/1.8*DH5))*16;
	fsp_err_t err;
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
	
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	
	uint32_t time_out =0xFFFF;
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
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	
}

void run(void)
{
	fsp_err_t err;
	canfd0_tx_frame.id=CAN_ID_0;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_P;
	canfd0_tx_frame.options=0;
	canfd0_tx_frame.data[0]=0xFF;
	canfd0_tx_frame.data[1]=0x66;
	canfd0_tx_frame.data[2]=0x6B;
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	
}

//
void Claw_Control()
{
	fsp_err_t err;
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
	//力矩电流限制200
	canfd0_tx_frame.data[7]=0x00;
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	
	uint32_t time_out =0xFFFF;
	canfd0_tx_frame.id=CAN_ID_FP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_P;
	canfd0_tx_frame.data[0]=0xC6;
	//接上面的电流限制
	canfd0_tx_frame.data[1]=0xC8;
	//校验码
	canfd0_tx_frame.data[2]=0x6B;
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
}
void Claw_Open()
{
	fsp_err_t err;
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
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	
	uint32_t time_out =0xFFFF;
	canfd0_tx_frame.id=CAN_ID_FP;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_P;
	canfd0_tx_frame.data[0]=0xC6;
	//接上面的电流限制
	canfd0_tx_frame.data[1]=0xE8;
	//校验码
	canfd0_tx_frame.data[2]=0x6B;
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
}
//读取夹爪状态
void Read_Claw()
{
	uint32_t time_out =0xFFFF;
	fsp_err_t err;
	canfd0_tx_frame.id=CAN_ID_F;
	canfd0_tx_frame.id_mode=CAN_ID_MODE_EXTENDED;
	canfd0_tx_frame.type=CAN_FRAME_TYPE_DATA;
	canfd0_tx_frame.data_length_code=CAN_DATA_LENGTH_CODE_CLAW;
	canfd0_tx_frame.options=0;
	canfd0_tx_frame.data[0]=0x17;
	canfd0_tx_frame.data[1]=0x6B;
	
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	err=R_CANFD_Write(&g_canfd0_ctrl,CAN_MAILBOX_NUMBER_0,&canfd0_tx_frame);
	assert(err==FSP_SUCCESS);
	while((true!=canfd0_tx_complete_flag)&& (--time_out));
	canfd0_tx_complete_flag=false;
	if(time_out==0)
	{
		return;
	}
	R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
	
}


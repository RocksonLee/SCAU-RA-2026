#ifndef __CANFD0_H__
#define __CANFD0_H__

#include "hal_data.h"

void CANFD0_Init(void);
void CANFD0_Operation(void);
void CANFD0_Operation_P(void);
bool CANFD0_Operation_1(int32_t angle);
bool CANFD0_Operation_2(int32_t angle);
bool CANFD0_Operation_3(int32_t angle);
bool CANFD0_Operation_4(int32_t angle);
bool CANFD0_Operation_5(int32_t angle);
bool Claw_Control(void);
bool run(void);
bool Read_Claw(void);
bool Claw_Open(void);
uint32_t CANFD0_Claw_Completion_Snapshot(void);
bool CANFD0_Wait_Claw_Complete(uint32_t completion_snapshot, uint32_t timeout_ms);


#define MAX_MOTOR_CNT 5  // 电机数量



#define CAN_ID   0x0100 //地址1的格式为0x0100
#define CAN_ID_0 0x0000
#define CAN_ID_1 0x0001 //地址1的packet包格式为0x0101，以此类推
#define CAN_ID_2 0x0002 
#define CAN_ID_3 0x0003 
#define CAN_ID_4 0x0004
#define CAN_ID_5 0x0005
#define CAN_ID_6 0x0006
#define CAN_ID_7 0x0007
#define CAN_ID_8 0x0008
#define CAN_ID_9 0x0009
#define CAN_ID_10 0x0010
#define CAN_ID_11 0x0011
#define CAN_ID_12 0x0012
#define CAN_ID_13 0x0013
#define CAN_ID_14 0x0014

#define CAN_ID_P 0X0001
#define CAN_ID_A 0x0100  
#define CAN_ID_AP 0x0101 
#define CAN_ID_B 0x0200  
#define CAN_ID_BP 0x0201 
#define CAN_ID_C 0x0300  
#define CAN_ID_CP 0x0301 
#define CAN_ID_D 0x0400  
#define CAN_ID_DP 0x0401 
#define CAN_ID_E 0x0500  
#define CAN_ID_EP 0x0501 
#define CAN_ID_F 0x0600  
#define CAN_ID_FP 0x0601 


#define CAN_DATA_LENGTH_CODE 8 //单个packet包发送最长只能是8个字节，虽然板载是canfd但电机只支持can通信，所以特别shit
#define CAN_DATA_LENGTH_CODE_P 3
#define CAN_DATA_LENGTH_CODE_CLAW 2
#define CAN_MAILBOX_NUMBER_0	(CANFD_TX_MB_0)

//减速比，一下测试均为大致值，眼测这一块
#define DH1 150 //测试完毕
#define DH2 140  //测试完毕
#define DH3 75	//测试完毕
#define DH4 18  //测试完毕
#define DH5 9.3 //测试完毕

//公式：角度/1.8*dh*16

//如果是多电机指令的话下面的参数就不用改
#define CANFD_FILTER_ID 0x00000000 //例如地址1的话就是0x00000100
#define MASK_ID 				0x1FFFF000
#define MASK_ID_MODE 		(0)

#endif
//直通限速位置式控制
//		canfd0_tx_frame.data[0]=0xFB;
//		canfd0_tx_frame.data[1]=0x00;
//		canfd0_tx_frame.data[2]=0x4E;
//		canfd0_tx_frame.data[3]=0x20;
//		canfd0_tx_frame.data[4]=0x00;
//		canfd0_tx_frame.data[5]=0x00;
//		canfd0_tx_frame.data[6]=0x0D;
//		canfd0_tx_frame.data[7]=0x00;
//x固件packet包测试
//	canfd0_tx_frame.data[0]=0xFB;
//	canfd0_tx_frame.data[1]=0x02;
//	canfd0_tx_frame.data[2]=0x00;
//	canfd0_tx_frame.data[3]=0x6B;

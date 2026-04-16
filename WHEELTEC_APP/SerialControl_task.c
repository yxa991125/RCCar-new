/**
 * @file    SerialControl_task.c
 * @brief   串口协议解析任务。
 * @author  WHEELTEC
 * @date    2025-07-10
 * @version 1.0.0
 *
 * @details
 * - 串口中断收到数据后写入队列，本任务负责按协议解析控制帧。
 * - 速度控制协议：7B 00 00 VX_H VX_L VY_H VY_L VZ_H VZ_L BCC 7D
 * - 自动回充协议：7B 01 00 VX_H VX_L VY_H VY_L VZ_H VZ_L BCC 7D
 * - 灯带设置协议：7B 04 01 R G B 00 00 00 BCC 7D
 * - 速度单位为 mm/s。
 */

// FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// BSP Include File
#include "bsp_RGBLight.h"

// APP Include File
#include "robot_select_init.h"
#include "RobotControl_task.h"
#include "RGBStripControl_task.h"
#include "servo_basic_control.h"

// BCC 校验函数
uint8_t Calculate_BCC(const uint8_t* checkdata,uint16_t datalen)
{
	char bccval = 0;
	for(uint16_t i=0;i<datalen;i++)
	{
		bccval ^= checkdata[i];
	}
	return bccval;
}

static uint8_t SerialControl_IsZeroMotion(const RobotControlCMDType_t *cmd)
{
	return (cmd->Vx == 0.0f && cmd->Vy == 0.0f && cmd->Vz == 0.0f) ? 1U : 0U;
}

static void SerialControl_SendZeroRosCommand(void)
{
	RobotControlCMDType_t stop_cmd = {
		.cmdsource = ROS_CMD,
		0.0f, 0.0f, 0.0f
	};

	(void)WriteRobotControlQueue(&stop_cmd, 0);
}

void SerialControlTask(void* param)
{
	extern QueueHandle_t g_xQueueROSserial;

	uint8_t recv = 0;     // 用于接收队列数据
	uint8_t roscmdBuf[20];
	uint8_t roscmdCount = 0;
	const uint8_t cmdLen = 11; // 一帧通信数据长度
	static uint8_t s_rc_override_blocking = 0U;

	(void)param;

	while( 1 )
	{
		if( pdPASS == xQueueReceive(g_xQueueROSserial,&recv,portMAX_DELAY) )
		{
			// 支持复位进入 BootLoader。
			_System_Reset_FromAPP_RTOS(recv);

			// 支持设置调试等级。
			RobotControl_SetDebugLevel(recv);

			roscmdBuf[roscmdCount] = recv;
			if( recv == 0x7B || roscmdCount>0 ) roscmdCount++;
			else roscmdCount = 0;

			if( cmdLen==roscmdCount )
			{
				roscmdCount = 0;

				// 检查帧尾与校验位。
				if( roscmdBuf[cmdLen-1]==0x7D && roscmdBuf[cmdLen-2]==Calculate_BCC(roscmdBuf,cmdLen-2) )
				{
					RobotControlCMDType_t cmd = {
						.cmdsource = ROS_CMD,
						0,0,0
					};
					uint8_t flag_stop = 0U;
					uint8_t writeflag = 1U;
					uint8_t motion_is_zero = 0U;
					uint8_t rc_override_active = 0U;
					uint8_t allow_serial_motion = 1U;
					static uint8_t idleCount = 0U;

					// 速度命令数据解析。
					if( roscmdBuf[1]<3 )
					{
						cmd.Vx = (float)((short)(roscmdBuf[3]<<8 | roscmdBuf[4]))/1000.0f;
						cmd.Vy = (float)((short)(roscmdBuf[5]<<8 | roscmdBuf[6]))/1000.0f;
						cmd.Vz = (float)((short)(roscmdBuf[7]<<8 | roscmdBuf[8]))/1000.0f;
					}

					// V1 协议。
					if( roscmdBuf[1]==0 )
					{
						RobotControlParam.ChargeMode=0;
						RobotControlParam.SecurityLevel = roscmdBuf[2] & 0x01;
						RobotControlParam.softwareEnflag = (roscmdBuf[2] & 0x80) ? 1 : 0;
					}
					else if( roscmdBuf[1]==1||roscmdBuf[1]==2 )
					{
						RobotControlParam.ChargeMode=1;
					}
					else if( roscmdBuf[1]==4 )
					{
						if( roscmdBuf[2]==1 )
						{
							userdefine_rgb[0]=1;
							userdefine_rgb[1]=roscmdBuf[3];
							userdefine_rgb[2]=roscmdBuf[4];
							userdefine_rgb[3]=roscmdBuf[5];
						}
						else
						{
							userdefine_rgb[0]=0;
						}
					}

					// V2 协议，暂未启用。
//					if( roscmdBuf[1]==0 )
//					{
//						if( roscmdBuf[2]==0xB0 ) RobotControlParam.SecurityLevel=0;
//						else if( roscmdBuf[2]==0xB1 ) RobotControlParam.SecurityLevel=1;
//					}
//					else if( roscmdBuf[1]==1||roscmdBuf[1]==2 )
//					{
//						if( roscmdBuf[2]==0xA1 ) RobotControlParam.ChargeMode=1;
//						else if( roscmdBuf[2]==0xA0 ) RobotControlParam.ChargeMode=0;
//					}
//					else if( roscmdBuf[1]==4 )
//					{
//						if( roscmdBuf[2]==1 )
//							rgb->SetColorFade(roscmdBuf[3],roscmdBuf[4],roscmdBuf[5]);
//						else rgb->turnoff();
//					}

					// 持续空闲时不重复写入非零控制队列。
					if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
					else idleCount=0;
					if( idleCount>10 ) writeflag=0,idleCount=10;

					motion_is_zero = SerialControl_IsZeroMotion(&cmd);
					if( roscmdBuf[1]==0 )
					{
						flag_stop = (roscmdBuf[2] & 0x80) ? 1U : 0U;
					}

					rc_override_active = ServoBasic_IsRcOverrideActive();
					if( rc_override_active != 0U )
					{
						if( s_rc_override_blocking == 0U )
						{
							SerialControl_SendZeroRosCommand();
							s_rc_override_blocking = 1U;
						}
						allow_serial_motion = (motion_is_zero != 0U || flag_stop != 0U) ? 1U : 0U;
					}
					else
					{
						s_rc_override_blocking = 0U;
					}

					if( allow_serial_motion != 0U && (writeflag != 0U || motion_is_zero != 0U || flag_stop != 0U))
					{
						WriteRobotControlQueue(&cmd,0);
					}

					// Orin 运动学到 PWM 的路径。
					if( roscmdBuf[1]==0 && allow_serial_motion != 0U )
					{
						ServoBasic_UpdateFromOrin(cmd.Vx, cmd.Vy, cmd.Vz, flag_stop);
					}
				}
			}
		}
	}
}

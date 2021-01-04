#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include "ros/ros.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv , "control_motor");
	printf(">>this is first test!\r\n");//指示程序已运行

	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0x80000000;
	config.AccMask=0xFFFFFFFF;
	config.Filter=0;//接收所有帧
	config.Timing0=0x00;/*波特率1000 Kbps  0x00  0x14*/
	config.Timing1=0x14;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	//需要发送的帧，结构体设置

	usleep(100000);
	VCI_CAN_OBJ send[2];
	send[0].ID=0x10;
	send[0].SendType=1;
	send[0].DataLen=8;
	send[0].ExternFlag=0;
	send[0].RemoteFlag=0;
	send[1].ID=0x20;
	send[1].SendType=1;
	send[1].DataLen=8;
	send[1].ExternFlag=0;
	send[1].RemoteFlag=0;

	int i=0;
	for(i = 0; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = 0x55;
		send[1].Data[i] = 0x55;
	}


	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 2) == 2)
	{
		printf("reset succes!\n");
	}
	else
	{
		printf("reset fail!\n");
	        usleep(100000);//延时100ms。
		VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
		exit(1);
	}

	usleep(500000);
	send[0].ID=0x11;
	send[1].ID=0x21;

	send[0].Data[0] = 0x06;
	send[1].Data[0] = 0x06;
	for(i = 1; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = 0x55;
		send[1].Data[i] = 0x55;
	}


	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 2) == 2)
	{
		printf("set mode succes!\n");
	}
	else
	{
		printf("set mode fail!\n");
	        usleep(100000);//延时100ms。
		VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
		exit(1);
	}

	usleep(500000);
	send[0].ID=0x17;
	send[1].ID=0x27;

	send[0].Data[0] = 0x0F;
	send[1].Data[0] = 0x0F;
	send[0].Data[1] = 0xA0;
	send[1].Data[1] = 0xA0;
	send[0].Data[2] = 0x01;
	send[1].Data[2] = 0xFE;
	send[0].Data[3] = 0xA4;
	send[1].Data[3] = 0x5C;

	for(i = 4; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = 0x55;
		send[1].Data[i] = 0x55;
	}


	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 2) == 2)
	{
		printf("speed 360 succes!\n");
	}
	else
	{
		printf("speed 360 fail!\n");
	        usleep(100000);//延时100ms。
		VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
		exit(1);
	}

	usleep(60000000);//延时100ms。
//	send[0].ID=0x10;
//	send[1].ID=0x20;

	send[0].Data[0] = 0x0F;
	send[1].Data[0] = 0x0F;
	send[0].Data[1] = 0xA0;
	send[1].Data[1] = 0xA0;
	send[0].Data[2] = 0x00;
	send[1].Data[2] = 0x00;
	send[0].Data[3] = 0x00;
	send[1].Data[3] = 0x00;

	for(i = 4; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = 0x55;
		send[1].Data[i] = 0x55;
	}


	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 2) == 2)
	{
		printf("stop succes!\n");
	}
	else
	{
		printf("stop fail!\n");
	        usleep(100000);//延时100ms。
		VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
		exit(1);
	}
	
	int reclen=0;
	int ind=1;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
         {
		int j;
         	for(j=0;j<reclen;j++)
                {
                        printf("CAN%d RX ID:0x%08X", ind, rec[j].ID);//ID
                        if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
                        if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
                        if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数            
                        if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
                        if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
                        printf(" data:0x");     //数据 
                        for(i = 0; i < rec[j].DataLen; i++)
                        {
                                printf(" %02X", rec[j].Data[i]);
                        }
                        printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
                        printf("\n");
                  }
          }
	else
	{
		printf("receive fail!\n");
	}

	usleep(1000000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
}

i# WindyPendulum -- 风力摆

***  

### 贾一帆、邓卓 、陈灿思    
卓俊之光   

### 个人主页：  
#### 贾一帆：[LOFTER ]( http://nottechnology.lofter.com /)   

* * *  

##目录  
#### [项目说明](#Project)  
#### [引脚说明](#Pins)  
#### [功能说明](#Function)  
#### [代码版本分支](#Branches)  
#### [过程记录](#Process)  

****
## <a name = "Project" /> 项目说明  
**July 18, 2015 7:29 PM**
	基于STM32F1系列微控制器的风力摆，利用空心杯电机产生的风力推动摆杆做有一定轨迹的运动。
****

## <a name = "Pins" /> 引脚说明  
	
### 三色LED  
| RLED | GLED  | BLED  |  
|------|-------|-------|  
| PB.3 | PB.4 | PB.5 |  

### HC05    
| USART1Tx | USART1Rx |  
|----------|----------|  
|   PA.9   |   PA.10  |  

### MPU6050
| USART2Tx | USART2Rx |  
|----------|----------|  
|   PA.2   |   PA.3  |  

### Motor
| Motor1 | Motor2 |  Motor3  |  Motor4  |  
|----------|----------|-------|-------|  
|   PB.6   |   PB.7  |  PB.8  |  PB. |  
****

## <a name = "Function" /> 功能说明  
### 功能目录  
[HC-05](#HC05func)  


- - -

### <a name = "HC05func" />  HC-05  
**2015-07-06 16:31:07**    
	HC-05蓝牙串口 通讯模块   
	可用串口：USART1~USART3   
	关键文件：`hc05.c`，`hc05.h`    
   	文件目录：`Hardware/HC05`   
	使用方法：     
1. 在`hc05.h`的`Define`中，定义是否使用DMA发送、DMA接收（默认使能）    
2. 在`hc05.h`的`Define`中，定义发送和接收数组长度  
3. 在`hc05.h`的`Define`中，定义Key和LED引脚  
4. 在程序开始前，对结构体参数如下赋值： 
``` C  
	/* HC05 -------------------------------------------------------*/  
	HC05.USARTBASE = USART2;		//使用串口2	  
	HC05.KeyBase = GPIOC_BASE;		//Key引脚  
	HC05.KeyPin = GPIO_Pin_7;  

	HC05.LEDBase = GPIOC_BASE;		//LED引脚  
	HC05.LEDPin = GPIO_Pin_8;   
```   
5. 调用`HC05Init(&HC05)`函数，若检测到HC-05模块，返回值为ErrorStatus类型的SUCCESS  
6. 检测成功后，可与配对的蓝牙模块互发数据  
7. 发送方法一：使用`HC05printf`函数（此函数用法与printf函数相同）
``` c  
    void HC05printf(HC05Str * HC05,char* fmt,...)  
    例： HC05printf(&HC05,"print a float %f\r\n",102.324);      
    //函数内已包含检测上一次传输是否完成的函数  
```
8. 方法二：使用`UARTxDMASend`函数  
	写好`HC05.TxData`数组内容，然后将需要发送的字节数与DMA通道号（USART2为通道7）作为参数开启一次		DMA传输  
``` c  
	UARTxDMASend(HC05Str * HC05,u16 Len);
	例：for(i = 0;i<HC05.RxLen;i++)HC05.TxData[i]=HC05.RxData[i];
	UARTxDMASend(&HC05,HC05.RxLen);
```  
9. 接收使用USART空闲中断，中断中得到接收到的数据包长度，并且进行处理，详情见`hc05.c`中串口2的中断服		务函数    	


- - -
### <a name = "MPU6050func" />  MPU6050  
**July 21, 2015 11:10 AM**  
	九轴姿态传感器模块  串口通讯  
	可用串口：USART1~USART3  
	关键文件：`jy901.c`，`jy901.h`  
    文件目录：`Hardware/JY901`
	使用方法：    
1.在jy901.h中定义接收帧个数为3（加速度、角速度、角度）。
2.在初始化中设置模块通讯所用串口。
3.编写甄别包头的UART2中断函数（只需修改FH数组中第二个数据的值即可）。包头为第一个数据的帧头，若根据1中设置，包头即为0x55，0x51。
4.编写DMA发送完成中断服务函数，修改memcpy函数中的参数，初步定为在此函数内对动力摆进行PID调节。
5.根据需要修改串口、DMA中断的优先级，在.c文件的最下方xxNVICInit函数中。

* * *


## <a name = "Branches" />代码版本分支  
### 分支目录  
####  主线分支  
[master](#master)   
[dev](#dev)  

#### feature分支  


- - -

### <a name = "master"/>master分支  
#### 分支说明 
	发布稳定版本的分支  
#### 版本号目录  
[8876d89](#8876d89master)  


_ _ _


#### <a name = "8876d89master" />8876d89--blank project   
	新建空白工程  

- - -

### <a name = "dev"/> dev分支  
#### 分支说明  
	发布待稳定版本的分支  
#### 版本号目录  
[a9aa0b6](#a9aa0b6dev)  
[8876d89](#8876d89dev)  
[440c68a](#440c68adev)  
[dbaef4b](#dbaef4bdev)  
[33f75ad](#33f75addev)  
[b822961](#b822961dev)  
_ _ _
#### <a name = "b822961dev" /> b822961 --MotorReversal  
**July 25, 2015 3:06 PM**
	修复一个Bug，由于JY901坐标轴方向与MPU6050坐标轴相反，导致电机方向相反。

#### <a name = "33f75addev" /> 33f75ad --UseJY901  
**July 24, 2015 20:02 PM**  
	由于MPU6050模块输出数据在电机转速大时极不稳定，取消MPU6050模块，改用JY901十轴姿态传感器模块

#### <a name = "dbaef4bdev" /> dbaef4b  --filter  
**July 24, 2015 15:59 PM**  
	增加对MPU6050采集数据的滤波  

#### <a name = "440c68adev" /> 440c68a  
**July 21, 2015 9:34 PM**  
	改编一些程序  
1.将各种上位机控制函数放入upmac.c中。
2.将电机输入输出、检测零漂由上电启动改为发送指令控制。
3.使用XJI上位机，增加检测指令函数。

	修复一些BUG
1.将HC-05 DMA接收通道（RxDMA）的DMA优先级改为最低，该BUG导致第一次向动力摆发送数据后该通道堵占DMA控制器。
2.修复停止电机的函数Bug，该Bug导致关闭TIM4后PWM仍继续输出。
3.检测零漂结束后清空i值，该Bug导致只能检测一次零漂。

#### <a name = "fd86bc4dev" />fd86bc4--Merge branch 'checkPIDout' into dev  
**July 21, 2015 11:22 AM**  
	修复PID的一些Bug
1.零漂前的符号改为+号。
2.PIDout改为自加之前的值。
3.修改PIDout对于电机的编号

#### <a name = "a9aa0b6dev" />a9aa0b6--compile whole project of PID control   
**July 21, 2015 11:15 AM**  
	编译PID控制程序
    
#### <a name = "8876d89dev" />8876d89--blank project   
	新建空白工程  

****

## <a name = "Process" /> 过程记录   

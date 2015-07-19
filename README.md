# WindyPendulum -- 风力摆

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

* * *


## <a name = "Branches" />代码版本分支  
### 分支目录  
####  主线分支  
[master](#master)   
[dev](#dev)  

#### feature分支  
##### [HC-05](#HC-05branch)  

- - -

### <a name = "master"/>master分支  
#### 分支说明 
	发布稳定版本的分支  
#### 版本号目录  



### <a name = "dev"/> dev分支  
#### 分支说明  
	发布待稳定版本的分支  
#### 版本号目录  


- - -

****

## <a name = "Process" /> 过程记录   

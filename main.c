/**           引脚
 *板载LED      PF1、PF2、(PF3)    红蓝绿
 *GPIOOUT     PD2、PD3
 *UART0       PA0和PA1          中断函数USART0_Handler
 *UART1       PB0\PB1           蓝牙
 *UART2       PD6,PD7   RT      与上位机通讯
 *TIMER0                        中断时间1ms(系统时钟80Mhz,预分频200，重装载值4000，向下计数)
 *TIMER1                        中断时间0.1s(系统时钟80Mhz,预分频200，重装载值40000，向下计数)
 *PWM1-0-1    PD0、PD1          设置驱动频率1K,PWM1发生器0分开两路，预分频为64，向下计数，装载值1250
 *
 *QEI0        PD6(A)、PD7(B)          PD6->PHA0,PD7->PHB0        2的31次方 AB相信号不交换
 *QEI1        PC5(A)\PC6(B)           A B相信号交换
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"//GPIO
#include "driverlib/sysctl.h"//SYS
#include "driverlib/uart.h"//串口中断
#include "utils/uartstdio.h"
#include "driverlib/timer.h"//定时器
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/qei.h"
//PID
struct gesture
{
    float angr;//角度
    float angx;
    float angy;
    float w;//角速度
    float v1;//左轮速度
    float v2;//右轮速度
    float x1;//左轮位移
    float x2;//右轮位移
    float x;//总位移
};
struct pidstruct
{
    float kp,ki,kd;
    float p ,i ,d;
    float thisde, lastde;
};  //thisde当前
struct pidstruct initPID(float kp, float ki, float kd, struct pidstruct e)//初始化PID
{
    e.kp=kp;
    e.ki=ki;
    e.kd=kd;
    e.p=0;//有必要时不用初始化给0
    e.i=0;
    e.d=0;
    e.lastde =0;
    e.thisde=0;
    return e;
}
struct gesture gest;//当前姿态
struct gesture tgest;//目标姿态
struct gesture globalgest;//初始姿态
struct pidstruct angrpid;//角度PID
struct pidstruct angxpid;//x轴角度PID
struct pidstruct angypid;
struct pidstruct angzpid;
struct pidstruct x1pid;//位移PID
struct pidstruct x2pid;
struct pidstruct v1pid;
struct pidstruct v2pid;
float PID(struct pidstruct *e,float err, float outhigh , float outlow)//返回PID的值 outhigh和outlow是限幅
{
    float out;
    e->thisde=err;
    e->p = e->kp * e->thisde ;
    e->i = e->ki * (e->i+e->lastde );
    e->d = e->kd * (e->thisde -e->lastde );
    e->lastde = e->thisde ;
    out=e->p+e->i +e->d;
    if(out>outhigh ) out=outhigh ;//限幅
    if(out<outlow ) out =outlow ;
    return out;
}
//冒烟函数
void AcquireVelocity_Distance(void);  //获取速度和距离
//串口中断服务函数
void UART0_Handler(void);//UART0中断函数
void TIMER0_IRQHandler(void);//TIMER0中断函数
void UART1_Handler_BuleTooth(void);//UART1中断函数
void UART2_Handler(void);//UART2中断函数
//配置函数
void ConfigureSYS(void);
void ConfigureGPIO(void);
void ConfigureUART(void);//初始化串口打印函数
void ConfigureUART0(void);
void ConfigureUART_BuleTooth(void);//配置蓝牙UART1
void ConfigureUART2(void);//上位机通讯
void ConfigurePWM(void);
void ConfigureTIME0(void);
void ConfigureQEI0(void);
void ConfigureQEI1(void);
int main(void)
{
    ConfigureSYS();
    ConfigureTIME0();
    ConfigureGPIO();
    ConfigurePWM();
    ConfigureUART_BuleTooth();
    //ConfigureUART0();
    ConfigureUART();
    ConfigureQEI0();
    ConfigureQEI1();
    while(1)
    {
//      v1pid=initPID(10.5,1,0.01,v1pid);
//      v2pid=initPID(11,1,0,v2pid);
        UARTprintf("%u\n",gest.v1);
    }

}
//
void AcquireVelocity_Distance()
{
    static float catch0[3], catch1[3];
    catch1[1]=QEIPositionGet(QEI0_BASE);
    catch1[2]=catch1[1]-catch1[0];
    catch1[0]=catch1[1];
    if(QEIDirectionGet(QEI0_BASE))
    {
        gest.x1+=0.52f;
        gest.v1=644429.2/catch1[2];
    }
    else
    {
        gest.x1-=0.52f;
        gest.v1=-644429.2/catch1[2];
    }
}
//配置函数
int led=0;
void TIMER0_IRQHandler(void)
{
    //获取中断状态
    uint32_t status=TimerIntStatus(TIMER0_BASE, true);//屏蔽后的中断状态
    //清除中断标志位
    TimerIntClear(TIMER0_BASE,status);
    AcquireVelocity_Distance();
//    闪灯程序
//    if(led)
//        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
//    else
//        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);
//    led=!led;
}
void UART1_Handler_BuleTooth(void)
{
    int32_t rxbuf;
    //读取中断状态
    uint32_t status=UARTIntStatus(UART1_BASE,true);
    //清除中断标志位
    UARTIntClear(UART1_BASE, status);
    //检查FIFO中是否有字符，若有，读取出来
    while(UARTCharsAvail(UART1_BASE))
    {
      rxbuf=UARTCharGetNonBlocking(UART1_BASE);
      UARTCharPutNonBlocking(UART1_BASE, rxbuf);//将读取到的字符再发送出去
    }
}
void UART0_Handler(void)
{
    int32_t rxbuf;
    //读取中断状态
    uint32_t status=UARTIntStatus(UART0_BASE,true);
    //清除中断标志位
    UARTIntClear(UART0_BASE, status);
    //检查FIFO中是否有字符，若有，读取出来
    while(UARTCharsAvail(UART0_BASE))
    {
      rxbuf=UARTCharGetNonBlocking(UART0_BASE);
      UARTCharPutNonBlocking(UART0_BASE, rxbuf);//将读取到的字符再发送出去
    }
}
void UART2_Handler(void)
{

}
void ConfigureSYS(void)
{
    //配置系统时钟，使用外部晶振驱动PLL，分频系数2.5，系统时钟80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //配置
}
void ConfigureGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//使能PF
}
void ConfigureUART2(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能PD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//使能UART2
    GPIOPinConfigure(GPIO_PD6_U2RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PD7_U2TX);//TX=PIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART2_BASE, SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    //使能FIFO并设置FIFO深度
    UARTFIFOEnable(UART2_BASE);
    UARTFIFOLevelSet(UART2_BASE, UART_FIFO_RX1_8,UART_FIFO_TX1_8);
    //配置串口中断
    UARTIntRegister(UART2_BASE,UART2_Handler);
    UARTIntEnable(UART2_BASE, UART_INT_RX|UART_INT_RT);//开启串口接收中断和接收超时中断
    UARTIntClear(UART2_BASE, UART2_BASE);
    //启用串口0的中断并开启处理器中断控制器
    IntEnable(INT_UART2);
    IntMasterEnable();
    //使能串口
    UARTEnable(UART2_BASE);
}
void ConfigureUART0(void)
{
    //使能外设
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//USART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//使能UART0，板载USB转TTL，端口COM6
    //配置PF1、PF2、PF3作为输出，连接至板载LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    //将PA0、PA1引脚配置成串口引脚
    GPIOPinConfigure(GPIO_PA0_U0RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PA1_U0TX);//TX=GPIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //配置串口波特时钟源及参数
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART0_BASE, SysCtlClockGet(),115200,           UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    //使能FIFO并设置FIFO深度
    UARTFIFOEnable(UART0_BASE);
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_RX1_8,UART_FIFO_TX1_8);
    //注册串口中断函数
    UARTIntRegister(UART0_BASE, UART0_Handler); //UART0中断函数
    UARTIntEnable(UART0_BASE, UART_INT_RX|UART_INT_RT);//开启串口接收中断和接收超时中断
    UARTIntClear(UART0_BASE, UART0_BASE);
    //启用串口0的中断并开启处理器中断控制器
    IntEnable(INT_UART0);
    IntMasterEnable();
    //使能串口
    UARTEnable(UART0_BASE);

}
void ConfigureUART(void)
{
    // 启用UART使用的GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // 使能 UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // 为UART配置GPIO引脚。
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // 初始化控制台I/O的UART
    UARTStdioConfig(0, 115200, SysCtlClockGet());//0-115200-16M
}
void ConfigureUART_BuleTooth(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能PB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//使能UART1
    GPIOPinConfigure(GPIO_PB0_U1RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PB1_U1TX);//TX=PIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART1_BASE, SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    //使能FIFO并设置FIFO深度
    UARTFIFOEnable(UART1_BASE);
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_RX1_8,UART_FIFO_TX1_8);
    //配置串口中断
    UARTIntRegister(UART1_BASE, UART1_Handler_BuleTooth);
    UARTIntEnable(UART1_BASE, UART_INT_RX|UART_INT_RT);//开启串口接收中断和接收超时中断
    UARTIntClear(UART1_BASE, UART1_BASE);
    //启用串口0的中断并开启处理器中断控制器
    IntEnable(INT_UART1);
    IntMasterEnable();
    //使能串口
    UARTEnable(UART1_BASE);
}
void ConfigurePWM(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能PD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);//使能PWM模块1时钟
    //配置PWM
    //配置PWM时钟（设置USEPWMDIV分频器）
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);//PWM时钟80M/64
    //使能外设（已使能）
    //使能引脚复用PWM功能
    GPIOPinTypePWM(GPIO_PORTD_BASE,GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTD_BASE,GPIO_PIN_1);
    //GPIO OUT
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 0);
    //PWM信号分配
    GPIOPinConfigure(GPIO_PD0_M1PWM0);//PD0->PWM模块1信号0
    GPIOPinConfigure(GPIO_PD1_M1PWM1);//PD1->PWM模块1信号1
    //配置PWM发生器
    //模块1->发生器0->向下计数，不同步
    //模块1->发生器0->向下计数，不同步
    PWMGenConfigure(PWM1_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    //配置PWM周期
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_0,1249);
    //配置PWM占空比（当前设置占空比为50%）
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_0,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0)+1)*0.5);
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_1,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0)+1)*0.5);
    //使能PWM模块1通道0和通道1输出
    PWMOutputState(PWM1_BASE,PWM_OUT_0_BIT,true);
    PWMOutputState(PWM1_BASE,PWM_OUT_1_BIT,true);
    //使能PWM发生器
    PWMGenEnable(PWM1_BASE,PWM_GEN_0);
}
void ConfigureTIME0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);//使能TIMER0
    //配置定时器，将Timer0拆分并配置TIMERA为连续向下计数模式
    TimerClockSourceSet(TIMER0_BASE,TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    //设置预分频值,TIMER0是16/32-bits定时器，当分成两个定时器时预分频才起作用，此时预分频为8位
    TimerPrescaleSet(TIMER0_BASE, TIMER_A,199);
    //设置装载值，TIMERA是以16位工作
    TimerLoadSet(TIMER0_BASE, TIMER_A,3999);
    //注册中断服务函数
    TimerIntRegister(TIMER0_BASE,TIMER_A, TIMER0_IRQHandler);
    //开启定时器A超时中断
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    //设置中断优先级
    IntPrioritySet(INT_TIMER0A,  0);//高四位有效，越低优先级越高，0xE0-0x00 0-14
    //使能中断
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    //使能定时器
    TimerEnable( TIMER0_BASE, TIMER_A);
}
void ConfigureTIME1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);//使能TIMER0
    //配置定时器，将Timer0拆分并配置TIMERA为连续向下计数模式
    TimerClockSourceSet(TIMER1_BASE,TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER1_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    //设置预分频值,TIMER0是16/32-bits定时器，当分成两个定时器时预分频才起作用，此时预分频为8位
    TimerPrescaleSet(TIMER1_BASE, TIMER_A,199);
    //设置装载值，TIMERA是以16位工作
    TimerLoadSet(TIMER1_BASE, TIMER_A,39999);
    //注册中断服务函数
    TimerIntRegister(TIMER1_BASE,TIMER_A, AcquireVelocity_Distance);
    //开启定时器A超时中断
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
    //设置中断优先级
    IntPrioritySet(INT_TIMER1A,  0);//高四位有效，越低优先级越高，0xE0-0x00 0-14
    //使能中断
    IntEnable(INT_TIMER1A);
    IntMasterEnable();
    //使能定时器
    TimerEnable( TIMER1_BASE, TIMER_A);
}
void ConfigureQEI0(void)
{
    //使能QEI与GPIOD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //解锁PD7引脚
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    //复用GPIO
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD6_PHA0);//PD6->编码器A相输入
    GPIOPinConfigure(GPIO_PD7_PHB0);//PD7->编码器B相输入
    //配置QEI0模块，A、B两相信号边沿均产生计数，不开启索引脉冲复位，正交相位模式，A、B两相信号不交换，最大计数值为0xffffffff
    QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
    //禁用QEI0的所有中断
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    //使能QEI0
    QEIEnable(QEI0_BASE);
    //设置QEI0模块当前位置计数器值为0
    QEIPositionSet(QEI0_BASE,2^31);
}
void ConfigureQEI1(void)
{
    //使能QEI1与GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //复用GPIO
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PC5_PHA1);//PC5->编码器A相输入
    GPIOPinConfigure(GPIO_PC6_PHB1);//PC6->编码器B相输入
    //配置QEI1模块，A、B两相信号边沿均产生计数，不开启索引脉冲复位，正交相位模式，A、B两相信号交换，最大计数值为0xffffffff
    QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_SWAP,0xffffffff);
    //禁用QEI1的所有中断
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    //使能QEI1
    QEIEnable(QEI1_BASE);
    //设置QEI1模块当前位置计数器值为2^31
    QEIPositionSet(QEI1_BASE,2^31);
}
/*延时n秒
 * SysCtlDelay(SysCtlClockGet()*n/3);
 *快速注解/去注解操作 ctrl+/
 * 字体调节Ctrl+shif+"+" Ctrl+"-"
 * 板载LED PF1 PF2 PF3 分别对应红蓝绿
 * UARTprintf();
 * uint32_t pos=QEIPositionGet(QEI0_BASE);
 *M1PWM0->PD0  M1PWM1->PD1
 *M1PWM4->PF0  M1PWM5->PF1  M1PWM6->PF2  M1PWM7->PF3
 */

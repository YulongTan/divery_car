/**           ����
 *����LED      PF1��PF2��(PF3)    ������
 *GPIOOUT     PD2��PD3
 *UART0       PA0��PA1          �жϺ���USART0_Handler
 *UART1       PB0\PB1           ����
 *UART2       PD6,PD7   RT      ����λ��ͨѶ
 *TIMER0                        �ж�ʱ��1ms(ϵͳʱ��80Mhz,Ԥ��Ƶ200����װ��ֵ4000�����¼���)
 *TIMER1                        �ж�ʱ��0.1s(ϵͳʱ��80Mhz,Ԥ��Ƶ200����װ��ֵ40000�����¼���)
 *PWM1-0-1    PD0��PD1          ��������Ƶ��1K,PWM1������0�ֿ���·��Ԥ��ƵΪ64�����¼�����װ��ֵ1250
 *
 *QEI0        PD6(A)��PD7(B)          PD6->PHA0,PD7->PHB0        2��31�η� AB���źŲ�����
 *QEI1        PC5(A)\PC6(B)           A B���źŽ���
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
#include "driverlib/uart.h"//�����ж�
#include "utils/uartstdio.h"
#include "driverlib/timer.h"//��ʱ��
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
    float angr;//�Ƕ�
    float angx;
    float angy;
    float w;//���ٶ�
    float v1;//�����ٶ�
    float v2;//�����ٶ�
    float x1;//����λ��
    float x2;//����λ��
    float x;//��λ��
};
struct pidstruct
{
    float kp,ki,kd;
    float p ,i ,d;
    float thisde, lastde;
};  //thisde��ǰ
struct pidstruct initPID(float kp, float ki, float kd, struct pidstruct e)//��ʼ��PID
{
    e.kp=kp;
    e.ki=ki;
    e.kd=kd;
    e.p=0;//�б�Ҫʱ���ó�ʼ����0
    e.i=0;
    e.d=0;
    e.lastde =0;
    e.thisde=0;
    return e;
}
struct gesture gest;//��ǰ��̬
struct gesture tgest;//Ŀ����̬
struct gesture globalgest;//��ʼ��̬
struct pidstruct angrpid;//�Ƕ�PID
struct pidstruct angxpid;//x��Ƕ�PID
struct pidstruct angypid;
struct pidstruct angzpid;
struct pidstruct x1pid;//λ��PID
struct pidstruct x2pid;
struct pidstruct v1pid;
struct pidstruct v2pid;
float PID(struct pidstruct *e,float err, float outhigh , float outlow)//����PID��ֵ outhigh��outlow���޷�
{
    float out;
    e->thisde=err;
    e->p = e->kp * e->thisde ;
    e->i = e->ki * (e->i+e->lastde );
    e->d = e->kd * (e->thisde -e->lastde );
    e->lastde = e->thisde ;
    out=e->p+e->i +e->d;
    if(out>outhigh ) out=outhigh ;//�޷�
    if(out<outlow ) out =outlow ;
    return out;
}
//ð�̺���
void AcquireVelocity_Distance(void);  //��ȡ�ٶȺ;���
//�����жϷ�����
void UART0_Handler(void);//UART0�жϺ���
void TIMER0_IRQHandler(void);//TIMER0�жϺ���
void UART1_Handler_BuleTooth(void);//UART1�жϺ���
void UART2_Handler(void);//UART2�жϺ���
//���ú���
void ConfigureSYS(void);
void ConfigureGPIO(void);
void ConfigureUART(void);//��ʼ�����ڴ�ӡ����
void ConfigureUART0(void);
void ConfigureUART_BuleTooth(void);//��������UART1
void ConfigureUART2(void);//��λ��ͨѶ
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
//���ú���
int led=0;
void TIMER0_IRQHandler(void)
{
    //��ȡ�ж�״̬
    uint32_t status=TimerIntStatus(TIMER0_BASE, true);//���κ���ж�״̬
    //����жϱ�־λ
    TimerIntClear(TIMER0_BASE,status);
    AcquireVelocity_Distance();
//    ���Ƴ���
//    if(led)
//        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
//    else
//        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);
//    led=!led;
}
void UART1_Handler_BuleTooth(void)
{
    int32_t rxbuf;
    //��ȡ�ж�״̬
    uint32_t status=UARTIntStatus(UART1_BASE,true);
    //����жϱ�־λ
    UARTIntClear(UART1_BASE, status);
    //���FIFO���Ƿ����ַ������У���ȡ����
    while(UARTCharsAvail(UART1_BASE))
    {
      rxbuf=UARTCharGetNonBlocking(UART1_BASE);
      UARTCharPutNonBlocking(UART1_BASE, rxbuf);//����ȡ�����ַ��ٷ��ͳ�ȥ
    }
}
void UART0_Handler(void)
{
    int32_t rxbuf;
    //��ȡ�ж�״̬
    uint32_t status=UARTIntStatus(UART0_BASE,true);
    //����жϱ�־λ
    UARTIntClear(UART0_BASE, status);
    //���FIFO���Ƿ����ַ������У���ȡ����
    while(UARTCharsAvail(UART0_BASE))
    {
      rxbuf=UARTCharGetNonBlocking(UART0_BASE);
      UARTCharPutNonBlocking(UART0_BASE, rxbuf);//����ȡ�����ַ��ٷ��ͳ�ȥ
    }
}
void UART2_Handler(void)
{

}
void ConfigureSYS(void)
{
    //����ϵͳʱ�ӣ�ʹ���ⲿ��������PLL����Ƶϵ��2.5��ϵͳʱ��80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //����
}
void ConfigureGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//ʹ��PF
}
void ConfigureUART2(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��PD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//ʹ��UART2
    GPIOPinConfigure(GPIO_PD6_U2RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PD7_U2TX);//TX=PIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART2_BASE, SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    //ʹ��FIFO������FIFO���
    UARTFIFOEnable(UART2_BASE);
    UARTFIFOLevelSet(UART2_BASE, UART_FIFO_RX1_8,UART_FIFO_TX1_8);
    //���ô����ж�
    UARTIntRegister(UART2_BASE,UART2_Handler);
    UARTIntEnable(UART2_BASE, UART_INT_RX|UART_INT_RT);//�������ڽ����жϺͽ��ճ�ʱ�ж�
    UARTIntClear(UART2_BASE, UART2_BASE);
    //���ô���0���жϲ������������жϿ�����
    IntEnable(INT_UART2);
    IntMasterEnable();
    //ʹ�ܴ���
    UARTEnable(UART2_BASE);
}
void ConfigureUART0(void)
{
    //ʹ������
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//USART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//ʹ��UART0������USBתTTL���˿�COM6
    //����PF1��PF2��PF3��Ϊ���������������LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    //��PA0��PA1�������óɴ�������
    GPIOPinConfigure(GPIO_PA0_U0RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PA1_U0TX);//TX=GPIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //���ô��ڲ���ʱ��Դ������
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART0_BASE, SysCtlClockGet(),115200,           UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    //ʹ��FIFO������FIFO���
    UARTFIFOEnable(UART0_BASE);
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_RX1_8,UART_FIFO_TX1_8);
    //ע�ᴮ���жϺ���
    UARTIntRegister(UART0_BASE, UART0_Handler); //UART0�жϺ���
    UARTIntEnable(UART0_BASE, UART_INT_RX|UART_INT_RT);//�������ڽ����жϺͽ��ճ�ʱ�ж�
    UARTIntClear(UART0_BASE, UART0_BASE);
    //���ô���0���жϲ������������жϿ�����
    IntEnable(INT_UART0);
    IntMasterEnable();
    //ʹ�ܴ���
    UARTEnable(UART0_BASE);

}
void ConfigureUART(void)
{
    // ����UARTʹ�õ�GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // ʹ�� UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // ΪUART����GPIO���š�
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // ��ʼ������̨I/O��UART
    UARTStdioConfig(0, 115200, SysCtlClockGet());//0-115200-16M
}
void ConfigureUART_BuleTooth(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��PB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//ʹ��UART1
    GPIOPinConfigure(GPIO_PB0_U1RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PB1_U1TX);//TX=PIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART1_BASE, SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    //ʹ��FIFO������FIFO���
    UARTFIFOEnable(UART1_BASE);
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_RX1_8,UART_FIFO_TX1_8);
    //���ô����ж�
    UARTIntRegister(UART1_BASE, UART1_Handler_BuleTooth);
    UARTIntEnable(UART1_BASE, UART_INT_RX|UART_INT_RT);//�������ڽ����жϺͽ��ճ�ʱ�ж�
    UARTIntClear(UART1_BASE, UART1_BASE);
    //���ô���0���жϲ������������жϿ�����
    IntEnable(INT_UART1);
    IntMasterEnable();
    //ʹ�ܴ���
    UARTEnable(UART1_BASE);
}
void ConfigurePWM(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��PD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);//ʹ��PWMģ��1ʱ��
    //����PWM
    //����PWMʱ�ӣ�����USEPWMDIV��Ƶ����
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);//PWMʱ��80M/64
    //ʹ�����裨��ʹ�ܣ�
    //ʹ�����Ÿ���PWM����
    GPIOPinTypePWM(GPIO_PORTD_BASE,GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTD_BASE,GPIO_PIN_1);
    //GPIO OUT
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3, 0);
    //PWM�źŷ���
    GPIOPinConfigure(GPIO_PD0_M1PWM0);//PD0->PWMģ��1�ź�0
    GPIOPinConfigure(GPIO_PD1_M1PWM1);//PD1->PWMģ��1�ź�1
    //����PWM������
    //ģ��1->������0->���¼�������ͬ��
    //ģ��1->������0->���¼�������ͬ��
    PWMGenConfigure(PWM1_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    //����PWM����
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_0,1249);
    //����PWMռ�ձȣ���ǰ����ռ�ձ�Ϊ50%��
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_0,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0)+1)*0.5);
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_1,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0)+1)*0.5);
    //ʹ��PWMģ��1ͨ��0��ͨ��1���
    PWMOutputState(PWM1_BASE,PWM_OUT_0_BIT,true);
    PWMOutputState(PWM1_BASE,PWM_OUT_1_BIT,true);
    //ʹ��PWM������
    PWMGenEnable(PWM1_BASE,PWM_GEN_0);
}
void ConfigureTIME0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);//ʹ��TIMER0
    //���ö�ʱ������Timer0��ֲ�����TIMERAΪ�������¼���ģʽ
    TimerClockSourceSet(TIMER0_BASE,TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    //����Ԥ��Ƶֵ,TIMER0��16/32-bits��ʱ�������ֳ�������ʱ��ʱԤ��Ƶ�������ã���ʱԤ��ƵΪ8λ
    TimerPrescaleSet(TIMER0_BASE, TIMER_A,199);
    //����װ��ֵ��TIMERA����16λ����
    TimerLoadSet(TIMER0_BASE, TIMER_A,3999);
    //ע���жϷ�����
    TimerIntRegister(TIMER0_BASE,TIMER_A, TIMER0_IRQHandler);
    //������ʱ��A��ʱ�ж�
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    //�����ж����ȼ�
    IntPrioritySet(INT_TIMER0A,  0);//����λ��Ч��Խ�����ȼ�Խ�ߣ�0xE0-0x00 0-14
    //ʹ���ж�
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    //ʹ�ܶ�ʱ��
    TimerEnable( TIMER0_BASE, TIMER_A);
}
void ConfigureTIME1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);//ʹ��TIMER0
    //���ö�ʱ������Timer0��ֲ�����TIMERAΪ�������¼���ģʽ
    TimerClockSourceSet(TIMER1_BASE,TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER1_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    //����Ԥ��Ƶֵ,TIMER0��16/32-bits��ʱ�������ֳ�������ʱ��ʱԤ��Ƶ�������ã���ʱԤ��ƵΪ8λ
    TimerPrescaleSet(TIMER1_BASE, TIMER_A,199);
    //����װ��ֵ��TIMERA����16λ����
    TimerLoadSet(TIMER1_BASE, TIMER_A,39999);
    //ע���жϷ�����
    TimerIntRegister(TIMER1_BASE,TIMER_A, AcquireVelocity_Distance);
    //������ʱ��A��ʱ�ж�
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
    //�����ж����ȼ�
    IntPrioritySet(INT_TIMER1A,  0);//����λ��Ч��Խ�����ȼ�Խ�ߣ�0xE0-0x00 0-14
    //ʹ���ж�
    IntEnable(INT_TIMER1A);
    IntMasterEnable();
    //ʹ�ܶ�ʱ��
    TimerEnable( TIMER1_BASE, TIMER_A);
}
void ConfigureQEI0(void)
{
    //ʹ��QEI��GPIOD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //����PD7����
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    //����GPIO
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD6_PHA0);//PD6->������A������
    GPIOPinConfigure(GPIO_PD7_PHB0);//PD7->������B������
    //����QEI0ģ�飬A��B�����źű��ؾ������������������������帴λ��������λģʽ��A��B�����źŲ�������������ֵΪ0xffffffff
    QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
    //����QEI0�������ж�
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    //ʹ��QEI0
    QEIEnable(QEI0_BASE);
    //����QEI0ģ�鵱ǰλ�ü�����ֵΪ0
    QEIPositionSet(QEI0_BASE,2^31);
}
void ConfigureQEI1(void)
{
    //ʹ��QEI1��GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //����GPIO
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PC5_PHA1);//PC5->������A������
    GPIOPinConfigure(GPIO_PC6_PHB1);//PC6->������B������
    //����QEI1ģ�飬A��B�����źű��ؾ������������������������帴λ��������λģʽ��A��B�����źŽ�����������ֵΪ0xffffffff
    QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_SWAP,0xffffffff);
    //����QEI1�������ж�
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    //ʹ��QEI1
    QEIEnable(QEI1_BASE);
    //����QEI1ģ�鵱ǰλ�ü�����ֵΪ2^31
    QEIPositionSet(QEI1_BASE,2^31);
}
/*��ʱn��
 * SysCtlDelay(SysCtlClockGet()*n/3);
 *����ע��/ȥע����� ctrl+/
 * �������Ctrl+shif+"+" Ctrl+"-"
 * ����LED PF1 PF2 PF3 �ֱ��Ӧ������
 * UARTprintf();
 * uint32_t pos=QEIPositionGet(QEI0_BASE);
 *M1PWM0->PD0  M1PWM1->PD1
 *M1PWM4->PF0  M1PWM5->PF1  M1PWM6->PF2  M1PWM7->PF3
 */

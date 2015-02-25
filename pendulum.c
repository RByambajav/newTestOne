#include<p18f452.h>
#include<math.h>
#include<timers.h>
#include<delays.h>
#include<portb.h>
#include<usart.h>
#include<pwm.h>
#include<adc.h>
#include<xlcd.h>
#include"D:\MY PROJECTS\Pendulum\pend_header.h"
/**	System config bits **/
#pragma config WDT = OFF
#pragma config OSC = HS
#pragma config LVP = OFF
#pragma config BOR = OFF
/** L298 control lines **/
#define LED	PORTAbits.RA2
#define In0	PORTDbits.RD0
#define In1 PORTDbits.RD1	
#define encoder0_dir	PORTBbits.RB2
#define encoder1_dir	PORTDbits.RD2
/** global data declarations **/
unsigned char lcd_data0[16]={0x64,0x74,0x3d,0x20,0x20,0x20,0x20,0x20,0x45,0x31,0x3a,0x20,0x20,0x20,0x20,0x20};
unsigned char lcd_data1[16]={0x20,0x4b,0x70,0x3d,0x20,0x20,0x4b,0x69,0x3d,0x2e,0x20,0x4b,0x64,0x3d,0x2e,0x20};

/** function declarations **/
void config_adc(void);
int read_adc(char);
void config_pwms(char);
void config_timers(void);
void config_interrupts(void);
void convertto_LCD(void);
/*motor control functions*/
void go_right(void);
void go_left(void);
void brake_motor(void);
void do_PID(void);
void clock0(void);
void check_limit(void);

/*EXTERNAL LCD functions*/
void display(void);
void DelayFor18TCY(void)
{
	Delay10TCYx(20);//at least 20 Tcy
}
void DelayPORXLCD(void)
{
	Delay10KTCYx(15);//at least 15ms
}
void DelayXLCD(void)
{
	Delay10KTCYx(10);//at least 10ms
}
void init_LCD(void)// configure external LCD
{
	OpenXLCD(FOUR_BIT & LINES_5X7);
	WriteCmdXLCD(BLINK_OFF&CURSOR_OFF);
	WriteCmdXLCD(SHIFT_DISP_LEFT);
	WriteCmdXLCD(0x80);
}
void print_LCD(void)
{
	unsigned char i;
	for(i=0;i<16;i++)
	{
		while(BusyXLCD());
		WriteDataXLCD(lcd_data0[i]);
	}
	while(BusyXLCD());
	WriteCmdXLCD(0xc0);
	for(i=1;i<16;i++)
	{
		while(BusyXLCD());
		WriteDataXLCD(lcd_data1[i]);
	}
}
void display()
{
	convertto_LCD();
	while(BusyXLCD());
	WriteCmdXLCD(0x01);
	while(BusyXLCD());
	print_LCD();
	while(BusyXLCD());
}
void go_right()
{
	LED = 0;
	In0 = 0;//Right
	In1 = 1;//
}
void go_left()
{
	LED = 1;
	In0 = 1;//Left
	In1 = 0;//
}
void brake_motor()
{
	SetDCPWM1(1023);
	In0 = 0;//brake
	In1 = 0;//	
}

struct
{
	signed int counter;
	signed int prev_val;
	signed int ax;
	unsigned int j;

}encoder0;
struct
{
	signed int counter;
}encoder1;
struct
{
	signed int error;	//e(n)
	signed int prev_error;//e(n-1)
	signed int Kp;
	signed int Ki;
	signed int Kd;

	signed int set_point;
	signed int integral;
	signed int derivative;
	signed int duty;
}PID;
struct
{
	unsigned char enable_dec;
}status;
struct
{
	unsigned int five_ms;
	unsigned char sec;
	unsigned char time_alarm;
}clock;
void main(void)
{
	char data=0x36;
	char i;
	unsigned char ddram=0x00;
	status.enable_dec = 0; // critical variables
	
	TRISAbits.TRISA2 = 0; //LED
		
	TRISDbits.TRISD0 = 0;//L298 In0
	TRISDbits.TRISD1 = 0;//L298 In1
	TRISDbits.TRISD2 = 1;//Encoder_1 dir
	TRISDbits.TRISD3 = 0;
	TRISD &=0x0f;

	TRISBbits.TRISB0 = 1;//Encoder_0 INT0
	TRISBbits.TRISB1 = 1;//Encoder_1 INT1
	TRISBbits.TRISB2 = 1;//Encoder_0 dir

	TRISCbits.TRISC0 = 1;//start button
	TRISCbits.TRISC3 = 0;//test point
	TRISCbits.TRISC4 = 0;//test point
	TRISCbits.TRISC5 = 0;//test point
	TRISCbits.TRISC6 = 0;//TX
	TRISCbits.TRISC7 = 1;//RX

	TRISEbits.TRISE0 = 0;
	TRISEbits.TRISE1 = 0;
	TRISEbits.TRISE2 = 0;

	TRISEbits.PSPMODE = 0;
	TRISEbits.IBOV = 0;
	TRISEbits.IBF = 0;
	TRISEbits.OBF = 0;
	
	config_adc();//configuration of ADC and analog ports
	config_timers();// configuration of timers
	config_pwms(255);//configuration of PWM generator
	init_LCD(); // initialization of LCD controller
	config_interrupts();// configuration of interrupts 
	OpenPWM1(255);//init PWM module

	
	encoder0.counter = 50;
	encoder1.counter = 0;
	PID.Kp=0;
	PID.Ki=0;
	PID.Kd=0;
	PID.set_point = 1000;
	PID.integral = 0;
	PID.derivative = 0;
	PID.duty = 0;
	PID.error = 0;
	PID.prev_error = 0;

	encoder1.counter = 1000;
	status.enable_dec=1;
	clock.five_ms=0;
	clock.sec = 0;
	T0CONbits.TMR0ON = 1; //turn on timer0
	while(1)
	{
		display();
		PID.Kp = read_adc(0)/10;
		PID.Ki = read_adc(1)/100;
		PID.Kd = read_adc(3)/100;
		Delay10KTCYx(150);
	}//forever loop
	

}
/*************************************/
void convertto_LCD()
{
	unsigned int temp,temp1;
	temp = encoder0.counter;
//	lcd_data0[3]=temp/10000+0x30;
//	lcd_data0[4]=(temp-(temp/10000)*10000-temp%1000)/1000+0x30;
//	lcd_data0[5]=(temp-(temp/1000)*1000-temp%100)/100+0x30;
//  lcd_data0[6]=(temp-(temp/100)*100-temp%10)/10+0x30;
//  lcd_data0[7]=temp%10+0x30;
	
	temp1 = encoder1.counter;
	lcd_data0[11]=temp1/10000+0x30;
	lcd_data0[12]=(temp1-(temp1/10000)*10000-temp1%1000)/1000+0x30;
	lcd_data0[13]=(temp1-(temp1/1000)*1000-temp1%100)/100+0x30;
    lcd_data0[14]=(temp1-(temp1/100)*100-temp1%10)/10+0x30;
    lcd_data0[15]=temp1%10+0x30;

	lcd_data1[4]=PID.Kp/10+0x30;
	lcd_data1[5]=PID.Kp%10+0x30;
	//lcd_data1[9]=PID.Ki/10+0x30;
	lcd_data1[10]=PID.Ki%10+0x30;
	//lcd_data1[14]=PID.Kd/10+0x30;
	lcd_data1[15]=PID.Kd%10+0x30;
	//lcd_data1[14]=clock.sec/10 + 0x30;
	//lcd_data1[15]=clock.sec%10+0x30;

	lcd_data0[3]=PID.duty/1000 + 0x30;
	lcd_data0[4]=(PID.duty%1000)/100 + 0x30;
	lcd_data0[5]=(PID.duty%100)/10 + 0x30;
	lcd_data0[6]=PID.duty%10 + 0x30;
	
}
// high priority interrupt vector
#pragma code isrcode = 0x08
void isrhandler(void)
{
	_asm
		goto intHandlerHigh
	_endasm
}
#pragma code

#pragma interrupt intHandlerHigh
void intHandlerHigh(void)
{
		if(INTCONbits.INT0IF)
		{
			INTCONbits.INT0IF = 0;
			if(encoder0_dir==1)encoder0.counter--;
			else encoder0.counter++;		
		}//3
		if(INTCON3bits.INT1IF)
		{
			INTCON3bits.INT1IF = 0;
			if(encoder1_dir==1)encoder1.counter--;
			else encoder1.counter++;
		}//4
}

// low priority interrupt vector
#pragma code lowISRcode = 0x18
void lowISRhandler(void)
{
	_asm
		goto intHandlerLow
	_endasm
}
#pragma code

#pragma interruptlow intHandlerLow
void intHandlerLow(void)  // every 5ms
{

	if(INTCONbits.TMR0IF) //timer0 int occurs every 5ms
    {
        INTCONbits.TMR0IF = CLEAR;
		WriteTimer0(0xe795);	// 5ms

		do_PID();
		clock0();
	//	check_limit();
		
    }
}
/****************************************/
void do_PID()
{
	PID.error = PID.set_point - encoder1.counter;
	if(PID.error>100||PID.error<-100)
	{
		PID.duty = 0;
		SetDCPWM1(PID.duty);
		while(1);
	}
	if(PID.error>40)PID.error=40;
	if(PID.error<-40)PID.error=-40;
	if(PID.error>0&&PID.error<10)PID.error=0;
	if(PID.error<0&&PID.error>-10)PID.error=0;

	PID.integral +=PID.error; 
	if(PID.integral>0&&PID.integral>1000)PID.integral=1000;
	if(PID.integral<0&&PID.integral<-1000)PID.integral=-1000;
	PID.derivative = PID.error - PID.prev_error;
	PID.prev_error = PID.error;
	if(PID.derivative>0&&PID.derivative>1000)PID.derivative=1000;
	if(PID.derivative<0&&PID.derivative<-1000)PID.derivative=-1000;

	if(PID.error<0)
	{
		PID.duty = PID.Kp*PID.error + (PID.Ki*PID.integral)/10 + (PID.Kd*PID.derivative)/10;
		
		if(PID.duty<0)
		{
			PID.duty = 0-PID.duty;
		}

		if(PID.duty>1023)PID.duty=1023;
		SetDCPWM1(PID.duty);
		go_left();
	}
	else
	{
		PID.duty = PID.Kp*PID.error + (PID.Ki*PID.integral)/10 + (PID.Kd*PID.derivative)/10;
		if(PID.duty<0)
		{
			PID.duty = 0-PID.duty;
		}
		
		if(PID.duty>1023)PID.duty=1023;
		SetDCPWM1(PID.duty);
		go_right();
	}

}
/*****************************************************/
void clock0()
{
	clock.five_ms++;
	if(clock.five_ms==200)
	{
		clock.sec++;
		clock.five_ms=0;
		if(clock.sec>99)clock.sec=0;
	}

}
void check_limit()
{
	if(PID.duty>1020)
	{
		encoder0.ax = encoder0.prev_val - encoder0.counter;
		if(encoder0.ax>-10&&encoder0.ax<10)
		{
			encoder0.j++;
			if(encoder0.j>600)
			{
				T0CONbits.TMR0ON = 0; //turn off timer0 to kill system operation
				INTCONbits.TMR0IE = DISABLE;	// disable TMR0 int 
				
				while(1)SetDCPWM1(0);
			}
		}
		else
		{
			encoder0.j=0;
		}	
		encoder0.prev_val = encoder0.counter;
	}
	
}
/*******************************************/
void config_adc()
{
	ADCON1 = 0b00000100; //AN4-AN7,AN2 digital AN0,AN1,AN3 are analog
	OpenADC(ADC_FOSC_32 	&
			ADC_RIGHT_JUST	&
			ADC_3ANA_0REF	&
			ADC_CH0			&
			ADC_CH1			&
			ADC_CH3			,
			ADC_INT_OFF);		
}
int read_adc(char channel)
{
	int result;
	channel=channel<<3;   	//1
	channel&=0b00111000;	//2
	ADCON0&=0b11000111;		//3
	ADCON0|=channel;		//1, 2, 3 SetChanADC(channel);
	Delay10TCYx(5);		// Delay For 50TCY
	ConvertADC();		// Start conversion
	while(BusyADC());	// Wait for completion
	result = ReadADC();	// Read result
//	CloseADC();			// Disable A/D converter
//	result=result/4;	// 8 bit result 
	return result;		// Return ADC value
}
void config_timers()
{
	/*timer0 used for time trigger */
	OpenTimer0(	TIMER_INT_ON 	&
				T0_16BIT		&
				T0_SOURCE_INT	&
				T0_EDGE_RISE	&
				T0_PS_1_4);
	WriteTimer0(0xe795);	// 65535 - 0xe795 = 6250*0.8us = 5ms
	T0CONbits.TMR0ON = 0; //turn off timer0
	/*timer2 used for PWM generator*/
	OpenTimer2(	TIMER_INT_OFF	&
				T2_PS_1_4		&	// prescaler can be 1:1,1:4,1:16
				T2_POST_1_1		   // postscaler can be 1:1,1:2,1:15,1:16 
			  );
	
}
void config_pwms(char period)
{
	OpenPWM1(period);
	OpenPWM2(period);
}
void config_interrupts()
{
	RCONbits.IPEN = ENABLE;			// enable Interrupt Priority
	INTCONbits.GIEH = ENABLE; 		// Enable all high priority int
	INTCONbits.GIEL = ENABLE; 		// enable all low priority int

/*timer0 interrupt*/
	INTCONbits.TMR0IE = ENABLE;	// enable TMR0 int 
	INTCONbits.TMR0IF = CLEAR; 		// clear TMR0 int flag
	INTCON2bits.TMR0IP	= LOW;		// TMR0 interrupt low priority

/*timer1 interrupt*/
	PIR1bits.TMR1IF = CLEAR;		// clear TMR1 overflow int flag
	PIE1bits.TMR1IE = ENABLE;		// disable TMR1 int
	IPR1bits.TMR1IP = LOW;			// TMR1 overflow int low priority

/*timer2 interrupt*/
	PIR1bits.TMR2IF = CLEAR;		// clear TMR2 = PR2 int flag
	PIE1bits.TMR2IE = DISABLE;		// disable TMR2 = PR2	int 
	IPR1bits.TMR2IP = LOW;			// TMR2 = PR2 low priority

/*timer3 interrupt*/
	PIR2bits.TMR3IF = CLEAR;		// clear TMR3 overflow flag 
	PIE2bits.TMR3IE = ENABLE;		// TMR3 overflow int
	IPR2bits.TMR3IP = LOW;			// TMR3 overflow int low 

/*external interrupt 0 */
	INTCONbits.INT0IE = ENABLE; 	// enable ext int0
	INTCONbits.INT0IF = CLEAR;		// clear ext int0 flag
	INTCON2bits.INTEDG0 = 1;		// External int0 on rising edge
	/*EXT INT0 is always high priority*/

/*external interrupt 1 */
	INTCON3bits.INT1IE = ENABLE;	// enable int1
	INTCON2bits.INTEDG1 = 1;		// External int1 on rising edge
	INTCON3bits.INT1IP = HIGH;		// external int1 high priority
	INTCON3bits.INT1IF = CLEAR;		// clear ext int1 flag

/*external interrupt 2 */	
	INTCON2bits.INTEDG2 = 0;		// External int2 on falling edge
	INTCON3bits.INT2IP = LOW;		// external int2 low priority
	INTCON3bits.INT2IE = DISABLE;	// disable external int2 
	INTCON3bits.INT2IF = CLEAR;		// clear int2 flag

/*RB Port change interrupt */
	INTCONbits.RBIE = DISABLE; 		// disable RB Port change int
	INTCONbits.RBIF = CLEAR;		// clear RB Port change flag
	INTCON2bits.RBIP = LOW;			// RB Port change int low priority


}

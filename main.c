#include "MK20D7.h"
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "spi.h"
#include "can.h"
#include "cdc.h"

#define BIT0             (0x00000001)
#define BIT1             (0x00000002)
#define BIT2             (0x00000004)
#define BIT3             (0x00000008)
#define BIT4             (0x00000010)
#define BIT5             (0x00000020)
#define BIT6             (0x00000040)
#define BIT7             (0x00000080)
#define BIT8             (0x00000100)
#define BIT9             (0x00000200)
#define BIT10            (0x00000400)
#define BIT11            (0x00000800)
#define BIT12            (0x00001000)
#define BIT13            (0x00002000)
#define BIT14            (0x00004000)
#define BIT15            (0x00008000)
#define BIT16			 (0x00010000)
#define BIT17			 (0x00020000)
#define BIT18			 (0x00040000)
#define BIT19			 (0x00080000)
#define BIT20			 (0x00100000)
#define BIT21			 (0x00200000)
#define BIT22			 (0x00400000)
#define BIT23			 (0x00800000)
#define BIT24			 (0x01000000)
#define BIT25			 (0x02000000)
#define BIT26			 (0x04000000)
#define BIT27			 (0x08000000)
#define BIT28			 (0x10000000)
#define BIT29			 (0x20000000)
#define BIT30			 (0x40000000)
#define BIT31			 (0x80000000)

#define GMLAN_PRI_MASK		(0x1C000000)
#define GMLAN_PRI_SHIFT		(0x1A)
#define GMLAN_ARB_MASK		(0x03FFE000)
#define GMLAN_ARB_SHIFT		(0x0D)
#define GMLAN_SENDER_MASK	(0x00001FFF)
#define GMLAN_SENDER_SHIFT	(0x00)

extern volatile uint32_t SysTick;

// Timing variables
uint32_t Time=0, TextTimeout=0;

// 0x108
volatile uint16_t RawSpeed=0;

// 0x120
volatile int8_t RawYawRate=0;
volatile int16_t RawSteeringAngle=0;

// 0x165
volatile uint16_t RawRPM=0;

// 0x170
volatile uint8_t RawOilPress=0;
volatile uint8_t RawTransTemp=0;
volatile uint8_t RawOilTemp=0;
volatile uint8_t RawIntakeTemp=0;
volatile uint8_t RawCoolantTemp=0;

// 0x212
volatile int16_t RawOutside1=0, RawOutside2=0;

// 0x21C
volatile uint8_t Status=0x00;

// 0x400
volatile uint16_t RawFuelLevel=0;

// 0x425
volatile uint8_t units=0;

// Display switch
volatile uint8_t temp=0;

// Steering wheel button press lockout and flag for displaying text on displays
uint8_t lock=0, text=0;

// String for GMLAN DIC nav text
char String[64], *Ptr;

uint32_t micros(void)
{
	uint32_t count, current, istatus;

	__asm__ volatile ("CPSID i");
	current=SYST_CVR;
	count=SysTick;
	istatus=SCB_ICSR;
	__asm__ volatile ("CPSIE i");

	if((istatus&SCB_ICSR_PENDSTSET_MASK)&&current>50)
		count++;

	current=((72000000/1000)-1)-current;

	return count*1000+current/(72000000/1000000);
}

void DelayMS(uint32_t ms)
{
	uint32_t start=micros();

	if(ms>0)
	{
		while(1)
		{
			while((micros()-start)>=1000)
			{
				ms--;

				if(ms==0)
					return;

				start+=1000;
			}
		}
	}
}

void DelayUS(uint32_t usec)
{
	uint32_t n=usec*24; // 72MHz

	if(n==0)
		return;

	__asm__ volatile(
		"L_%=_delayMicroseconds:\n"
		"subs   %0, #1\n"
		"bne    L_%=_delayMicroseconds\n"
		: "+r" (n) : );
}

uint8_t _display_buf[8]={ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t _digit[]={ 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E }; // digits 0 to F, 0xBF = '-', 0x7F = '.', 0xFF = off
/*
Segment bits (1 = off, 0 = on)

  -0-
5|   |1
  -6-
4|   |2
  -3-
      .7

A = 10001000/0x88
b = 10000011/0x83
C = 11000110/0xC6
d = 10100001/0xA1
E = 10000110/0x86
F = 10001110/0x8E
g = 10010000/0x90
H = 10001001/0x89
I = 11001111/0xCF
J = 11100001/0xE1
K = 10001001/0x89
L = 11000111/0xC7
M = 11101010/0xEA
n = 10101011/0xAB
o = 10100011/0xA3
P = 10001100/0x8C
Q = 10011000/0x98
R = 10101111/0xAF
S = 10010010/0x92
T = 11111000/0xF8
u = 11100011/0xE3
V = 11110001/0xF1
W = 11010101/0xD5
X = 10001001/0x89
y = 10010001/0x91
Z = 10100100/0xA4
*/

void _display_custom(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
	_display_buf[7]=d0;
	_display_buf[6]=d1;
	_display_buf[5]=d2;
	_display_buf[4]=d3;
	_display_buf[3]=d4;
	_display_buf[2]=d5;
	_display_buf[1]=d6;
	_display_buf[0]=d7;

	GPIOD_PCOR|=BIT4;
	SPI_WriteBuf(_display_buf, 8);
	GPIOD_PSOR|=BIT4;
}

void _display(int16_t value, uint8_t disp, uint8_t dp, uint8_t extra)
{
	uint8_t neg=0, i;

	// Return, if selected display is outside range
	if(disp>2)
		return;

	// If forcing an extra display digit, knock off a digit on the max input value
	if(extra)
	{
		if(value>=999)
			value=999;

		if(value<=-99)
			value=-99;
	}
	else
	{
		if(value>=9999)
			value=9999;

		if(value<=-999)
			value=-999;
	}

	// If it's a negative value, remember that and negate the value
	if(value<0)
	{
		neg=1;
		value=-value;
	}

	// Index into the buffer for the selected display
	i=4*disp;

	// Clear the selected display
	_display_buf[4*disp+0]=0xFF;
	_display_buf[4*disp+1]=0xFF;
	_display_buf[4*disp+2]=0xFF;
	_display_buf[4*disp+3]=0xFF;

	// If an extra digit is wanted (0 to F)
	if(extra>0&&extra<=0xF)
		_display_buf[i++]=_digit[extra];

	// If value is 0, force a 0 digit (otherwise it will be blank)
	if(value==0)
		_display_buf[i++]=_digit[0];
	// Divide down the value by 10, to extract the individual digits
	else while(value)
	{
		_display_buf[i++]=_digit[value%10];
		value/=10;
	}

	// If it's a negative value, insert a '-' (limits the total display value)
	if(neg)
		_display_buf[i++]=0xBF;

	// If a decimal point is desired, clear that bit on that selected digit (1 = off, 0 = on)
	if(dp)
		_display_buf[4*disp+(dp-1)]&=0x7F;

	// Send it to the SPI port
	GPIOD_PCOR|=BIT4;
	SPI_WriteBuf(_display_buf, 8);
	GPIOD_PSOR|=BIT4;
}

// Display nothing on two display modules
void _displayOff(void)
{
	uint8_t buf[8]={ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	GPIOD_PCOR|=BIT4;
	SPI_WriteBuf(buf, 8);
	GPIOD_PSOR|=BIT4;
}

void sendNEC(uint8_t Addr, uint8_t Cmd)
{
	uint32_t mask;
	struct
	{
		union
		{
			struct
			{
				uint8_t command_inv;	// Inverse command bits
				uint8_t command;		// Command bits
				uint8_t address_inv;	// Inverse address bits
				uint8_t address;		// Address bits
			};
			uint32_t raw;				// Raw IR code data
		};
	} data;

	data.address=Addr;
	data.address_inv=~Addr;
	data.command=Cmd;
	data.command_inv=~Cmd;

	GPIOC_PSOR|=BIT5;
	DelayUS(9000);
	GPIOC_PCOR|=BIT5;
	DelayUS(4500);

	for(mask=1<<31;mask;mask>>=1)
	{
		if(data.raw&mask)
		{
			GPIOC_PSOR|=BIT5;
			DelayUS(560);
			GPIOC_PCOR|=BIT5;
			DelayUS(1690);
		}
		else
		{
			GPIOC_PSOR|=BIT5;
			DelayUS(560);
			GPIOC_PCOR|=BIT5;
			DelayUS(560);
		}
	}

	GPIOC_PSOR|=BIT5;
	DelayUS(560);
	GPIOC_PCOR|=BIT5;
}

void SendGMLAN(uint8_t Pri, uint16_t Arb, uint16_t Sender, uint8_t Length, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4, uint8_t Data5, uint8_t Data6, uint8_t Data7)
{
	CAN_Frame_t tx;

	tx.MessageID=CAN_MESSAGE_ID_EXT|((Pri<<GMLAN_PRI_SHIFT)&GMLAN_PRI_MASK)|((Arb<<GMLAN_ARB_SHIFT)&GMLAN_ARB_MASK)|((Sender<<GMLAN_SENDER_SHIFT)&GMLAN_SENDER_MASK);
	tx.Length=Length;
	tx.FrameType=CAN_DATA_FRAME;
	tx.Data[0]=Data0;
	tx.Data[1]=Data1;
	tx.Data[2]=Data2;
	tx.Data[3]=Data3;
	tx.Data[4]=Data4;
	tx.Data[5]=Data5;
	tx.Data[6]=Data6;
	tx.Data[7]=Data7;
	CAN_SendFrame(tx);
}

void SendCAN11bit(uint16_t Arb, uint8_t Length, uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4, uint8_t Data5, uint8_t Data6, uint8_t Data7)
{
	CAN_Frame_t tx;

	tx.MessageID=Arb;
	tx.Length=Length;
	tx.FrameType=CAN_DATA_FRAME;
	tx.Data[0]=Data0;
	tx.Data[1]=Data1;
	tx.Data[2]=Data2;
	tx.Data[3]=Data3;
	tx.Data[4]=Data4;
	tx.Data[5]=Data5;
	tx.Data[6]=Data6;
	tx.Data[7]=Data7;
	CAN_SendFrame(tx);
}

// CAN ID = 0x10438040 (Arb 0x21C)
// Volume up = 0x01					0x9D/0x28 = Volume up
// Volume down = 0x02				0x9D/0xA8 = Volume down
// Up = 0x03						0x9D/0xD0 = Next
// Down = 0x04						0x9D/0x50 = Prev
// Source = 0x05					0x9D/0xC8 = Source
// Pick up/Voice command = 0x06		0x9D/0x49 = Phone answer
// Hang up/Mute = 0x07				0x9D/0x70 = Enter/Play/Pause

uint8_t CurrentGear=0;

void GearPos(void)
{
#define NUM_GEARS 7						// Number of gear ratios
	const float TireDiameter=28.6f;		// Wheel diameter in inches (28.6")
	const float FinalRatio=3.27f; 		// Rear end gear ratio (3.27)
	const float GearRatios[NUM_GEARS]=	// Gear ratios
	{
		3.67f,	// Rev, 3.67
		4.48f,	// 1st, 4.48
		2.58f,	// 2nd, 2.58
		1.63f,	// 3rd, 1.63
		1.19f,	// 4th, 1.19
		1.00f,	// 5th, 1.00
		0.75f	// 6th, 0.75
	};
	uint16_t ExpectRPM[NUM_GEARS], i;
	uint16_t LowRPM, HighRPM, RPM=RawRPM/4;
	float MPH=(float)RawSpeed/103.0f;

	// Calculate an array of expected RPMs for each gear ratio
	for(i=0;i<NUM_GEARS;i++)
		ExpectRPM[i]=(((GearRatios[i]*FinalRatio)*336.13f*MPH)/TireDiameter);

	for(i=0;i<NUM_GEARS;i++)
	{
		// Calculate high end of expected RPM range from "this" gear and the previous gear (higher expected RPM).
		if(i>0)
			HighRPM=ExpectRPM[i]+((ExpectRPM[i-1]-ExpectRPM[i])/3);
		else
			HighRPM=ExpectRPM[0];

		// Calculate low end of expected RPM range from "this" gear and the next gear (lower expected RPM).
		if(i<(NUM_GEARS-1))
			LowRPM=ExpectRPM[i]-((ExpectRPM[i]-ExpectRPM[i+1])/3);
		else
			LowRPM=ExpectRPM[NUM_GEARS-1];

		// If the actual RPM falls between this range, it's that gear
		if((RPM>LowRPM)&&(RPM<HighRPM))
			CurrentGear=i;
	}
}

int main(void)
{
	// LED/Steering wheel remote output
	PORTC_PCR5=PORT_PCR_MUX(0x01);
	GPIOC_PDDR|=BIT5;
	GPIOC_PCOR|=BIT5;

	// SPI Chip Select
	PORTD_PCR4=PORT_PCR_MUX(0x01);
	GPIOD_PDDR|=BIT4;
	GPIOD_PSOR|=BIT4;

	// SWCAN transceiver mode pins
	// Mode0	Mode1
	// L		L		= Sleep
	// H		L		= High speed mode
	// L		H		= High voltage wake up mode
	// H		H		= Normal mode
	
	// Mode 0
	PORTB_PCR16=PORT_PCR_MUX(1);
	GPIOB_PDDR|=BIT16;
	GPIOB_PSOR|=BIT16;

	// Mode 1
	PORTB_PCR17=PORT_PCR_MUX(1);
	GPIOB_PDDR|=BIT17;
	GPIOB_PSOR|=BIT17;

	// Initalize the SPI module
	SPI_Init();

	// Initalize the CANbus module
	CAN_Init();

	// Initalize USB CDC serial
//	CDC_Init();

	_displayOff();

	// Set initial time
	Time=SysTick;

	// Set text timeout to initial time + 2 seconds
	TextTimeout=SysTick+2000;
	// Set text display
	text=1;

	while(1)
	{
		if((SysTick-Time)>200)
		{
			Time=SysTick;

			// Calculate gear position
			GearPos();

			if(text)
			{
				// Keep displaying until 2 seconds have passed, then turn off text display
				if((TextTimeout-SysTick)<2000)
				{
					if(temp==0)
						_display_custom(0xA3, 0xCF, 0xC7, 0xFF, 0xC6, 0xA3, 0xA3, 0xC7 /* oIL CooL */);
					else if(temp==1)
						_display_custom(0xA3, 0xCF, 0xC7, 0x8C, 0xF8, 0xAF, 0x88, 0xAB /* oILPTrAn */);
					else if(temp==2)
						_display_custom(0x91, 0x88, 0xD5, 0xFF, 0x92, 0xF8, 0x86, 0xAF /* YAW STEr */);
					else if(temp==3)
						_display_custom(0xFF, 0xFF, 0xFF, 0xFF, 0x8E, 0xE3, 0x86, 0xC7 /* FuEL     */);
					else if(temp==4)
						_display_custom(0xA3, 0xE3, 0xF8, 0x92, 0xCF, 0xA1, 0x86, 0xFF /* OuTSIdE  */);
					else if(temp==5)
						_display_custom(0xCF, 0x88, 0xF8, 0xFF, 0x90, 0x86, 0x88, 0xAF /* IAT gEAr */);
				}
				else
					text=0;
			}
			else
			{
				if(temp==0)
				{
					_display(units?(int)(RawOilTemp-40):(int)((RawOilTemp-40)*1.8f+32.0f), 1, 0, units?0xC:0xF);
					_display(units?(int)(RawCoolantTemp-40):(int)((RawCoolantTemp-40)*1.8f+32.0f), 0, 0, units?0xC:0xF);
				}
				else if(temp==1)
				{
					_display(units?(int)(RawOilPress*4.0f):(int)(RawOilPress*5.8f), 1, 2, 0);
					_display(units?(int)(RawTransTemp-40):(int)((RawTransTemp-40)*1.8f+32.0f), 0, 0, units?0xC:0xF);
				}
				else if(temp==2)
				{
					_display(RawYawRate, 1, 0, 0);
					_display(RawSteeringAngle/16, 0, 0, 0);
				}
				else if(temp==3)
				{
					_display((int)((RawFuelLevel*0.3921568627451f)*10.0f), 0, 2, 0);
				}
				else if(temp==4)
				{
					_display(units?(int)(RawOutside1-40):(int)((RawOutside1-40)*1.8f+32.0f), 1, 0, units?0xC:0xF);
					_display(units?(int)(RawOutside2-40):(int)((RawOutside2-40)*1.8f+32.0f), 0, 0, units?0xC:0xF);
				}
				else if(temp==5)
				{
					_display(units?(int)(RawIntakeTemp-40):(int)((RawIntakeTemp-40)*1.8f+32.0f), 1, 0, units?0xC:0xF);
					_display(CurrentGear, 0, 0, 0);
				}
			}

			memset(String, 0, 64);
			sprintf(String, "Gear: %d\nIAT: %d %c", CurrentGear, units?(int)(RawIntakeTemp-40):(int)((RawIntakeTemp-40)*1.8f+32.0f), units?'C':'F');

			SendGMLAN(0x4, 0x57B, 0x000, 4, 0x00, 0x04, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00);
			SendGMLAN(0x4, 0x592, 0x000, 8, 0x10, 6+7+7+7, 0x02, 0x01, 0x16, 0x02, 0x00, String[0]);
			SendGMLAN(0x4, 0x592, 0x000, 8, 0x21, String[1], String[2], String[3], String[4], String[5], String[6], String[7]);
			SendGMLAN(0x4, 0x592, 0x000, 8, 0x22, String[8], String[9], String[10], String[11], String[12], String[13], String[14]);
			SendGMLAN(0x4, 0x592, 0x000, 8, 0x23, String[15], String[16], String[17], String[18], String[19], String[20], String[21]);
		}

		if(Status&&lock==0)
		{
			lock=1;

			if(		Status==0x01)	// Volume up (Volume up)
				sendNEC(0x9D, 0x28);
			else if(Status==0x02)	// Volume down (Volume down)
				sendNEC(0x9D, 0xA8);
			else if(Status==0x03)	// Up (Next)
				sendNEC(0x9D, 0xD0);
			else if(Status==0x04)	// Down (Prev)
				sendNEC(0x9D, 0x50);
			else if(Status==0x05)	// Source (Source)
				sendNEC(0x9D, 0xC8);
			else if(Status==0x06)	// Pick up/Voice command (Phone answer)
				sendNEC(0x9D, 0x49);
			else if(Status==0x07)	// Hang up/Mute (Enter/Play/Pause)
				sendNEC(0x9D, 0x70);
		}

		if(Status==0x00)
			lock=0;
	}

	return 0;
}

//char buffer[128];

void can_isr(void)
{
	uint32_t status=CAN0_IFLAG1, i=0;
	CAN_Frame_t rx;

	// Check all message buffers
	for(i=0;i<16;i++)
	{
		// Skip over any that didn't trigger the IRQ
		if((status&(1UL<<i))==0)
			continue;

		// Read in CAN frame
		CAN_ReadFrame(&rx, i);

		rx.MessageID&=~CAN_MESSAGE_ID_EXT;	// Remove the extended ID check bit
		rx.MessageID&=~GMLAN_ARB_MASK;		// Mask out anything that isn't the GMLAN ARB ID
		rx.MessageID>>=GMLAN_ARB_SHIFT;		// Shift down the GMLAN ARB ID

		// 0x108 - Trip status? (contains current speed)
		if(rx.MessageID==0x108)
			RawSpeed=(((rx.Data[0]&0xFF)<<8)|(rx.Data[1]&0xFF));

		// 0x212 - Ambient air temperature
		if(rx.MessageID==0x212)
		{
			RawOutside1=rx.Data[1];
			RawOutside2=rx.Data[2];
		}

		// 0x400 - Fuel consumption
		if(rx.MessageID==0x400)
			RawFuelLevel=rx.Data[1];

		// 0x120 - Traction/Anti-slip control status
		if(rx.MessageID==0x120)
		{
			RawSteeringAngle=(int16_t)(((rx.Data[4]&0xFF)<<8)|(rx.Data[5]&0xFF));
			RawYawRate=(int8_t)rx.Data[7];
		}

		// 0x165 - Engine status 1
		if(rx.MessageID==0x165)
		{
			RawRPM=(((rx.Data[2]&0xFF)<<8)|(rx.Data[3]&0xFF));
		}

		// 0x170 - Engine status 2
		if(rx.MessageID==0x170)
		{
			RawOilPress=rx.Data[1];
			RawTransTemp=rx.Data[3];
			RawOilTemp=rx.Data[5];
			RawIntakeTemp=rx.Data[6];
			RawCoolantTemp=rx.Data[7];
		}

		// 0x425 - Cluster units selection
		if(rx.MessageID==0x425)
		{
			if(rx.Data[0]==0x05)
				units=0; // Switch to US units
			else
				units=1; // Switch to Metric units
		}

		// 0x392 - Heated seat push buttons
		if(rx.MessageID==0x392)
		{
			if(rx.Data[0]==0x01)	// Driver seat
			{
				SendGMLAN(0x4, 0x200, 0x000, 5, 0x84, 0, 1, 50, 0, 0, 0, 0); // Make a beep
				temp--; // Switch to next 'screen'
				text=1; // Trigger text display
				TextTimeout=SysTick+2000; // Text display timeout, 2 seconds

				if(temp<0)
					temp=0;
			}

			if(rx.Data[0]==0x08)	// Passenger seat
			{
				SendGMLAN(0x4, 0x200, 0x000, 5, 0x84, 0, 1, 50, 0, 0, 0, 0); // Make a beep
				temp++; // Switch to previous'screen'
				text=1; // Trigger text display
				TextTimeout=SysTick+2000; // Text display timeout, 2 seconds

				if(temp>5)
					temp=5;
			}
		}

		// 0x21C - Steering wheel controls
		if(rx.MessageID==0x21C)
			Status=rx.Data[0];

#if 0
		// For debugging and logging CAN messages
		sprintf(buffer, "%d %x %d %x %x %x %x %x %x %x %x\r\n",
				SysTick,
				rx.MessageID,
				rx.Length,
				rx.Data[0], rx.Data[1], rx.Data[2], rx.Data[3],
				rx.Data[4], rx.Data[5], rx.Data[6], rx.Data[7]);
		CDC_SendString(buffer);
#endif

		// Clear IRQ flag
		CAN0_IFLAG1|=1<<i;
	}
}

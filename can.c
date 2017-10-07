/*
	Copyright 2017 Matt Williams
	K20 CANBUS driver implementation
	Fully interrupt driven, makes use of all MBs
*/

#include "mk20d7.h"
#include "can.h"

#define CAN_MB_RX_INACTIVE			0x0			/**< MB is not active */
#define CAN_MB_RX_BUSY				0x1			/**< FlexCAN is updating the contents of the MB */
#define CAN_MB_RX_FULL				0x2			/**< MB is full */
#define CAN_MB_RX_EMPTY				0x4			/**< MB is active and empty */
#define CAN_MB_RX_OVERRUN			0x6			/**< MB is being overwritten into a full buffer */
#define CAN_MB_RX_RANSWER			0xA			/**< A frame was configured to recognize a Remote Request Frame and transmit a ResponseFrame in return */

#define CAN_MB_TX_INACTIVE			0x8			/**< MB is not active */
#define CAN_MB_TX_ABORT				0x9			/**< MB is aborted */
#define CAN_MB_TX_ONCE				0xC			/**< MB is Tx once */
#define CAN_MB_TX_TANSWER			0xA			/**< MB is a Tx Response Frame from an incoming Remote Request Frame */

inline void NVIC_EnableIRQ(uint8_t IRQ, uint8_t Priority);

void CAN_Init(void)
{
	int i;

	PORTA_PCR12=PORT_PCR_MUX(0x02);										// Set PTA12 to alternative MUX (CAN module Tx)
	PORTA_PCR13=PORT_PCR_MUX(0x02)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;	// Set PTA13 to alternative MUX (CAN module Rx, with pullup for Melexis SWCAN transciever)

	SIM_SCGC6|=SIM_SCGC6_FLEXCAN0_MASK;

	CAN0_MCR|=CAN_MCR_MDIS_MASK;	// Disable CAN module

	OSC_CR|=OSC_CR_ERCLKEN_MASK;			// Enable external oscillator
	CAN0_CTRL1&=~CAN_CTRL1_CLKSRC_MASK;		// Select external clock

	// Set up interrupt priority and enable it
	NVIC_EnableIRQ(INT_CAN0_ORed_Message_buffer, 0x80);

	CAN0_MCR&=~CAN_MCR_MDIS_MASK;	// Enable module
	CAN0_MCR|=CAN_MCR_SOFTRST_MASK;	// Soft reset

	while((CAN0_MCR&CAN_MCR_SOFTRST_MASK)==0x02000000);	// Wait for soft reset
	while(!(CAN0_MCR&CAN_MCR_FRZACK_MASK));				// Wait to enter freeze mode

	CAN0_MCR|=CAN_MCR_SRXDIS_MASK;	// Disable self reception mode
	CAN0_MCR|=CAN_MCR_IRMQ_MASK;	// Enable per-mailbox filtering
	CAN0_MCR|=CAN_MCR_MAXMB(0x0F);	// Max message boxes

//	CAN0_CTRL1=CAN_CTRL1_PRESDIV(0x01)|CAN_CTRL1_PSEG1(0x02)|CAN_CTRL1_PSEG2(0x03)|CAN_CTRL1_PROPSEG(0x07); // 500kbps (Processor Expert generated)

//	CAN0_CTRL1=CAN_CTRL1_PRESDIV(0x03)|CAN_CTRL1_PSEG1(0x02)|CAN_CTRL1_PSEG2(0x03)|CAN_CTRL1_PROPSEG(0x07); // 250kbps (Processor Expert generated)

//	CAN0_CTRL1=CAN_CTRL1_PRESDIV(0x03)|CAN_CTRL1_PSEG1(0x02)|CAN_CTRL1_PSEG2(0x07)|CAN_CTRL1_PROPSEG(0x07); // 200kbps (Processor Expert generated)

//	CAN0_CTRL1=CAN_CTRL1_PRESDIV(0x07)|CAN_CTRL1_PSEG1(0x07)|CAN_CTRL1_PSEG2(0x01)|CAN_CTRL1_PROPSEG(0x04); // 125kbps (Processor Expert generated)

	CAN0_CTRL1=CAN_CTRL1_PRESDIV(0x17)|CAN_CTRL1_PSEG1(0x02)|CAN_CTRL1_PSEG2(0x07)|CAN_CTRL1_PROPSEG(0x07); // 33.333kbps (Processor Expert generated)

	CAN0_CTRL1&=~CAN_CTRL1_LOM_MASK;		// Disable listen only mode
	CAN0_CTRL1&=~CAN_CTRL1_LPB_MASK;		// Disable loopback mode

	CAN0_IMASK1=CAN_IMASK1_BUFLM(0xFFFF);	// Enable Tx/Rx interrupt for message buffers

	// Split up MBs

	// 0-4 are RX extended IDs
	for(i=0;i<4;i++)
	{
		CAN0_RXIMR(i)=0;

		CAN0_ID(i)=CAN_ID_EXT_MASK;
		CAN0_CS(i)=CAN_CS_CODE(CAN_MB_RX_EMPTY)|CAN_CS_IDE_MASK;
	}

	// 5-10 are RX standard IDs
	for(i=5;i<10;i++)
	{
		CAN0_RXIMR(i)=0;

		CAN0_ID(i)=CAN_ID_STD_MASK;
		CAN0_CS(i)=CAN_CS_CODE(CAN_MB_RX_EMPTY);
	}

	// 11-15 are TX
	for(i=11;i<16;i++)
		CAN0_CS(i)=CAN_CS_CODE(CAN_MB_TX_INACTIVE);

	CAN0_MCR&=~CAN_MCR_HALT_MASK;			// Exit freeze mode

	while(CAN0_MCR&CAN_MCR_FRZACK_MASK);	// Wait to exit freeze mode
	while(CAN0_MCR&CAN_MCR_NOTRDY_MASK);	// Wait for ready
}

int CAN_ReadFrame(CAN_Frame_t *Frame, uint8_t index)
{
	uint32_t RxMBCode=(CAN0_CS(index)&CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT;
	uint8_t i;

	// Read CAN frame if the MB is full, or got overwritten
	if(RxMBCode==CAN_MB_RX_FULL||RxMBCode==CAN_MB_RX_OVERRUN)
	{
		CAN0_CS(index)&=~CAN_CS_CODE_MASK;					// Clear CS code bits
		CAN0_CS(index)|=CAN_CS_CODE(CAN_MB_RX_INACTIVE);	// Hold message buffer by setting it inactive

		Frame->Length=(CAN0_CS(index)&CAN_CS_DLC_MASK)>>CAN_CS_DLC_SHIFT;	// Get frame data length

		if(CAN0_CS(index)&CAN_CS_IDE_MASK)	// Check if extended ID bit is set
			Frame->MessageID=(CAN0_ID(index)&(CAN_ID_STD_MASK|CAN_ID_EXT_MASK))|CAN_MESSAGE_ID_EXT; // Get extended ID
		else
			Frame->MessageID=(CAN0_ID(index)&CAN_ID_STD_MASK)>>CAN_ID_STD_SHIFT;	// Get standard ID

		// If it's not a remote frame, it should have data
		if(Frame->FrameType!=CAN_REMOTE_FRAME)
		{
			// Get first word of frame data bytes
			Frame->Data[0]=CAN0_WORD0(index)>>24;
			Frame->Data[1]=CAN0_WORD0(index)>>16;
			Frame->Data[2]=CAN0_WORD0(index)>>8;
			Frame->Data[3]=CAN0_WORD0(index);

			// Don't bother with second word if the length isn't long enough
			if(Frame->Length>3)
			{
				// Otherwise, get second word bytes
				Frame->Data[4]=CAN0_WORD1(index)>>24;
				Frame->Data[5]=CAN0_WORD1(index)>>16;
				Frame->Data[6]=CAN0_WORD1(index)>>8;
				Frame->Data[7]=CAN0_WORD1(index);
			}

			// Set unused bytes to 0
			for(i=Frame->Length;i<8;i++)
				Frame->Data[i]=0x00;
		}

		// Set empty MB and reset filters
		if(index<5)
		{
			// Extended IDs
			CAN0_RXIMR(index)=0;

			CAN0_ID(index)=CAN_ID_EXT_MASK;
			CAN0_CS(index)=CAN_CS_CODE(CAN_MB_RX_EMPTY)|CAN_CS_IDE_MASK;
		}
		else
		{
			// Standard IDs
			CAN0_RXIMR(index)=0;

			CAN0_ID(index)=CAN_ID_STD_MASK;
			CAN0_CS(index)=CAN_CS_CODE(CAN_MB_RX_EMPTY);
		}
	}

	return index;
}

void CAN_SendFrame(CAN_Frame_t Frame)
{
	int index=-1, i;

	// Find first inactive MB to transmit on
	for(i=11;i<16;i++)
	{
		if(((CAN0_CS(i)&CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT)==CAN_MB_TX_INACTIVE)
		{
			index=i;
			break;
		}
	}

	// Didn't find one
	if(index<0)
		return;

	// Hold the inactive MB
	CAN0_CS(index)=CAN_CS_CODE(CAN_MB_TX_INACTIVE);

	// Set the message frame ID
	if((Frame.MessageID&CAN_MESSAGE_ID_EXT)!=0x00)
	{
		// Extended ID
		CAN0_ID(index)&=~(CAN_ID_STD_MASK|CAN_ID_EXT_MASK);		// Clear ID bitfield
		CAN0_ID(index)|=(Frame.MessageID&~CAN_MESSAGE_ID_EXT);	// Set ID bits

		CAN0_CS(index)|=CAN_CS_IDE_MASK;						// Set extended ID bit
	}
	else
	{
		// Standard ID
		CAN0_ID(index)&=~CAN_ID_STD_MASK;						// Clear ID bitfield
		CAN0_ID(index)|=(Frame.MessageID<<CAN_ID_STD_SHIFT);	// Set ID bits

		CAN0_CS(index)&=~CAN_CS_IDE_MASK;						// Clear extended ID bit
	}

	// Set first frame data word
	CAN0_WORD0(index) =Frame.Data[0]<<24;
	CAN0_WORD0(index)|=Frame.Data[1]<<16;
	CAN0_WORD0(index)|=Frame.Data[2]<<8;
	CAN0_WORD0(index)|=Frame.Data[3];

	// Only set second word if the length is long enough
	if(Frame.Length>3)
	{
		// Otherwise, set them
		CAN0_WORD1(index) =Frame.Data[4]<<24;
		CAN0_WORD1(index)|=Frame.Data[5]<<16;
		CAN0_WORD1(index)|=Frame.Data[6]<<8;
		CAN0_WORD1(index)|=Frame.Data[7];
	}

	// Set message frame data length
	CAN0_CS(index)&=~CAN_CS_DLC_MASK;

	// Set message to transmit once
	CAN0_CS(index)|=(Frame.Length<<CAN_CS_DLC_SHIFT);
	CAN0_CS(index)|=CAN_CS_CODE(CAN_MB_TX_ONCE);
}

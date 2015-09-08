// File: MAX3421E.c
// Author: Jaka Kordez
// Version: 0.1

#include "MAX3421E.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define BYTE uint8_t
#define WORD uint16_t

// Set transfer bounds
#define NAK_LIMIT 200
#define RETRY_LIMIT 3

extern UART_HandleTypeDef huart1;

// Global variables
static BYTE XfrData[2000];      // Big array to handle max size descriptor data
static BYTE maxPacketSize;		// discovered and filled in by Get_Descriptor-device request in enumerate_device()
static WORD VID,PID,nak_count,IN_nak_count,HS_nak_count;
static unsigned int last_transfer_size;	
unsigned volatile long timeval;			// incremented by timer0 ISR
WORD inhibit_send;

SPI_HandleTypeDef hspi5;
uint8_t dataOut[80];
uint8_t dataIn[5];

static void MX_SPI5_Init(void);
void Hwreg(uint8_t reg, uint8_t val);
uint8_t Hrreg(uint8_t reg);
void MAX_Reset(void);
void detect_device(void);
void waitframes(uint8_t num);
void enumerate_device(void);
BYTE CTL_Write_ND(BYTE *pSUD);
BYTE print_error(BYTE err);
BYTE CTL_Read(BYTE *pSUD);
BYTE Send_Packet(BYTE token,BYTE endpoint);
BYTE IN_Transfer(BYTE endpoint,WORD INbytes);

void printfUART(uint8_t *text, ...){
	HAL_UART_Transmit(&huart1, text, strlen((const char *)text), 1000);
}

void Hwreg(uint8_t reg, uint8_t val){
	int delay = 50;
	dataOut[0] = reg | 2;
	dataOut[1] = val;
	MAX_CS_PORT->BSRR = (uint32_t)MAX_CS_PIN << 16;
	HAL_SPI_Transmit(&hspi5, dataOut, 2, 1000);
	MAX_CS_PORT->BSRR = MAX_CS_PIN;
	while(delay--);
}

uint8_t Hrreg(uint8_t reg){
	int delay = 50;
	dataOut[0] = reg;
	dataOut[1] = 0;
	MAX_CS_PORT->BSRR = (uint32_t)MAX_CS_PIN << 16;
	HAL_SPI_TransmitReceive(&hspi5, dataOut, dataIn, 2, 1000);
	MAX_CS_PORT->BSRR = MAX_CS_PIN;
	while(delay--);
	return dataIn[1];
}

void Hwritebytes(BYTE reg, BYTE N, BYTE *p)
{
	int delay = 50;
	dataOut[0] = reg+0x02;			// command byte into the FIFO. 0x0002 is the write bit
	for(int j = 0; j < N; j++)
	{
		dataOut[j+1] = *p++;				// send the next data byte
	}
	MAX_CS_PORT->BSRR = (uint32_t)MAX_CS_PIN << 16;
	HAL_SPI_Transmit(&hspi5, dataOut, N+1, 1000);
	MAX_CS_PORT->BSRR = MAX_CS_PIN;
	while(delay--);
}

/* SPI5 init function */
void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi5.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi5);

}

void MAX_Init(){
	MX_SPI5_Init();
	__HAL_SPI_ENABLE(&hspi5);
	
	dataOut[0] = (17<<3) | 2;
	dataOut[1] = 0x10;
	GPIOF->BSRR = (uint32_t)GPIO_PIN_6 << 16;
	HAL_SPI_Transmit(&hspi5, dataOut, 2, 1000);
	GPIOF->BSRR = GPIO_PIN_6;
	Hwreg(rUSBIEN, bmOSCOKIE);
	Hrreg(rUSBIEN);
	Hrreg(18<<3);
	MAX_Reset();
	Hwreg(rIOPINS1,0x00);		// seven-segs off
	Hwreg(rIOPINS2,0x00);		// and Vbus OFF (in case something already plugged in)
	HAL_Delay(200);
	VBUS_ON;
	
}

void MAX_Process(void){
	detect_device();
	waitframes(200); 			// Some devices require this
  enumerate_device();
}

void detect_device(void)
{
	int busstate;
	// Activate HOST mode & turn on the 15K pulldown resistors on D+ and D-
	Hwreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmHOST)); // Note--initially set up as a FS host (LOWSPEED=0)
	Hwreg(rHIRQ,bmCONDETIRQ);  // clear the connection detect IRQ

	do 		// See if anything is plugged in. If not, hang until something plugs in
	{
		Hwreg(rHCTL,bmSAMPLEBUS);			// update the JSTATUS and KSTATUS bits
		busstate = Hrreg(rHRSL);			// read them
		busstate &= (bmJSTATUS|bmKSTATUS);	// check for either of them high	
	} 
	while (busstate==0);
	if (busstate==bmJSTATUS)    // since we're set to FS, J-state means D+ high
	{
		Hwreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmHOST|bmSOFKAENAB));  // make the MAX3421E a full speed host
		printfUART("Full-Speed Device Detected\n");
	}
	if (busstate==bmKSTATUS)  // K-state means D- high
	{
		Hwreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmHOST|bmLOWSPEED|bmSOFKAENAB));  // make the MAX3421E a low speed host        
		printfUART("Low-Speed Device Detected\n");
	}
}

void enumerate_device(void)
{
	static BYTE HR,iCONFIG,iMFG,iPROD,iSERIAL;
	static WORD TotalLen,ix;
	static BYTE len,type,adr,pktsize;
	// SETUP bytes for the requests we'll send to the device
	static BYTE Set_Address_to_7[8]      		= {0x00,0x05,0x07,0x00,0x00,0x00,0x00,0x00};
	static BYTE Get_Descriptor_Device[8]  		= {0x80,0x06,0x00,0x01,0x00,0x00,0x00,0x00}; // code fills in length field
	static BYTE Get_Descriptor_Config[8]  		= {0x80,0x06,0x00,0x02,0x00,0x00,0x00,0x00};
	static BYTE str[8] = {0x80,0x06,0x00,0x03,0x00,0x00,0x40,0x00};	// Get_Descriptor-String template. Code fills in idx at str[2].

	// Issue a USB bus reset
	printfUART("Issuing USB bus reset\n");
	Hwreg(rHCTL,bmBUSRST);           // initiate the 50 msec bus reset
	while(Hrreg(rHCTL) & bmBUSRST);  // Wait for the bus reset to complete

	// Wait some frames before programming any transfers. This allows the device to recover from
	// the bus reset.
	waitframes(200);  

	// Get the device descriptor.
	maxPacketSize = 8;				// only safe value until we find out
	Hwreg(rPERADDR,0);   			// First request goes to address 0
	Get_Descriptor_Device[6]=8;		// wLengthL
	Get_Descriptor_Device[7]=0;		// wLengthH
	printfUART("First 8 bytes of Device Descriptor ");
	HR = CTL_Read(Get_Descriptor_Device);  	// Get device descriptor into XfrData[]
	if(print_error(HR)) return;	// print_error() does nothing if HRSL=0, returns the 4-bit HRSL.

	printfUART("(%u/%u NAKS)\n",IN_nak_count,HS_nak_count);		// Show NAK count for data stage/status stage
	maxPacketSize = XfrData[7];
	for (ix=0; ix<last_transfer_size;ix++)
		//printfUART("%02X ",(BYTE*)XfrData[ix]);
	printfUART("\n");
	printfUART("EP0 maxPacketSize is %02u bytes\n",maxPacketSize);

	// Issue another USB bus reset
	printfUART("Issuing USB bus reset\n");
	Hwreg(rHCTL,bmBUSRST);           // initiate the 50 msec bus reset
	while(Hrreg(rHCTL) & bmBUSRST);  // Wait for the bus reset to complete
	waitframes(200);

	// Set_Address to 7 (Note: this request goes to address 0, already set in PERADDR register).
	printfUART("Setting address to 0x07\n");
	HR = CTL_Write_ND(Set_Address_to_7);   // CTL-Write, no data stage
	if(print_error(HR)) return;

	waitframes(30);           // Device gets 2 msec recovery time
	Hwreg(rPERADDR,7);       // now all transfers go to addr 7

	// Get the device descriptor at the assigned address. 
	Get_Descriptor_Device[6]=0x12;			// fill in the real device descriptor length
	printfUART("\nDevice Descriptor ");
	HR = CTL_Read(Get_Descriptor_Device);  // Get device descriptor into XfrData[]
	if(print_error(HR)) return;
	printfUART("(%u/%u NAKS)\n",IN_nak_count,HS_nak_count);
	printf  ("-----------------\n");

	VID 	= XfrData[8] + 256*XfrData[9];
	PID 	= XfrData[10]+ 256*XfrData[11];
	iMFG 	= XfrData[14];
	iPROD 	= XfrData[15];
	iSERIAL	= XfrData[16];

		for (ix=0; ix<last_transfer_size;ix++)
			//printfUART("%02X ",(BYTE*)XfrData[ix]);
	printfUART("\n");
	printfUART("This device has %u configuration\n",XfrData[17]);
	printfUART("Vendor  ID is 0x%04X\n",VID);
	printfUART("Product ID is 0x%04X\n",PID);
	//
	str[2]=0;	// index 0 is language ID string
	str[4]=0;	// lang ID is 0
	str[5]=0;
	str[6]=4;	// wLengthL
	str[7]=0;	// wLengthH

	HR = CTL_Read(str);  	// Get lang ID string
	if (!HR)				// Check for ACK (could be a STALL if the device has no strings)
		{
		printfUART("\nLanguage ID String Descriptor is ");
		for (ix=0; ix<last_transfer_size;ix++)
			//printfUART("%02X ",(BYTE*)XfrData[ix]);
		str[4]=XfrData[2]; 	// LangID-L
		str[5]=XfrData[3];	// LangID-H
		str[6]=255; 		// now request a really big string
		}
	if(iMFG)
		{
		str[2]=iMFG; 			// fill in the string index from the device descriptor
		HR = CTL_Read(str);  	// Get Manufacturer ID string
		printfUART("\nManuf. string is  \"");
		for (ix=2; ix<last_transfer_size;ix+=2)
			//printfUART("%c",(BYTE*)XfrData[ix]);
		printfUART("\"\n");
		}
	else printfUART("There is no Manuf. string\n");

	if(iPROD)
		{
		str[2]=iPROD;
		HR = CTL_Read(str);  // Get Product ID string
		printfUART("Product string is \"");
		for (ix=2; ix<last_transfer_size;ix+=2)
			//printfUART("%c",(BYTE*)XfrData[ix]);
		printfUART("\"\n");
		}
	else printfUART("There is no Product string\n");

	if(iSERIAL)
		{
		str[2]=iSERIAL;
		HR = CTL_Read(str);  // Get Serial Number ID string
		printfUART("S/N string is     \"");
		for (ix=2; ix<last_transfer_size;ix+=2)
			//printfUART("%c",(BYTE*)XfrData[ix]);
		printfUART("\"\n");
		}
	else printfUART("There is no Serial Number");

	// Get the 9-byte configuration descriptor

	printfUART("\n\nConfiguration Descriptor ");
	Get_Descriptor_Config[6]=9;	// fill in the wLengthL field
	Get_Descriptor_Config[7]=0;	// fill in the wLengthH	field

	HR = CTL_Read(Get_Descriptor_Config);  // Get config descriptor into XfrData[]
	if(print_error(HR)) return;
	printfUART("(%u/%u NAKS)\n",IN_nak_count,HS_nak_count);
	printf  ("------------------------\n");

	for (ix=0; ix<last_transfer_size;ix++)
		//printfUART("%02X ",(BYTE*)XfrData[ix]);

	// Now that the full length of all descriptors (Config, Interface, Endpoint, maybe Class)
	// is known we can fill in the correct length and ask for the full boat.

	Get_Descriptor_Config[6]=XfrData[2];	// LengthL
	Get_Descriptor_Config[7]=XfrData[3];	// LengthH
	HR = CTL_Read(Get_Descriptor_Config);  // Get config descriptor into XfrData[]

	printfUART("\nFull Configuration Data");
	for (ix=0; ix<last_transfer_size;ix++)
		{
		if((ix&0x0F)==0) printfUART("\n");		// CR every 16 numbers
		//printfUART("%02X ",(BYTE*)XfrData[ix]);
		}
	iCONFIG = XfrData[6];	// optional configuration string

	printfUART("\nConfiguration %01X has %01X interface",XfrData[5],XfrData[4]);
	if (XfrData[4]>1) printfUART("s");
	printfUART("\nThis device is ");
	if(XfrData[7] & 0x40)	printfUART("self-powered\n");
	else					printfUART("bus powered and uses %03u milliamps\n",XfrData[8]*2);
	//
	// Parse the config+ data for interfaces and endpoints. Skip over everything but
	// interface and endpoint descriptors. 
	//
	TotalLen=last_transfer_size;
	ix=0;
		do
		{
		len=XfrData[ix];		// length of first descriptor (the CONFIG descriptor)
		type=XfrData[ix+1];
		adr=XfrData[ix+2];
		pktsize=XfrData[ix+4];

		if(type==0x04)			// Interface descriptor?
			printfUART("Interface %u, Alternate Setting %u has:\n",XfrData[ix+2],XfrData[ix+3]);
		else if(type==0x05)		// check for endpoint descriptor type
			{
			printfUART("--Endpoint %u",(adr&0x0F));
			if (XfrData[ix+2]&0x80) printfUART("-IN  ");
			else printfUART("-OUT ");
			printfUART("(%02u) is type ",(BYTE)pktsize);

			switch(XfrData[ix+3]&0x03)
				{
				case 0x00:
				printfUART("CONTROL\n"); break;
				case 0x01:
				printfUART("ISOCHRONOUS\n"); break;
				case 0x02:
				printfUART("BULK\n"); break;
				case 0x03:
				printfUART("INTERRUPT with a polling interval of %u msec.\n",XfrData[ix+6]);
				}
			}
		ix += len;				// point to next descriptor
		}
		while (ix<TotalLen);
	//
	if(iCONFIG)
		{
		str[2]=iCONFIG;
		HR = CTL_Read(str);  // Get Config string
		printfUART("\nConfig string is \"");
		for (ix=2; ix<last_transfer_size;ix+=2)
			//printfUART("%c",(BYTE*)XfrData[ix]);
		printfUART("\"\n");
		}
	else printfUART("There is no Config string\n");

}

// ----------------------------------------------------
// CONTROL-Read Transfer. Get the length from SUD[7:6].
// ----------------------------------------------------
BYTE CTL_Read(BYTE *pSUD)
{
  BYTE  resultcode;
  WORD	bytes_to_read;
  bytes_to_read = pSUD[6] + 256*pSUD[7];

// SETUP packet
  Hwritebytes(rSUDFIFO,8,pSUD);      		// Load the Setup data FIFO
  resultcode=Send_Packet(tokSETUP,0);   	// SETUP packet to EP0
  if (resultcode) return (resultcode); 		// should be 0, indicating ACK. Else return error code.
// One or more IN packets (may be a multi-packet transfer)
  Hwreg(rHCTL,bmRCVTOG1);            		// FIRST Data packet in a CTL transfer uses DATA1 toggle.
//  last_transfer_size = IN_Transfer(0,bytes_to_read);     // In transfer to EP-0 (IN_Transfer function handles multiple packets)
  resultcode = IN_Transfer(0,bytes_to_read); 
  if(resultcode) return (resultcode);

  IN_nak_count=nak_count;
// The OUT status stage
  resultcode=Send_Packet(tokOUTHS,0);
  if (resultcode) return (resultcode);   // should be 0, indicating ACK. Else return error code. 
  return(0);    // success!
}

// -----------------------------------------------------------------------------------
// IN Transfer to arbitrary endpoint. Handles multiple packets if necessary. Transfers
// "length" bytes.
// -----------------------------------------------------------------------------------
// Do an IN transfer to 'endpoint'. Keep sending INS and saving concatenated packet data 
// in array Xfr_Data[] until 'numbytes' bytes are received. If no errors, returns total
// number of bytes read. If any errors, returns a byte with the MSB set and the 7 LSB
// indicating the error code from the "launch transfer" function. 
//
BYTE IN_Transfer(BYTE endpoint,WORD INbytes)
{
BYTE resultcode,j;
BYTE pktsize;
unsigned int xfrlen,xfrsize;

xfrsize = INbytes;
xfrlen = 0;

while(1) // use a 'return' to exit this loop.
  {
  resultcode=Send_Packet(tokIN,endpoint);     	// IN packet to EP-'endpoint'. Function takes care of NAKS.
  if (resultcode) return (resultcode);  		// should be 0, indicating ACK. Else return error code.  
  pktsize=Hrreg(rRCVBC);                        // number of received bytes
  for(j=0; j<pktsize; j++)                      // add this packet's data to XfrData array
      XfrData[j+xfrlen] = Hrreg(rRCVFIFO);         
  Hwreg(rHIRQ,bmRCVDAVIRQ);                     // Clear the IRQ & free the buffer
  xfrlen += pktsize;                            // add this packet's byte count to total transfer length
//
// The transfer is complete under two conditions:
// 1. The device sent a short packet (L.T. maxPacketSize)
// 2. 'INbytes' have been transferred.
//
  if ((pktsize < maxPacketSize) || (xfrlen >= xfrsize))    // have we transferred 'length' bytes?
// 	 return xfrlen;
		{
		last_transfer_size = xfrlen;
		return(resultcode);
		}
  }
}

BYTE CTL_Write_ND(BYTE *pSUD)
{
  BYTE resultcode;
  Hwritebytes(rSUDFIFO,8,pSUD);
// 1. Send the SETUP token and 8 setup bytes. Device should immediately ACK.
  resultcode=Send_Packet(tokSETUP,0);    // SETUP packet to EP0
  if (resultcode) return (resultcode);   // should be 0, indicating ACK.

// 2. No data stage, so the last operation is to send an IN token to the peripheral
// as the STATUS (handshake) stage of this control transfer. We should get NAK or the
// DATA1 PID. When we get the DATA1 PID the 3421 automatically sends the closing ACK.
  resultcode=Send_Packet(tokINHS,0);   // This function takes care of NAK retries.
  if(resultcode) return (resultcode);  // should be 0, indicating ACK.
  else  return(0);
}

void MAX_Reset(void)
{
	Hwreg(rUSBCTL,bmCHIPRES);  // chip reset This stops the oscillator
	Hwreg(rUSBCTL,0x00);       // remove the reset
	while(!(Hrreg(rUSBIRQ) & bmOSCOKIRQ)) ;  // hang until the PLL stabilizes
}

void waitframes(BYTE num)
{
	BYTE k;
	Hwreg(rHIRQ,bmFRAMEIRQ);     // clear any pending 
	k=0;
	while(k!=num)               // do this at least once
	{
		//HAL_Delay(1);
		while(!(Hrreg(rHIRQ)& bmFRAMEIRQ));
		Hwreg(rHIRQ,bmFRAMEIRQ); // clear the IRQ
		k++;
	}
}

// -----------------------------------------------------------------------------------
// Send a packet.
// ENTRY: PERADDR preset to the peripheral address.
// EXIT: Result code. 0 indicates success.
// 1. Launch the packet, wait for the host IRQ, reset the host IRQ.
// 2. Examine the result code.
//    If NAK, re-send the packet up to NAK_LIMIT times.
//    If bus timeout (no response), re-send packet up to RETRY_LIMIT times.
//    Otherwise, return the result code: 0=success, nonzero=error condition.
// -----------------------------------------------------------------------------------
BYTE Send_Packet(BYTE token,BYTE endpoint)
{
BYTE resultcode,retry_count;
retry_count = 0;
nak_count = 0;
//
while(1) 	// If the response is NAK or timeout, keep sending until either NAK_LIMIT or RETRY_LIMIT is reached.
			// Returns the HRSL code.
  {                                     
  Hwreg(rHXFR,(token|endpoint));         // launch the transfer
  while(!(Hrreg(rHIRQ) & bmHXFRDNIRQ));  // wait for the completion IRQ
  Hwreg(rHIRQ,bmHXFRDNIRQ);              // clear the IRQ
  resultcode = (Hrreg(rHRSL) & 0x0F);    // get the result
  if (resultcode==hrNAK) 

    {
    nak_count++;
    if(nak_count==NAK_LIMIT) break;
    else continue;
    }

  if (resultcode==hrTIMEOUT)
    {
    retry_count++;
    if (retry_count==RETRY_LIMIT) break;    // hit the max allowed retries. Exit and return result code
    else continue;
    }
  else break;                           	// all other cases, just return the success or error code
  }
return(resultcode);
}

BYTE print_error(BYTE err)
{
if(err)
	{
	printfUART(">>>>> Error >>>>> ");
	switch(err)
		{
		case 0x01: printfUART("MAX3421E SIE is busy "); break;
		case 0x02: printfUART("Bad value in HXFR register "); break;
		case 0x04: printfUART("Exceeded NAK limit"); break;
		case 0x0C: printfUART("LS Timeout "); break;
		case 0x0D: printfUART("FS Timeout "); break;
		case 0x0E: printfUART("Device did not respond in time "); break;
		case 0x0F: printfUART("Device babbled (sent too long) "); break;
		default:   printfUART("Programming error %01X,",err);
		}
	}
return(err);
}

// File: MAX3421E.c
// Author: Jaka Kordez
// Version: 0.1

#include "MAX3421E.h"

SPI_HandleTypeDef hspi5;
uint8_t dataOut[5];
uint8_t dataIn[5];

static void MX_SPI5_Init(void);
void Hwreg(uint8_t reg, uint8_t val);
uint8_t Hrreg(uint8_t reg);
void MAX_Reset(void);
void detect_device(void);

void Hwreg(uint8_t reg, uint8_t val){
	
	dataOut[0] = (reg<<3) | 2;
	dataOut[1] = val;
	MAX_CS_PORT->BSRR = (uint32_t)MAX_CS_PIN << 16;
	HAL_SPI_Transmit(&hspi5, dataOut, 2, 1000);
	MAX_CS_PORT->BSRR = MAX_CS_PIN;
}

uint8_t Hrreg(uint8_t reg){
	dataOut[0] = reg<<3;
	dataOut[1] = 0;
	MAX_CS_PORT->BSRR = (uint32_t)MAX_CS_PIN << 16;
	HAL_SPI_TransmitReceive(&hspi5, dataOut, dataIn, 2, 1000);
	MAX_CS_PORT->BSRR = MAX_CS_PIN;
	return dataIn[1];
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
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
	MAX_Reset();
	Hwreg(rIOPINS1,0x00);		// seven-segs off
	Hwreg(rIOPINS2,0x00);		// and Vbus OFF (in case something already plugged in)
	HAL_Delay(200);
	VBUS_ON;
}

void MAX_Process(void){
	detect_device();
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
		//Full-Speed Device Detected
	}
	if (busstate==bmKSTATUS)  // K-state means D- high
	{
		Hwreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmHOST|bmLOWSPEED|bmSOFKAENAB));  // make the MAX3421E a low speed host        
		//Low-Speed Device Detected
	}
}

void MAX_Reset(void)
{
	Hwreg(rUSBCTL,bmCHIPRES);  // chip reset This stops the oscillator
	Hwreg(rUSBCTL,0x00);       // remove the reset
	while(!(Hrreg(rUSBIRQ) & bmOSCOKIRQ)) ;  // hang until the PLL stabilizes
}

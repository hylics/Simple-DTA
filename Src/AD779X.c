/***************************************************************************//**
 *   @file   AD779X.c
 *   @brief  Implementation of AD779X Driver.
 *   @author Bancisor MIhai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 501
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD779X.h"				// AD779X definitions.
#include "Communication.h"		// Communication definitions.
//#include "stm32f0xx_hal_conf.h" //for assert
//#include <assert.h>

//#define AD7793
//#define TIMEOUT_COMMAND 2     //address <-> command
//#define TIMEOUT_PACKET  50    //address,command <-> address,command
//#define TIMEOUT_LH      1     //CS LOW or HIGH <-> transmit or receive
//#define TIMEOUT_RESET   2000  //after reset

/******************************************************************************/
/* Private functions prototypes                                               */
/******************************************************************************/
//static uint32_t AD779X_GetRegisterValue(uint8_t regAddress, uint8_t size, uint8_t modifyCS);
//static void AD779X_SetRegisterValue(uint8_t regAddress, uint32_t regValue, uint8_t size, uint8_t modifyCS);
static void AD779X_WaitRdyGoLow(void);


/***************************************************************************//**
 * @brief Initializes the AD779X and checks if the device is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
uint8_t AD779X_Init(void)
{ 
	uint8_t status = 0x1;
    
  //SPI_Init(0, 1000000, 1, 1);
	ADI_PART_CS_HIGH;
	AD779X_GetRegisterValue(AD779X_REG_ID, 1, 1);
  if((AD779X_GetRegisterValue(AD779X_REG_ID, 1, 1) & 0x0F) != AD7793_ID)
		{
			status = 0x0;
		}
    
	return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @return  None.    
*******************************************************************************/
void AD779X_Reset(void)
{
	ADI_PART_CS_HIGH;
	SCK_HIGH;
	MOSI_LOW;
	uint8_t dataToSend[6] = {0x03, 0xff, 0xff, 0xff, 0xff, 0xff};
	
  ADI_PART_CS_LOW;
  //ADI_DELAY(TIMEOUT_LH);
	SPI_Write(dataToSend, 5);
	//ADI_DELAY(TIMEOUT_LH);
	ADI_PART_CS_HIGH;
	ADI_DELAY(TIMEOUT_RESET);
}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
uint32_t AD779X_GetRegisterValue(uint8_t regAddress, uint8_t size, uint8_t modifyCS) {	
	uint8_t data[4]      = {0x00};
	uint32_t receivedData = 0x00;
  uint8_t i            = 0x00; 
    
	data[0] = AD779X_COMM_READ |  AD779X_COMM_ADDR(regAddress);
	//Modifying CS by this function or external
	if(modifyCS != 0) {
		ADI_PART_CS_LOW;
		//ADI_DELAY(TIMEOUT_LH);
	}
	
	SPI_Read(data, size);
	//Modifying CS by this function or external
	if(modifyCS != 0) {
		//ADI_DELAY(TIMEOUT_LH);
		ADI_PART_CS_HIGH;
	}
	//ADI_DELAY(TIMEOUT_PACKET);
	
	for(i = 0; i < size; i++)
    {
        receivedData = (receivedData << 8) + data[i];
    }
    
    return (receivedData);
} //END AD779X_GetRegisterValue()


/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD779X_SetRegisterValue(uint8_t regAddress, uint32_t regValue, uint8_t size, uint8_t modifyCS) {
	uint8_t data[4]      = {0x00};	
	uint8_t* data_ptr = (uint8_t*)&regValue;
  uint8_t bytesNr      = size;
    
  data[0] = AD779X_COMM_WRITE |  AD779X_COMM_ADDR(regAddress);
  while(bytesNr != 0) {
      data[bytesNr] = *data_ptr;
      data_ptr++;
      bytesNr--;
  }
  //Modifying CS by this function or external
	if(modifyCS !=0) {
		ADI_PART_CS_LOW;
		//ADI_DELAY(TIMEOUT_LH);
	}	
	SPI_Write(data, size);
	//Modifying CS by this function or external
	if(modifyCS != 0) {
		//ADI_DELAY(TIMEOUT_LH);
		ADI_PART_CS_HIGH;
	}
	//ADI_DELAY(TIMEOUT_PACKET);
	
} //END AD779X_SetRegisterValue()

/***************************************************************************//**
 * @brief  Waits for RDY pin to go low.
 *
 * @return None.
*******************************************************************************/
static void AD779X_WaitRdyGoLow(void) {
    while( AD779X_RDY_STATE ) {
			//add osDelay() ???
      ;
    }
}

/**
* @brief write config to ad779X
* @param *adc_instance
* @param type - choice all register been written or specific
* @return ADI_StatusTypeDef
*/
ADI_StatusTypeDef AD779X_conf(AD779X_HandleTypeDef *adc_instance, op_mode_TypeDef type) {
	/* Check the AD779X handle allocation */
	if(adc_instance == NULL) {
		return ADI_ERROR;
	}
	
	/* Check the parameters */
	//assert_param();
	
	adc_instance->state = ADI_BUSY;
	
	if(adc_instance->lock == ADI_LOCKED) {
		return ADI_BUSY;
	}
	else {
		adc_instance->lock = ADI_LOCKED;
	}
	
	uint8_t channel = adc_instance->conf & 0x07;
	
	// REG CONF
	if(type==reg_all || type==reg_conf) {
		AD779X_SetRegisterValue(AD779X_REG_CONF, adc_instance->conf, 2, 1);
	}
	
	// REG MODE
	if(type==reg_all || type==reg_mode) {
		AD779X_SetRegisterValue(AD779X_REG_MODE, adc_instance->mode, 2, 1);
	}
	
	// REG IO
	if(type==reg_all || type==reg_io) {
		AD779X_SetRegisterValue(AD779X_REG_IO, adc_instance->io, 1, 1);
	}
	
	// REG OFFSET
	if(type==reg_all || type==reg_offset) {
		if(channel<=3) {
			AD779X_SetRegisterValue(AD779X_REG_OFFSET, adc_instance->offset[channel], 2, 1);
		}
		else return ADI_ERROR;
	}
	
	// REG FULLSCALE
	if(type==reg_all || type==reg_full_scale) {
		if(channel<=3) {
		  AD779X_SetRegisterValue(AD779X_REG_FULLSCALE, adc_instance->fullscale[channel], 2, 1);
		}
		else return ADI_ERROR;
	}
	
	adc_instance->lock = ADI_UNLOCKED;
	adc_instance->state = ADI_OK;
	
	return ADI_OK;
}

/*my func*/
/*int AD779X_conf(uint32_t gain, uint32_t channel, uint32_t current) {
	uint32_t command;
	int retval = 0;
	
	command = AD779X_GetRegisterValue(AD779X_REG_CONF, 2, 1); // CS is modified by SPI read/write functions.
	command &= ~AD779X_CONF_GAIN(0xFF);
  command |= AD779X_CONF_GAIN(gain);
	
	command &= ~AD779X_CONF_CHAN(0xFF);
  command |= AD779X_CONF_CHAN(channel);
	
	command |= AD779X_CONF_UNIPOLAR;
	
	
	AD779X_SetRegisterValue(AD779X_REG_CONF, command, 2, 1);
	//if( (AD779X_GetRegisterValue(AD779X_REG_CONF, 2, 1) & command) != command ) {
		  //retval = 1;
	}
	
	command &= ~AD7702_RATE_1_120ms;
	command |= AD7702_RATE_1_120ms;
  AD779X_SetRegisterValue(AD779X_REG_MODE, command, 2, 1); // CS is modified by SPI read/write functions.
	//if( (AD779X_GetRegisterValue(AD779X_REG_MODE, 2, 1) & command) != command) {
      //retval = 2;
	}
	

	command = 0x0;
	command |= AD779X_IEXCEN(current);
	AD779X_SetRegisterValue(AD779X_REG_IO, command, 1, 1);
	//if( (AD779X_GetRegisterValue(AD779X_REG_IO, 1, 1) & command) != command) {
		  //retval = 3;
	}
	
	return retval;
}*/



/***************************************************************************//**
 * @brief Selects the channel of AD779X.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
void AD779X_SetChannel(uint32_t channel) {
    uint32_t command;
    
    command = AD779X_GetRegisterValue(AD779X_REG_CONF, 2, 1); // CS is modified by SPI read/write functions.
    command &= ~AD779X_CONF_CHAN(0xFF);
    command |= AD779X_CONF_CHAN(channel);
    AD779X_SetRegisterValue(AD779X_REG_CONF, command, 2, 1); // CS is modified by SPI read/write functions.
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 * the offset calibration should be 
 * performed before the system full-scale calibration is initiated
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD779X_Calibrate(AD779X_HandleTypeDef *adc_instance, uint8_t mode, uint8_t channel) {
	adc_instance->mode &= !AD779X_MODE_SEL(mode);
	adc_instance->mode |= AD779X_MODE_SEL(mode);
	adc_instance->conf &= ~AD779X_CONF_CHAN(0xFF);
	adc_instance->conf |= AD779X_CONF_CHAN(channel);
	AD779X_conf(adc_instance, reg_conf);
  ADI_PART_CS_LOW; 
  AD779X_SetRegisterValue(AD779X_REG_MODE, adc_instance->mode, 2, 0); // CS is not modified by SPI read/write functions.
  AD779X_WaitRdyGoLow();
  ADI_PART_CS_HIGH;
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
uint32_t AD779X_SingleConversion(AD779X_HandleTypeDef *adc_instance)
{
    //uint32_t command = 0x2009; //default 0x200A
    uint32_t regData = 0x0;
    
	  adc_instance->mode &= ~AD779X_MODE_SEL(0xF);
    adc_instance->mode |= AD779X_MODE_SEL(AD779X_MODE_SINGLE);
	
    ADI_PART_CS_LOW;
	  //ADI_DELAY(TIMEOUT_LH);
	  //AD779X_conf(adc_instance, reg_mode); // remeber about CS!
    //AD779X_SetRegisterValue(AD779X_REG_MODE, command, 2, 0);// CS is not modified by SPI read/write functions.
	  AD779X_SetRegisterValue(AD779X_REG_MODE, adc_instance->mode, 2, 0);// CS is not modified by SPI read/write functions.
    AD779X_WaitRdyGoLow();
	  //ADI_DELAY(TIMEOUT_WRGL);
	  //ADI_DELAY(5);
    regData = AD779X_GetRegisterValue(AD779X_REG_DATA, 3, 0); // CS is not modified by SPI read/write functions.
    //ADI_DELAY(TIMEOUT_LH);
	  ADI_PART_CS_HIGH;
	
	  /*bad solution for problem reading 2 bytes as 3 bytes*/
	  /*solved if soft SPI used?*/
	  /*if((regData & 0xF00000) != 0) {
		  regData>>=8;
	  }*/

    return(regData);
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
uint32_t AD779X_ContinuousReadAvg(uint8_t sampleNumber) {
    uint32_t samplesAverage = 0x0;
    uint32_t command        = 0x0;
    uint8_t count          = 0x0;
    
    command = AD779X_MODE_SEL(AD779X_MODE_CONT);
    ADI_PART_CS_LOW;
	  //ADI_DELAY(TIMEOUT_LH);
    AD779X_SetRegisterValue(AD779X_REG_MODE, command, 2, 0);// CS is not modified by SPI read/write functions.
    for(count = 0;count < sampleNumber;count ++)
    {
        AD779X_WaitRdyGoLow();
        samplesAverage += AD779X_GetRegisterValue(AD779X_REG_DATA, 2, 0);  // CS is not modified by SPI read/write functions.
    }
		//ADI_DELAY(TIMEOUT_LH);
    ADI_PART_CS_HIGH;
    samplesAverage = samplesAverage / sampleNumber;
    
    return(samplesAverage);
}

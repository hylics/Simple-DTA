/***********************
*
*/

#include "adi.h"

AD779X_HandleTypeDef adi1;

void ADI_Init(void) {
	//
	adi1.conf &= ( ~AD779X_CONF_GAIN(0xFF) | ~AD779X_CONF_CHAN(0xFF) | ~AD779X_CONF_UNIPOLAR | AD779X_CONF_BUF);
	adi1.conf |= ( AD779X_CONF_GAIN(AD779X_GAIN_1) | AD779X_CONF_CHAN(AD779X_CH_AIN2P_AIN2M) | AD779X_CONF_UNIPOLAR | AD779X_CONF_BUF);
	
	adi1.mode &= ~(AD779X_MODE_RATE(0xF) | AD779X_MODE_SEL(0xFF));
	adi1.mode |= (AD779X_MODE_RATE(AD7702_RATE_1_120ms) | AD779X_MODE_SEL(AD779X_MODE_IDLE));
	
	adi1.io |= ( AD779X_IEXCDIR(AD779X_DIR_IEXC1_IOUT1_IEXC2_IOUT2) | AD779X_IEXCEN(AD779X_EN_IXCEN_210uA) );
	
	//adi1.offset[AD779X_CH_AIN2P_AIN2M] = AD779X_GetRegisterValue(AD779X_REG_OFFSET, 2, 1);
	adi1.offset[AD779X_CH_AIN2P_AIN2M] = 0x8000;
	//adi1.fullscale[AD779X_CH_AIN2P_AIN2M] = AD779X_GetRegisterValue(AD779X_REG_FULLSCALE, 2, 1);
	adi1.fullscale[AD779X_CH_AIN2P_AIN2M] = 0x53A0; //0x54a3
	
	adi1.cs.gpio = GPIOB;
	adi1.cs.pin = GPIO_PIN_12;
	adi1.rdy.gpio = GPIOC;
	adi1.rdy.pin = GPIO_PIN_6;
	
	//AD779X_conf(&adi1, reg_all);
	
	if(AD779X_Init() == 1) {
		//
		AD779X_conf(&adi1, reg_all);
	}
}

uint32_t rec_filter(uint32_t data, uint8_t Nb, uint8_t k) {
  static uint32_t y = 0;
  static uint64_t z = 0;
  z += data;
	z -= y;
  return y = (Nb * z) >> k;

//	static uint32_t res;
//	res += data;
//	res /= 2;
//	return res;
}




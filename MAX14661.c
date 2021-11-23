#include "transfer_handler.h"
#include "MAX14661.h"

bool SetTerminal(uint8_t posTerm, uint8_t negTerm) {
  
	if(posTerm > 16 && negTerm > 16)
		return false;
	posTerm -= 1;
	negTerm -= 1;
	
	uint8_t configData[2];
	configData[0] = 0x14;
    configData[1] = negTerm;
	
	iic_send(MAX14661_ADDR, configData, 2, false);
	
	configData[0] = 0x15;
    configData[1] = posTerm;
	
	iic_send(MAX14661_ADDR, configData, 2, false);
	
	return true;
	
}

bool SetGainRes(uint8_t gainRes) { 
	
	if(gainRes > 7)
		return false;
	
	(gainRes & 0x01) ? nrf_gpio_pin_set(MUX_A0) : nrf_gpio_pin_clear(MUX_A0);
	(gainRes & 0x02) ? nrf_gpio_pin_set(MUX_A1) : nrf_gpio_pin_clear(MUX_A1);
	(gainRes & 0x04) ? nrf_gpio_pin_set(MUX_A2) : nrf_gpio_pin_clear(MUX_A2);
	
	return true;

}

bool initGainRes(void){

	nrf_gpio_cfg_output(MUX_A0);
	nrf_gpio_cfg_output(MUX_A1);
	nrf_gpio_cfg_output(MUX_A2);
	
	SetGainRes(GAIN_RES_1);
    SetTerminal(1, 2);
	
    return true;
	
}



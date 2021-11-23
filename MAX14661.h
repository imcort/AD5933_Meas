#ifndef MAX14661_h
#define MAX14661_h

#include <stdint.h>
#include <stdbool.h>

#define MAX14661_ADDR 0x4c

#define MAX14661_REG_DIR0 0x00
#define MAX14661_REG_DIR1 0x01
#define MAX14661_REG_DIR2 0x02
#define MAX14661_REG_DIR3 0x03

#define MUX_A0  8
#define MUX_A1	15
#define MUX_A2 	18

#define GAIN_RES_1        0
#define GAIN_RES_10				1
#define GAIN_RES_100			2
#define GAIN_RES_1K				3
#define GAIN_RES_10K			4
#define GAIN_RES_100K			5
#define GAIN_RES_1M				6
#define GAIN_RES_10M			7


bool SetTerminal(uint8_t posTerm, uint8_t negTerm);
bool SetGainRes(uint8_t gainRes);
bool initGainRes(void);
#endif

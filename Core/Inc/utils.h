/*
 * utils.h
 *
 *  Created on: Sep 8, 2021
 *      Author: perez
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

typedef struct{
    uint8_t *Buff;
    uint8_t iw;
    uint8_t ir;
    uint8_t ISCMD;
    uint8_t header;
    uint8_t state;
    uint8_t cks;
    uint8_t ckst;
    uint8_t nbytes;					//Nbytes[cantidad de bytes de mi buffer]
    uint8_t IntPutBytes;
    uint8_t iData;					//iData[indice para saber donde comienzan mis datos;
    uint8_t maskSize;
    uint8_t timeout;
    uint8_t i;
}_Rx;

typedef struct{
	uint8_t *Buff;
    uint8_t iw;
    uint8_t ir;
    uint8_t length;
    uint8_t OutPutBytes;
    uint8_t maskSize;                //Nbytes[cantidad de bytes de mi buffer]; iData[indice para saber donde comienzan mis datos; _rx.ckst[]
    uint8_t maskBuf;
    uint8_t DataSent;
}_Tx;

typedef union{
	uint8_t     u8[16];
	int8_t      i8[4];
	uint16_t    u16[16];
	int16_t     i16[2];
	uint32_t    u32;
	int32_t     i32;
}_work;

typedef union{
	struct
	{
		uint8_t b0: 1;
		uint8_t b1: 1;
		uint8_t b2: 1;
		uint8_t b3: 1;
		uint8_t b4: 1;
		uint8_t b5: 1;
		uint8_t b6: 1;
		uint8_t b7: 1;
	}bit;
	uint8_t byte;
}_sFlag;

#endif /* INC_UTILS_H_ */

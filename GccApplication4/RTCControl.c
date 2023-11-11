/*
 * RTCControl.c
 *
 * Created: 11/9/2023 7:49:48 PM
 *  Author: alber
 */ 
#include "RTCControl.h"

void startCond(){
	SERCOM3->I2CM.CTRLB.bit.CMD = 0x1;                 /* Sending repeated start condition */
}

void stopCond(){	
	SERCOM3->I2CM.CTRLB.bit.CMD = 0x3;
}


void sendI2CDataArray(uint8_t SlAddr, uint8_t prt, uint8_t ptrData[MAX_LEN], uint8_t Size){
	
	SERCOM3->I2CM.ADDR.reg = (SlAddr << 1) | 0;    /* Sending slave address in write mode */
	while(!SERCOM3->I2CM.INTFLAG.bit.MB);;
		
	SERCOM3->I2CM.DATA.reg = prt;
	while(!SERCOM3->I2CM.INTFLAG.bit.MB);;

	int i;
	for(i=0; i<Size; i++) {
		uint8_t val = ptrData[i];
		/* placing the data from transmitting buffer to DATA register*/
		SERCOM3->I2CM.DATA.reg = val;
		while(!SERCOM3->I2CM.INTFLAG.bit.MB);;
	}
	
	stopCond();
}

void sendI2CData(uint8_t SlAddr, uint8_t data){
		
	SERCOM3->I2CM.ADDR.reg = (SlAddr << 1) | 0;    /* Sending slave address in write mode */
	while(!SERCOM3->I2CM.INTFLAG.bit.MB);;

	SERCOM3->I2CM.DATA.reg = data;
	while(!SERCOM3->I2CM.INTFLAG.bit.MB);;
			
	stopCond();
}

void receiveI2CDataArray(uint8_t SlAddr, uint8_t prt, uint8_t ptrData[MAX_LEN], uint8_t Size){
	
	SERCOM3->I2CM.ADDR.reg = (SlAddr << 1) | 0;    /* Sending slave address in write mode */
	while(!SERCOM3->I2CM.INTFLAG.bit.MB);;
	

	SERCOM3->I2CM.DATA.reg = prt;
	while(!SERCOM3->I2CM.INTFLAG.bit.MB);			
			
	SERCOM3->I2CM.ADDR.reg = (SlAddr << 1) | 1;
	while(!SERCOM3->I2CM.INTFLAG.bit.SB);	
		
	int i;
	for(i=0; i< Size; i++) {
		uint8_t val = SERCOM3->I2CM.DATA.reg;
		
		if(i + 1 == Size){
			SERCOM3->I2CM.CTRLB.bit.ACKACT = 1;
		}else{
			SERCOM3->I2CM.CTRLB.bit.ACKACT = 0;
		}
		
		SERCOM3->I2CM.CTRLB.bit.CMD = 0x2;	
		while (SERCOM3->I2CM.SYNCBUSY.bit.SYSOP);	
			
		ptrData[i] = val;
		while(!SERCOM3->I2CM.INTFLAG.bit.SB);
		

	}
	stopCond();
}


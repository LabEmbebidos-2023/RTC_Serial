/*
 * RTCControl.h
 *
 * Created: 11/9/2023 7:43:29 PM
 *  Author: alber
 */ 

#include "sam.h"

#ifndef RTCCONTROL_H_
#define RTCCONTROL_H_
#define MAX_LEN 20

struct Timestamp{
	int day;
	int month;
	int year;
	
	int hour;
	int minutes;
	int seconds;
};

void startCond();
void stopCond();

void sendI2CDataArray(uint8_t SlAddr, uint8_t prt, uint8_t ptrData[MAX_LEN], uint8_t Size);
void sendI2CData(uint8_t SlAddr, uint8_t data);
void receiveI2CDataArray(uint8_t SlAddr, uint8_t prt, uint8_t ptrData[MAX_LEN], uint8_t Size);


#endif /* RTCCONTROL_H_ */
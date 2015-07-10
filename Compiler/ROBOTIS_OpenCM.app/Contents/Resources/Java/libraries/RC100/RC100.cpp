/*
 * RC100.cpp
 *
 *  Created on: 2013. 4. 24.
 *      Author: ROBOTIS,.LTD.
 */

#include "RC100.h"
#include "rc100_core.h"


RC100::RC100() {
	// TODO Auto-generated constructor stub

}

RC100::~RC100() {
	// TODO Auto-generated destructor stub

}

void RC100::begin(int num){
	if(num == 1) rc100Initialize(57600);
	else if(num == 2) rc100Initialize(1900);
	check_mode = num;
}
void RC100::end(void){
	rc100Terminate();
}
int RC100::writeData(int data){
	return rc100TxData(data);
}
void RC100::writeRaw(byte temp){
	TxDByteUart2(temp);
}
byte RC100::readRaw(void){
	return RxDByteUart2();
}
int RC100::available(void){
	return rc100RxCheck();
}
int RC100::readData(void){
	return rc100RxData();
}
void RC100::setChannel(byte IR_channel){
	rc100channel(IR_channel);
}
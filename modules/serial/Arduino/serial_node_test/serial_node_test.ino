#include <Arduino.h>
#include <cstring>

#define PACKAGE_LEN 6
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define NOT_FOUND -1
#define BUFFER_SIZE 11

typedef union Data{
	uint8_t b[4];
	float f;
}data;

uint8_t buffer[BUFFER_SIZE];

data a[4];
uint8_t b[4];

void setup(){
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial1.setTimeout(10);
	for(int i=0;i<4;i++){
		a[i].f = 0;
	}
}

void loop(){
	Serial1.write(PACKAGE_HEAD);
	transmit(a[0]);
	transmit(a[1]);
	transmit(a[2]);
	transmit(a[3]);
	Serial1.write(PACKAGE_TAIL);
	for(int i=0;i<4;i++){
		a[i].f += 0.3;
	}
	int num = Serial1.readBytes(buffer,BUFFER_SIZE);
	int index = findPackage(buffer,num);
	if(index!=NOT_FOUND){
		memcpy(b,buffer+index,4);
		Serial.print(b[0]);
		Serial.print(" ");
		Serial.print(b[1]);
		Serial.print(" ");
		Serial.print(b[2]);
		Serial.print(" ");
		Serial.println(b[3]);
	}
}

void transmit(data a){
	for(int i=0;i<4;i++)
		Serial1.write(a.b[i]);
}

int findPackage(uint8_t* buffer,int size){
    if(size>=PACKAGE_LEN){
        for(int i=0;i<=(size-PACKAGE_LEN);i++){
            if(buffer[i] == PACKAGE_HEAD && buffer[i+PACKAGE_LEN-1] == PACKAGE_TAIL){
                return i+1;
            }
        }
    }
    return NOT_FOUND;
}
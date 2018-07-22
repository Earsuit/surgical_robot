#include <cstring>

#define PACKAGE_LEN 14
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define NOT_FOUND -1

typedef union Data{
	uint8_t b[4];
	float f;
}data;

data a[3];

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);
    for(int i=0;i<4;i++){
		a[i].f = 0;
	}
}

void loop(){
    Serial1.write(PACKAGE_HEAD);
    transmit(a[0]);
	transmit(a[1]);
	transmit(a[2]);
	Serial1.write(PACKAGE_TAIL);
	for(int i=0;i<3;i++){
		a[i].f += 0.3;
	}
}

void transmit(data a){
	for(int i=0;i<4;i++)
		Serial1.write(a.b[i]);
}
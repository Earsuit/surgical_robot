#include "ATMEGA2560.h"
#include <math.h>

#define TRUE 1
#define FALSE 0

#define DEBUG TRUE

#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71

#define OUTPUT_COMPARE_ENCODER  0xF424 //set to 1s

#define ONE_REVOLUTION_1 89400.0
#define ONE_REVOLUTION_2 44700.0
#define ONE_REVOLUTION_3 67050.0
#define ONE_REVOLUTION_4 67050.0

//measurement interval interrupt
#define MEASUREMENT_OUTPUT_COMPARE 0xEA6 //3750, 15ms

#define MY_2PI 6.2831853072
#define MY_PI 3.1415926536

#define SATURATION 6.8

typedef union FLOAT{
	float data;
	uint8_t bytes[4];
}Float;

Float commands[4];

void setup(){
	commands[0].data = 0;
	commands[1].data = 0;
	commands[2].data = 0;
	commands[3].data = 0;
	Serial.begin(115200);
	Serial2.begin(115200);
	SET_AIN1_PIN_1_OUT;
	SET_AIN2_PIN_1_OUT;

	SET_AIN1_PIN_2_OUT;
	SET_AIN2_PIN_2_OUT;

	SET_AIN1_PIN_3_OUT;
	SET_AIN2_PIN_3_OUT;
	SET_C2_READ_PIN_3_IN;

	SET_AIN1_PIN_4_OUT;
	SET_AIN2_PIN_4_OUT;
	PWM_setup();	
}

void loop(){
	if(Serial.available()){
		Serial.readBytes(commands[0].bytes, 16);
		Serial2.print(commands[0].data);
		Serial2.print(",");
		Serial2.print(commands[1].data);
		Serial2.print(",");
		Serial2.print(commands[2].data);
		Serial2.print(",");
		Serial2.println(commands[3].data);
	}
	driveMotor_1(commands[0].data);
	driveMotor_2(commands[1].data);
	driveMotor_3(commands[2].data);
	driveMotor_4(commands[3].data);
}

inline void driveMotor_1(float command){
	if(command>=0){
		SET_AIN1_PIN_1(LOW);
		SET_AIN2_PIN_1(HIGH);
		PWM_1(uint16_t(command));
	}else{
		SET_AIN1_PIN_1(HIGH);
		SET_AIN2_PIN_1(LOW);
		PWM_1(-uint16_t(command));
	}
}

inline void driveMotor_2(float command){
	if(command>=0){
		SET_AIN1_PIN_2(LOW);
		SET_AIN2_PIN_2(HIGH);
		PWM_2(uint16_t(command));
	}else{
		SET_AIN1_PIN_2(HIGH);
		SET_AIN2_PIN_2(LOW);
		PWM_2(-uint16_t(command));
	}
}

inline void driveMotor_3(float command){
	if(command>=0){
		SET_AIN1_PIN_3(LOW);
		SET_AIN2_PIN_3(HIGH);
		PWM_3(uint16_t(command));
	}else{
		SET_AIN1_PIN_3(HIGH);
		SET_AIN2_PIN_3(LOW);
		PWM_3(-uint16_t(command));
	}
}

inline void driveMotor_4(float command){
	if(command>=0){
		SET_AIN1_PIN_4(LOW);
		SET_AIN2_PIN_4(HIGH);
		PWM_4(uint8_t(command));
	}else{
		SET_AIN1_PIN_4(HIGH);
		SET_AIN2_PIN_4(LOW);
		PWM_4(uint8_t(-command));
	}
}

void PWM_setup(){
	SET_PWM_PIN_1_OUTPUT;
	SET_PWM_PIN_2_OUTPUT;
	SET_PWM_PIN_3_OUTPUT;
	SET_PWM_PIN_4_OUTPUT;
	
	// 10-bit Fasr PWM Mode, Set OC5A on Compare Match when up-counting Clear OC2A on Compare Match when down-counting,
	// TOP = 0x3FF, CLK/8
	TCCR5A = _BV(COM5A1)  | _BV(WGM51) | _BV(WGM50);
	TCCR5B = _BV(CS51) | _BV(WGM52);

	TCCR4A = _BV(COM4A1)  | _BV(WGM41) | _BV(WGM40);
	TCCR4B = _BV(CS41) | _BV(WGM42);

	TCCR3A = _BV(COM3A1)  | _BV(WGM31) | _BV(WGM30);
	TCCR3B = _BV(CS31) | _BV(WGM32);

	// motor 4, 8-bit 
	TCCR0A =  _BV(COM0B1)  | _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(CS00) | _BV(CS01);
}

inline void PWM_1(uint16_t value){
	OCR5A = value;
}

inline void PWM_2(uint16_t value){
	OCR4A = value;
}

inline void PWM_3(uint16_t value){
	OCR3A = value;
}

inline void PWM_4(uint8_t value){
	OCR0B = value;
}
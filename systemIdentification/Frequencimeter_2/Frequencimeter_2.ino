#include "ATMEGA2560.h"
#include "I2C.h"
#include <math.h>

#define TRUE 1
#define FALSE 0

#define DEBUG FALSE

#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71

#define OUTPUT_COMPARE_ENCODER  0xF424 //set to 1s
#define FACTOR 64285.625
#define ONE_REVOLUTION 350
#define DEGREES_PER_PULSE 1.0286
//X1 Encoding
#define POSITIVE_DIR 0x40		//bits 5,6 -> 10, actually this is equal to true
#define NEGATIVE_DIR 0x00		//bits 5,6 -> 00, actually this is false

#define NEGATIVE_MASK 0x8000

#define INA219_ADDRESS 0x40
#define ARDUINO_ADDRESS 0x52

// INA219 Register Address
#define CONFIG_REG 0x00
#define SHUNT_VOL_REG 0x01
#define BUS_VOL_REG 0x02
#define POWER_REG 0x03
#define CURRENT_REG 0x04
#define CALIB_REG 0x05

#define BUS_VOL_LSB 0.004f  //Volt
#define CURRENT_LSB 0.000013f	//Amp

//measurement interval interrupt
#define MEASUREMENT_OUTPUT_COMPARE 0xEA6 //3750, 15ms

#define MY_PI 3.14159265358979

#define RUNTIME 15.0

volatile uint8_t encoderPulseShared = 0x00;
volatile int32_t countShared = 0;
volatile uint8_t output = FALSE;
volatile uint8_t updated = FALSE;
int32_t countLocal = 0;
int32_t countTotal = 0;

typedef union FLOAT{
	float data;
	uint8_t bytes[4];
}Float;

Float vel,degree;

int16_t voltRegValue, currentRegVal;
Float volt,current;

using namespace TWI;

float t = 0;
float dt = 0.015;  //15ms
float k = 1;

void setup(){
	vel.data = degree.data = volt.data = current.data = 0;
	Serial.begin(115200);
	SET_STBY_PIN_OUT;
	SET_AIN1_PIN_1_OUT;
	SET_AIN1_PIN_1_OUT;
	SET_C1_READ_PIN_1_IN;
	SET_C2_READ_PIN_1_IN;
	SET_STBY_PIN(HIGH);
	// SET_AIN2_PIN_1(HIGH);
	SET_AIN1_PIN_1(LOW);
	INA219Setup();
	PWM_setup();
	timerSetup();
}

void loop(){
	if(t<RUNTIME){
		if(output){
			//for now the output part runs 2.164ms
			output = FALSE;
			if(updated){
				// turn off input capture interrupt
				TIMSK4 = 0x00;
				updated = FALSE;
				countLocal = countShared;
				// turn on input capture interrupt
				TIMSK4 = _BV(ICIE4);
			}
			//reset counter
			countShared = 0;
			countTotal += countLocal;
			int16_t v = chrip();
			if(v>=0){
				SET_AIN1_PIN_1(LOW);
				PWM(v);
			}else{
				SET_AIN1_PIN_1(HIGH);
				PWM(1023+v);
			}
			//if the count is not updated, the vel is 0
			vel.data = DEGREES_PER_PULSE*countLocal/dt;
			degree.data = (countTotal/ONE_REVOLUTION)*360+countTotal%ONE_REVOLUTION*DEGREES_PER_PULSE;
			getVolt();
			getCueent();
			#if DEBUG
				Serial.print(degree.data);
				Serial.print(",");
				Serial.print(vel.data);
				Serial.print(",");
				Serial.print(volt.data);
				Serial.print(",");
				Serial.println(current.data);
			#else
				Serial.write(PACKAGE_HEAD);
				Serial.write(degree.bytes,4);
				Serial.write(vel.bytes,4);
				Serial.write(current.bytes,4);
				Serial.write(volt.bytes,4);
				Serial.write(PACKAGE_TAIL);
			#endif
		}
	}else{
		SET_STBY_PIN(LOW);
	}
			
	// if(Serial.available()){
	// 	int v = Serial.parseInt();
	// 	if(v & NEGATIVE_MASK){
	// 		SET_AIN2_PIN_1(LOW);
	// 		SET_AIN1_PIN_1(HIGH);
	// 		PWM(-v);
	// 	}else{
	// 		SET_AIN2_PIN_1(HIGH);
	// 		SET_AIN1_PIN_1(LOW);
	// 		PWM(v);
	// 	}
	// }
}

inline int16_t chrip(){
	t += dt;
	return 1023*sin(2*MY_PI*k*t*t);
}

void INA219Setup(){
	I2CSetup(ARDUINO_ADDRESS,400);
	
	startTrans(INA219_ADDRESS,WRITE);
	write(CONFIG_REG);
	//change the Bus Voltage Range to 16V and shunt voltage to 80mV (max 0.8A)
	write(0x9);	
	write(0x1F,true);	

	//program the calibration register to 0x7B13
	startTrans(INA219_ADDRESS,WRITE);
	write(CALIB_REG);
	// only change the Bus Voltage Range to 16V
	write(0x7B);	
	write(0x13,true);	
}

inline void getVolt(){
	int16_t voltRegValue;
	startTrans(INA219_ADDRESS,WRITE);
	write(BUS_VOL_REG);
	requestFrom(INA219_ADDRESS,1,true);
	voltRegValue = ((readBuffer()<<8) | readBuffer())>>3;
	if(voltRegValue  & 0x1000)		//check if negative
		voltRegValue |= 0xE000;
	volt.data = voltRegValue*BUS_VOL_LSB;
}

inline void getCueent(){
	int16_t currentRegVal;
	startTrans(INA219_ADDRESS,WRITE);
	write(CURRENT_REG);
	requestFrom(INA219_ADDRESS,1,true);
	currentRegVal = (readBuffer()<<8) | readBuffer();
	current.data = currentRegVal*CURRENT_LSB;
}

// counter 2
void PWM_setup(){
	// Fasr PWM Mode, Set OC2B on Compare Match when up-counting Clear OC2A on Compare Match when down-counting,
	// TOP = 0xFF, CLK/128
	// if set TOP to OCRA, has to set OCR0A as well.
	SET_PWM_PIN_OUTPUT;
	// TCCR0A = _BV(COM0B1)  | _BV(WGM01) | _BV(WGM00);
	// TCCR0B = _BV(CS00) | _BV(CS01);

	// 10-bit Fasr PWM Mode, Set OC5A on Compare Match when up-counting Clear OC2A on Compare Match when down-counting,
	// TOP = 0x3FF, CLK/8
	TCCR5A = _BV(COM5A1)  | _BV(WGM51) | _BV(WGM50);
	TCCR5B = _BV(CS51) | _BV(WGM52);
}

void PWM(uint16_t value){
	OCR5A = value;
}

void encoderClockSetup(){
    //Timer/Counter 4
    TCCR4A = 0x00;
    TCCR4B = (_BV(WGM42)) | (_BV(CS42)) | (_BV(ICES4));  //CTC mode, clk/256, rising (positive) edge will trigger the capture.
    OCR4A = OUTPUT_COMPARE_ENCODER; //set to 1s
    TCNT4 = 0x00; //initialise the counter
	TIMSK4 = _BV(ICIE4);   //enable Input Capture Interrupt
}

void measurementIntervalSetup(){
    //Timer/Counter 1
    TCCR1A = 0x00;
    TCCR1B = (_BV(WGM12)) | (_BV(CS11)) | (_BV(CS10));  //CTC mode, clk/64
    OCR1A = MEASUREMENT_OUTPUT_COMPARE;
    TCNT1 = 0x00; //initialise the counter
    TIMSK1 = _BV(OCIE1A);  //Output Compare A Match Interrupt Enable
}

// To calculate time
void calculateIntervalSetup(){
	//Timer/Counter 3
    TCCR3A = 0x00;
    TCCR3B = (_BV(WGM32)) | (_BV(CS31)) | (_BV(CS30));  //CTC mode, clk/64
    OCR3A = 0xFFFF;
}

void timerSetup(){
	cli();
	encoderClockSetup();
	measurementIntervalSetup();
	// calculateIntervalSetup();
	sei();
}

ISR(TIMER4_CAPT_vect){
	encoderPulseShared = READ_C2_1;
	countShared = (encoderPulseShared)?countShared-1:countShared+1;
	updated = TRUE;
}

ISR(TIMER1_COMPA_vect){
	output = TRUE;
}
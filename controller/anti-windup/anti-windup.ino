#include "ATMEGA2560.h"
#include "I2C.h"
#include <math.h>

#define TRUE 1
#define FALSE 0

#define DEBUG TRUE
#define OPTICAL_ENCODER 0x01
#define HALL_EFFECT_ENCODER 0x02

#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71

#define OUTPUT_COMPARE_ENCODER  0xF424 //set to 1s

#define ENCODER HALL_EFFECT_ENCODER
#if ENCODER == OPTICAL_ENCODER
#define ONE_REVOLUTION 250.0
#define DEGREES_PER_PULSE 1.44
#define RADIANS_PER_PULSE 0.025133
#elif ENCODER == HALL_EFFECT_ENCODER
#define ONE_REVOLUTION 44700.0f
#define DEGREES_PER_PULSE 1.0286
#define RADIANS_PER_PULSE 0.0179524567
#endif

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
#define CURRENT_LSB 2.44e-5	//Amp

//measurement interval interrupt
#define MEASUREMENT_OUTPUT_COMPARE 0xEA6 //3750, 15ms

#define MY_2PI 6.2831853072
#define MY_PI 3.1415926536

#define RUNTIME 150

#define TO_VOLTAGE 0.006647		//6.8/1023

#define SATURATION 6.8

//
#define PIDF 0x01
#define PI 0x02
#define CONTROLLER PIDF

volatile uint8_t encoderPulseShared = 0x00;
volatile int32_t countShared = 0;
volatile uint8_t output = FALSE;
volatile uint8_t updated = FALSE;
int32_t countLocal = 0;
int32_t prev_countLocal = 0;
int16_t prev_v = 0;

typedef union FLOAT{
	float data;
	uint8_t bytes[4];
}Float;
//
Float vel,angle;

float ref = 0;

// controller
#if CONTROLLER == PIDF
	float int_y_k = 0;		//y[k]
	float de_y_k_1 = 0;		//y[k-1]
	float de_u_k_1 = 0;		//u[k-1]
	float diff = 0;			//difference between the wanted control and the actual control
	float kp = 17.5;
	float ki = 5;
	float kd = 0.6;
	float Tf = 0.0237;
	float b = 0.871;
	float c = 0.358;
	float dt = 0.015;
	float k_anti = 1;
#elif CONTROLLER == PI	
	float int_y_k = 0;		//y[k]
	float de_y_k_1 = 0;		//y[k-1]
	float de_u_k_1 = 0;		//u[k-1]
	float diff = 0;			//difference between the wanted control and the actual control
	float kp = 2;
	float ki = 1;
	float kd = 2.1;
	float Tf = 0.0237;
	float b = 0.871;
	float c = 0.358;
	float dt = 0.015;
	float k_anti = 0.005;
#endif

using namespace TWI;

void setup(){
	angle.data = 0;
	Serial.begin(115200);
	SET_STBY_PIN_OUT;
	SET_AIN1_PIN_1_OUT;
	SET_AIN1_PIN_1_OUT;
	SET_C1_READ_PIN_1_IN;
	SET_C2_READ_PIN_1_IN;
	SET_STBY_PIN(HIGH);
	PWM_setup();
	timerSetup();
}

void loop(){
	if(Serial.available()){
		ref = Serial.parseFloat();
	}
	if(output){
		output = FALSE;
		if(updated){
			// turn off input capture interrupt
			cli();
			updated = FALSE;
			countLocal = countShared;
			// turn on input capture interrupt
			sei();
		}
		angle.data = (countLocal/ONE_REVOLUTION)*MY_2PI;
		
		int16_t v = controller(ref,angle.data);
		if(prev_countLocal == countLocal && v!=0){
			if(v>0){
				v += 50;
				if(v-prev_v<2)
					v += 70;
				
			}else{
				v = -280;
			}
		}
		prev_v = v;
		if(v>=0){
			SET_AIN1_PIN_1(LOW);
			PWM(v);
		}else{
			SET_AIN1_PIN_1(HIGH);
			PWM(1023+v);
		}
		#if DEBUG
			Serial.println(angle.data*180/MY_PI);
		#else
			Serial.write(PACKAGE_HEAD);
			Serial.write(angle.bytes,4);
			Serial.write(PACKAGE_TAIL);
		#endif
		prev_countLocal = countLocal;
	}
}

/*
	Compute the control signal in PWM 
	@param ref: reference 
	@param feedback: angle
	@return the PWM signal
*/
inline int16_t controller(float ref, float feedback){
	#if CONTROLLER == PIDF
		float v = kp*(b*ref-feedback)+integrator(ref-feedback-k_anti*diff)+PIDFderivative(kd*(c*ref-feedback));
	#elif CONTROLLER == PI
		float error = ref-feedback;
		float v = kp*error+integrator(error-k_anti*diff);	
	#endif

	if(v>SATURATION){
		diff = v-SATURATION;
		v = SATURATION;
	}else if(v<-SATURATION){
		diff = v-SATURATION;
		v = -SATURATION;
	}else
		diff = 0;
	return 1023*(v/SATURATION);
}

inline float integrator(float u_k){
	float tmp = int_y_k;
	int_y_k = int_y_k + ki*dt*u_k;
	return tmp;
}

inline float PIDFderivative(float u_k){
	de_y_k_1 = ((Tf-dt)*de_y_k_1+kd*u_k-kd*de_u_k_1)/Tf;
	de_u_k_1 = u_k;
	return de_y_k_1;
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

void pinChangeInterruptSetup(){
	//The rising edge of INT0 generates asynchronously an interrupt request
	EICRA = _BV(ISC00) | _BV(ISC01);
	EIMSK = _BV(INT0);
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
	pinChangeInterruptSetup();
	measurementIntervalSetup();
	// calculateIntervalSetup();
	sei();
}

ISR(TIMER1_COMPA_vect){
	output = TRUE;
}

ISR(INT0_vect){
	encoderPulseShared = READ_C2_1;
	countShared = (encoderPulseShared)?countShared-1:countShared+1;
	updated = TRUE;
}
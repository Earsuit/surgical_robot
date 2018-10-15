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

volatile uint8_t encoderPulseShared_1 = 0x00;
volatile uint8_t encoderPulseShared_2 = 0x00;
volatile uint8_t encoderPulseShared_3 = 0x00;
volatile uint8_t encoderPulseShared_4 = 0x00;
volatile int32_t countShared_1 = 0;
volatile int32_t countShared_2 = 0;
volatile int32_t countShared_3 = 0;
volatile int32_t countShared_4 = 0;
volatile uint8_t output = FALSE;
volatile uint8_t updated = 0x00;
int32_t countLocal_1 = 0;
int32_t countLocal_2 = 0;
int32_t countLocal_3 = 0;
int32_t countLocal_4 = 0;
int32_t prev_count_1 = 0;
int32_t prev_count_2 = 0;
int32_t prev_count_3 = 0;
int32_t prev_count_4 = 0;

typedef union FLOAT{
	float data;
	uint8_t bytes[4];
}Float;

Float angle_1,angle_2,angle_3,angle_4;
Float ref[4];

// controller
float int_y_k[4] = {0,0,0,0};		//y[k]
float de_y_k_1[4] = {0,0,0,0};		//y[k-1]
float de_u_k_1[4] = {0,0,0,0};		//u[k-1]
float diff[4] = {0,0,0,0};			//difference between the wanted control and the actual control
float kp[4] = {17,18,18,18};
float ki[4] = {5,5,5,5};
float kd[4] = {0.6,0.6,0.6,0.6};
float Tf = 0.0237;
float b = 0.871;
float c = 0.358;
float dt = 0.015;
float k_anti[4] = {1,1,1,1};

void setup(){
	angle_1.data = 0;
	angle_2.data = 0;
	angle_3.data = 0;
	angle_4.data = 0;
	ref[0].data = 0;
	ref[1].data = 0;
	ref[2].data = 0;
	ref[3].data = 0;
	Serial.begin(115200);
	Serial2.begin(115200);
	SET_AIN1_PIN_1_OUT;
	SET_AIN2_PIN_1_OUT;
	SET_C2_READ_PIN_1_IN;

	SET_AIN1_PIN_2_OUT;
	SET_AIN2_PIN_2_OUT;
	SET_C2_READ_PIN_2_IN;

	SET_AIN1_PIN_3_OUT;
	SET_AIN2_PIN_3_OUT;
	SET_C2_READ_PIN_3_IN;

	SET_AIN1_PIN_4_OUT;
	SET_AIN2_PIN_4_OUT;
	SET_C2_READ_PIN_4_IN;
	PWM_setup();
	timerSetup();	
}
int count = 0;
void loop(){
	if(Serial.available()){
		// ref_1 = Serial.parseFloat();
		// ref_2 = Serial.parseFloat();
		// ref_3 = Serial.parseFloat(); 
		// ref_4 = Serial.parseFloat(); 
		Serial.readBytes(ref[0].bytes, 16);
	}
	if(output){
		output = FALSE;
		if(updated){
			// turn off input capture interrupt
			cli();
			if(updated & 0x01)
				countLocal_1 = countShared_1;
			if(updated & 0x02)
				countLocal_2 = countShared_2;
			if(updated & 0x04)
				countLocal_3 = countShared_3;
			if(updated & 0x08)
				countLocal_4 = countShared_4;
			// turn on input capture interrupt
			updated = 0x00;
			sei();
		}
		angle_1.data = (countLocal_1/ONE_REVOLUTION_1)*MY_2PI;
		angle_2.data = (countLocal_2/ONE_REVOLUTION_2)*MY_2PI;
		angle_3.data = (countLocal_3/ONE_REVOLUTION_3)*MY_2PI;
		angle_4.data = (countLocal_4/ONE_REVOLUTION_4)*MY_2PI;

		driveMotor_1(ref[0].data,angle_1.data);
		driveMotor_2(ref[1].data,angle_2.data);
		driveMotor_3(ref[2].data,angle_3.data);
		driveMotor_4(ref[3].data);

		#if DEBUG
			// Serial2.print(angle_1.data*180/MY_PI);	
			// Serial2.print(" ");
			// Serial2.print(angle_2.data*180/MY_PI);
			// Serial2.print(" ");
			// Serial2.print(angle_3.data*180/MY_PI);	
			// Serial2.print(" ");
			// Serial2.println(angle_4.data*180/MY_PI);
			Serial2.print(ref[0].data);	
			Serial2.print(" ");
			Serial2.print(ref[1].data);
			Serial2.print(" ");
			Serial2.print(ref[2].data);	
			Serial2.print(" ");
			Serial2.println(ref[3].data);
		#else
			Serial.write(PACKAGE_HEAD);
			Serial.write(angle.bytes,4);
			Serial.write(PACKAGE_TAIL);
		#endif
	}
}

inline void driveMotor_1(float ref, float feedback){
	int16_t v = controller(0,ref,feedback);
	//overcome the friction
	if(prev_count_1 == countLocal_1 && abs(v)<280 && v!=0){	
		if(v>0){
			v += 150;
		}else if(v<0){
			v -= 200;
		}
	}
	
	prev_count_1 = countLocal_1;
	if(v>=0){
		SET_AIN1_PIN_1(LOW);
		SET_AIN2_PIN_1(HIGH);
		PWM_1(v);
	}else{
		SET_AIN1_PIN_1(HIGH);
		SET_AIN2_PIN_1(LOW);
		PWM_1(-v);
	}
}

inline void driveMotor_2(float ref, float feedback){
	int16_t v = controller(1,ref,feedback);
	//overcome the friction
	if(prev_count_2 == countLocal_2 && abs(v)<280 && v!=0){	
		if(v>0){
			v += 50;
		}else if(v<0){
			v -= 80;
		}
	}
	prev_count_2 = countLocal_2;
	if(v>=0){
		SET_AIN1_PIN_2(LOW);
		SET_AIN2_PIN_2(HIGH);
		PWM_2(v);
	}else{
		SET_AIN1_PIN_2(HIGH);
		SET_AIN2_PIN_2(LOW);
		PWM_2(-v);
	}
}

inline void driveMotor_3(float ref, float feedback){
	int16_t v = controller(2,ref,feedback);
	//overcome the friction
	if(prev_count_3 == countLocal_3 && abs(v)<280 && v!=0){	
		if(v>0){
			v += 50;
		}else if(v<0){
			v -= 80;
		}
	}
	prev_count_3 = countLocal_3;
	if(v>=0){
		SET_AIN1_PIN_3(LOW);
		SET_AIN2_PIN_3(HIGH);
		PWM_3(v);
	}else{
		SET_AIN1_PIN_3(HIGH);
		SET_AIN2_PIN_3(LOW);
		PWM_3(-v);
	}
}

inline void driveMotor_4(float ref){
	if(ref>=0){
		SET_AIN1_PIN_4(LOW);
		SET_AIN2_PIN_4(HIGH);
		PWM_4(uint8_t(ref));
	}else{
		SET_AIN1_PIN_4(HIGH);
		SET_AIN2_PIN_4(LOW);
		PWM_4(uint8_t(-ref));
	}
}

/*
	Compute the control signal in PWM 
	@param ref: reference 
	@param feedback: angle
	@return the PWM signal
*/
inline int16_t controller(int motor, float ref, float feedback){
	float v = kp[motor]*(b*ref-feedback)+integrator(motor,ref-feedback-k_anti[motor]*diff[motor])+PIDFderivative(motor,kd[motor]*(c*ref-feedback));

	if(v>SATURATION){
		diff[motor] = v-SATURATION;
		v = SATURATION;
	}else if(v<-SATURATION){
		diff[motor] = v-SATURATION;
		v = -SATURATION;
	}else
		diff[motor] = 0;
	if(motor==3)
		return 255*(v/SATURATION);
	return 1023*(v/SATURATION);
}

inline float integrator(int motor,float u_k){
	float tmp = int_y_k[motor];
	int_y_k[motor] = int_y_k[motor] + ki[motor]*dt*u_k;
	return tmp;
}

inline float PIDFderivative(int motor,float u_k){
	de_y_k_1[motor] = ((Tf-dt)*de_y_k_1[motor]+kd[motor]*u_k-kd[motor]*de_u_k_1[motor])/Tf;
	de_u_k_1[motor] = u_k;
	return de_y_k_1[motor];
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

void pinChangeInterruptSetup(){
	//The rising edge of INT0 generates asynchronously an interrupt request
	EICRA = _BV(ISC00) | _BV(ISC01) | _BV(ISC10) | _BV(ISC11) | _BV(ISC20) | _BV(ISC21) | _BV(ISC30) | _BV(ISC31);
	EIMSK = _BV(INT0) | _BV(INT1) | _BV(INT2) | _BV(INT3);
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
// void calculateIntervalSetup(){
// 	//Timer/Counter 3
//     TCCR3A = 0x00;
//     TCCR3B = (_BV(WGM32)) | (_BV(CS31)) | (_BV(CS30));  //CTC mode, clk/64
//     OCR3A = 0xFFFF;
// }

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
	encoderPulseShared_1 = READ_C2_1;
	countShared_1 = (encoderPulseShared_1)?countShared_1-1:countShared_1+1;
	updated |= 0x01;
}

ISR(INT1_vect){
	encoderPulseShared_2 = READ_C2_2;
	countShared_2 = (encoderPulseShared_2)?countShared_2-1:countShared_2+1;
	updated |= 0x02;
}

ISR(INT2_vect){
	encoderPulseShared_3 = READ_C2_3;
	countShared_3 = (encoderPulseShared_3)?countShared_3-1:countShared_3+1;
	updated |= 0x04;
}

ISR(INT3_vect){
	encoderPulseShared_4 = READ_C2_4;
	countShared_4 = (encoderPulseShared_4)?countShared_4-1:countShared_4+1;
	updated |= 0x08;
}
#include "ATMEGA2560.h"
#include "I2C.h"

#define OUTPUT_COMPARE_TIME_INTERVAL 0x1388		//set to 20ms
#define OUTPUT_COMPARE_ENCODER  0xF424 //set to 1s
#define TICK_PER_SEC_GEARBOX 10812.5
#define ONE_REVOLUTION 2085
#define DEGREES_PER_PULSE 0.173
//X1 Encoding
#define POSITIVE_DIR 0x40		//bits 5,6 -> 10, actually this is equal to true
#define NEGATIVE_DIR 0x00		//bits 5,6 -> 00, actually this is false

#define TRUE 1
#define FALSE 0

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

volatile uint16_t clockTickShared;
volatile uint8_t encoderPulseShared = 0x00;
volatile int32_t countShared = 0;
uint8_t encoderPulseLocal = 0x00;
uint8_t encoderStatesLocal = 0x00;
uint16_t clockTickLocal = 0x00;
uint8_t updated = 0x00;

typedef union FLOAT{
	float data;
	uint8_t bytes[4];
}Float;

Float vel,degree;
int32_t countLocal = 0;

int16_t voltRegValue, currentRegVal;
float volt = 0;
float current = 0;

using namespace TWI;
uint8_t tt = 0;

void setup(){
	vel.data = degree.data = 0;
	Serial.begin(115200);
	SET_STBY_PIN_OUT;
	SET_AIN1_PIN_1_OUT;
	SET_AIN1_PIN_1_OUT;
	SET_C1_READ_PIN_1_IN;
	SET_C2_READ_PIN_1_IN;
	SET_STBY_PIN(HIGH);
	SET_AIN2_PIN_1(HIGH);
	SET_AIN1_PIN_1(LOW);
	// INA219Setup();
	encoderClockSetup();
	PWM_setup();
	PWM(255);
}

void loop(){ 
	if(updated){
		cli();
		updated = false;
		encoderStatesLocal = encoderPulseShared;
		clockTickLocal = clockTickShared;
		countLocal = countShared;
		sei();
		vel.data = encoderStatesLocal ? -TICK_PER_SEC_GEARBOX/clockTickLocal : TICK_PER_SEC_GEARBOX/clockTickLocal;
		degree.data = (countLocal/ONE_REVOLUTION)*360+countLocal%ONE_REVOLUTION*DEGREES_PER_PULSE;
		Serial.print(vel.data);
		Serial.print(",");
		Serial.println(degree.data);
	}
	if(Serial.available()){
		int v = Serial.parseInt();
		if(v & NEGATIVE_MASK){
			SET_AIN2_PIN_1(LOW);
			SET_AIN1_PIN_1(HIGH);
			PWM(-v);
		}else{
			SET_AIN2_PIN_1(HIGH);
			SET_AIN1_PIN_1(LOW);
			PWM(v);
		}
	}
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
	volt = voltRegValue*BUS_VOL_LSB;
}

inline void getCueent(){
	int16_t currentRegVal;
	startTrans(INA219_ADDRESS,WRITE);
	write(CURRENT_REG);
	requestFrom(INA219_ADDRESS,1,true);
	currentRegVal = (readBuffer()<<8) | readBuffer();
	current = currentRegVal*CURRENT_LSB;
}

// void pinChangeSetup(){
// 	cli();
// 	//INT
// 	// EICRA = _BV(ISC10) | _BV(ISC00);    //Any logical change on INT1 and INT0 generates an interrupt request.
// 	// EIMSK = _BV(INT0) | _BV(INT1);		//interrupt enable
// 	//PCINT
// 	PCMSK0 = _BV(PCINT0);		//Set PCINT0
// 	PCICR = _BV(PCIE0);			//Enable PCINT
// 	sei();
// }

// counter 2
void PWM_setup(){
	// Fasr PWM Mode, Set OC2B on Compare Match when up-counting Clear OC2A on Compare Match when down-counting,
	// TOP = 0xFF, CLK/128
	// if set TOP to OCRA, has to set OCR0A as well.
	SET_PWM_PIN_OUTPUT;
	TCCR0A = _BV(COM0B1)  | _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(CS00) | _BV(CS01);
}

void PWM(uint8_t value){
	OCR0B = value;
}

void timeIntervalSetup(){
	cli();  //disable the global interrupt
    //Timer/Counter 1
    TCCR3A = 0x00;
    TCCR3B = (_BV(WGM32)) | (_BV(CS31)) | (_BV(CS30));  //CTC mode, clk/64
    OCR3A = OUTPUT_COMPARE_TIME_INTERVAL; //set to 200ms
    TCNT3 = 0x00; //initialise the counter
    TIMSK3 = _BV(OCIE3A);  //Output Compare A Match Interrupt Enable
    sei(); //enable global interrupt
}

void encoderClockSetup(){
	cli();  //disable the global interrupt
    //Timer/Counter 4
    TCCR4A = 0x00;
    TCCR4B = (_BV(WGM42)) | (_BV(CS42)) | (_BV(ICES4));  //CTC mode, clk/256, rising (positive) edge will trigger the capture.
    OCR4A = OUTPUT_COMPARE_ENCODER; //set to 1s
    TCNT4 = 0x00; //initialise the counter
	TIMSK4 = _BV(ICIE4);   //enable Input Capture Interrupt
    sei(); //enable global interrupt
}

ISR(TIMER4_CAPT_vect){
	TCNT4 = 0x00;
	encoderPulseShared = READ_C2_1;
	clockTickShared = ICR4;
	countShared = (encoderPulseShared)?countShared-1:countShared+1;
	updated = TRUE;
}

ISR(TIMER3_COMPA_vect){
	updated = TRUE;
}
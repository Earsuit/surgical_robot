#ifndef ATMEGA2560_H_
#define ATMEGA2560_H_

#define IN  0x00
#define OUT 0xFF
#define HIGH 0xFF
#define LOW 0x00

//Motor driver 1
#define STBY_PIN PG5        //D8
//Motor 1
#define PWM_PIN_1 PG5       //D4, OC2B
#define AIN1_PIN_1 PA0      //D22, need to set as an output
#define AIN2_PIN_1 PA1      //D23, need to set as an output
#define C1_COUNT_PIN_1 PL0  //D49, use PCINT0, belongs to PCMSK0
#define C2_COUNT_PIN_1 PJ0  //RXD3,(NOT USED) use PCINT9, belongs to PCMSK1
#define C1_READ_PIN_1 PC5   //D32, need to set as an input
#define C2_READ_PIN_1 PC6   //D31, need to set as an input
//For system identification
#define VELOCITY_PIN_1 PL0  //D49, use ICP4

#define SET(PORT,MASK,VALUE)    PORT = ( (MASK & VALUE) | (PORT & ~MASK) )
#define GET(PORT,MASK)         (PORT & MASK)

#define SET_AIN1_PIN_1_OUT          SET(DDRA,_BV(AIN1_PIN_1),OUT)
#define SET_AIN2_PIN_1_OUT          SET(DDRA,_BV(AIN2_PIN_1),OUT)
#define SET_C1_READ_PIN_1_IN        SET(DDRC,_BV(C1_READ_PIN_1),IN)
#define SET_C2_READ_PIN_1_IN        SET(DDRC,_BV(C2_READ_PIN_1),IN)
#define SET_PWM_PIN_OUTPUT          SET(DDRG,_BV(PWM_PIN_1),OUT)
#define SET_STBY_PIN_OUT            SET(DDRH,_BV(STBY_PIN),OUT)

#define SET_AIN1_PIN_1(VAL) SET(PORTA,_BV(AIN1_PIN_1),VAL)
#define SET_AIN2_PIN_1(VAL) SET(PORTA,_BV(AIN2_PIN_1),VAL)
#define SET_STBY_PIN(VAL) SET(PORTH,_BV(STBY_PIN),VAL)

#define READ_C1_1 GET(PINC,_BV(C1_READ_PIN_1))
#define READ_C2_1 GET(PINC,_BV(C2_READ_PIN_1))

#endif
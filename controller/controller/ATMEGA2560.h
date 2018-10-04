#ifndef ATMEGA2560_H_
#define ATMEGA2560_H_

#define IN  0x00
#define OUT 0xFF
#define HIGH 0xFF
#define LOW 0x00

//Motor 1, M1 to 2, M2 to 1
#define PWM_PIN_1 PL3       //D46, OC5A, connect to A1 in
#define AIN1_PIN_1 PA0      //D22, need to set as an output, connect to A2 in
#define AIN2_PIN_1 PA1      //D23, need to set as an output, connect to A2 in
#define C1_COUNT_PIN_1 PD0  //D21, Phase B, SCL, use INT0
#define C2_READ_PIN_1 PC6   //Phase A, D31, need to set as an input
//Motor 2 
#define PWM_PIN_2 PH3       //D6, OC4A, connect to A1 in
#define AIN1_PIN_2 PA2      //D24, need to set as an output
#define AIN2_PIN_2 PA3      //D25, need to set as an output
#define C1_COUNT_PIN_2 PD1  //D20, Phase B, SDA, use INT1
#define C2_READ_PIN_2 PC5   //Phase A, D32, need to set as an input
//Motor 3
#define PWM_PIN_3 PE3       //D5, OC3A, connect to A1 in
#define AIN1_PIN_3 PA4      //D26, need to set as an output, connect to A2 in
#define AIN2_PIN_3 PA5      //D27, need to set as an output, connect to A2 in
#define C1_COUNT_PIN_3 PD2  //D19, Phase B, use INT2
#define C2_READ_PIN_3 PC4   //Phase A, D33, need to set as an input
//Motor 4
#define PWM_PIN_4 PG5       //D4, OC0B, connect to A1 in
#define AIN1_PIN_4 PA6      //D28, need to set as an output, connect to A2 in
#define AIN2_PIN_4 PA7      //D29, need to set as an output, connect to A2 in
#define C1_COUNT_PIN_4 PD3  //D18, Phase B, use INT3
#define C2_READ_PIN_4 PC3   //Phase A, D34, need to set as an input

#define SET(PORT,MASK,VALUE)    PORT = ( (MASK & VALUE) | (PORT & ~MASK) )
#define GET(PORT,MASK)         (PORT & MASK)

#define SET_AIN1_PIN_1_OUT          SET(DDRA,_BV(AIN1_PIN_1),OUT)
#define SET_AIN2_PIN_1_OUT          SET(DDRA,_BV(AIN2_PIN_1),OUT)
#define SET_C2_READ_PIN_1_IN        SET(DDRC,_BV(C2_READ_PIN_1),IN)
#define SET_PWM_PIN_1_OUTPUT        SET(DDRL,_BV(PWM_PIN_1),OUT)

#define SET_AIN1_PIN_2_OUT          SET(DDRA,_BV(AIN1_PIN_2),OUT)
#define SET_AIN2_PIN_2_OUT          SET(DDRA,_BV(AIN2_PIN_2),OUT)
#define SET_C2_READ_PIN_2_IN        SET(DDRC,_BV(C2_READ_PIN_2),IN)
#define SET_PWM_PIN_2_OUTPUT        SET(DDRH,_BV(PWM_PIN_2),OUT)

#define SET_AIN1_PIN_3_OUT          SET(DDRA,_BV(AIN1_PIN_3),OUT)
#define SET_AIN2_PIN_3_OUT          SET(DDRA,_BV(AIN2_PIN_3),OUT)
#define SET_C2_READ_PIN_3_IN        SET(DDRC,_BV(C2_READ_PIN_3),IN)
#define SET_PWM_PIN_3_OUTPUT        SET(DDRE,_BV(PWM_PIN_3),OUT)

#define SET_AIN1_PIN_4_OUT          SET(DDRA,_BV(AIN1_PIN_4),OUT)
#define SET_AIN2_PIN_4_OUT          SET(DDRA,_BV(AIN2_PIN_4),OUT)
#define SET_C2_READ_PIN_4_IN        SET(DDRC,_BV(C2_READ_PIN_4),IN)
#define SET_PWM_PIN_4_OUTPUT        SET(DDRG,_BV(PWM_PIN_4),OUT)

#define SET_AIN1_PIN_1(VAL) SET(PORTA,_BV(AIN1_PIN_1),VAL)
#define SET_AIN2_PIN_1(VAL) SET(PORTA,_BV(AIN2_PIN_1),VAL)
#define SET_AIN1_PIN_2(VAL) SET(PORTA,_BV(AIN1_PIN_2),VAL)
#define SET_AIN2_PIN_2(VAL) SET(PORTA,_BV(AIN2_PIN_2),VAL)
#define SET_AIN1_PIN_3(VAL) SET(PORTA,_BV(AIN1_PIN_3),VAL)
#define SET_AIN2_PIN_3(VAL) SET(PORTA,_BV(AIN2_PIN_3),VAL)
#define SET_AIN1_PIN_4(VAL) SET(PORTA,_BV(AIN1_PIN_4),VAL)
#define SET_AIN2_PIN_4(VAL) SET(PORTA,_BV(AIN2_PIN_4),VAL)

#define READ_C2_1 GET(PINC,_BV(C2_READ_PIN_1))
#define READ_C2_2 GET(PINC,_BV(C2_READ_PIN_2))
#define READ_C2_3 GET(PINC,_BV(C2_READ_PIN_3))
#define READ_C2_4 GET(PINC,_BV(C2_READ_PIN_4))

#endif
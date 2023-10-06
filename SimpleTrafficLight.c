// SimpleTrafficLight.c
// Runs on TM4C123
// Index implementation of a Moore FSM to operate a traffic light.
// Daniel Valvano, Jonathan Valvano
// July 20, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
 // Modified by Dr. Min He on 08/14/2020: removed PLL and SysTick, used software 
 // loop to generate delay, moved port inititlization code to init functions,
 // changed the FSM timing to: 2s for green, 1s for yellow, and
 // migrated to keil5.

// east facing red light connected to PB5
// east facing yellow light connected to PB4
// east facing green light connected to PB3
// north facing red light connected to PB2
// north facing yellow light connected to PB1
// north facing green light connected to PB0
// north facing car detector connected to PE1 (1=car present)
// east facing car detector connected to PE0 (1=car present)
#include <stdint.h>   // for data type alias


#define LIGHT                   (*((volatile unsigned long *)0x400050FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002400C)) // bits 1-0
#define SENSOR                  (*((volatile unsigned long *)0x4002400C))

#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control

void Delay(uint8_t n);
void Light_Init(void);
void Sensor_Init(void);

// FSM state data structure
struct State {
  uint32_t Out; 
  uint32_t Time;  
  uint32_t Next[4];
}; 

typedef const struct State STyp;

// Constants definitions
#define goN   0
#define waitN 1
#define goE   2
#define waitE 3

STyp FSM[4]={
 {0x21,2,{goN,waitN,goN,waitN}}, 
 {0x22,1,{goE,goE,goE,goE}},
 {0x0C,2,{goE,goE,waitE,waitE}},
 {0x14,1,{goN,goN,goN,goN}}};

 
int main(void){ 
  uint32_t Input; 
  uint32_t S;  // index to the current state 
	
	Light_Init();
	Sensor_Init();
  S = goN;                     // FSM start with green on north, 
    
  while(1){
    LIGHT = FSM[S].Out;  // set lights
    Delay(FSM[S].Time);
    Input = SENSOR;     // read sensors
    S = FSM[S].Next[Input];  
  }
}

void Delay(uint8_t n){
	volatile uint32_t time;
	
  time = n*727240*100/91;  // 0.1sec
  while(time){
		time--;
  }
}

void Light_Init(void){
	SYSCTL_RCGC2_R |= 0x02;      // Activate Port B clocks
	while ((SYSCTL_RCGC2_R&0x02)!=0x02){} // wait for clock to be active
		
	GPIO_PORTB_AMSEL_R &= ~0x3F; // Disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // Enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // Outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // Regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // Enable digital signals on PB5-0
}

void Sensor_Init(void){
	SYSCTL_RCGC2_R |= 0x10;      // Activate Port E clocks
	while ((SYSCTL_RCGC2_R&0x10)!=0x10){} // wait for clock to be active
		
  GPIO_PORTE_AMSEL_R &= ~0x03; // Disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // Enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x03;   // Inputs on PE1-0
  GPIO_PORTE_AFSEL_R &= ~0x03; // Regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x03;    // Enable digital on PE1-0
}

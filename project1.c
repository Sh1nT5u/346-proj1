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
#include "SysTick.h"


#define LIGHT                   (*((volatile unsigned long *)0x400050FC))

#define LEDS										(*((volatile unsigned long *)0x40025018))
//0x40025018

#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002400C)) // bits 1-0
#define SENSOR                  (*((volatile unsigned long *)0x4002400C))
#define board_button            (*((volatile unsigned long *)0x40025004))

#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control

#define STCTRL 									(*((volatile unsigned long *)0xE000E010))
#define STRELOAD 								(*((volatile unsigned long *)0xE000E014))
#define STCURRENT							 	(*((volatile unsigned long *)0xE000E018))
	

#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control

	
void Delay(uint8_t n);
void Light_Init(void);
void Sensor_Init(void);


// FSM state data structure
struct State {
  uint32_t Out;
	uint32_t on_board_out;
  uint32_t Time;  
  uint32_t Next[8];
}; 


typedef const struct State STyp;

// Constants definitions
//#define goN   0
//#define waitN 1
//#define goE   2
//#define waitE 3

//STyp FSM[4]={
 //{0x21,2,{goN,waitN,goN,waitN}}, 
 //{0x22,1,{goE,goE,goE,goE}},
 //{0x0C,2,{goE,goE,waitE,waitE}},
 //{0x14,1,{goN,goN,goN,goN}}};

 // output, time, next
 // Constants definitions
#define West   0
#define WaitWestS 1
#define WaitWestRT 2
#define South   3
#define WaitSouth 4
#define Rightturn 5
#define WaitRightS 6
#define WaitRightW 7

STyp FSM[8]={
    // (West) default = west green, South and RT red
    {0x0C,0x02, 4,{West, West, WaitWestS, WaitWestS, WaitWestRT, WaitWestRT, WaitWestRT, WaitWestRT}},
 
    // (WaitWestS) Change west to yellow then red and wait to turn South green
    {0x14,0x02, 1,{South, South, South, South, South, South, South, South}},
 
    // (WaitWestRT) Change west to yellow then red and wait to turn on RT and South
    {0x14,0x02, 1,{WaitWestRT, WaitWestRT, WaitWestRT, WaitWestRT, WaitWestRT, WaitWestRT, WaitWestRT, WaitWestRT}},
 
    // (South) South green, RT & West red
    {0x21,0x02, 3,{WaitSouth, WaitSouth, South, WaitSouth, Rightturn, Rightturn, Rightturn, Rightturn}},
 
    // (WaitSouth) Change South to yellow then red and wait to turn West Green
    {0x22,0x02, 1,{West, West, West, West, West, West, West, West }},
 
    // (RightTurn) RT and South Green, West red
    {0x21,0x40, 4,{WaitRightW, WaitRightW, WaitRightS, WaitRightS, Rightturn, WaitRightW, Rightturn, WaitRightW }},
 
    // (waitRightS) South Green, RT yellow then red, West red
    {0x22,0x06, 1,{South, South, South, South, South, South, South, South}},
 
    // (waitRightW) SOuth and Right turn yellow then red, then turn West green
    {0x22,0x06, 1,{West, West, West, West, West, West, West, West}},
};

 
 
 
 
//turn light
// PF3 green
// PF3+PF1 “yellow” 
// PF1 red LED 
// determine on board switch to toggle turn lights
 
// signals start with red on south and right turn, and green on west 
//when no sensors activated stay in current state
// if one sensor is true, then turn that side to green for duration of sensor staying true
//if right turn is true, then south is also true, but south can activate independantly of right turn.
 // If both are true, cycle through with the following: 
 // Green 6s
 //Yellow 2s
 //Red 8s
 
int main(void){ 
  uint32_t Input; 
  uint32_t S;  // index to the current state 
	SysTick_Init();
	
	Light_Init();
	LIGHT = 0x0C;
	Sensor_Init();
	LIGHT = 0xFF;
	S = West;                     // FSM start with the West light as green, 
    
  while(1){

    LIGHT = FSM[S].Out;  // set lights
		LEDS = FSM[S].on_board_out; //set onboard LEDS
		
		
		SysTick_Wait05s(FSM[S].Time);
		
		
    Input = SENSOR | board_button;     // read sensors
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
	SYSCTL_RCGC2_R |= 0x30;      // Activate Port E & F clocks
	while ((SYSCTL_RCGC2_R&0x10)!=0x10){} // wait for clock to be active
		
  GPIO_PORTE_AMSEL_R &= ~0x03; // Disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // Enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x03;   // Inputs on PE1-0
  GPIO_PORTE_AFSEL_R &= ~0x03; // Regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x03;    // Enable digital on PE1-0
		
	//port F  PF4 and PF 0 are onboard switches sw1, and sw2 respectively
	// PF 1, 2, 3 are red, blue, green respectively
	GPIO_PORTF_AMSEL_R &= ~0xFF; // Disable analog function on PF1-0
  GPIO_PORTF_PCTL_R &= ~0x000000FF; // Enable regular GPIO
  GPIO_PORTF_DIR_R &= ~0xFF;   // Inputs on PF1-0
  GPIO_PORTF_AFSEL_R &= ~0xFF; // Regular function on PF1-0
  GPIO_PORTF_DEN_R |= 0xFF;    // Enable digital on PF1-0
}

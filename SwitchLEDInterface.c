// ***** 0. Documentation Section *****
// SwitchLEDInterface.c for Lab 8
// Runs on LM4F120/TM4C123
// Use simple programming structures in C to toggle an LED
// while a button is pressed and turn the LED on when the
// button is released.  This lab requires external hardware
// to be wired to the LaunchPad using the prototyping board.
// January 15, 2016
//      Jon Valvano and Ramesh Yerraballi

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

#define PE0        		  (*((volatile unsigned long *)0x40024004))
#define PE1          		(*((volatile unsigned long *)0x40024008))
/*
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
*/
	

// ***** 2. Global Declarations Section *****
unsigned long SW1;  // input from PE0
unsigned short LEDStatus;

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

void PortE_Init(void);
void PortF_Init(void);
void Delay1ms(unsigned long msec);
void LEDOn(void);
void LEDOff(void);

// ***** 3. Subroutines Section *****

// PE0, PB0, or PA2 connected to positive logic momentary switch using 10k ohm pull down resistor
// PE1, PB1, or PA3 connected to positive logic LED through 470 ohm current limiting resistor
// To avoid damaging your hardware, ensure that your circuits match the schematic
// shown in Lab8_artist.sch (PCB Artist schematic file) or 
// Lab8_artist.pdf (compatible with many various readers like Adobe Acrobat).
int main(void){ 
//**********************************************************************
// The following version tests input on PE0 and output on PE1
//**********************************************************************
  TExaS_Init(SW_PIN_PE0, LED_PIN_PE1, ScopeOn);  // activate grader and set system clock to 80 MHz
  PortE_Init();
//	PortF_Init();
	
  EnableInterrupts();           // enable interrupts for the grader
	
	LEDOn();
	
  while(1){
				Delay1ms(100); //Wait 100 ms for system to stabilize
				
				SW1 = PE0;     // read PE0 into SW1
				LEDStatus = PE1;  //Read LED Status
				if (SW1)
				{
					if (LEDStatus == 0x02)
					{
						LEDOff();
					}
					else
					{
						LEDOn();
						
					}
				}
				else if (LEDStatus == 0x00)
				{
					LEDOn();
				}
  }
  
}

void PortE_Init(void)
	{ 
		volatile unsigned long delay;
		SYSCTL_RCGC2_R |= 0x00000010;     // 1) E clock 
		delay = SYSCTL_RCGC2_R;           // delay   
		GPIO_PORTE_LOCK_R = 0x4C4F434B;   // 2) unlock PortE PE0  
		GPIO_PORTE_CR_R = 0x1F;           // allow changes to PE4-0       
		GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog function
		GPIO_PORTE_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
		GPIO_PORTE_DIR_R = 0x0E;          // 5) PE4,PE0 input, PE3,PE2,PE1 output   
		GPIO_PORTE_AFSEL_R = 0x00;        // 6) no alternate function
		GPIO_PORTE_PUR_R = 0x00;          // enable pullup resistors on PE4,PE0       
		GPIO_PORTE_DEN_R = 0x1F;          // 7) enable digital pins PE4-PE0       
		
}
	
void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;      // 1) F clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize 
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  	
  GPIO_PORTF_AMSEL_R &= 0x00;        // 2) disable analog function
  GPIO_PORTF_PCTL_R &= 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R &= ~0x10;         // 4.1) PF4 input,
  GPIO_PORTF_DIR_R |= 0x0E;          // 4.2) PF3,2,1 output  
  GPIO_PORTF_AFSEL_R &= 0x00;        // 5) no alternate function
  GPIO_PORTF_PUR_R |= 0x10;          // 6) enable pullup resistor on PF4       
  GPIO_PORTF_DEN_R |= 0x1E;          // 7) enable digital pins PF4-PF1
}
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E

	
// Subroutine to delay in units of milliseconds
// Inputs:  Number of milliseconds to delay
// Outputs: None
// Notes:   assumes 80 MHz clock
void Delay1ms(unsigned long msec){
// write this function
  unsigned long i;
   
  while(msec > 0){
    i = 13333;  // this number means 1ms
    while(i > 0){
      i = i - 1;
    }
    msec = msec - 1; // decrements every 1 ms
  }
}

void LEDOn(void)
{
	PE1 = 0x02;       // LED is On at PE1;
	//LEDStatus =1;
}

void LEDOff(void)
{
	PE1 = 0x00;       // LED is Off at PE1;
	//LEDStatus =0;
}


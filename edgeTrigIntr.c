
//*****************************************************************************
//Edge-triggered Interrupt on PORTF pin 4 (left side swich on launchpad board)
// --flashes blue LED when edge-trigerred interrupt occurs on PF4
//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/tm4c123gh6pm.h"

#define RED_LED	  GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
#define MS_DELAY_500 375000
#define SYS_CLOCK 4000000

#define NVIC_EN0_INT30          0x40000000  // Interrupt 30 enable
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
//NVIC_EN0_R   IRQ 0 to 31 Set Enable Register
//NVIC_PRI7_R  IRQ 28 to 31 Priority Register

uint32_t static toggle =1;

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void SystemInit(){
	// set system clock ( 4 Mhz )
	SysCtlClockSet(SYSCTL_SYSDIV_50 | 
	SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

void flashLights(){
	if (toggle == 1){
		DisableInterrupts();
		SysCtlDelay(MS_DELAY_500);
		  GPIO_PORTF_DATA_R |= GPIO_PIN_2;
		toggle = 0;
		EnableInterrupts();
       //
		
	} else {
		DisableInterrupts();
		SysCtlDelay(MS_DELAY_500);
		GPIO_PORTF_DATA_R &= ~GPIO_PIN_2;
		toggle = 1;
		EnableInterrupts();
	}
	/*uint8_t i;
	for( i = 0; i < loops; i++){
	       // Toggle the LED.
        //
        GPIO_PORTF_DATA_R |= GPIO_PIN_2;
       //
	// wait for specified time
		   SysCtlDelay(MS_DELAY_500);
		
		 GPIO_PORTF_DATA_R &= ~GPIO_PIN_2;
		
		// wait for specified time
		    SysCtlDelay(MS_DELAY_500);
	}*/
}

void  GPIOPortF_Handler(){
	 GPIO_PORTF_ICR_R = GPIO_PIN_4;   // acknowledge flag4
	/*if (GPIOPinRead(GPIO_PORTF_DATA_R, GPIO_PIN_4)){

		
	} else {

		flashLights();
	}*/
	//toggle led
	SysCtlDelay(MS_DELAY_500);
	flashLights();
}

void alternate_gpioInit(){
	// **** Using Tivaware Peripheral Driver Library Function Calls ****
	// init port f ( use pin 4 for switch 1 )
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	// wait for port f enable
	 while ( !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF) ) {}
	// configure port f pin 4 as input
   GPIO_PORTF_DIR_R &= ~GPIO_PIN_4;
	// enable weak pull ups on pin 4
	 GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, 
										0x00, GPIO_PIN_TYPE_STD_WPU);
	// register interrupt for port f pin 4
	  GPIOIntRegister(GPIO_PORTF_BASE, GPIOPortF_Handler);
	// set interrupt type ( rising edge )
   GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
	// enable interrupts on port f ( rising edge )
  	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
	// initialize pin2 for the blue LED
	  GPIO_PORTF_DIR_R |= GPIO_PIN_2;
    GPIO_PORTF_DEN_R |= GPIO_PIN_2;
}
	
void gpioInit(){ 
	uint32_t fallingedges;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
	fallingedges = 0;  // short wait
	GPIO_PORTF_LOCK_R = 0x4C4F434B;  //  unlock Port F
	GPIO_PORTF_CR_R |= (GPIO_PIN_4 + GPIO_PIN_2);      //allow changes to PF0 and PF2
  GPIO_PORTF_DIR_R &= ~GPIO_PIN_2;    //  make PF4 input (built-in button)
	GPIO_PORTF_DIR_R |= 0x04;			//     and PF2 output (Blue LED)
  GPIO_PORTF_AFSEL_R &= ~(GPIO_PIN_4 + GPIO_PIN_2);  //     disable alt funct on PF4 and PF2
  GPIO_PORTF_DEN_R |= (GPIO_PIN_4 + GPIO_PIN_2);    //     enable digital I/O on PF4 and PF2   
  GPIO_PORTF_PCTL_R &= ~0x000F0F00; // configure PF4 and PF2 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //  disable analog functionality on PF
	GPIO_PORTF_PUR_R |= GPIO_PIN_4;     //  enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~GPIO_PIN_4;     //  PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~GPIO_PIN_4;    //  PF4 is not both edges
  GPIO_PORTF_IEV_R |= GPIO_PIN_4;     // PF4 rising edge event 
  GPIO_PORTF_ICR_R = GPIO_PIN_4;      //  clear flag0
  GPIO_PORTF_IM_R |= GPIO_PIN_4;      //  arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = NVIC_EN0_INT30;  // (h) enable interrupt 30 in NVIC
  EnableInterrupts();           // 
}

int main(){
	
	// init required gpio pins
	gpioInit();
	//flashLights(); // test LED's initially
	
	while ( 1 ) ; // wait for interrupts
	
}

#include "Dio.h"
#include "types.h"
#include "TM4C123GH6PM.h"
#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_sysctl.h"
/*initializations*/
void PortF_Init(void);
void PortB_Init(void);
void PortA_Init(void);
void PortD_Init(void);
void ports_inits();
void ManualController( void *pvParameters );
void PassengerTask( void *pvParameters );
void MotorController( void *pvParameters );
void JammTask(void *pvParameters);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Stop(void);
void delayMs(int n);
// Semaphores
SemaphoreHandle_t xUpModeSemaphore;
SemaphoreHandle_t xJammingSemaphore;
// Define task handles
xTaskHandle switchTaskHandle;
xTaskHandle passenger;
// Define Queue handles
xQueueHandle xMotor;
xQueueHandle xPassenger;
/*------------------Functions----------*/
void Motor_Forward(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,1);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,0);
}
void Motor_Backward(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,0);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,1);
}
void Motor_Stop(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,1);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,1);
}
void delayMs(int n){
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<3180;j++)
    {}
  }
}

/*----------Task to control Manual Buttons----------*/
void ManualController(void* parameters) {
	portBASE_TYPE xStatus;
    while (1) {
        if ((GPIO_PORTB_DATA_R & 0x4)==0x4) { //Driver Press Passenger Manual Up
					xStatus=xQueueSendToBack(xMotor,&(uint32_t){1},0);
				}
				if  ((GPIO_PORTB_DATA_R & 0x08)==0x08){  //Driver Press Passenger Manual Down
					xStatus=xQueueSendToBack(xMotor,&(uint32_t){2},0);
				}
				if (((GPIO_PORTB_DATA_R & 0x40)==0x40)&&((GPIO_PORTA_DATA_R & 0x40)!=0x40))   //Passenger press Manual Up
				{ 
					xStatus=xQueueSendToBack(xMotor,&(uint32_t){1},0);
				}
				if (((GPIO_PORTB_DATA_R & 0x80)==0x80) &&((GPIO_PORTA_DATA_R & 0x40)!=0x40))   //Passenger press Manual Down
				{ 
					xStatus=xQueueSendToBack(xMotor,&(uint32_t){2},0);
				}
        vTaskDelay(pdMS_TO_TICKS(10)); // debounce switch
    }
}
/*-------------------Jammer--------*/
void JammTask(void *pvParameters){
	long message=(long)pvParameters;
	portBASE_TYPE xStatus;
	xSemaphoreTake(xUpModeSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(xUpModeSemaphore,portMAX_DELAY); //Unlocked when automatic up is pressed either from driver or passenger 
		  xSemaphoreTake(xJammingSemaphore,portMAX_DELAY); //Unlocked when Jamming occurs 
			xStatus=xQueueSendToBack(xMotor,&(uint32_t){0},0);  //sends 0 to motor queue to stop the motor
	}
}
/*-----------MotorController-------*/
void MotorController(void *pvParameters){
	uint32_t message;
	for(;;)
{
	xQueueReceive(xMotor,&message,portMAX_DELAY);
	if(message==0){ //Jamming Mode where the Motor Stops then moves Backwards
		Motor_Stop();
		Motor_Backward();
		delayMs(500);
		Motor_Stop();
	}else if(message==1){ //Motor forward
		Motor_Forward();
	}else if(message==2){ //Motor backward
		Motor_Backward();
	}else if(message==3){  //Motor Stops
		Motor_Stop();
  }
 }
}

/*---------------------Passenger Window--------------*/
void PassengerTask(void *pvParameters){
  portBASE_TYPE xStatus;
	uint32_t message;
	for(;;)
{ 
	xQueueReceive(xPassenger,&message,portMAX_DELAY);
	switch (message){
		case 1:  //Driver press passenger Automatic Up
		  xSemaphoreGive(xUpModeSemaphore);
      xStatus=xQueueSendToBack(xMotor,&(uint32_t){1},0);
			break;
	 }
  }
 }
/*------------------------------------------------------------------------*/

                         /*main function*/
/*------------------------------------------------------------------------*/
int main( void )
{ 
	ports_inits();
	xMotor=xQueueCreate(20,sizeof(uint32_t));	
	xPassenger=xQueueCreate(20,sizeof(uint32_t));
	xUpModeSemaphore = xSemaphoreCreateBinary();
	xJammingSemaphore = xSemaphoreCreateBinary();
	__ASM("CPSIE i");
	
	if( (xUpModeSemaphore && xJammingSemaphore&& xMotor && xPassenger) != NULL )
		{
			xTaskCreate(ManualController, "Switch Task", configMINIMAL_STACK_SIZE, NULL, 1, switchTaskHandle);
			xTaskCreate( PassengerTask, "Passenger", configMINIMAL_STACK_SIZE, NULL, 2, passenger );
			xTaskCreate( MotorController, "Motor",configMINIMAL_STACK_SIZE, NULL, 3, NULL );
			xTaskCreate (JammTask, "Jammer", configMINIMAL_STACK_SIZE,NULL,4,NULL);
			/* Start the scheduler so the created tasks start executing. */
			vTaskStartScheduler();
		}
    for( ;; );
}

/*-----------------------------Initializations-------------------------------------------*/
void ports_inits(){
	PortA_Init();
  PortF_Init();
	PortB_Init();
	PortD_Init();
	}
void PortF_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) F clock
  GPIOF->LOCK = 0x4C4F434B;  				 // 2) unlock PortF PF0  
  GPIOF->CR = 0x1F;          				 // allow changes to PF4-0       
  GPIOF->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOF->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOF->DIR = 0x0E;         				 // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIOF->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOF->PUR = 0x11;       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0x1F;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	GPIOF->ICR = 0x11;     // Clear any Previous Interrupt 
	GPIOF->IM |=0x11;      // Unmask the interrupts for PF0 and PF4
	GPIOF->IS |= 0x11;     // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~0x11;   // Sense on Low Level
	NVIC_EN0_R |= 0x40000000;
  NVIC_PRI7_R |= 0xe0000;  // Enable the Interrupt for PortF in NVIC
}
void PortB_Init(void){

	SYSCTL->RCGCGPIO |= 0x00000002; 	// initialize clock                           
	GPIOB->LOCK = 0x4C4F434B;
	GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOB->CR |= 0x000000ff;           // unlocking commit register for switch 1 & switch 2 
  GPIOB->DIR |= 0x00000000;          // detrmining the output pins                                   
  GPIOB->AFSEL = 0x00; 
	GPIOB->DEN |= 0x000000ff;          // detrmining the pins direction 
  GPIOB->DATA |= 0x00000000;
	GPIOB -> PDR = 0xff; 	
	GPIOB->ICR = 0xff;     // Clear any Previous Interrupt
  GPIOB->IM |=0xff;      // Unmask the interrupts for PF0 and PF4	
	NVIC_EN0_R |= 0x02;
	NVIC_PRI0_R |= 0xe000;
}
void PortD_Init(void){

	SYSCTL->RCGCGPIO |= 0x00000008; 	// initialize clock                           
	GPIOD->LOCK = 0x4C4F434B;
	GPIOD->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOD->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOD->CR |= 0x000000ff;           // unlocking commit register for switch 1 & switch 2 
  GPIOD->DIR |= 0x00000000;          // detrmining the output pins                                   
  GPIOD->AFSEL = 0x00; 
	GPIOD->DEN |= 0x000000ff;          // detrmining the pins direction 
  GPIOD->DATA |= 0x00000000;
	GPIOD -> PDR = 0xff; 	
	GPIOD->ICR = 0xff;     // Clear any Previous Interrupt
  GPIOD->IM |=0xff;      // Unmask the interrupts for PF0 and PF4	
	NVIC_EN0_R |= 0x08; 
	NVIC_PRI0_R |= 0xe000; 
}
void PortA_Init(void){
	SYSCTL_RCGCGPIO_R |= 0x00000001;             // initialize clock
	while((SYSCTL_PRGPIO_R&0x00000001) == 0){}   // looping until clock is initiallized
	GPIO_PORTA_CR_R |= 0x000000ff;               // unlocking commit register for switch 1 & switch 2 
  GPIO_PORTA_DIR_R |= 0x0000000c;              // detrmining the output pins                                   
  GPIO_PORTA_DEN_R |= 0x000000ff;              // detrmining the pins direction 
  GPIO_PORTA_DATA_R |= 0x00000000;
	GPIOA -> PDR = 0x40; 
}
/*----------------------------Port-F handler--------------------------------------------*/
void GPIOF_Handler(void){
 uint32_t pinFlags;
 pinFlags = GPIO_PORTF_MIS_R;
/*--------------------------Jammer Button--------------------*/
	 if(pinFlags & (1 << 0)){
  GPIOF->ICR = 0xff; 
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xJammingSemaphore,&xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}
/*-----------------------------Port B Handler-------------------------------*/
void GPIOB_isrHandler(void){

 uint32_t pinFlags;
 pinFlags = GPIO_PORTB_MIS_R;
	//Case Driver Press Passenger Automatic Up
	 if(pinFlags & (1 << 0)){
	 GPIOB->ICR = 0xff;        // clear the interrupt flag of PORTB
	 xQueueSendFromISR(xPassenger,&(uint32_t){1},0);	 
  }
	//Driver Press Passenger Automatic Down
 if(pinFlags & (1 << 1)){
	 GPIOB->ICR = 0xff;        
  xQueueSendFromISR(xMotor,&(uint32_t){2},0);		 
  }
 //Driver Press Passenger Manual Up
	 if(pinFlags & (1 << 2)){	 
		 xQueueSendFromISR(xMotor,&(uint32_t){3},0);	
		 GPIOB->ICR = 0xff;        	
  }
	 //Driver Press Passenger Manual Down
		 if(pinFlags & (1 << 3)){	 
		 xQueueSendFromISR(xMotor,&(uint32_t){3},0);	
		 GPIOB->ICR = 0xff;        
  }
	//Passenger  Press Automatic Up
 if(pinFlags & (1 << 4)){
	  if((GPIO_PORTA_DATA_R & 0x40)!= 0x40){    //Check on Lock Button
			xQueueSendFromISR(xPassenger,&(uint32_t){1},0);	
   }
		GPIOB->ICR = 0xff; 
 }
	//Passenger Press Automatic Down
 if(pinFlags & (1 << 5)){	 
	     if((GPIO_PORTA_DATA_R & 0x40)!= 0x40){    //Check on Lock Button
	 xQueueSendFromISR(xMotor,&(uint32_t){2},0);
 }
	 GPIOB->ICR = 0xff; 
}
	//Passenger Press Manual Up
	if(pinFlags & (1 << 6)){	 
	         // clear the interrupt flag of PORTB
		 xQueueSendFromISR(xMotor,&(uint32_t){3},0);	
		 GPIOB->ICR = 0xff;
}	
	//Passenger Press Manual Down
	 if(pinFlags & (1 << 7)){	 
		  xQueueSendFromISR(xMotor,&(uint32_t){3},0);	
			GPIOB->ICR = 0xff; 
  }
}
/*----Port D Handler---------*/
void GPIOD_isrHandler(void){
 uint32_t pinFlags;
 pinFlags = GPIO_PORTD_MIS_R;
/*-Limit Switch 1----*/
	 if(pinFlags & (1 << 0)){
		 GPIOD->ICR = 0xff;  
     xQueueSendFromISR(xMotor,&(uint32_t){3},0);	
  }
/*---Limit Switch 2---*/
	 if (pinFlags & (1 << 1)){
		 GPIOD->ICR = 0xff;  
    xQueueSendFromISR(xMotor,&(uint32_t){3},0);	
 }
}
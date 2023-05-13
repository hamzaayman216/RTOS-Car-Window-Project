#include "tm4c123gh6pm.h"
#include "bitwise_operation.h"

void DIO_init(void)
{
    SYSCTL_RCGCGPIO_R |= 0X20;
    while((SYSCTL_PRGPIO_R & 0X20)==0);
      GPIO_PORTF_CR_R|= 0X1F;
      GPIO_PORTF_DIR_R|= 0X0E;
      GPIO_PORTF_LOCK_R=0x4c4f434b;
      GPIO_PORTF_PUR_R |= 0x11;
      GPIO_PORTF_DEN_R|= 0X1F;
} 
void DIO_WritePort (unsigned long volatile * Port ,unsigned char value)
{
  *Port = value;
}
void DIO_WritePin(unsigned long  volatile* Port , int Pin, unsigned char value)
{
  if(value == 0)
  {
    clearBit( *Port,  Pin);
  }
  else if(value == 1)
  {
    setBit( *Port,  Pin);
  }
}
//unit8 ReadPin(Port, Pin);

unsigned char DIO_ReadPin( unsigned long  volatile* Port, int Pin) {
  
  return getBit(*Port, Pin);
  
}

unsigned int DIO_ReadPort( unsigned long  volatile* Port) {
  
  return (*Port);
  
}

void DIO_TogglePin(unsigned long  volatile* Port, int Pin) {
  
  toggleBit(*Port, Pin);

}

void DIO_ClearPort (unsigned long volatile * Port)
{
  *Port = 0x00;
}



//unit32 ReadPort(Port);
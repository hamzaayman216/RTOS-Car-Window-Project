void DIO_init(void);
void DIO_WritePort (unsigned long volatile * Port ,unsigned char value);
void DIO_WritePin(unsigned long  volatile* Port , int Pin, unsigned char value);
unsigned char DIO_ReadPin( unsigned long  volatile* Port, int Pin);
unsigned int DIO_ReadPort( unsigned long  volatile* Port);
void DIO_TogglePin(unsigned long  volatile* Port, int Pin);
void DIO_ClearPort (unsigned long volatile * Port);

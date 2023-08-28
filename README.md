# STM32F103-CMSIS-I2C-EEPROM-lib
STM32F103 (Blue Pill) routines and sample memory monitor program for the 24CL64 I2C EEPROM
## To clone this library and sample EEPROM memory monitor:
+ ```git clone http://github.com/sandynomike/STM32-CMSIS-I2C-EEPROM-lib```
+ ```cd STM32-CMSIS-I2C-EEPROM-lib```
+ ```make clean && make```
## Routines included in the STM32F104-CMSIS-I2C-EEPROM-lib.c library:
+ void EE24_init( I2C_TypeDef *thisI2C, uint32_t deviceAdd, uint32_t bytes, uint32_t pageSize )<br>
  Associate and initialize the I2C interface to this I2C EEPROM device
+ void EE24_read( uint32_t address, uint8_t *data, uint32_t length )<br>
  Read in a block of data from the EEPROM device
+ void EE24_write( uint32_t address, uint8_t *data, uint32_t length, uint32_t fill )<br>
  Write data to the EEPROM device. Can either write data from a buffer (*data) or fill the specified
  block with a constant value located at data[0].
+ void EE24_dump( uint32_t address, uint32_t length )<br>
  Perform a memory dump from the EEPROM to the serial port. Note that this routine uses the STM32F103-CMSIS-USART-lib library
  and requires that the USART port be initialized and a serial port be open.

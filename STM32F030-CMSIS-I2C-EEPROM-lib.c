//  STM32F103-CMSIS-I2C-EEPROM-lib.c
//    Routines to write/fill and read data on the 24CL64 I2C EEPROM IC.
//    Mike Shegedin (EZdenki.com)
//
//        Version 1.1    9 Aug 2023   Updated I2C and Delay libraries
//        Version 1.0   18 Jul 2023   Updated comments and core files
//        Started          Jul 2023
//
//  Target Microcontroller: STM32F103 (Blue Pill)
//
//  Target devices:
//    EEPROM: 24CL64 I2C EEPROM
//    This EEPROM device has 8 kB of storage with a 32-byte page size. The code may have to be
//    modified to work with other devices.
//
//  Required Libraries:
//    STM32F103-CMSIS-I2C-lib
//    STM32F103-CMSIS-USART-lib
//      (This requirement can be eliminated if the EE24_dump routine is commented out.)
//  -------------------------------------------------------------------------------------------
//     ***  HARDWARE SETUP for 24CL64 ***          
//                                +----+-----+----- VCC
//             .-------.          |    |     |
//     ;--- A0 |1 o   8| VCC -----+   5.6k  5.6k
//     |--- A1 |2     7| WP -- NC      |     |
//     |--- A2 |3     6| SCL ----------+-----|----- Blue Pill I2C SCL pin
//     |-- GND |4     5| SDA ----------------+----- Blue Pill I2C SDA pin 
//     |       '-------' 
//    _|_
//    \-/
//     V
//
//  -------------------------------------------------------------------------------------------

#ifndef __STM32F030_CMSIS_I2C_EEPROM_LIB_C
#define __STM32F030_CMSIS_I2C_EEPROM_LIB_C


#include "stm32f030x6.h"                  // Primary CMSIS header file
#include "STM32F030-CMSIS-I2C-lib.c"      // I2C library
#include "STM32F030-CMSIS-USART-lib.c"    // USART library
#include "STM32F030-Delay-lib.c"          // For microsecond delay routine

//  -------------------------------------------------------------------------------------------
//  Globals for EE24 Routines Defined in EE24_init() Routine
//  -------------------------------------------------------------------------------------------

I2C_TypeDef *EE24_I2C;    // Global variable to point to the I2C interface used for the EEPROM
uint32_t EE24_BYTES;      // Global of the total number of bytes in this EEPROM
uint32_t EE24_PAGESIZE;   // Maximum writable page size in bytes
uint32_t EE24_ADD;        // EEPROM 7-bit address including any chip-select pins


//  -------------------------------------------------------------------------------------------
//  EE24 Routines
//  -------------------------------------------------------------------------------------------

//  void
//  EE24_init( I2C_TypeDef *thisI2C, uint32_t I2CSpeed, uint32_t deviceAdd,
//             uint32_t bytes, uint32_t pageSize )
//  Set and initialize I2C bus and set parameters to work with this I2C EEPROM.
//  Example for a 24LC64 EEPROM with all chip-select pins grounded on the I2C1 interface:
//    EE24_init( I2C1, 0x50, 0x2000, 32 );
void
EE24_init( I2C_TypeDef *thisI2C, uint32_t I2CSpeed, uint32_t deviceAdd,
           uint32_t bytes, uint32_t pageSize )
{
  EE24_I2C      = thisI2C;    // Set the I2C interface used to talk to this EEPROM device
  EE24_ADD      = deviceAdd;  // Set the I2C address for this device (incl. chip-select pins)
  EE24_BYTES    = bytes;      // Set the number of bytes of storage for this device
  EE24_PAGESIZE = pageSize;   // Set the maximum writable page size for this device

  I2C_init( thisI2C, I2CSpeed );  // Init the I2C interface used for the EEPROM
}


//  void
//  EE24_read( uint32_t address, uint8_t *data, uint32_t length)
//  Reads in a block of data from the EEPROM device.
//    address: The starting address in the EEPROM to read from
//      *data: Points to a byte array that will store the data. Note that the memory for this
//             array must be allocated ahead fo time, and the routine does not check the size
//             of the array, so it is possible for the routine to perform a buffer overflow.
//     length: The number of bytes to read from the EEPROM. Note buffer overflow potential.
//  Returns 0 if read address and length are within the bounds of the device, otherwise will
//  return a value of 1 to indicate the address and/or length are out-of-bounds and no read
//  was performed.

uint32_t
EE24_read( uint32_t address, uint8_t *data, uint32_t length)
{
  int32_t x;  // Counter

  if( (address + length - 1 ) > EE24_BYTES )    // If the address and/or length are too high
    return 1;                                   // then return 1 to indicate that no read
                                                // operation was performed.

                                                // Address Command Frame:
  I2C_setAddress( EE24_I2C, EE24_ADD );         // Set address of EEPROM
  I2C_setNBytes( EE24_I2C, 2 );                 // Set number of bytes to write
  I2C_start( EE24_I2C );                        // Start I2C transaction
  I2C_write( EE24_I2C, (address >> 8) & 0x1F ); // Send EEPROM memory address upper byte
  I2C_write( EE24_I2C, address & 0xFF );        // Send lower byte
  I2C_stop( EE24_I2C );                         // Stop I2C transaction

                                                // Data Read Frame
  I2C_setNBytes( EE24_I2C, length );            // Set number of bytes to read
  I2C_setReadMode( EE24_I2C );                  // Turn on Read Mode
  I2C_start( EE24_I2C );                        // Start I2C transaction
  for( x = 0; x<length; x++)                    // Loop to read in bytes
    data[x] = I2C_read( EE24_I2C );             // Read in one byte per iteration
  I2C_stop( EE24_I2C );                         // Send stop bit
  I2C_setWriteMode( EE24_I2C );                 // Return to Write Mode (default)

  return 0;                                     // Return zero to indicate that read was okay
}


//  void
//  EE24_write( uint32_t address, uint8_t *data, uint32_t length, uint32_t fill)
//  Writes data to EEPROM. 
//  "address" is the starting address to write to
//  "*data" points to the array of 8-bit data to be written. In the case of a fill operation,
//  the first byte ( data[0] ) will be used as the fill value.
//  "length" is the number of bytes to write
//  If "fill" = 0, then the data pointed to by *data will be written to the EEPROM.
//  If "fill" = 1, then the value in data[0] will be used to fill the specified EEPROM area.
//
//  If the write operation would go beyond the memory boundary of the device, then the
//  routine will exit and return a 1 (error), and no data will be written to the device.
//  No other error checking is performed.
//
//  Examples:
//      Write 50 bytes of data contained in myData[] from address 0x0010:
//        EE24_write( 0x0010, myData, 50, 0 );
//
//      Fill with 100 bytes of 0xFF from address 0x0100:
//        myByte = 0xFF;
//        EE24_write( 0x0100, &myByte, 100, 1 );
uint32_t
EE24_write( uint32_t address, uint8_t *data, uint32_t length, uint32_t fill)
{

  uint32_t endAddress;        // Last address to write to
  uint32_t pageStartAddress;  // Starting address for current page
  uint32_t pageEndAddress;    // Ending address for current page
  uint8_t  addHighByte;       // Address High Byte
  uint8_t  addLowByte;        // Address Low Byte
  uint32_t bytesWritten;      // Count of bytes written used for return value.
  
  if( address >= EE24_BYTES )
    return 1;
 
  // First page
  endAddress = address + length - 1;          // Last address to write
  if( endAddress < address )                  // If end address rolls over to become less than
    return 1;                                 // start address then return error code.
  if( endAddress > EE24_BYTES - 1)            // If end address is beyond the write capacity
    return 1;                                 // of the EEPROM then return error code.

  bytesWritten     = 0;                       // Reset bytesWritten count

  pageStartAddress = address;                 // Starting address for this page
  pageEndAddress   = ( address / 32 + 1 ) * 32 - 1;  // Ending address for this page
  

  while( bytesWritten < length )              // Main write loop
  {
    if( endAddress < pageEndAddress )         // If ending in the middle of a page, then
      pageEndAddress = endAddress;            // send the end-of-page accordingly.
  
    addHighByte  = (pageStartAddress >> 8) & 0x1F;  // Split 16-bit address into two 8-bit
    addLowByte   = pageStartAddress & 0xFF;         // addresses.
    uint32_t writeLength = pageEndAddress - pageStartAddress + 3;

    I2C_setAddress( EE24_I2C, EE24_ADD );     // Set I2C address of EEPROM 
    I2C_setNBytes( EE24_I2C, writeLength );      // Set number of bytes to write
    I2C_start( EE24_I2C );                    // Send start bit
    I2C_write( EE24_I2C, addHighByte );       // Send high and low bytes of address
    I2C_write( EE24_I2C, addLowByte );        // to write to.

    
    if( fill )                                // Do "fill" with single value in data[0]
      for( int32_t x = pageStartAddress; x<= pageEndAddress; x++ )
      {
        I2C_write( EE24_I2C, data[0] );       // Write data[0] as fill value for this page
        bytesWritten++;
      }
    else                                      // Do write using data in data[]
      for( int32_t x = pageStartAddress; x<= pageEndAddress; x++ )
      {
        I2C_write( EE24_I2C, data[bytesWritten] );  // Write data[] for this page
        bytesWritten++;
      }

    I2C_stop( EE24_I2C );                     // End frame
    delay_us( 15e3 );                          // Pause to allow for write to complete
    pageStartAddress = pageEndAddress + 1;    // Set next page start and end addresses
    pageEndAddress = pageStartAddress + 31;
  }                                           // End of main write loop

  return 0;                                   // No error
}


//  void
//  EE24_dump( uint32_t address, uint32_t length )
//  Perform memory dump from EEPROM to the serial port.
//  NOTE: Requires STM32F103-CMSIS-USART.lib library and that the USART port be initialized
//        via the "USART_init( USARTx )" routine.
//
//  address: The starting address and length is the number of bytes to display.
//   length: The number of bytes to read from the specified address. Data will be displayed
//           on 0x20 boundaries such that the first and last bytes are shown.
//  Data is displayed in hex and in ASCII. ASCII data will only show printable 7-bit ASCII
//  codes. Non-printable ASCII codes will be shown as a period (.). The routine attempts to
//  make sane adjustments to address and length parameters if they would go beyond the maximum
//  capacity of the device.
void
EE24_dump( uint32_t address, uint32_t length )
{
  uint32_t startAddress, endAddress;
  uint8_t  eeData[16];

  if( address >= EE24_BYTES )             // If address is beyond the max address, then set
    address = EE24_BYTES - 1;             // it to be the max address. 
  
  if( address + length -1 >= EE24_BYTES ) // If length goes beyond max, then set it to end 
    length = EE24_BYTES - address;        // at the last address.

  // Compute starting and ending addresses to fall on 0x0 and 0xF boundaries respectively.
  startAddress = ( address >> 4 ) << 4;
  endAddress   = ( ( address + length -1 ) | 0xF );

  // Print Header, including beginning newline
//  USART_puts( "\nAddress 0. 1. 2. 3. 4. 5. 6. 7. 8. 9. A. B. C. D. E. F. ______ASCII_____\n");

  // Display in 16-byte blocks
  for( uint32_t blockCnt = startAddress; blockCnt < endAddress; blockCnt += 16 )
  {
    EE24_read( blockCnt, eeData, 16 );          // Read in 16 bytes from EEPROM
    USART_puth( blockCnt, 7 );                  // Print Address
    USART_putc( ' ' );

    for( uint32_t x = 0; x<16; x++ )            // Display 16 bytes in hex
    {
      USART_puth( eeData[x], 2 );
      USART_putc( ' ' );
    }

    for( uint32_t x = 0; x<16; x++ )            // Display 16 bytes in ASCII
    {
      char thisChar = eeData[x];                
      if( (thisChar<32) | (thisChar>126) )      // Display non-printable characters as '.'
        USART_putc( '.' );
      else
        USART_putc( thisChar );
    }
  USART_putc('\n');                             // End with newline
  }
}

#endif /* #define __STM32F030_CMSIS_I2C_EEPROM_LIB_C */

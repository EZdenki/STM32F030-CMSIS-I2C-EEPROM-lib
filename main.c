//  ==========================================================================================
//  main.c for STM32F030-CMSIS-I2C-EEPROM-lib.c
//  ------------------------------------------------------------------------------------------
//  Implements an EEPROM memory monitor that allows reading and writing of data to a 24LC64
//  I2C EEPROM using the STM32F030-CMSIS-I2C-EEPROM-lib.c library.
//  ------------------------------------------------------------------------------------------
//  https://github.com/EZdenki/STM32F030-CMSIS-I2C-EEPROM-lib
//  Released under the MIT License
//  Copyright (c) 2023
//  Mike Shegedin, EZdenki.com
//    Version 0.9   27 Aug 2023   Started port of STM32F103-CMSIS-I2C-EEPROM-lib code
//  ------------------------------------------------------------------------------------------
//  Target Devices:
//    STM32F030F4xx running at 8 MHz internal clock
//    24LC64 EEPROM IC
//    USB Serial Dongle
//  ------------------------------------------------------------------------------------------
//  Hardware Setup:
//
//                                          STM32F030F4xx
//                                           .---. .---.
//                                     BOOT0 |1o  V  20| PA14 / SWCLK
//                                    OSC_IN |2      19| PA13 / SWDIO
//                                   OSC_OUT |3      18| PA10 / I2C1_SDA ------,
//     GND --- [10K] --- [BUTTON] ----- NRST |4      17| PA9  / I2C1_SCL ---,  |
//                                      VDDA |5      16| VCC --- VCC        |  |
//                                       PA0 |6      15| GND --- GND        |  |
//   ,----------------,                  PA1 |7      14| PB1                |  |
//   | USB SERIAL| RX |----- USART1_TX / PA2 |8      13| PA7                |  |
//   | DONGLE    | TX |----- USART1_RX / PA3 |9      12| PA6                |  |
//   |           |GND |--,               PA4 |10     11| PA5                |  |
//   '----------------'  |                   '---------'                    |  |
//                       |                                      VCC   VCC   |  |
//                      GND                                      |     |    |  |
//                                        24CL64              [5.6K]   |    |  |
//                                       .-------.               |  [5.6K]  |  |
//                               ;--- A0 |1 o   8| VCC --- VCC   |     |    |  |
//                               |--- A1 |2     7| WP  --- NC    |     |    |  |
//                               |--- A2 |3     6| SCL ----------^-----|----'  |
//                               |-- GND |4     5| SDA ----------------^-------'
//                               |       '-------'               
//                               | 
//                              GND
//
//  **IMPORTANT** Pins PA2 and PA3 on the STM32F030 are *not* 5V tolerant! Make sure the
//                serial dongle is operating at 3.3 V!
//  ------------------------------------------------------------------------------------------
//  Command Operation from the Serial Terminal:
//
//    Type in one-character command plus numerical value in hex with no space between the
//    command character and the hex value. Obviously incorrect entries will be ignored.
//    In the following, "x" indicates a hex value.
//
//    Sx    Enter the starting address
//    Ex    Enter the ending address (will adjust length accordingly)
//    Lx    Enter the length (will adjust ending address accordingly)
//    Vx    Enter 8-bit value to write
//    R     Perform read operation -- display EEPROM contents for the specified area
//    W     Write value from Vx to the specified area on the EEPROM
//
//    Commands may be entered in upper or lower case.
//
//    Example:
//    S10   Set starting address to 0x10 and adjust End parameter accordingly
//    E1FF  Set ending address to 0x1FF and adjust length parameter accordingly
//    V55   Set the write value to 0x55
//    R     Display the specified area on the EEPROM
//    W     Write "value" to all of the area on the EEPROM from the specified start address
//          to the specified end address.
//  Reads in temperature and humidity data from an AHT10 I2C temperature and humidity
//  sensor, and displays this data along with comfort level phrases to an LCD module.
//
// ===========================================================================================


#include <stdlib.h>                           // Needed for itoa function
#include "stm32f030x6.h"                      // Primary CMSIS header file
#include "STM32F030-CMSIS-USART-lib.c"        // For serial terminal output
#include "STM32F030-CMSIS-I2C-EEPROM-lib.c"   // I2C EEPROM library


// ===========================================================================================
// main 
// ===========================================================================================
int
main()
{
  uint32_t startAdd = 0x0000;   // Starting address of EEPROM for read/write
  uint32_t endAdd   = 0x007F;   // Ending address of EEPROM for read/write
  uint32_t length;              // Number of bytes for read/write operation
  uint8_t  setValue = 0x00;     // Value used to set/fill for EEPROM write operation
  uint8_t  inpLen;              // Number of characters in serial input string
  char     inpCmd;              // Command letter of input (S, E, L, V, R or W)
  uint32_t input;               // Numerical input gotten from serial command
  uint32_t goodInp;             // Non-zero means got good input. Zero means bad input
  #define  STRLEN 10            // Maximum number of characters to accept from serial input
  char     myStr[ STRLEN + 1 ]; // Command from serial input (minus any numerical input)

  
  USART_init( USART1, 115200 );     // Only USART1 is available on the STM32F030F4.
                                    // Init. USART1 and associate with USART routines
  USART_puts( "UART Connected!" );


// ==================================================================  
//     \/   \/   \/   Main EEPROM work starts here   \/   \/   \/
// ==================================================================  

  
  EE24_init( I2C1, 100e3, 0x50, 0x2000, 32 );     // Init I2C port for this EEPROM device.
                                                  // Specify I2C port, I2C speed, I2C
                                                  // address, number of bytes on this device,
                                                  // and page size.
                                                  // Only I2C1 available on STM32F030F4.
                                                  // Note: May fail at I2C speeds below
                                                  // 2 kHz due to conflict with high USART
                                                  // speeds.
  USART_puts( "\n\n\nEEPROM Monitor\n" );         // Display title
    length = endAdd - startAdd + 1;               // Initialize length parameter
 
  EE24_dump( startAdd, length );                  // Dump this area of the EEPROM
  
  while( 1 )                                      // Main Loop
  {
    USART_puts( "\n[S]tart: 0x" );                // Display Command Prompt
    USART_puth( startAdd, 6 );
    USART_puts( "  [E]nd: 0x" );
    USART_puth( endAdd, 6 );
    USART_puts( "  [L]ength: 0x");
    USART_puth( length, 6 );
    USART_puts( "\n[V]alue: 0x" );
    USART_puth( setValue, 2 );
    USART_puts( "      [R]ead           [W]rite\n");
    USART_puts( "Enter command:" );

    do                                            // Loop here until good input is entered
    {
      goodInp = 1;                                // Assume good input to start

      inpLen = USART_gets( myStr, STRLEN );       // Read keyboard input
      inpCmd = myStr[0];                          // Get command letter

      if( inpLen > 1 )                            // If likely got a value after the command,
        input = strtol( &(myStr[1]), NULL, 16 );  // then extract the value.

      if( inpCmd == 'S' || inpCmd == 's'  )       // Set starting address
      {
        if( input < EE24_BYTES )                  // Keep start address within the boundries
          startAdd = input;                       // of the device. Adjust the end address
        endAdd = startAdd + length;               // address as necessary.
        if( endAdd >= EE24_BYTES )                // If the end address is beyond the device
        {                                         // capacity, then readjust the end address
          length = EE24_BYTES - startAdd;         // and length so they point to the last 
          endAdd = EE24_BYTES - 1;                // EEPROM address.
        }
      }

      else if( inpCmd == 'E' || inpCmd == 'e' )   // Set ending address
      {
        if( input < EE24_BYTES && input > startAdd )
        {                                         // Update end address as long as it looks
          endAdd = input;                         // normal. And also update the length
          length = endAdd - startAdd + 1;         // to match the start/end addresses.
        }
      }

      else if( inpCmd == 'L' || inpCmd == 'l' )   // Set length (number of bytes to write
      {
        if( input>=1 && input + startAdd <= EE24_BYTES )
        {                                         // if length is within device boundry, then
          length = input;                         // set the length, and update the end
          endAdd = startAdd + input - 1;          // address accordingly.
        }
      }

      else if( inpCmd == 'V' || inpCmd == 'v' )   //  Set write value
      {
        if( input == ( input & 0xFF ))            // Update setValue only if it is within
          setValue = input;                       // 0 to 255.
      }

      else if( inpCmd == 'R' || inpCmd == 'r' )   //  Read and dump EEPROM
      {
        USART_putc( '\n' );
        EE24_dump( startAdd, length );
      }

      else if( inpCmd == 'W' || inpCmd == 'w' )       // Write setValue to EEPROM from 
        EE24_write( startAdd, &setValue, length, 1 ); // start address end address.

      else
      {    
        USART_puts( "\nInvalid input. Please enter Sxx, Exx, Lxx, Vxx, R, or W:" );
        goodInp = 0;  // Flag bad input
      }

    } while ( ! goodInp );
    
  } // END of main repeating block
  return 1;
}

/*
 This example code requires Atmel I2C drivers for Xmega distributed as part of AVR1308
 Available here: http://www.atmel.com/dyn/resources/prod_documents/AVR1308.zip

 You only need twi_master_driver.c,.h and twi_slave_driver.c,.h

 To test devices other then ds2782 as shown here, change SLAVE_ADDRESS to your device I2C addr
 and connect to PORTC/TWIC pins 0,1 (SDA,SCL)
*/

#include <stdio.h>
#include <avr\io.h>
#include <stddef.h>
#include <util\delay.h>
#include <avr\interrupt.h>

#define bool unsigned char

#include "twi_master_driver.h"
#include "twi_slave_driver.h"


/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x34

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        8

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       2000000
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)


/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */


/*! Buffer with test data to send.*/
uint8_t sendBuffer[NUM_BYTES];

int main(void)
{
	// configure led
   PORTF.DIR |= (1<<0); // EVAL-USB boards
   // PORTF.DIR |= (1<<2); // EVAL-01 boards
   // PORTD.DIR |= (1<<0); // EVAL-04 boards

   // this example configuration is for ds2782 fuel gauge on PORTC TWIC with 
   // power provided on PORTC:2,3

   // turn on power to ds2782
   PORTC.OUT |= (1<<3);            // PORTC:3 hi (power), PORTC:2 lo (gnd)
   PORTC.DIR |= (1<<2) | (1<<3);
	
	// Enable internal pull-up on PC0, PC1.. Uncomment if you don't have external pullups
//	PORTCFG.MPCMASK = 0x03; // Configure several PINxCTRL registers at the same time
//	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc; //Enable pull-up to get a defined level on the switches

	
	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	               &TWIC,
	               TWI_MASTER_INTLVL_LO_gc,
	               TWI_BAUDSETTING);

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();

    // loop forever
	// do 16-bit master i2c read of ds2782 register
	while (1)
	{
	  // toggle led
	  PORTF.OUT ^= (1<<0); // EVAL-USB boards
   // PORTF.OUT |= (1<<2); // EVAL-01 boards
   // PORTD.OUT |= (1<<0); // EVAL-04 boards

      // modify this address for the register you wish to read specific to your i2c device
      sendBuffer[0]=0x0C; // read from 16bit voltage register
	
	  TWI_MasterWriteRead(&twiMaster,
		                    SLAVE_ADDRESS,
		                    &sendBuffer[0],
		                    1,
		                    2);

      // results of 16bit read from volt register will be loaded in sendBuffer[0-1]

	  while (twiMaster.status != TWIM_STATUS_READY); // wait for transaction to complete

      _delay_ms(100);
	}
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

#include <stdio.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>
#include <avr\interrupt.h>


#define AC_STATUS_AC0STATE 0x10
#define AC_STATUS_AC1STATE 0x20

// uncomment for EVAL-01 board
//#define USER_LED_OUT() PORTF.DIR |= (1<<2)
//#define USER_LED_ON()  PORTF.OUT |= (1<<2)
//#define USER_LED_OFF() PORTF.OUT &= ~(1<<2)
// uncomment for EVAL-USB, Lite boards
#define USER_LED_OUT() PORTF.DIR |= (1<<0)
#define USER_LED_ON()  PORTF.OUT |= (1<<0)
#define USER_LED_OFF() PORTF.OUT &= ~(1<<0)

void Config32MHzClock(void);


int main(void)
{
  Config32MHzClock();

// output clock on PORTE:7 for test purposes
  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); // clkout

  USER_LED_OUT();

// setup PORTA:0-3 (0=hi, 1=hi, 2=samp, 3=gnd) to drive voltage divider with 
// test voltage on PORTA:2 to be input to analog comparator POS input
  PORTA.DIR = 0xB;
  PORTA.OUT = 0x3;

//
// Uncomment for Analog comparator referenced to internal 1.1V bandgap
//

  // configure AC0 to be PORTA:2 - VBandGap (1.1V)
  ACA.AC0CTRL = AC_INTMODE_RISING_gc | AC_INTLVL_OFF_gc | AC_HYSMODE_NO_gc | 0x1;
  ACA.AC0MUXCTRL = AC_MUXPOS_PIN2_gc | AC_MUXNEG_BANDGAP_gc;


//
// Uncomment for Analog comparator referenced to DAC (adjustable reference)
//
/*
  // configure AC0 to be PORTA:2 - DAC output
  ACA.AC0CTRL = AC_INTMODE_RISING_gc | AC_INTLVL_OFF_gc | AC_HYSMODE_NO_gc | 0x1;
  ACA.AC0MUXCTRL = AC_MUXPOS_PIN2_gc | AC_MUXNEG_DAC_gc;
  // configure DAC output for comparator (this is an adjustable reference)
  // use default VCC=3.3V reference for DAC output
  DACB.CTRLA = (1<<4) | (1<<2) | (1<<0);    // enable internal output, DACB, CH0 output
  DACB.CTRLC = DAC_REFSEL_AVCC_gc;          // select AVCC as reference
  DACB.CH0DATA = 0x0800;                    // half scale VDAC = 3.3/2 (VDAC=3.3*CH0DATA/4096)
*/

//
// Uncomment for high level interrupt on rising edge, compare to DAC output
//
/*
  ACA.AC0CTRL = AC_INTMODE_RISING_gc | AC_INTLVL_HI_gc | AC_HYSMODE_NO_gc | 0x1;
  ACA.AC0MUXCTRL = AC_MUXPOS_PIN2_gc | AC_MUXNEG_DAC_gc;
  // configure DAC output for comparator (this is an adjustable reference)
  // use default VCC=3.3V reference for DAC output
  DACB.CTRLA = (1<<4) | (1<<2) | (1<<0);    // enable internal output, DACB, CH0 output
  DACB.CTRLC = DAC_REFSEL_AVCC_gc;          // select AVCC as reference
  DACB.CH0DATA = 0x0800;                    // half scale VDAC = 3.3/2 (VDAC=3.3*CH0DATA/4096)

  PMIC.CTRL = PMIC_HILVLEN_bm; // enable high level interrupts
  sei();

  // turn LED off when output of comparator is 0 (will be turned on by interrupt)
  while(1)
   { if(!(ACA.STATUS & AC_STATUS_AC0STATE)) 
    USER_LED_OFF(); }; // turn off LED when AC output low
*/


  // When Analog Comparator 0 is hi, turn on user LED
  //  otherwise turn LED off
  // This is a polling method of monitoring output of Analog Comparator
  while(1)
  {
	if(ACA.STATUS & AC_STATUS_AC0STATE)
       USER_LED_ON();
    else
	   USER_LED_OFF();
  };


};


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
};


ISR(ACA_AC0_vect)
{
  USER_LED_ON();
  _delay_ms(150);
};

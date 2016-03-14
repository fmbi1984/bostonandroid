#include <stdio.h>
#include <avr\io.h>
#include <stddef.h>
#define F_CPU 2000000UL
#include <util\delay.h>
#include <avr\interrupt.h>
#include <avr\pgmspace.h>

void Config32MHzClock(void);
uint8_t ReadCalibrationByte( uint8_t index );

volatile int data12[100];
volatile int Reading;
volatile int gADC_CH0_ZeroOffset=0;

int main(void)
{
  // implicit 2MHz RC oscillator

  // read low ADCA calibration byte from NVM signature row into register
  ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );

  // read high ADCA calibration byte from NVM signature row into register
  ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );

   // configure PORTF:0 as output to LED
   PORTF.DIR=(1<<0);   // Eval-01 64A3 RevA, Eval-USB
//  PORTF.DIR=(1<<2);   // Eval-01 128A3,64A3 RevB


  // Setup heartbeat trigger every 10ms
  TCC0.PER = 2500;   // ~10ms
  TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
  TCC0.CTRLB = TC0_CCAEN_bm;

  // setup event0 when TCC0 overflows
  EVSYS.CH0MUX = 0xC0; // TCC0 Overflow

// setup ADC input on PORTA:0-3 (0=nc, 1=hi, 2=samp, 3=gnd(external input))
// and power PORTA:1 to create voltage divider
// connect actual system GND to PORTA:3, VLO is ~0.3V and will not work for measuring GND offset
  PORTA.DIR = 0x2;
  PORTA.OUT = 0x2;

/*
// UNCOMMENT FOR SINGLE ENDED UNSIGNED MODE 
//
  // setup adc for single ended, unsigned sampling on PORTA:3 to calibrate ADC offset
  ADCA.CTRLA |= 0x1;                                   // enable adc
  ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;                // 12 bit conversion
  ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | 0x02;           // internal 1V bandgap reference
  ADCA.PRESCALER = ADC_PRESCALER_DIV16_gc;             // peripheral clk/16 (2MHz/16=125kHz)
  ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;     // single ended
  ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;            // PORTA:3 (GND)

  // trigger single conversion
  ADCA.CTRLA |= 0x4; 
  // wait for result
  while(!ADCA.CH0.INTFLAGS);                 // wait for conversion complete flag
  ADCA.CH0.INTFLAGS=ADC_CH_CHIF_bm;          // clear int flags (cleared by writing 1)
  gADC_CH0_ZeroOffset = ADCA.CH0RES;         // read 12 bit value and save as zero offset

  // setup adc to read from PORTA:2 and interrupt on conversion complete
  ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc;            // PORTA:2
  ADCA.CH0.INTCTRL = ADC_CH_INTLVL_HI_gc;              // hi level interrupt
  ADCA.EVCTRL = ADC_EVSEL_0123_gc | ADC_EVACT_CH0_gc;  // trigger ch0 conversion on event0
*/

// UNCOMMENT FOR DIFFERENTIAL SIGNED MODE
//
  // setup adc to read from PORTA:2 (diff mode)
  ADCA.CTRLA |= 0x1;                                   // enable adc
  ADCA.CTRLB = 0x10 | ADC_RESOLUTION_12BIT_gc;         // 12 bit signed conversion (pos 11bits)
  ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | 0x02;           // internal 1V bandgap reference
  ADCA.PRESCALER = ADC_PRESCALER_DIV16_gc;             // peripheral clk/16 (2MHz/16=125kHz)
  ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFF_gc;            // differential
  ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc | ADC_CH_MUXNEG_PIN3_gc;      // PORTA:2 wrt A3
  ADCA.CH0.INTCTRL = ADC_CH_INTLVL_HI_gc;              // hi level interrupt
  ADCA.EVCTRL = ADC_EVSEL_0123_gc | ADC_EVACT_CH0_gc;  // trigger ch0 conversion on event0

  PMIC.CTRL = PMIC_HILVLEN_bm; // enable high level interrupts

  sei(); // enable global interrupts

  while(1) {
	  if(Reading > 0x400)     // for signed mode this is middle of pos range, unsigned 1/4 range
    	PORTF.OUT |= 0x01;    // turn on LED
  	  else
    	PORTF.OUT &= ~(0x01); // off
	};

/* // uncomment for EVAL-01 Rev B 64A3,128A3
  while(1) {
	  if(Reading > 0x400)     // for signed mode this is middle of pos range, unsigned 1/4 range
    	PORTF.OUT |= 0x01;    // turn on LED
  	  else
    	PORTF.OUT &= ~(0x01); // off
	};
*/
};


uint8_t ReadCalibrationByte( uint8_t index ) 
{ 
uint8_t result; 

/* Load the NVM Command register to read the calibration row. */ 
NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; 
result = pgm_read_byte(index); 

/* Clean up NVM Command register. */ 
NVM_CMD = NVM_CMD_NO_OPERATION_gc; 

return( result ); 
} 


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
};


// interrupt should be called after each ADC conversion is complete
ISR(ADCA_CH0_vect)
{
  Reading = (ADCA.CH0RES - gADC_CH0_ZeroOffset);
};

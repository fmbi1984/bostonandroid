#include <stdio.h>
#include <avr\io.h>

void Config32MHzClock(void);

int main(void)
{
   Config32MHzClock(); // configure sysclk=32MHz RC oscillator

   CLK.PSCTRL = 0x00;  // no division on peripheral clock

   // make clkout on PORTE:7
   PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
   PORTE.DIR = (1<<7); // clkout

   // configure timer/counter0
   TCC0.CTRLA = TC_CLKSEL_DIV1024_gc;   // clk/1024

   // configure PORTF:0 as output to LED
   PORTF.DIR=(1<<0);   // Eval-01 64A3 RevA, Eval-USB
//  PORTF.DIR=(1<<2);   // Eval-01 128A3,64A3 RevB

   // configure PORTB to generate PWM signal on PORTB0,1
   // PORTB:0,1 both rise to 1 at start of cycle
   // PORTB:0 falls when CNT = CCA
   // PORTB:1 falls when CNT = CCB
   // when CNT=PER cycle repeats

   TCD0.PER = 200;    //  200/32M = 6.25us period
   TCD0.CTRLA = TC_CLKSEL_DIV1_gc; 
   TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCBEN_bm | TC0_CCAEN_bm;   // 2 COMP outputs

   TCD0.CCA = 70;     //  70/32M  = 2.20us
   TCD0.CCB = 120;    // 120/32M  = 3.75us

   // configure PORTD as all output
   PORTD.DIR = 0xFF;

   // blink debug LED at ~2Hz 
   while(1){
     while(TCC0.CNT < 7082);  // wait for CNT==7082
     // Blink LED
     PORTF.OUT ^= (1<<0);   // Eval-01 64A3 RevA, Eval-USB
//     PORTF.OUT ^= (1<<2);   // Eval-01 128A3, 64A3 RevB
     TCC0.CNT=0;              // reset count
   };

return 0;
};


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL |= OSC_RC32MEN_bm; // enable 32MHz oscillators
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
};



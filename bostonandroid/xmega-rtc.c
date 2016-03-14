#include <stdio.h>
#include <avr\io.h>
#include <avr\interrupt.h>

void Config32MHzClock(void);
void Config32KHzRTC(void);

volatile long int gRTCSeconds;

int main(void)
{
  Config32MHzClock(); // configure sysclk=32MHz RC oscillator, turn on 32KHz osc
  Config32KHzRTC();   // configure with 1 tick per second

  CLK.PSCTRL = 0x00;  // no division on peripheral clock

  // make clkout on PORTE:7
  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); // clkout

  // configure timer/counter0
  TCC0.CTRLA = TC_CLKSEL_DIV1024_gc;   // clk/1024

  // configure PORTF:0 as output to LED 
  PORTF.DIR=(1<<0);   // Eval-01 64A3 RevA, Eval-USB
//  PORTF.DIR=(1<<2);   // Eval-01 128A3,64A3 RevB


  // setup RTC counter params
  RTC.PER = 0;                         // overflow after 1 second
  RTC.INTCTRL |= RTC_OVFINTLVL_HI_gc;  // set high level interrupt on RTC overflow
  RTC.CNT = 0;                         // clear initial count

  PMIC.CTRL = PMIC_HILVLEN_bm; // enable high level interrupts
  sei(); // enable interrupts

  while(1); // forever

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

void Config32KHzRTC(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  OSC.CTRL |= OSC_RC32KEN_bm; // enable internal 32KHz Osc
  // select RTC clk source
  CLK.RTCCTRL = CLK_RTCSRC_TOSC32_gc;  // internal 32KHz Osc source RTC enable
  RTC.CTRL = RTC_PRESCALER_DIV1024_gc;     // 1KHz/1024 = 1second/tick

  // wait for RTC SYNC status not busy before returning
  while(RTC.STATUS & RTC_SYNCBUSY_bm);
};

ISR(RTC_OVF_vect)
{
gRTCSeconds++;
    PORTF.OUT ^= (1<<0);   // Eval-01 64A3 RevA, Eval-USB
//    PORTF.OUT ^= (1<<2);   // Eval-01 128A3, 64A3 RevB
};

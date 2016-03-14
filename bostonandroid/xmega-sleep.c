#include <stdio.h>
#include <avr\io.h>
#include <avr\sleep.h>
#include <avr\interrupt.h>
#include <util\delay.h>

void Config32MHzClock(void);
#define F_CPU 32000000


int main(void)
{
  Config32MHzClock(); // configure sysclk=32MHz RC oscillator, turn on 32KHz osc

  CLK.PSCTRL = 0x00;  // no division on peripheral clock

  // use the sleep.h set_sleep_mode() to configure sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // lowest setting; wake on external interrupts, TWI
//  set_sleep_mode(SLEEP_MODE_PWR_SAVE);    // same as Powerdown but RTC on/wake source
//  set_sleep_mode(SLEEP_MODE_STANDBY);     // same as Powerdown but System clock on
//  set_sleep_mode(SLEEP_MODE_EXT_STANDBY); // almost idle, but cpu/peripheral clocks off
//  set_sleep_mode(SLEEP_MODE_IDLE);        // highest setting; wake on any interrupt

  // configure PORTF:0 as output to LED 
  PORTF.DIR=(1<<0);   // Eval-01 64A3 RevA, Eval-USB
//  PORTF.DIR=(1<<2);   // Eval-01 128A3,64A3 RevB

  // setup PORTC TimerCounter to generate overflow after some period
  // configure timer/counter0
  TCC0.CTRLA = TC_CLKSEL_DIV1024_gc;   // clk/1024
  TCC0.PER   = 7812;
  TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc; // set low level interrupt on overflow
  TCC0.CNT   = 0;

  // enable interrupts on any PORTD level change
  PORTD.INT0MASK = 0xff;  // any pin on PORTD
  PORTD.INTCTRL = 0x1; // enable low level interrupt
   
  PMIC.CTRL = PMIC_LOLVLEN_bm; // enable low level interrupts
  sei(); // enable interrupts

  while(1)
  {
    sleep_enable(); 
    sleep_cpu(); // wake on any enabled interrupt sources
    PORTF.OUT ^= (1<<0);   // Eval-01 64A3 RevA, Eval-USB
//    PORTF.OUT ^= (1<<2);   // Eval-01 128A3, 64A3 RevB
    _delay_ms(3000);
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


ISR(TCC0_OVF_vect)
{
//    PORTF.OUT ^= (1<<0);   // Eval-01 64A3 RevA, Eval-USB
//    PORTF.OUT ^= (1<<2);   // Eval-01 128A3, 64A3 RevB
};

ISR(PORTD_INT0_vect)
{
//    PORTF.OUT ^= (1<<0);   // Eval-01 64A3 RevA, Eval-USB
//    PORTF.OUT ^= (1<<2);   // Eval-01 128A3, 64A3 RevB
};

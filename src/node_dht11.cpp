#include <Arduino.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

//
//////////////////////////////////////////////////////////////////////
/// TX433 ON PORT 0, DHT-11 ON PORT 1, PIR is hardwirded as PCINT4 ///
//////////////////////////////////////////////////////////////////////
//
// To save battery, we don't use the timer potentiometer on the PIR
// instead we handle the timer for motion off in software by sleeping
// the ATTINY85
//
// DEFINE THE NODE
#define BLUE          // my RF nodes are white, green, blue, red
#define DHTPIN 1      // DHT connected to ATTiny95 Digital Pin 1
#define TXPIN  0      // 433Mhz transmit pin
#define VDHTCOUNT 450 // number of (4s) WDT events needed for a read of the DHT and voltage
                      // 450 => voltage/temp/RH every 30mins
#define PIRCOUNT 75   // number of (4s) WDT events needed w/o motion to reset motion
                      // 75 => motion is reset to 0 only after 5 minutes
#define HAS_DHT       // is there a DHT attached?


// SHOULDN'T NEED TO CHANGE THESE BELOW
#define PULSE_US 250
#define F_CPU 1000000UL
#define TX_HIGH PORTB |= 1<<TXPIN    // write a 1 to PORTB bit 1, leaving other bits unchanged
#define TX_LOW PORTB &= ~(1<<TXPIN)  // write a 0 to PORTB bit 1, leaving other bits unchanged
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
//
volatile int pulse=1;        // this flips sign every PULSE_US via WDT ISR
volatile int motion=0;       // set to 1 when there is motion via pin change ISR
volatile int i_4s_wdt=0;     // incremented by the watchdog timer ISR every 4 seconds
int recent_motion=1;         // rate limits motion detection, set to 1 if motion is 'active'
int txd=0;                   // set to 1 if the 433mhz transmitter has been used recently




//
///////////////////
/// DHT-11 CODE ///
///////////////////
//
/*
 * Tinu DHT - Tinusaur DHT11 Library
 *
 * @file: tinudht.h
 * @created: 2014-07-08
 * @author: neven
 *
 * Source code available at: https://bitbucket.org/tinusaur/tinudht
 *
 */


typedef struct {
  uint16_t humidity;
  uint16_t temperature;
} TinuDHT;
TinuDHT tinudht;

// ----------------------------------------------------------------------------
#define TINUDHT_OK				0
#define TINUDHT_ERROR_CHECKSUM	-1
#define TINUDHT_ERROR_TIMEOUT	-2
#define TINUDHT_RCV_TIMEOUT 255
#define TINUDHT_RCV_DELAY 10
#define TINUDHT_RCV_LENGTH 3
// ----------------------------------------------------------------------------

uint8_t tinudht_read(TinuDHT *ptinudht, uint8_t dht_pin) {
  // Buffer to received data
  uint8_t data[5];
  // Empty the buffer
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  // timeout integer
  uint8_t timeout;
  
  // Send request to DHT-11
  DDRB |= (1 << dht_pin);       // Set port as output
  PORTB &= ~(1 << dht_pin);	// Set to 0 (LOW)
  _delay_ms(20);	        // Data sheet says to pull down for >18 ms
  PORTB |= (1 << dht_pin);      // Set to 1
  _delay_us(40);	        // Data sheet says wait 40 us with HIGH
  // Receive response from DHT-11
  DDRB &= ~(1 << dht_pin);	// Set port as input
  //PORTB |= (1 << dht_pin);// Set to 1, optional pull-up.
  
  //DHT-11 resets line to zero after 40us high
  timeout = TINUDHT_RCV_TIMEOUT;
  while (bit_is_clear(PINB, dht_pin)) {	// Wait for 1
    if (timeout-- == 0) {
      PORTB &= ~(1 << dht_pin);	// Set to 0 (LOW)
      return TINUDHT_ERROR_TIMEOUT;
    }
  }

  timeout = TINUDHT_RCV_TIMEOUT;
  while (bit_is_set(PINB, dht_pin)) {  // Wait for 0
    if (timeout-- == 0) {
      PORTB &= ~(1 << dht_pin);	// Set to 0 (LOW)
      return TINUDHT_ERROR_TIMEOUT;
    }
  }
  uint8_t bit_index = 7;
  uint8_t byte_index = 0;
  // Usual output is 40 bits
  // Humidty integer | Humidity decimal | Temp integer | Temp decimal | Checksum
  // on DHT-11 I think the decimals are not given.
  for (uint8_t i = 0; i < 32; i++) {
    // Wait for start
    timeout = TINUDHT_RCV_TIMEOUT;
    while(bit_is_clear(PINB, dht_pin)) {// Wait for 1
      if (timeout-- == 0) {
	PORTB &= ~(1 << dht_pin);	// Set to 0 (LOW)
	return TINUDHT_ERROR_TIMEOUT;
      }
    }
    // Determine the bit value
    uint8_t len = 0;
    while(bit_is_set(PINB, dht_pin)) {	// Wait for 0
      _delay_us(TINUDHT_RCV_DELAY);
      if (len++ == TINUDHT_RCV_TIMEOUT) {
	PORTB &= ~(1 << dht_pin);	// Set to 0 (LOW)
	return TINUDHT_ERROR_TIMEOUT;
      }
    }
    // a HIGH for more than 30us is a 1 otherwise a 0
    if (len >= TINUDHT_RCV_LENGTH) {
      data[byte_index] |= (1 << bit_index);
    }

    if (bit_index == 0) {
      bit_index = 7;	  // restart at MSB
      byte_index++;	  // next byte!
    }
    else bit_index--;
  } // end data loop
  
  // skip the checksum
  // uint8_t checksum = data[0] + data[2];
  // if (data[4] != checksum) return TINUDHT_ERROR_CHECKSUM;
  
  // On DHT11 data[1],data[3] are always zero so not used.
  ptinudht->humidity = data[0];
  ptinudht->temperature = data[2];
  
  return TINUDHT_OK;
}

//
/////////////////////////////////////////////////////
/// TIMER COMPARE INTERRUPT, FIRES EVERY PULSE_US ///
/////////////////////////////////////////////////////
// only used for the tx433 routine timing

// Timer/Counter0 Compare Match A interrupt handler
ISR (TIMER0_COMPA_vect) {
  pulse *= -1; // invert the pulse value
}

// enable the interrupt
void enable_compare_timer_interrupt() {
  // configure 240us timer interrupt
  TCNT0 = 0;               // Count up from 0 using the (8bit) TimerCounter0 register
  TCCR0A = 2 << WGM00;     // Timer counter control register A: set CTC mode
  TCCR0B = 1 << CS00;      // Set prescaler to /1 (1uS at 1Mhz)
  //if (CLKPR == 3)        // If clock set to 1MHz
  //    TCCR0B = (1<<CS00);// Set prescaler to /1 (1uS at 1Mhz)
  //else                   // Otherwise clock set to 8MHz
  //    TCCR0B = (2<<CS00);// Set prescaler to /8 (1uS at 8Mhz)
  GTCCR |= 1 << PSR0;      // Reset prescaler
  OCR0A = PULSE_US;        // 240 + 1 = 241 microseconds (4KHz)
  TIFR = 1 << OCF0A;       // Clear output compare interrupt flag
  TIMSK |= 1 << OCIE0A;    // Enable output compare interrupt
}

// disable the interrupt
void disable_compare_timer_interrupt()
{
  TIMSK &= ~(1 << OCIE0A);    // disable output compare interrupt bit leaving others unchanged
}

// RF data out put is via 433MHz transmitter. 
// I use the HomeEasy protocol, but send an integer value(<1024) as the Remote ID sender, with a
// user defined channel number (0-16).
// BIT PATTERN
// bit 0 -------- bit 25 : the HomeEasy remote ID
// bit 26 & bit 27       : Group and On/Off
// bit 28, 29, 30, 31    : Channel 4bit value
//
// Here it is hijacked as
// bit  0 -- 15 : we need a node ID somewhere in here for our 4 nodes
// bit 16 -- 25 : the 10 bit sensor value
// bit 26 & 27  : group and on/off are ignored here
// bit 28 -- 31 : channel number to identify the measurement (voltage, temp etc )
//
// bits 0 -- 25 are mapped into a 4-byte array on receipt.
// to make the node ID align with a byte boundary and cover 4 nodes, we will
// identify the node using bits 8 & 9
void tx433( long int value, int channel )
{
  // enable an interrupt that fires around every 240us
  enable_compare_timer_interrupt();
  //
  // define an empty signal vector (inverted binary)
  int signal[32] = {1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1, 1,0, 1,1,1,1};
  //
  // identify this node
  // which is stored in bits (0-9) but we only use 8 & 9.
#ifdef WHITE
  // identify this node as ID = 0 (White)
  signal[8]=1; signal[9]=1;
#endif
#ifdef GREEN
  // identify this node as ID = 1 (Green)
  signal[8]=1; signal[9]=0;
#endif
#ifdef BLUE
  // identify this node as ID = 2 (Blue)
  signal[8]=0; signal[9]=1;
#endif
#ifdef RED
  // identify this node as ID = 3 (Red)
  signal[8]=0; signal[9]=0;
#endif
  //
  uint32_t binary_value = value;
  // ADC values can be covered by a 10 bit number
  for ( unsigned i = 0; i<10; ++i ) {
    // AND the binary value with a binary 1 to get the least significant bit
    // shouldnt this be 0x01 if its going into 8 bits, or 0x00000001 if and-ing with a 32 bit??
    uint8_t bit =  binary_value & 0x0001;
    if ( bit == 1) {
      // put this bit into the receiver code which is the first 26 bits
      signal[25-i] = 0;
    } else {
      // put this bit into the receiver code which is the first 26 bits
      signal[25-i] = 1;
    }
    // shift the binary value right by one bit
    binary_value = binary_value >> 1;
  }
  //
  uint8_t binary_channel = channel;
  // channel is only a 4 bit number
  for ( unsigned i = 0; i<4; ++i ) {
    // as above, get the least significant bit by an AND with a hex(01)
    uint8_t bit =  binary_channel & 0x01;
    if ( bit == 1) {
      // put the bit into the channel code
      signal[31-i] = 0;
    } else {
      // put the bit into the channel code
      signal[31-i] = 1;
    }
    binary_channel = binary_channel >> 1;
  }
  // cheap hack to synchronise with a 240us pulse signal
  for ( unsigned k = 0; k<5; ++k ) {
    pulse = 1;
    // a sync bit
    while ( pulse > 0 ) TX_HIGH;
    while ( pulse < 0 ) TX_LOW;
    while ( pulse > 0 ) TX_LOW;
    while ( pulse < 0 ) TX_LOW;
    while ( pulse > 0 ) TX_LOW;
    while ( pulse < 0 ) TX_LOW;
    while ( pulse > 0 ) TX_LOW;
    while ( pulse < 0 ) TX_LOW;
    while ( pulse > 0 ) TX_LOW;
    while ( pulse < 0 ) TX_LOW;
    while ( pulse > 0 ) TX_LOW;
    for ( int i = 0; i < 32; ++i ) {
      if (signal[i] == 1) {
        // a 1 bit_
        while ( pulse < 0 ) TX_HIGH;
        while ( pulse > 0 ) TX_LOW;
        while ( pulse < 0 ) TX_HIGH;
        while ( pulse > 0 ) TX_LOW;
        while ( pulse < 0 ) TX_LOW;
        while ( pulse > 0 ) TX_LOW;
        while ( pulse < 0 ) TX_LOW;
        while ( pulse > 0 ) TX_LOW;
      } else {
        // a 0 bit_
        while ( pulse < 0 ) TX_HIGH;
        while ( pulse > 0 ) TX_LOW;
        while ( pulse < 0 ) TX_LOW;
        while ( pulse > 0 ) TX_LOW;
        while ( pulse < 0 ) TX_LOW;
        while ( pulse > 0 ) TX_LOW;
        while ( pulse < 0 ) TX_HIGH;
        while ( pulse > 0 ) TX_LOW;
      }
    }
    while ( pulse < 0 ) TX_HIGH;
    // TX_LOW;
    for ( int i = 0; i<20; ++i ) {
      while ( pulse > 0 ) TX_LOW;
      while ( pulse < 0 ) TX_LOW;
    }
  }
  txd = 1; // set the "I've just transmitted a signal" flag
  // turn the pulse interrupt back off again.
  disable_compare_timer_interrupt();
}


///////////////////////////////////////
/// WATCHDOG TIMER FOR SLEEP WAKEUP ///
///////////////////////////////////////
//
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9=8 sec
void setup_watchdog(int ii) {
  byte bb;
  //int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  //ww=bb;
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  i_4s_wdt += 1;  // set global flag
}

//////////////////////////////////////////
/// PIR INTERRUPT FOR MOTION DETECTION ///
//////////////////////////////////////////
//
ISR (PCINT0_vect) {
  // check that the pin change is PB4 for the PIR
  // not sure that this is needed since the PCMSK is set for PB4 only
  if bit_is_set(PINB, 4) {
      motion=1;
    }
}

/////////////////////////////
/// INTERNAL VOLTAGE READ ///
/////////////////////////////
// READ THE SELF VOLTAGE BY REFERENCE TO INTERNAL 1.1v (APPROX)
//
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  _delay_ms(10); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;
  // result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
#ifdef WHITE
  result =    1122000L / result; // Calculate Vcc (in mV); factor is calibration w.r.t multimeter for WHITE node
#endif
#ifdef GREEN
  result =    1100000L / result; // Calculate Vcc (in mV); factor is calibration w.r.t multimeter for GREEN node
#endif
#ifdef BLUE
  result =    1093000L / result; // Calculate Vcc (in mV); factor is calibration for Uno 3.3v for BLUE node
#endif
  return result;
}

////////////////////////////
/// SYSTEM SLEEP ROUTING ///
////////////////////////////
// set system into the sleep state
// system wakes up when wtchdog is timed out
// the "watch_pir" flag specifies if we should watch for pin changes on PB4.
void system_sleep(int watch_pir) {
  pinMode(TXPIN,INPUT);                // set all used ports to intput to save power
  if ( watch_pir == 1 ) {
    sbi(PCMSK,PCINT4);                 // want pin PCINT4/PB4/pin 3 for the PIR
    sbi(GIFR,PCIF);                    // clear any outstanding interrupts
    sbi(GIMSK,PCIE);                   // enable pin change interrupts
  }
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
	cbi(PCMSK,PCINT4);                   // disable PIR interrupt
  sleep_disable();                     // System continues execution here when watchdog timed out
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  pinMode(TXPIN,OUTPUT);               // set port 0 as output again
}


void setup() {
  //#ifdef BLUE
  //OSCCAL -= -3;
  //#endif
  pinMode(4,INPUT);        // PIR input
  // pinMode(DHTPIN,OUTPUT); not needed becuase it is set in the data call and is 1-wire
  pinMode(TXPIN,OUTPUT);   // transmitter output
  //
  setup_watchdog(8);       // 4 second watchdog
  // flash LED 5 times on startup
  for ( unsigned i = 0; i < 5; ++i ) {
    digitalWrite( TXPIN, HIGH );
    _delay_ms(100);
    digitalWrite( TXPIN, LOW );
    _delay_ms(900);
  }
  recent_motion = 0;
}

void loop() {
  // we periodically report the node voltage, temperature and humidity
  // iwdt +=1 every 4 seconds
  if (i_4s_wdt >= VDHTCOUNT) { 
    i_4s_wdt=0;        // reset counter back to zero
    // compute the battery velocity relative to 5v
    long voltage_value = 1023*readVcc()/5000;
    tx433( voltage_value, 0 );
    //
#ifdef HAS_DHT
    uint8_t tinudht_result = tinudht_read(&tinudht, DHTPIN);
    tx433( tinudht.temperature, 2 );
    tx433( tinudht.humidity, 3 );
#endif
  }
  // transmit no motion signal
  if ( (i_4s_wdt >= PIRCOUNT) && (recent_motion == 1) ) {
    // transmit motion off: 0 sent to "channel" 1
    tx433( 0, 1 );
    // reset motion flag
    motion = 0;
    // reset recent motion flag
    recent_motion = 0;
  }
  if ( motion == 1 ) {
    // motion detected, but this is the first motion for a while
    if ( recent_motion == 0 ) {
      // transmit motion on
      tx433( 255, 1 );
      recent_motion = 1; 
    }
    // reset the wdt to restart counting again for no motion
    i_4s_wdt = 0;
    // reset motion -- if there is a PIR signal it will be reset to 1 by ISR
    motion = 0;
  }
  // if we have just used the transmitter
  if ( txd == 1 ) {
    //
    // RF interference from above tx433 calls can false-trigger the PIR, which is then
    // turned on for 2.5 seconds (assuming the potentiometer is set to lowest value).
    // To avoid false positives we now wait for 4seconds
    system_sleep(0);       // sleep w/o watching the PIR for pin changes
    txd = 0;               // not txd has expired
  }
  //
  system_sleep(1);       // the ISR for the PIR is ONLY active when sleeping
}

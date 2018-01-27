/*
 *   Arduino program to sample a seismometer signal at three different amplitudes.  With a signal 16 times 
 *   another, the 10 bit A/D is extended to 14 bits of dynamic range( but still 10 bits of resolution ).
 *   
 *   The circuit has a low pass filter followed by 2 stages with gains of 4 each. 
 *   
 *   LM324 op-amps cannot drive to the positive rail.  The 3.3 voltage source is used as the AREF to keep
 *   the A/D converter in the active region of the op-amp when powered by 5 volts.
 */


#include <TimerOne.h>         // Paul Stoffregen library.  

const int led = LED_BUILTIN;  // the pin with a LED
volatile byte sample_ready;
volatile int sig0;
volatile int sig1;
volatile int sig2;
volatile byte sat0, sat1, sat2;
int zp0, zp1, zp2;            // slow tracking of zero points for each A/D
unsigned int quake, quake_detect;

void setup(void)
{
  pinMode(led, OUTPUT);
  Timer1.initialize(166667);
  Timer1.attachInterrupt(sample);
  Serial.begin(9600);
  analogReference(EXTERNAL);        // 3.3 volts
  zp0 = zp1 = zp2 = 512;
}

void sample(void){    // interrupt 6 times a second

  sig0 = analogRead(A0);    // the signal with the most amplitude
  sig1 = analogRead(A1);    // 2 bits down in amplitude
  sig2 = analogRead(A2);    // another 2 bits down in amplitude
  // sig1 is inverted in phase from the other 2, so correct the reading
  sig1 = 1023 - sig1;

  sat0 = sat1 = sat2 = 0;   // the op-amp may not drive completely to zero or 1023
  if( sig0 < 30 || sig0 > 1000 ) sat0 = 1;
  if( sig1 < 30 || sig1 > 1000 ) sat1 = 1;
  if( sig2 < 30 || sig2 > 1000 ) sat2 = 1;
  
  sample_ready = 1;

}


void loop(void){

int v0, v1, v2;
int val;
int f;        // shift or mult factor

static unsigned long timer;
static unsigned int sec_,min_,hour_;
static byte tick;
static byte led_holdoff;

  noInterrupts();
  if( sample_ready ){
    sample_ready = 0;
    
    if( tick ){           // top of the minute, look at zero points
      tick = 0;
      if( sat0 == 0 ){
        if( sig0 > zp0 ) ++zp0;    // miving these once per minute is very slow
        if( sig0 < zp0 ) --zp0;    // it will take 500 minutes to move half the A/D range
      }
      if( sat1 == 0 ){
        if( sig1 > zp1 ) ++zp1;
        if( sig1 < zp1 ) --zp1;
      }
      if( sat2 == 0 ){
        if( sig2 > zp2 ) ++zp2;
        if( sig2 < zp2 ) --zp2;
      }
    }
    
    v0 = sig0 - zp0;   v1 = sig1 - zp1;  v2 = sig2 - zp2;
    interrupts();

    // which signal is not saturated
    val = v2;   f = 4;              // default.  We will use this one even if it is saturated
    if( sat1 == 0 ) val = v1, f = 2;
    if( sat0 == 0 ) val = v0, f = 0;  

    val = val << f;                 // shift up the signals sampled at lower amplification

    //Serial.print(zp0);  Serial.write(' ');
    Serial.println(val);            // serial data for AmaSeis program

    //Serial.print(sig0); Serial.write(' ');
    //Serial.print(sig1); Serial.write(' ');
    //Serial.println(sig2);

    // light the LED if it looks like we are receiving an earthquake
    if( val > 20 || val < -20 ){
      digitalWrite(led,HIGH);
      if( hour_ > 2 ) ++quake_detect;     // avoid false detect on startup
    }
    else{
      if( led_holdoff == 0 ) digitalWrite(led,LOW);
      if( quake_detect ) --quake_detect;
    }
    
    if( quake_detect > 3600 ){             // large signals for 10 minutes
      ++quake;
      quake_detect = 0;
      hour_ = 2;             // no more detects for an hour
    }
  }
  
  interrupts();

  // keeping track of some time
  if( timer != millis() ){
    timer = millis();
    
    if( led_holdoff && (timer % 250) == 0 ) led_holdoff = 0;  // allow led out 1/4 second after on
    
    if( ( timer % 1000 ) == 0 ){
      // each second blink the led if we have detected quakes in the past
      if( quake ) digitalWrite(led,HIGH), led_holdoff = 1; 
      
      if( ++sec_ == 60 ){
        sec_ = 0,
        tick = 1;
        if( ++min_ == 60 ){
          min_ = 0;
          ++hour_;
          if( hour_ > 26 ){        // time out quakes, display for one day
            hour_ = 3;
            if( quake ) --quake;
          }
        }
      }
    }
  }
  
  
}


/*
 *   Arduino program to sample a seismometer signal at three different amplitudes.  With a signal 16 times 
 *   another, the 10 bit A/D is extended to 14 bits of dynamic range( but still 10 bits of resolution ).
 *   
 *   The circuit has a low pass filter followed by 2 stages with gains of 4 each. 
 *   
 *   LM324 op-amps cannot drive to the positive rail.  The 3.3 voltage source is used as the AREF to keep
 *   the A/D converter in the active region of the op-amp when powered by 5 volts.
 *   
 *   Feb 2018 - a simple FIR filter added to filter 60 hz energy in the signal.  Sample rate increased to 240 hz.
 *   At counts 0 - 6, samples are saved.  The samples are filtered and one value is output to the serial port.
 *   At counts 7 - 39, the samples are ignored.  Decimation is 40 but a decimation filter is not used.
 *   The rate of samples sent to the PC is still 6 per second.
 */


#include <TimerOne.h>         // Paul Stoffregen library.  

const int led = LED_BUILTIN;  // the pin with a LED
volatile byte sample_ready;
volatile byte tick;
int sig0;
int sig1;
int sig2;

int zp0, zp1, zp2;            // slow tracking of zero points for each A/D
int samples[7];

byte sec_, min_,hour_;
byte quakes;

void setup(void){
  pinMode(led, OUTPUT);
  Timer1.initialize(166667/40);
  Timer1.attachInterrupt(sample);
  Serial.begin(9600);
  analogReference(EXTERNAL);        // 3.3 volts
  zp0 = zp1 = zp2 = 512;
}

void sample(void){    // interrupt 240 times a second.
int val,f,zp;
static byte counter;

  if( ++counter > 39 ) counter = 0;    // aquire 6 samples at 240 hz rate, skip processing discarded samples
  if( counter < 7 ){ 
    sig0 = analogRead(A0);    // the signal with the most amplitude
    sig1 = analogRead(A1);    // 2 bits down in amplitude
    sig2 = analogRead(A2);    // another 2 bits down in amplitude
  // sig1 is inverted in phase from the other 2, so correct the reading
    sig1 = 1023 - sig1;
    
  // pick a signal that is not saturating the A/D converter
    f = 0, val = sig0, zp = zp0;
    if( sig0 < 30 || sig0 > 1000 ) f = 2, val = sig1, zp = zp1;
    if( sig1 < 30 || sig1 > 1000 ) f = 4, val = sig2, zp = zp2;
  //  correct value to +- 0 point and scale
    val = val - zp;
    val <<= f;

    samples[counter] = val;
    if( counter == 6 && sample_ready == 1 ) digitalWrite(led,HIGH);   // out of gas !!!
    if( counter == 6 ) sample_ready = 1;
  }
  else if( tick ){             // adjust the zero points once per minute during decimation count 7 to 39
    tick = 0;                  // when processor is lightly loaded
    if( sig0 > zp0 ) ++zp0;
    if( sig0 < zp0 ) --zp0; 
    if( sig1 > zp1 ) ++zp1;
    if( sig1 < zp1 ) --zp1;
    if( sig2 > zp2 ) ++zp2;
    if( sig2 < zp2 ) --zp2;
  }
}


void loop(void){
static int val;
static byte counter;
static byte flash_count;

   if( sample_ready ){                             // 6 times a second
      val = filter();
      noInterrupts();
      sample_ready = 0;
      interrupts();
      Serial.println(val);
      ++counter;
      flash_count = flash_led(flash_count);
      if( flash_count == 0 ) signal_level(val);     // signal level also turns on the led
   }

   if( counter == 6 ){   // one second time
      counter = 0;
      keep_time();
      flash_count = quakes << 1;                    // twice amount to count both on and off times
      quake_detect(val);
   }
   
}

byte flash_led( byte count ){ // flash the number of quakes detected
static byte led_state;

  if( count == 0 ) return 0;
  
  led_state ^= 1;
  digitalWrite(led,led_state);
  return (count - 1); 
}

void keep_time(){            // called once a second

  if( ++sec_ == 60 ){
    sec_ = 0;
    noInterrupts();
    tick = 1;                // flag to run the zero point code
    interrupts();
    if( ++min_ == 60 ){
      min_ = 0;
      if(++hour_ == 24 ){
        if( quakes ) --quakes;   // removed quake detected after 24 hours of quiet
        hour_ = 0;
      }
    }
  }
  
}

void signal_level(int val){     // led on if value is more than some amount
static byte led_on;             // did this routine turn the led on
                                // 3 processes are writing the led, so it is a bit overloaded.

  if( val > 50 || val < -50 ) led_on = 1, digitalWrite(led,HIGH);
  else if( led_on ) led_on = 0, digitalWrite(led,LOW);
}

void quake_detect( int val ){       // called once a second
static int detector;
static int holdoff;

  if( holdoff ){              // avoid counting the same quake twice
    --holdoff;
    return;
  }

  if( val > 20 || val < -20 ) ++detector;
  else if( detector > 0 ) --detector;
  
  if( detector > 300 ){
    if( quakes < 3 ) ++quakes;       // can only blink 3 a second, so not counting above 3
    detector = 0;
    hour_ = 0;                       // timeout in 24 hours
    holdoff = 3600;                  // no detects for next hour
  }
  
}


// run a simple FIR filter to reject 60 hz energy.  Coefficients are .25 .5 .75 1 .75 .5 .25
int filter(){
int val;        // if all these samples are max, then 4 * 8192 == 32k, at the limit of an integer.
                // in theory there should be no overflow

   val = samples[0] >> 2;
   val += samples[1] >> 1;
   val += samples[2] >> 2;
   val += samples[2] >> 1;
   val += samples[3];
   val += samples[4] >> 1;
   val += samples[4] >> 2;
   val += samples[5] >> 1;
   val += samples[6] >> 2;
   val >>= 1;                // filter has a gain of 4, returning a times by 2 value

   return val;
}


#include <Adafruit_NeoPixel.h>
#include <Servo.h>  //ライブラリ<Servo.h>を組み込む


#define  INT_T   40  // Interrupt time [ms]

struct repeating_timer st_tm1ms;

int global_time_ms = 0;         // 時刻 [ms]


//**********************************************************************
// Big RGB LED (NeoPixel) is controlled by serial command via NEOPIX_PIN (GPIO12).
// Small RGB LEDs are connected to GPIO 17,16 and 25.  
//**********************************************************************
#define  TIME_MS 200 // Wait time [ms]

// Big RGB LED (NeoPixel)
int NEO_PWR = 11;
int NEOPIX_PIN  = 12;
#define NUMPIXELS 1
Adafruit_NeoPixel pixels( NUMPIXELS, NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

// Small RGB LEDs
int rgb_LED[3] = { 17, 16, 25};




// GM6020 control
#define GM6020_PWM      D0

Servo sv; //svをServoの変数とする



 
void setup() {

  int ch;

  //[*] Serial to PC
  Serial.begin( 921600);
  

  //[*] Big RGB LED setting
  pixels.begin();
  pinMode( NEO_PWR, OUTPUT);
  digitalWrite( NEO_PWR, HIGH);


  //[*] Small RGB LED setting
  for(ch=0;ch<3;ch++){
    pinMode( rgb_LED[ch], OUTPUT);
    digitalWrite( rgb_LED[ch], HIGH);
  }


  //[*] GM6020 PWM control (50Hz)
  //
  // analogWriteは100Hz未満は設定できない．
  //
  pinMode( GM6020_PWM, OUTPUT);  
  // Pulse width: 1000-2000[us] 
  sv.attach( GM6020_PWM, 1000, 2000);
  
  // initial setting for GM6020 PWM
  sv.writeMicroseconds( 2000) ;   // [us]
  delay( 2000);                   // [ms]
  sv.writeMicroseconds( 1000) ;
  delay( 3000);  

  // timer 割り込み 
  /* タイマーの初期化(割込み間隔はusで指定) */
  add_repeating_timer_us( INT_T*1000, flip, NULL, &st_tm1ms);
}




////--------------------------------------------------------------
////---------------------- timer 割り込み関数 ----------------------
////--------------------------------------------------------------
bool flip(struct repeating_timer *t){

  float out;
  int duty;
  
  //[*] ------------- 時刻 [ms] -------------------------
  global_time_ms += INT_T;
  global_time_ms = (global_time_ms < 1000*1000) ? global_time_ms : 0; 


  out = ( 1 + sin( 2*PI*0.2*(float)global_time_ms/1000.0) )/2.0;
  duty = (int)( (1920 - 1080)*out + 1080 ); // [us]

  sv.writeMicroseconds( duty) ;

  Serial.printf( "AA,%f[s],%f[-],%f[-],%f[-],%f[-],%f[-],%f[-]\r\n", (float)global_time_ms/1000.0, out, (float)duty, 0.0, 0.0, 0.0, 0.0);

  return true;
}


 



////--------------------------------------------------------------
////---------------------- main loop -----------------------------
////--------------------------------------------------------------
void loop() { 
  
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(15, 25, 205));
  delay( TIME_MS);
  pixels.show();
  digitalWrite( rgb_LED[0], LOW);
  digitalWrite( rgb_LED[1], HIGH);
  digitalWrite( rgb_LED[2], HIGH);
  
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(103, 25, 205));
  delay( TIME_MS);
  pixels.show();
  digitalWrite( rgb_LED[0], HIGH);
  digitalWrite( rgb_LED[1], LOW);
  digitalWrite( rgb_LED[2], HIGH);
  
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(233, 242, 205));
  delay( TIME_MS);
  pixels.show();
  digitalWrite( rgb_LED[0], HIGH);
  digitalWrite( rgb_LED[1], HIGH);
  digitalWrite( rgb_LED[2], LOW);
  
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(233, 23, 23));
  delay( TIME_MS);
  pixels.show();
  digitalWrite( rgb_LED[0], LOW);
  digitalWrite( rgb_LED[1], LOW);
  digitalWrite( rgb_LED[2], LOW);
  
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(12, 66, 101));
  delay( TIME_MS);
  pixels.show();
  digitalWrite( rgb_LED[0], HIGH);
  digitalWrite( rgb_LED[1], HIGH);
  digitalWrite( rgb_LED[2], HIGH);
   
}

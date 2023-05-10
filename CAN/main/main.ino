#include "Adafruit_NeoPixel.h"
#include <Servo.h>  //ライブラリ<Servo.h>を組み込む
#include "mcp_can.h" //https://github.com/coryjfowler/MCP_CAN_lib 


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

#define MOTOR_ID        1                   // GM6020モータID
#define SEND_ID0        0x1FF               // 送信用ID(CAN通信)
#define READ_ID         (0x204 + MOTOR_ID)  // 受信用ID(CAN通信)
#define READ_WAIT_US    5000                // 受信最大待機時間 [us]

// CAN via MCP2515
#define CS0             D1  // CSピン
#define INT_PIN         D2  // 受信割り込みピン
// XIAO RP2040 の場合は，MCP_CAN CAN0( CS0) だけだとフリーズする．
MCP_CAN CAN0( &SPI, CS0);   // CAN0 CS: pin D1




 
void setup() {

  int ch;

  //[*] Serial to PC
  //
  // PC側の Serial port で，DTRを有効にしないと受信できない．
  // 1200 bps だと強制リセットが掛かる仕様．
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
  sv.writeMicroseconds( 1000) ;   // [us]
  delay( 2000);                   // [ms]  


  //[*] initialize CAN0 bus, baudrate: 1Mbps@20MHz  
  pinMode( INT_PIN, INPUT); // 受信割込のためのピンを設定

  if( CAN0.begin( MCP_ANY, CAN_1000KBPS, MCP_20MHZ) == CAN_OK ){
      
    Serial.printf( "CAN0: Init OK!\r\n");
    CAN0.setMode( MCP_NORMAL);
  }else{ 
    Serial.printf( "CAN0: Init Fail!\r\n");
  }
  

  //[*] timer 割り込み 
  /* タイマーの初期化(割込み間隔はusで指定) */
  add_repeating_timer_us( INT_T*1000, flip, NULL, &st_tm1ms);
}




////--------------------------------------------------------------
////---------------------- timer 割り込み関数 ----------------------
////--------------------------------------------------------------
bool flip(struct repeating_timer *t){

  float out;
  int duty;

  int read_time_start_us = 0, read_time_us = 0;

  int data = 0;
  byte send_data0[8] = { 0 };
  long unsigned int rxId = 0;
  byte read_len = 0;
  byte read_data[8] = { 0 };
  
  float angle = 0; // [rad]
  float angle_vel = 0; // [rad/s]
  float current = 0;
  float temp = 0;      // [degree] 

  char serial_buff[150] = { 0 };

  
  
  //[*] ------------- 時刻 [ms] -------------------------
  global_time_ms += INT_T;
  global_time_ms = (global_time_ms < 1000*1000) ? global_time_ms : 0; 



  //[*] GM6020 からの受信
  read_time_start_us = micros();
  while( digitalRead( INT_PIN) & (read_time_us < READ_WAIT_US) )    // If INT_PIN pin is low, read receive buffer
    read_time_us = micros() - read_time_start_us;

  if( read_time_us < READ_WAIT_US ){
    while( rxId != READ_ID || read_len != 8 )
      CAN0.readMsgBuf( &rxId, &read_len, read_data); 
  }  
    
  angle = 2*PI/8191.0*(short)( (read_data[0] << 8) | read_data[1] );    // [rad]
  angle_vel = 2*PI/60.0*(short)( (read_data[2] << 8) | read_data[3] );  // [rad/s]
  current = (short)( (read_data[4] << 8) | read_data[5] );
  temp = (float)read_data[6];                                           // モータの温度 [degree]


  out = ( 1 + sin( 2*PI*0.2*(float)global_time_ms/1000.0) )/2.0;
  //[*] PWM送信(角度制御)
  duty = (int)( (1920 - 1080)*out + 1080 ); // [us]
  //[*] CAN送信(電圧制御)
  data = (int)( 30000*(2*out - 1) );  
  send_data0[0] = (data >> 8) & 0xFF;
  send_data0[1] = data & 0xFF;
  
  
  //[*] GM6020 への指令送信
  sv.writeMicroseconds( duty) ;
  CAN0.sendMsgBuf( SEND_ID0, 0, 8, send_data0);

  

  //[*] 送信
  sprintf( serial_buff, "AA,%f[s],%f[-],%f[-],%f[-],%f[-],%f[-],%f[-]\r\n", (float)global_time_ms/1000.0,   out,        (float)duty,  angle, 
                                                                                                            angle_vel,      current,  temp);
  Serial.print( serial_buff);

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

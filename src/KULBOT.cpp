/*
The firmware is developed by 3S JSC.
@ Written by HungDang
*/
//========================ROBOT KULBOT==============================================
#include "KULBOT.h"
#include "KULBOTSPIFFs.h"
 uint8_t STATIC_IP_ARRAY_STA_WIFI[12];
 IPAddress AP_LOCAL_IP(192, 168, 4, 100);
 IPAddress AP_GATEWAY_IP(192, 168, 1, 4);
 IPAddress AP_NETWORK_MASK(255, 255, 255, 0);
const char *AP_PASSWORD_DEFAULT=NULL;
/* Variable count pulse encoder*/
volatile long _count_pluseA=0;
volatile long _count_pluseB=0;
volatile long _last_count_pluseA=0;
volatile long _last_count_pluseB=0;
double _kul_kp=0.0;
double _kul_ki=0.0;
double _kul_kd=0.0;
double _input_PV_encoder = 0, _output_enocoder = 0, _input_SV_encoder = 0;
PID KULBOT_PID(&_input_PV_encoder,&_output_enocoder, &_input_SV_encoder, _kul_kp, _kul_ki, _kul_kd, DIRECT);  
/* Variable count pulse encoder*/

// portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
// hw_timer_t* timer = NULL; 
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 
/* Sensor init obj*/


Adafruit_NeoPixel _RGB_WS2812(RGB_LED_COUNT,WS1812_PIN,NEO_GRB + NEO_KHZ800);
TCA9548A I2CMux; 
TwoWire DRIVER_MOTOR(1);
TwoWire DRIVER_SENSOR(0);
PCA9685 DRIVE_PWM_MOTOR =PCA9685(DRIVE_PWM_MOTOR_ADDR,DRIVER_MOTOR);
LiquidCrystal_I2C KULBOT_LCD(_DRIVE_LCD_ADDRESS,_DRIVE_LCD_NUM_CELLS,_DRIVE_LCD_NUM_COLUMN); 
PCF8574 KULBOT_LINE(&DRIVER_SENSOR,_LINE_SENSOR_ADDRESS);
PCF8574 KULBOT_TOUCH(&DRIVER_SENSOR,_TOUCH_SENSOR_ADDRESS);
PCF8574 KULBOT_IR(&DRIVER_SENSOR,_IR_SENSOR_ADDRESS);
PCF8574 KULBOT_TRAFFIC(&DRIVER_SENSOR,_TRAFFIC_SENSOR_ADDRESS);
PCF8574 KULBOT_BUTTON_LED(&DRIVER_SENSOR,_BUTTON_LED_ADDRESS);
DHT12   KULBOT_DHT(&DRIVER_SENSOR);
//COLOR_TCS34725 KULBOT_COLOR(TCS34725_INTEGRATIONTIME_50MS,TCS34725_GAIN_4X);
PCF8591 KULBOT_VOLUME(&DRIVER_SENSOR,_VOLUME_SENSOR_ADDRESS,DRIVE_MOTOR_SDA,DRIVE_MOTOR_SCL);
PCF8591 KULBOT_SOIL_HUM(&DRIVER_SENSOR,_SOIL_HUM_SENSOR_ADDRESS);
PCF8591 KULBOT_LIGHT(&DRIVER_SENSOR,_LIGHT_SENSOR_ADDRESS);
PCF8591 KULBOT_GAS(&DRIVER_SENSOR,_GAS_SENSOR_ADDRESS);
PCF8591 KULBOT_JOYSTICK(&DRIVER_SENSOR,_JOYSTICK_SENSOR_ADDRESS);
/* Sensor init obj */
uint8_t _pinout_PCF8574_arrry_config[8]={P0,P1,P2,P3,P4,P5,P6,P7};
/*--------------------------------------------SYSTEM VARIBALE-----------------------------*/
static bool _starting_system;
static int _count_button_power_on =0;
static int _count_button_power_off =0;
static int _count_button_power_stop =5;
//=====================================================================================
/*static void Manager_Power(void * parameter)
{
  for(;;)
  {
     unsigned long _time_start_system =millis();
    if(millis() - _time_start_system >= 1000UL && _starting_system ==1)
    {
        _time_start_system = millis();
        if(digitalRead(INPUT_BUTTON_POWER) == 0) _count_button_power_off++;
        else _count_button_power_off=0;
        if(_count_button_power_off >= _count_button_power_stop)
        {
            digitalWrite(LED_GREEN_POWER,HIGH);
            digitalWrite(LED_RED_POWER,LOW);
            delay(1000);
            digitalWrite(CONTROL_POWER,LOW);
        }    
    }
    vTaskDelay(10);
  }
}
static int PROID_TIME_WIFI_AP_BLINKY =1000;
void static Blinky_WIFI_AP_DISCONNECT(void)
{
  digitalWrite(LED_GREEN_POWER,LOW);
  digitalWrite(LED_RED_POWER,LOW);
  delay(PROID_TIME_WIFI_AP_BLINKY);
  digitalWrite(LED_GREEN_POWER,HIGH);
  digitalWrite(LED_RED_POWER,HIGH);
  delay(PROID_TIME_WIFI_AP_BLINKY);
}
static bool _status_connect_AP_Wifi;
static void _Check_conecttion_AP_WIFI(void * parameter)
{
  for(;;)
  {
    _status_connect_AP_Wifi = WiFi.softAPgetStationNum();
    if(_status_connect_AP_Wifi) digitalWrite(LED_GREEN_POWER,LOW);
    else Blinky_WIFI_AP_DISCONNECT();
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}*/
/*void KULBOT::KULBOT_SYSTEM_START(void)
{
  pinMode(CONTROL_POWER,OUTPUT);
  pinMode(INPUT_BUTTON_POWER,INPUT);
  pinMode(LED_GREEN_POWER,OUTPUT);
  pinMode(LED_RED_POWER,OUTPUT);
  digitalWrite(LED_GREEN_POWER,HIGH);
  digitalWrite(LED_RED_POWER,HIGH);
   while(_starting_system == 0)
 {
    unsigned long _time_starting_system=millis();
    if(millis() - _time_starting_system>= 1000UL)
    {
       _time_starting_system = millis();
       if(digitalRead(INPUT_BUTTON_POWER)==0) _count_button_power_on++;   
    }
    if(_count_button_power_on >= _count_button_power_stop) break;
 } 
 _starting_system =1; 
 digitalWrite(CONTROL_POWER,HIGH);
 digitalWrite(LED_GREEN_POWER,LOW);
 xTaskCreate(Manager_Power,"Manager",2000,NULL,0,NULL);
 xTaskCreate(_Check_conecttion_AP_WIFI,"AP_CON",5000,NULL,0,NULL);
}
//=====================================================================================
volatile uint32_t flag_timer_system =0;
void IRAM_ATTR onTimer() {   
  // portENTER_CRITICAL_ISR(&timerMux);
  //  flag_timer_system++;
  // portEXIT_CRITICAL_ISR(&timerMux); 
}
void KULBOT::KULBOT_ENABLE_TIMER_SYSTEM(void)
{
  // timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, 100000, true);
  // timerAlarmEnable(timer);
}*/
void IRAM_ATTR toggleLED()
{
delay(10);
digitalWrite(CONTROL_POWER,LOW);
delay(1000);
   }
void KULBOT::KULBOT_INIT(void)
{
 // this->KULBOT_SYSTEM_START();
//  SPIFFS_INIT();
   Serial.begin(115200);
   pinMode(CONTROL_POWER,OUTPUT);
   digitalWrite(CONTROL_POWER,HIGH);
   pinMode(INPUT_BUTTON_POWER, INPUT);
   attachInterrupt(digitalPinToInterrupt(INPUT_BUTTON_POWER), toggleLED,RISING);
   pinMode(LED_GREEN_POWER,OUTPUT);
   pinMode(LED_RED_POWER,OUTPUT);
   digitalWrite(LED_RED_POWER,0);
 _RGB_WS2812.clear();
}
/*--------------------------------------------KULBOT_WIFI_AP_INIT-----------------------------*/
/*void KULBOT::KULBOT_WIFI_AP_CONFIG_SYSTEM(void)
{
  String SSID  = SPIFFS_READFILE(SPIFFS, path_file_config); 
  if(SSID == "") SSID =(String)AP_NAME_SSID_DEFAULT;
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY_IP, AP_NETWORK_MASK);
  WiFi.softAP(SSID.c_str(),AP_PASSWORD_DEFAULT,AP_CHANEL_WIFI,AP_SSID_HIDEN_WIFI,AP_MAX_CONNECTION_WIFI);
}
/*--------------------------------------------KULBOT_WIFI_STA_INIT-----------------------------*/
/*static int _count_congfig_wifi_STA=0;
void KULBOT::KULBOT_WIFI_STA_INIT(String SSID,String PASSWORD,bool static_IP)
{
  WiFi.begin(SSID.c_str(),PASSWORD.c_str());
  if(static_IP)
  {
    IPAddress ip_static_local(STATIC_IP_ARRAY_STA_WIFI[0], STATIC_IP_ARRAY_STA_WIFI[1], STATIC_IP_ARRAY_STA_WIFI[2], STATIC_IP_ARRAY_STA_WIFI[3]);
    IPAddress gateway_static_local(STATIC_IP_ARRAY_STA_WIFI[4], STATIC_IP_ARRAY_STA_WIFI[5], STATIC_IP_ARRAY_STA_WIFI[6], STATIC_IP_ARRAY_STA_WIFI[7]);
    IPAddress subnet_static_local(STATIC_IP_ARRAY_STA_WIFI[8], STATIC_IP_ARRAY_STA_WIFI[9], STATIC_IP_ARRAY_STA_WIFI[10], STATIC_IP_ARRAY_STA_WIFI[11]);
    WiFi.config(ip_static_local,gateway_static_local,subnet_static_local);
  }
  else return;
  while(WiFi.status() != WL_CONNECTED)
  {
    _count_congfig_wifi_STA++;
    delay(1000);
    if(_count_congfig_wifi_STA>=30) break;
  }
  if(WiFi.status() == WL_CONNECTED) Serial.println(WiFi.localIP());
  else Serial.println("WiFi connect Faile!");
}
//======================================================================================
void KULBOT::KULBOT_GET_GATEWAY_STA_WIFI(uint8_t gt1,uint8_t gt2,uint8_t gt3,uint8_t gt4)
{
  STATIC_IP_ARRAY_STA_WIFI[4]=gt1;
  STATIC_IP_ARRAY_STA_WIFI[5]=gt2;
  STATIC_IP_ARRAY_STA_WIFI[6]=gt3;
  STATIC_IP_ARRAY_STA_WIFI[7]=gt4;
}
//======================================================================================
void KULBOT::KULBOT_GET_IP_STA_WIFI(uint8_t ip1,uint8_t ip2,uint8_t ip3,uint8_t ip4)
{
  STATIC_IP_ARRAY_STA_WIFI[0]=ip1;
  STATIC_IP_ARRAY_STA_WIFI[1]=ip2;
  STATIC_IP_ARRAY_STA_WIFI[2]=ip3;
  STATIC_IP_ARRAY_STA_WIFI[3]=ip4;
}
//======================================================================================
void KULBOT::KULBOT_GET_SUBNET_STA_WIFI(uint8_t sb1,uint8_t sb2,uint8_t sb3,uint8_t sb4)
{
  STATIC_IP_ARRAY_STA_WIFI[8]=sb1;
  STATIC_IP_ARRAY_STA_WIFI[9]=sb2;
  STATIC_IP_ARRAY_STA_WIFI[10]=sb3;
  STATIC_IP_ARRAY_STA_WIFI[11]=sb4;
}
/*--------------------------------------------Kulbot WS2812-----------------------------*/
void KULBOT::KULBOT_RGB_INIT(uint8_t WS2812_Brightness)
{
    _RGB_WS2812.begin();
    _RGB_WS2812.setBrightness(map(WS2812_Brightness,WS2812_Brightness_MIN,WS2812_Brightness_MAX,0,255));  
   // _RGB_WS2812.clear();
}
uint8_t KULBOT::_LED_CONV(uint8_t led)
{
  uint8_t led_conv;
  if(led ==1) led_conv =0;
  else if(led ==2) led_conv =1;
  else if(led ==3) led_conv =2;
  else if(led ==4) led_conv =3;
  else if(led ==5) led_conv =4;
  else if(led ==6) led_conv =5;
  else if(led ==7) led_conv =6;
  else if(led ==8) led_conv =7;
   return led_conv;
}
//======================================================================================
void KULBOT::KULBOT_RGB_ON(uint8_t led,uint8_t color)
{
   uint8_t r,g,b;
    if(color ==0) {r=255;g=0;b=0;} //red
    else if(color ==1) {r=255;g=127;b=0;}  //org
    else if(color ==2) {r=255;g=255;b=0;}  // yellow
    else if(color ==3) {r=0;g=255;b=0;}  // Green
    else if(color ==4) {r=0;g=0;b=255;}  // Blue
    else if(color ==5) {r=75;g=0;b=130;}  // INDIGO  {75,0,130};
    else if(color ==6) {r=143;g=0;b=255;}  // VIOLET  {143,0,255};
    else if(color ==7) {r=255;g=255;b=255;}  // RGB_COLOR_WHITE  {255,255,255};
    else if(color ==8) {r=0;g=0;b=0;}  // RGB_COLOR_WHITE  {255,255,255};
    _RGB_WS2812.setPixelColor(_LED_CONV(led),_RGB_WS2812.Color(r,g,b));
    _RGB_WS2812.show();
}
//======================================================================================
void KULBOT::KULBOT_RGB_OFF(uint8_t led)
{
  _RGB_WS2812.setPixelColor(_LED_CONV(led),_RGB_WS2812.Color(0,0,0));
  _RGB_WS2812.show();
}
//======================================================================================
void KULBOT::KULBOT_RGB_ALL_ON(uint8_t color)
{
  for(int i=0;i<RGB_LED_COUNT;i++)
  {
  this->KULBOT_RGB_ON(i,color);
 }
 }
//======================================================================================
void KULBOT::KULBOT_RGB_ALL_OFF()
{
 for(int i=0;i<RGB_LED_COUNT;i++)
  {
  this->KULBOT_RGB_ON(i,RGB_COLOR_BLACK);
  }
}
//======================================================================================
void KULBOT::KULBOT_RGB_SET_HUE(uint8_t led , uint8_t r,uint8_t g,uint8_t b)
{
   _RGB_WS2812.setPixelColor(_LED_CONV(led),_RGB_WS2812.Color(r,g,b));
   _RGB_WS2812.show();
}
//======================================================================================
void KULBOT::KULBOT_RGB_AlLSET_HUE(uint8_t r,uint8_t g,uint8_t b)
{
    for(int i=0;i<RGB_LED_COUNT;i++)
    {
        this->KULBOT_RGB_SET_HUE(i,r,g,b);
    }
}
//======================================================================================
/*--------------------------------------------Kulbot Motor-----------------------------*/
volatile int _POSI_ENCODER[] = {0,0};
const int ENCODER_CHA[]={DRIVE_HAL_MOTOR1_CHA,DRIVE_HAL_MOTOR2_CHA};
const int ENCODER_CHB[]={DRIVE_HAL_MOTOR1_CHB,DRIVE_HAL_MOTOR2_CHB};
template <int j>
void readEncoder(){
  int b = digitalRead(ENCODER_CHB[j]);
  if(b > 0){
   _POSI_ENCODER[j]++; //_POSI_ENCODER[j]++;
  }
  else{
    _POSI_ENCODER[j]--; //  _POSI_ENCODER[j]--;
  }
}
void IRAM_ATTR readEncoderA()
{
  int MSB = digitalRead(DRIVE_HAL_MOTOR1_CHA); 
  int LSB = digitalRead(DRIVE_HAL_MOTOR1_CHB); 
  int encoded = (MSB << 1) |LSB;
  int sum  = (_last_count_pluseA << 2) | encoded;
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) _count_pluseA++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) _count_pluseA--;
  _last_count_pluseA = encoded;
}
//======================================================================================
void IRAM_ATTR readEncoderB()
{
  int MSB = digitalRead(DRIVE_HAL_MOTOR2_CHA); 
  int LSB = digitalRead(DRIVE_HAL_MOTOR2_CHB); 
  int encoded = (MSB << 1) |LSB;
  int sum  = (_last_count_pluseB << 2) | encoded;
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) _count_pluseB++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) _count_pluseB--;
  _last_count_pluseB = encoded;
}
//======================================================================================
SimplePID pid[2];
void KULBOT::KULBOT_MOTORENCODER_INIT()
{  
  DRIVER_MOTOR.begin(DRIVE_MOTOR_SDA,DRIVE_MOTOR_SCL,DRIVE_MOTOR_FREQ);
  DRIVE_PWM_MOTOR.begin();
  DRIVE_PWM_MOTOR.setPWMFreq(DRIVE_PWM_FREQ);
 // DRIVE_PWM_MOTOR.setOscillatorFrequency(DRIVE_OscillatorFrequency);
  pinMode(DRIVE_HAL_MOTOR1_CHA,INPUT_PULLUP);
  pinMode(DRIVE_HAL_MOTOR1_CHB,INPUT_PULLUP);
  pinMode(DRIVE_HAL_MOTOR2_CHA,INPUT_PULLUP);
  pinMode(DRIVE_HAL_MOTOR2_CHB,INPUT_PULLUP);
  ledcSetup(LED_PWM_CHANEL1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LED_PWM_CHANEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(DRIVE_PWMA, LED_PWM_CHANEL1);
  ledcAttachPin(DRIVE_PWMB, LED_PWM_CHANEL2);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHA[0]),readEncoder<0>,FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHA[1]),readEncoder<1>,FALLING);
  // attachInterrupt(digitalPinToInterrupt(DRIVE_HAL_MOTOR1_CHA),readEncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(DRIVE_HAL_MOTOR1_CHB),readEncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(DRIVE_HAL_MOTOR2_CHA),readEncoderB, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(DRIVE_HAL_MOTOR2_CHB),readEncoderB, CHANGE);
  //this->KULBOT_ENABLE_TIMER_SYSTEM();
  this->PWM_MOTORA(MIN_SPEED_MOTOR);
  this->PWM_MOTORB(MIN_SPEED_MOTOR);
  //this->MOTOR_BRAKE(DRIVE_MOTORA);
  // KULBOT_PID.SetMode(AUTOMATIC);
  // KULBOT_PID.SetSampleTime(1);
  // KULBOT_PID.SetOutputLimits(-100, 100); 

 pid[0].setParams(0.5,0.0,0,1024);
}
//======================================================================================
void KULBOT::PWM_MOTORA(uint8_t speed)
{
 ledcWrite(LED_PWM_CHANEL1,map(speed,MIN_SPEED_MOTOR,MAX_SPEED_MOTOR,PWM_MIN_DUTY_CYCLE,PWM_MAX_DUTY_CYCLE));
}
//======================================================================================
void  KULBOT::PWM_MOTORB(uint8_t speed)
{
 ledcWrite(LED_PWM_CHANEL2,map(speed,MIN_SPEED_MOTOR,MAX_SPEED_MOTOR,PWM_MIN_DUTY_CYCLE,PWM_MAX_DUTY_CYCLE));
}
//======================================================================================
void KULBOT::MOTOR_RUN_FORWARD(int motor)
{
 if(motor == DRIVE_MOTORA)
 {
   DRIVE_PWM_MOTOR.setPWM(DIR_MA2,DRIVE_MAX_DUTY_CYCLE,DRIVE_MIN_DUTY_CYCLE);
   DRIVE_PWM_MOTOR.setPWM(DIR_MA1,DRIVE_MIN_DUTY_CYCLE,DRIVE_MAX_DUTY_CYCLE);
 }
 else if(motor == DRIVE_MOTORB)
 {
   DRIVE_PWM_MOTOR.setPWM(DIR_MB1,DRIVE_MAX_DUTY_CYCLE,DRIVE_MIN_DUTY_CYCLE);
   DRIVE_PWM_MOTOR.setPWM(DIR_MB2,DRIVE_MIN_DUTY_CYCLE,DRIVE_MAX_DUTY_CYCLE);
 }
}
//======================================================================================
void KULBOT::MOTOR_RUN_BACKWARD(int motor)
{
 if(motor == DRIVE_MOTORA)
 {
   DRIVE_PWM_MOTOR.setPWM(DIR_MA2,DRIVE_MIN_DUTY_CYCLE,DRIVE_MAX_DUTY_CYCLE);
   DRIVE_PWM_MOTOR.setPWM(DIR_MA1,DRIVE_MAX_DUTY_CYCLE,DRIVE_MIN_DUTY_CYCLE);
 }
 else if(motor == DRIVE_MOTORB)
 {
   DRIVE_PWM_MOTOR.setPWM(DIR_MB1,DRIVE_MIN_DUTY_CYCLE,DRIVE_MAX_DUTY_CYCLE);
   DRIVE_PWM_MOTOR.setPWM(DIR_MB2,DRIVE_MAX_DUTY_CYCLE,DRIVE_MIN_DUTY_CYCLE);
 }
}
//======================================================================================
void KULBOT::MOTOR_STOP(int motor)
{
    if(motor == DRIVE_MOTORA) this->PWM_MOTORA(MIN_SPEED_MOTOR);
    else if(motor == DRIVE_MOTORB) this->PWM_MOTORB(MIN_SPEED_MOTOR);
    else if(motor == DRIVE_MOTORAB) 
    {
        this->PWM_MOTORB(MIN_SPEED_MOTOR);
        this->PWM_MOTORA(MIN_SPEED_MOTOR);
    }
}
void KULBOT::MOTOR_BRAKE(int motor)
{
  if(motor == DRIVE_MOTORA)
  {
    DRIVE_PWM_MOTOR.setPWM(DIR_MA2,DRIVE_MAX_DUTY_CYCLE,DRIVE_MIN_DUTY_CYCLE);
   DRIVE_PWM_MOTOR.setPWM(DIR_MA1,DRIVE_MAX_DUTY_CYCLE,DRIVE_MIN_DUTY_CYCLE);
  }
}
//======================================================================================
void KULBOT::KULBOT_MOTORENCODER_RUN1(int motor,uint8_t speed,uint8_t dir)
{
 if(motor== DRIVE_MOTORA && dir == DRIVE_MOVE_FORWARD)
 {
  this->MOTOR_RUN_FORWARD(DRIVE_MOTORA);
  this->PWM_MOTORA(speed);
 }
 else if(motor== DRIVE_MOTORA && dir == DRIVE_MOVE_BACKWARD)
 {
  this->MOTOR_RUN_BACKWARD(DRIVE_MOTORA);
  this->PWM_MOTORA(speed);
 }
  if(motor== DRIVE_MOTORB && dir == DRIVE_MOVE_FORWARD)
 {
  this->MOTOR_RUN_FORWARD(DRIVE_MOTORB);
  this->PWM_MOTORB(speed);
 }

 else if(motor== DRIVE_MOTORB && dir == DRIVE_MOVE_BACKWARD)
 {
  this->MOTOR_RUN_BACKWARD(DRIVE_MOTORB);
  this->PWM_MOTORB(speed);
 }
}
void KULBOT::KULBOT_MOTORENCODER_RUN2(int motor,uint8_t speed,uint8_t dir,unsigned long timer)
{
  uint8_t Vspeed_min =0;
  this->KULBOT_MOTORENCODER_RUN1(motor,speed,dir);
  delay(timer);
 KULBOT_MOTORENCODER_RUN1(motor,Vspeed_min,dir);
 delay((unsigned long )timer/100);
}
 unsigned long prevT = 0;

void KULBOT::setMotor(int dir, int pwmVal){
  this->PWM_MOTORA(pwmVal);
  if(dir == 1){
    this->MOTOR_RUN_FORWARD(DRIVE_MOTORA);
  }
  else if(dir == -1){
   this->MOTOR_RUN_FORWARD(DRIVE_MOTORA);
  }
 
}
void KULBOT::KULBOT_MOTORENCODER_RUN3(int motor,uint8_t speed,uint8_t dir,int degree)
{
  int target[2];
  target[0] =34;
  unsigned long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6);
  // Serial.println("deltaT:" +(String)deltaT);
  prevT = currT;
  int pos[2];
  noInterrupts();
  pos[0] = _POSI_ENCODER[0];
  interrupts();
  int pwr;
  int dirM;
   pid[0].evalu(pos[0],target[0],deltaT,pwr,dirM);
   Serial.println(pos[0]);
  setMotor(dirM,pwr);
   

}
/*--------------------------------------------Kulbot Servo-----------------------------*/
void KULBOT::KULBOT_SERVO_INIT(void)
{
  DRIVER_MOTOR.begin(DRIVE_MOTOR_SDA,DRIVE_MOTOR_SCL,DRIVE_MOTOR_FREQ);
  DRIVE_PWM_MOTOR.begin();
  DRIVE_PWM_MOTOR.setPWMFreq(DRIVE_PWM_FREQ);
  DRIVE_PWM_MOTOR.setOscillatorFrequency(DRIVE_OscillatorFrequency);
}
//======================================================================================
uint8_t KULBOT::_selection_servo_port(uint8_t _port_servo)
{  
   if(_port_servo==1)        _num_servo_port = _SERVO_PORT_1;
   else if(_port_servo==2)   _num_servo_port = _SERVO_PORT_2;
   else if(_port_servo ==3)  _num_servo_port = _SERVO_PORT_3;
   else if(_port_servo ==4)  _num_servo_port = _SERVO_PORT_4;
   else if(_port_servo ==5)  _num_servo_port = _SERVO_PORT_5;
   else if(_port_servo ==6)  _num_servo_port = _SERVO_PORT_6;
   else if(_port_servo ==7)  _num_servo_port = _SERVO_PORT_7;
   else if(_port_servo ==8)  _num_servo_port = _SERVO_PORT_8;
   return _num_servo_port;
}
//======================================================================================
int KULBOT::_angleToPulse(uint8_t angle)
{
  int pulse_servo;
  pulse_servo= (int)(map(angle,SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_MIN_PULSE,SERVO_MAX_PULSE));
  return pulse_servo;
}
//======================================================================================
void KULBOT::KULBOT_SERVO_SET_ANGLE(uint8_t servo,uint8_t ang)
{
 DRIVE_PWM_MOTOR.setPWM(this->_selection_servo_port(servo),0,this->_angleToPulse(ang));
}
void KULBOT::KULBOT_MOVE_SERVO(int time, int servo_target[]){
  if (time > 10) {
    for (int i = 1; i < 9; i++) {
      increment[i] = ((servo_target[i]) - servo_position[i]) / (time / 10.0);
    }
    final_time =  millis() + time;

    for (int iteration = 1; millis() < final_time; iteration++) {
      partial_time = millis() + 10;

      for (int i = 1; i < 9; i++) {
        this->KULBOT_SERVO_SET_ANGLE(i,(int)(servo_position[i] + (iteration * increment[i])));
      }
      while (millis() < partial_time);
    }
  }
  else {
    for (int i = 1; i < 9; i++) {
      this->KULBOT_SERVO_SET_ANGLE(i,(int)servo_target[i]);
    }
  }
  for (int i = 1; i < 9; i++) {
    servo_position[i] = servo_target[i];
  }
}
/*--------------------------------------------Kulbot Sensor-----------------------------*/
void KULBOT::KULBOT_SENSOR_INIT()
{
  //DRIVE_SENSOR_SDA,DRIVE_SENSOR_SCL,_DRIVE_SENSOR_FREQ
  DRIVER_SENSOR.begin();
  I2CMux.begin(DRIVER_SENSOR); 
  //I2CMux.closeAll();
}
//======================================================================================
uint8_t KULBOT::_selection_sensor_port(uint8_t port)
{
  if(port==1) _port_sensor_selection=_SENSOR_PORT_1;
  else if(port==2) _port_sensor_selection =_SENSOR_PORT_2;
  else if(port==3) _port_sensor_selection =_SENSOR_PORT_3;
  else if(port==4) _port_sensor_selection =_SENSOR_PORT_4;
  else if(port==5) _port_sensor_selection =_SENSOR_PORT_5;
  else if(port==6) _port_sensor_selection =_SENSOR_PORT_6;
  else if(port==7) _port_sensor_selection =_SENSOR_PORT_7;
  else if(port==8) _port_sensor_selection =_SENSOR_PORT_8;
  return _port_sensor_selection;
}
/*--------------------------------------------Kulbot LCD-----------------------------*/
void KULBOT::KULBOT_LCD_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_LCD.init();
  KULBOT_LCD.clear();
  KULBOT_LCD.noCursor();
  KULBOT_LCD.backlight();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  
}
//======================================================================================
void KULBOT::KULBOT_LCD_PRINT_STRING(uint8_t port,uint8_t column,uint8_t cell,String data)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_LCD.setCursor(column,cell);
  KULBOT_LCD.print(data);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
void KULBOT::KULBOT_LCD_PRINT_NUMBER(uint8_t port, uint8_t column,uint8_t cell,long num)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_LCD.setCursor(column,cell);
  KULBOT_LCD.print(num);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
void KULBOT::KULBOT_LCD_CLEAR(uint8_t port)
{
 I2CMux.openChannel(this->_selection_sensor_port(port));
 KULBOT_LCD.clear();
 I2CMux.closeChannel(this->_selection_sensor_port(port));
}
uint8_t KULBOT::_read_byte_value_sensor(byte addr)
{
  volatile uint8_t _byte_requsetFrom_sensor;
  DRIVER_SENSOR.requestFrom(addr,(uint8_t)1,(uint8_t)true);
  if(DRIVER_SENSOR.available()) _byte_requsetFrom_sensor = (uint8_t)DRIVER_SENSOR.read();
  // Serial.println("byte read:"+(String)_byte_requsetFrom_sensor);
   return _byte_requsetFrom_sensor;
  
}
void KULBOT::_write_byte_value_sensor(byte addr,byte data)
{
  DRIVER_SENSOR.beginTransmission(addr);
  DRIVER_SENSOR.write(data);
  DRIVER_SENSOR.endTransmission();
}
/*--------------------------------------------Kulbot SENSOR----------------------------*/
void KULBOT::KULBOT_LINE_SENSOR_INIT(uint8_t port)
{
    I2CMux.openChannel(this->_selection_sensor_port(port));
    KULBOT_LINE.begin();
    KULBOT_LINE.pinMode(P0,INPUT);
    KULBOT_LINE.pinMode(P1,INPUT);
    I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
bool KULBOT::KULBOT_GET_LINE_SENSOR(uint8_t port,int line)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  if(line ==_LINE_SENSOR_LEFT) _value_line_sensor = KULBOT_LINE.digitalRead(P0);
  else if(line ==_LINE_SENSOR_RIGHT) _value_line_sensor = KULBOT_LINE.digitalRead(P1);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _value_line_sensor;
}
//======================================================================================
void KULBOT::KULBOT_TOUCH_SENSOR_INIT(uint8_t port)
{
    I2CMux.openChannel(this->_selection_sensor_port(port));
    KULBOT_TOUCH.begin();
    KULBOT_TOUCH.pinMode(P0,INPUT);
    I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
bool KULBOT::KULBOT_GET_TOUCH_SENSOR(uint8_t port)
{
    I2CMux.openChannel(this->_selection_sensor_port(port));
    _value_touch_sensor = KULBOT_TOUCH.digitalRead(P0);
    delay(10);
    I2CMux.closeChannel(this->_selection_sensor_port(port));
    return _value_touch_sensor;
    
}
//======================================================================================
void KULBOT::KULBOT_IR_SENSOR_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_IR.begin();
  KULBOT_IR.pinMode(P0,INPUT);
  for(int i=1;i<8;i++)
  {
    KULBOT_IR.pinMode(_pinout_PCF8574_arrry_config[i],OUTPUT);
    KULBOT_IR.digitalWrite(_pinout_PCF8574_arrry_config[i],HIGH);
  }
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
/*
  RED : P1,P4
  GREEN:P2,P5
  BLUE: P3,P6
*/
void KULBOT::IR_LED_RED_CONTROL(bool status)
{
  KULBOT_IR.digitalWrite(P1,status);
  KULBOT_IR.digitalWrite(P4,status);
}
void KULBOT::IR_LED_GREEN_CONTROL(bool status)
{
  KULBOT_IR.digitalWrite(P2,status);
  KULBOT_IR.digitalWrite(P5,status);
}
void KULBOT::IR_LED_BLUE_CONTROL(bool status)
{
  KULBOT_IR.digitalWrite(P3,status);
  KULBOT_IR.digitalWrite(P6,status);
}
void KULBOT::_IR_TURN_ON_RED()
{
  this->IR_LED_RED_CONTROL(LOW);
  this->IR_LED_BLUE_CONTROL(HIGH);
  this->IR_LED_GREEN_CONTROL(HIGH);
}
//======================================================================================
void KULBOT::_IR_TURN_ON_BLUE()
{
  this->IR_LED_RED_CONTROL(HIGH);
  this->IR_LED_BLUE_CONTROL(LOW);
  this->IR_LED_GREEN_CONTROL(HIGH);
}
//======================================================================================
void KULBOT::_IR_TURN_ON_GREEN()
{
  this->IR_LED_RED_CONTROL(HIGH);
  this->IR_LED_BLUE_CONTROL(HIGH);
  this->IR_LED_GREEN_CONTROL(LOW);
}
//======================================================================================
void KULBOT::_IR_TURN_ON_YELLOW()
{
  this->IR_LED_RED_CONTROL(LOW);
  this->IR_LED_BLUE_CONTROL(HIGH);
  this->IR_LED_GREEN_CONTROL(LOW);
}
//======================================================================================
void KULBOT::_IR_TURN_ON_CYN()
{
   this->IR_LED_RED_CONTROL(HIGH);
  this->IR_LED_BLUE_CONTROL(LOW);
  this->IR_LED_GREEN_CONTROL(LOW);
}
//======================================================================================
void KULBOT::_IR_TURN_ON_VIOLET()
{
  this->IR_LED_RED_CONTROL(LOW);
  this->IR_LED_BLUE_CONTROL(LOW);
  this->IR_LED_GREEN_CONTROL(HIGH);
}
//======================================================================================
void KULBOT::_IR_TURN_ON_WHITLE()
{
  this->IR_LED_RED_CONTROL(LOW);
  this->IR_LED_BLUE_CONTROL(LOW);
  this->IR_LED_GREEN_CONTROL(LOW);
}
//======================================================================================
void KULBOT::KULBOT_SET_IR_SENSOR_LED(uint8_t port,uint8_t color)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  if(color ==_IR_COLOR_RED) this->_IR_TURN_ON_RED();
  else if(color == _IR_COLOR_BLUE) this->_IR_TURN_ON_BLUE();
  else if(color==_IR_COLOR_GREEN) this->_IR_TURN_ON_GREEN();
  else if(color==_IR_COLOR_YELLOW) this->_IR_TURN_ON_YELLOW();
  else if(color==_IR_COLOR_CYN) this->_IR_TURN_ON_CYN();
  else if(color==_IR_COLOR_VIOLET) this->_IR_TURN_ON_VIOLET();
  else if(color==_IR_COLOR_WHITE) this->_IR_TURN_ON_WHITLE();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
bool KULBOT::KULBOT_GET_IR_SENSOR(uint8_t port)
{
    I2CMux.openChannel(this->_selection_sensor_port(port));
    _value_ir_sensor = KULBOT_IR.digitalRead(P0);
    delay(10);
    I2CMux.closeChannel(this->_selection_sensor_port(port));
    return _value_ir_sensor;
}
//======================================================================================
void KULBOT::KULBOT_DHT_SENSOR_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_DHT.begin();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
uint8_t KULBOT::KULBOT_GET_TEMP_DHT_SENSOR(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  int status = KULBOT_DHT.read();
  _value_temp_sensor  = KULBOT_DHT.getTemperature();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _value_temp_sensor;
}
//======================================================================================
uint8_t KULBOT::KULBOT_GET_HUM_DHT_SENSOR(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  int status = KULBOT_DHT.read();
  _value_temp_sensor  = KULBOT_DHT.getHumidity();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _value_temp_sensor;
}
//======================================================================================
uint8_t KULBOT::KULBOT_GET_SONAR_SENSOR(uint8_t port)
{
   I2CMux.openChannel(this->_selection_sensor_port(port));
   this->_write_byte_value_sensor(_SONAR_SENSOR_ADDRESS,_SONAR_SENSOR_CONFIG_READ);
   delay(5);
  _value_sonar_sensor = this->_read_byte_value_sensor(_SONAR_SENSOR_ADDRESS);
   I2CMux.closeChannel(this->_selection_sensor_port(port));
   return _value_sonar_sensor;
  // delay(1);
}
//======================================================================================
void KULBOT::KULBOT_TRAFFIC_LIGHT_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_TRAFFIC.begin();
  KULBOT_TRAFFIC.pinMode(P0,OUTPUT);
  KULBOT_TRAFFIC.pinMode(P1,OUTPUT);
  KULBOT_TRAFFIC.pinMode(P2,OUTPUT);
  KULBOT_TRAFFIC.digitalWrite(P0,1);
  KULBOT_TRAFFIC.digitalWrite(P1,1);
  KULBOT_TRAFFIC.digitalWrite(P2,1);
  I2CMux.openChannel(this->_selection_sensor_port(port));
  delay(10);
}
//======================================================================================
void KULBOT::KULBOT_SET_TRAFFIC_LIGHT(uint8_t port,uint8_t light,bool status)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  if(light==_red_traffic) KULBOT_TRAFFIC.digitalWrite(P2,status);
  // {
  //   if(status == _status_traffic_on) KULBOT_TRAFFIC.digitalWrite(P2,LOW);
  //   else if(status == _status_traffic_off) KULBOT_TRAFFIC.digitalWrite(P2,HIGH);
  // }
  else if(light==_yellow_traffic) KULBOT_TRAFFIC.digitalWrite(P1,status);
  // {
  //   if(status == _status_traffic_on) KULBOT_TRAFFIC.digitalWrite(P1,LOW);
  //   else if(status == _status_traffic_off) KULBOT_TRAFFIC.digitalWrite(P1,HIGH);
  // }
  else if(light == _green_traffic) KULBOT_TRAFFIC.digitalWrite(P0,status);
  // {
  //   if(status == _status_traffic_on) KULBOT_TRAFFIC.digitalWrite(P0,LOW);
  //   else if(status == _status_traffic_off) KULBOT_TRAFFIC.digitalWrite(P0,HIGH);
  // }
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================

void KULBOT::KULBOT_BUTTON_LED_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_BUTTON_LED.begin();
  KULBOT_BUTTON_LED.pinMode(P0,INPUT);
  KULBOT_BUTTON_LED.pinMode(P1,INPUT);
//  for(int i=2;i<8;i++)
//  {
//    KULBOT_BUTTON_LED.pinMode(_pinout_PCF8574_arrry_config[i],OUTPUT);
//    KULBOT_BUTTON_LED.digitalWrite(_pinout_PCF8574_arrry_config[i],HIGH);
//  }
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
bool KULBOT::KULBOT_GET_BUTTON_LED(uint8_t port,int button)
{
 I2CMux.openChannel(this->_selection_sensor_port(port));
 if(button == _LEFT_BUTTON_LED) _value_button_led = KULBOT_BUTTON_LED.digitalRead(P0);
 else if(button == _RIGHT_BUTTON_LED) _value_button_led = KULBOT_BUTTON_LED.digitalRead(P1);
 I2CMux.closeChannel(this->_selection_sensor_port(port));
 return _value_button_led;
}
/*
  RED : P2,P5
  GREEN:P3,P6
  BLUE: P4,P7
*/
void KULBOT::BUTTON_LED_RED_CONTROL(bool status)
{
 KULBOT_BUTTON_LED.digitalWrite(P2,status);
 KULBOT_BUTTON_LED.digitalWrite(P5,status);
}
void KULBOT::BUTTON_LED_GREEN_CONTROL(bool status)
{
 KULBOT_BUTTON_LED.digitalWrite(P3,status);
 KULBOT_BUTTON_LED.digitalWrite(P6,status);
}
void KULBOT::BUTTON_LED_BLUE_CONTROL(bool status)
{
 KULBOT_BUTTON_LED.digitalWrite(P4,status);
 KULBOT_BUTTON_LED.digitalWrite(P7,status);
}
void KULBOT::_BUTTON_LED_ON_RED()
{
 this->BUTTON_LED_RED_CONTROL(LOW);
 this->BUTTON_LED_GREEN_CONTROL(HIGH);
 this->BUTTON_LED_BLUE_CONTROL(HIGH);
}
void KULBOT::_BUTTON_LED_ON_GREEN()
{
 this->BUTTON_LED_RED_CONTROL(HIGH);
 this->BUTTON_LED_GREEN_CONTROL(LOW);
 this->BUTTON_LED_BLUE_CONTROL(HIGH);
}
void KULBOT::_BUTTON_LED_ON_BLUE()
{
 this->BUTTON_LED_RED_CONTROL(HIGH);
 this->BUTTON_LED_GREEN_CONTROL(HIGH);
 this->BUTTON_LED_BLUE_CONTROL(LOW);
}
void KULBOT::_BUTTON_LED_ON_YELLOW()
{
 this->BUTTON_LED_RED_CONTROL(LOW);
 this->BUTTON_LED_GREEN_CONTROL(LOW);
 this->BUTTON_LED_BLUE_CONTROL(HIGH);
}
void KULBOT::_BUTTON_LED_ON_CYN()
{
 this->BUTTON_LED_RED_CONTROL(HIGH);
 this->BUTTON_LED_GREEN_CONTROL(LOW);
 this->BUTTON_LED_BLUE_CONTROL(LOW);
}
void KULBOT::_BUTTON_LED_ON_VIOLET()
{
 this->BUTTON_LED_RED_CONTROL(LOW);
 this->BUTTON_LED_GREEN_CONTROL(HIGH);
 this->BUTTON_LED_BLUE_CONTROL(LOW);
}
void KULBOT::_BUTTON_LED_ON_WHITLE()
{
 this->BUTTON_LED_RED_CONTROL(LOW);
 this->BUTTON_LED_GREEN_CONTROL(LOW);
 this->BUTTON_LED_BLUE_CONTROL(LOW);
}
void KULBOT::KULBOT_SET_BUTTON_LED(uint8_t port,uint8_t color)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  if(color ==_BUTTON_COLOR_RED) this->_BUTTON_LED_ON_RED();
  else if(color == _BUTTON_COLOR_GREEN) this->_BUTTON_LED_ON_GREEN();
  else if(color==_BUTTON_COLOR_BLUE) this->_BUTTON_LED_ON_BLUE();
  else if(color==_BUTTON_COLOR_YELLOW) this->_BUTTON_LED_ON_YELLOW();
  else if(color==_BUTTON_COLOR_CYN) this->_BUTTON_LED_ON_CYN();
  else if(color==_BUTTON_COLOR_VIOLET) this->_BUTTON_LED_ON_VIOLET();
  else if(color==_BUTTON_COLOR_WHITE) this->_BUTTON_LED_ON_WHITLE();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
//======================================================================================
/*void KULBOT::KULBOT_GRYRO_SENSOR_INIT(uint8_t port)
{
 I2CMux.openChannel(this->_selection_sensor_port(port));
 KULBOT_GRYRO.begin();
 KULBOT_GRYRO.calcGyroOffsets(true);
 I2CMux.closeChannel(this->_selection_sensor_port(port));
}
double KULBOT::KULBOT_GET_GRYRO_SENSOR(uint8_t port,uint8_t _data_get)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_GRYRO.update();
  if(_data_get == _get_GRYRO_ANGLE_X) _gryro_data_get = KULBOT_GRYRO.getAngleX();
  else if(_data_get == _get_GRYRO_ANGLE_Y) _gryro_data_get = KULBOT_GRYRO.getAngleY();
  else if(_data_get == _get_GRYRO_ANGLE_Z) _gryro_data_get = KULBOT_GRYRO.getAngleZ();
  else if(_data_get == _get_GRYRO_ACC_X) _gryro_data_get = KULBOT_GRYRO.getAccX();
  else if(_data_get == _get_GRYRO_ACC_Y) _gryro_data_get = KULBOT_GRYRO.getAccY();
  else if(_data_get == _get_GRYRO_ACC_Z) _gryro_data_get = KULBOT_GRYRO.getAccZ();
  else if(_data_get == _get_GRYRO_X) _gryro_data_get = KULBOT_GRYRO.getGyroX();
  else if(_data_get == _get_GRYRO_Y) _gryro_data_get = KULBOT_GRYRO.getGyroY();
  else if(_data_get == _get_GRYRO_Z) _gryro_data_get = KULBOT_GRYRO.getGyroZ();
  else if(_data_get == _get_GRYRO_ACC_ANGLE_X) _gryro_data_get = KULBOT_GRYRO.getGyroAngleX();
  else if(_data_get == _get_GRYRO_ACC_ANGLE_Y) _gryro_data_get = KULBOT_GRYRO.getGyroAngleY();
  else if(_data_get == _get_GRYRO_ACC_ANGLE_Z) _gryro_data_get = KULBOT_GRYRO.getGyroAngleZ();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _gryro_data_get;
}*/
/*void KULBOT::KULBOT_COLOR_SENSOR_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_COLOR.begin();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}*/
/*uint16_t KULBOT::KULBOT_GET_COLOR_SENSOR(uint8_t port, uint8_t _data)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
    KULBOT_COLOR.getRawData(&_color_green,&_color_red,&_color_blue,&_color_clear);
    if(_data == 0) _get_color_read = _color_green;
    else if(_data == 1) _get_color_read = _color_red;
    else if(_data == 2) _get_color_read = _color_blue;
    else if(_data == 3) _get_color_read = _color_clear;
  I2CMux.closeChannel(this->_selection_sensor_port(port));
   return _get_color_read;
}*/
void KULBOT::KULBOT_VOLUME_INIT(uint8_t port)
{
 // I2CMux.openChannel(this->_selection_sensor_port(port));
 // KULBOT_VOLUME.begin();
 // I2CMux.closeChannel(this->_selection_sensor_port(port));
}
uint8_t KULBOT::KULBOT_GET_VOLUME_SENSOR(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  _volume_read_value =KULBOT_VOLUME.analogRead(AIN0);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _volume_read_value;
}
void KULBOT::KULBOT_SOIL_HUM_SENSOR_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_SOIL_HUM.begin();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
uint8_t KULBOT::KULBOT_GET_SOIL_HUM_SENSOR(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  _soil_hum_read_value = KULBOT_VOLUME.analogRead(AIN0);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _soil_hum_read_value;
}
void KULBOT::KULBOT_LIGHT_SENSOR_INTIT(uint8_t port)
{
   I2CMux.openChannel(this->_selection_sensor_port(port));
   KULBOT_LIGHT.begin();
   I2CMux.closeChannel(this->_selection_sensor_port(port));
}
uint8_t KULBOT::KULBOT_GET_LIGHT_SENSOR(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  _light_read_value =KULBOT_LIGHT.analogRead(AIN0);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _light_read_value;
}
/*void KULBOT::KULBOT_GAS_SENSOR_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_GAS.begin();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
uint8_t KULBOT::KULBOT_GET_GAS_SENSOR(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  _gas_read_value=KULBOT_GAS.analogRead(AIN0);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _gas_read_value;
}*/
void KULBOT::KULBOT_JOYSTICK_SENSOR_INIT(uint8_t port)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  KULBOT_JOYSTICK.begin();
  I2CMux.closeChannel(this->_selection_sensor_port(port));
}
uint8_t KULBOT::KULBOT::KULBOT_GET_JOYSTICK_SENSOR(uint8_t port,uint8_t type_get)
{
  I2CMux.openChannel(this->_selection_sensor_port(port));
  if(type_get == _Joystick_X) _joystick_read_value = KULBOT_JOYSTICK.analogRead(AIN1);
  else if(type_get == _Joystick_Y) _joystick_read_value = KULBOT_JOYSTICK.analogRead(AIN2);
  I2CMux.closeChannel(this->_selection_sensor_port(port));
  return _joystick_read_value;
}
//======================================================================================
/*void KULBOT::KULBOT_SD_CARD_INIT()
{
  SD.begin(SD_CARD_CS);
}
void KULBOT::KULBOT_SD_CARD_WRITE(fs::FS &fs, const char * path, const char * data)
{
 // Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file){
   // Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(data)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
void KULBOT::KULBOT_SD_CARD_READ(fs::FS &fs, const char * path)
{
  //Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if(!file){
    // Serial.println("Failed to open file for reading");
    return;
  }
  // Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}
void KULBOT::KULBOT_SD_CARD_APPEN(fs::FS &fs, const char * path, const char * data)
{
  // Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(data)){
      // Serial.println("Message appended");
  } else {
    // Serial.println("Append failed");
  }
  file.close();
}
void KULBOT::KULBOT_SD_CARD_DELETE(fs::FS &fs, const char * path)
{
    // Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    // Serial.println("File deleted");
  } else {
    // Serial.println("Delete failed");
  }
}*/
//======================================================================================

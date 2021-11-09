
#define debug 1
///////////////////// Steering Servo Control /////////////////////
#define RC_SERVO_PIN 8
#define NEURAL_ANGLE 90
#define LEFT_STEER_ANGLE  -30
#define RIGHT_STEER_ANGLE  30

#include <Servo.h>
Servo   Steeringservo;
int Steering_Angle = NEURAL_ANGLE;

void steering_control()
{
  if(Steering_Angle<= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  Steeringservo.write(Steering_Angle);  
}

///////////////////// Encoder /////////////////////

#include <SPI.h>

#define ENC1_ADD 41

signed long encoderPos= 0;

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 
}

unsigned int count_1, count_2, count_3, count_4;
void readEncoder(int encoder_no) {
  // Initialize temporary variables for SPI read
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  encoderPos = ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8) + (long)count_4;
  
  //return count_value;
}

void clearEncoderCount(int encoder_no) {
    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}

///////////////////// Steering Servo Control /////////////////////

/////////////////////// DC Motor Control /////////////////////

unsigned long lastTime,now;
double input = 0;

#define MOTOR_PWM 5 
#define MOTOR_DIR 4

int Motor_Speed = 0;

void motor_control(int dir, int motor_pwm)
{
  
  if(dir == 1) // forward
  { 
    digitalWrite(MOTOR_DIR,LOW);
    analogWrite(MOTOR_PWM,motor_pwm);    
  }
  else if(dir == -1) // backward
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM,motor_pwm);
  }
  else // stop
  {
    analogWrite(MOTOR_PWM,0);
  }
}

/////////////////////// DC Motor Control /////////////////////

////////////////////// Sonar Sensor Setup ////////////////////
#include <NewPing.h>
#define TRIG 9
#define ECHO 10
#define Sonar_MAX_Distance 150
NewPing sonar(9,10,Sonar_MAX_Distance);
int sonar_d = 0;

void sonar_sensor_read(void)
{
  sonar_d=sonar.ping_cm();
  if(sonar_d == 0) sonar_d = Sonar_MAX_Distance;  
}

////////////////////// Sonar Sensor Setup ////////////////////

/////////////////////////// I2C 통신 //////////////////////////

#include <Wire.h>
int sensor_flag = 0;

void receiveEvent(int howMany)
{
  unsigned char a[7];  // '#' +   '*'
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();
  a[3] = Wire.read();
  a[4] = Wire.read();
  a[5] = Wire.read();
  a[6] = Wire.read();

  
   
  if( (a[0]=='#') && (a[1] == 'C') && (a[6] == '*'))
  {
    Steering_Angle =  a[2]*256 + a[3];
       
    if( ((a[4]&0x80) >>7)  ==1 )  //check MSB bit is 1 -> negative
    {     
      Motor_Speed = ((255- a[4])+(256-a[5]))*-1;           
    }
    else 
    {
      Motor_Speed    =  a[4]*256 + a[5];
    }
    
    steering_control();  
     
    if(Motor_Speed> 0)      motor_control(1,Motor_Speed);
    else if(Motor_Speed< 0) motor_control(-1,-Motor_Speed);
   
    else motor_control(0,0);
  }

  if( (a[0]=='#') && (a[1] == 'S') && (a[6] == '*'))
  { 
    sensor_flag = 1;
  }
}
/////////////////////////// I2C 통신 //////////////////////////

void requestEvent()
{
  unsigned char s[8] = {0,};
  int temp;
  temp = sonar_d*10;
  s[0]='#';
  s[1]= (temp&0xff00)>>8; // sonar
  s[2]= (temp&0x00ff); // sonar
  // s[3]= (encoderPos&0xff000000)>>24;    // encoder MSB 8bit
  // s[4]= (encoderPos&0x00ff0000)>>16;
  // s[5]= (encoderPos&0x0000ff00)>>8;
  // s[6]= (encoderPos&0x000000ff);        // encoder LSB 8bit
  s[3]= (signed long)count_1<<24;    // encoder MSB 8bit
  s[4]= (signed long)count_2<<16;
  s[5]= (signed long)count_3<<8;
  s[6]= (signed long)count_4;        // encoder LSB 8bit
  s[7]='*'; 
  
  Wire.write(s,8); // respond 
  sensor_flag = 0;
}

void setup()
{
  Wire.begin(5);    // I2C bus  #5
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
  initEncoders();       Serial.println("Encoders Initialized...");  
  clearEncoderCount(1);  Serial.println("Encoder[1] Cleared...");
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(Steering_Angle);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(115200);
  delay(200);
}

void loop()
{

  now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=200)
  {
      lastTime=now;
      readEncoder(1);
  }
  sonar_sensor_read();
  if(debug == 1)
  {   
    ///////////// Steering Servo  ////////////// 
    Serial.print("Sonar : ");
    Serial.print(sonar_d);
    Serial.print("cm");    

    ///////////// Steering Servo  /////////////
    Serial.print("  Steering Angle : ");
    Serial.print(Steering_Angle);

    ////////////// encoder  ///////////////////
    Serial.print("  Encoder Pos : ");       
    Serial.print(encoderPos);
    
    /////////////// Motor PWM  ///////////////// 
    Serial.print("  Motor PWM : ");
    Serial.println(Motor_Speed);
  }
  
}

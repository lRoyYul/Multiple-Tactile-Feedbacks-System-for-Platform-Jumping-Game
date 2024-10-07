#include <math.h>
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int dirPin2 = 9; // 2nd direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor *NOT USED*
int fsrPin = A3; // input pin for FSR sensor *NOT USED*
int encPin1 = 2; // encoder read pin 1
int encPin2 = 3; // encoder read pin 2
int motorPin = 12;

double rs = 0.073152;   //[m]
double rp = 0.004191;
int updatedPos = 0;     // keeps track of the encoder position
volatile int lastRawPos = 0; // 'volatile' as it is modified in an interrupt
int EncLookup[16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};
//friction parameter
double asphaltResistance = 0.8;  
double gravelResistance = 1.2;   
double grassResistance = 1.5;  
double currentResistance = asphaltResistance;
// Kinematics variables
double xh = 0;           // Position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;

// Force output variables
double force = 0;           			// Force at the handle
double Tp = 0;              			// Torque of the motor pulley
double duty = 0;            			// Duty cylce (between 0 and 255)
unsigned int output = 0;    			// Output command to the motor

double Texture_Spatial_Frequency = 200; // Spatial frequency of texture rendering
double Texture_Damping = 0.1;             // Damping for texture rendering
double ax=0;
String Udata;
String Fdata;
unsigned long startTime;
const unsigned long timeout = 1000;
unsigned long lastDataTime = 0;

double x_wall=0;
double k_wall = 0;
void readEncoder() {
  int newRawPos = digitalRead(encPin1) * 2 + digitalRead(encPin2);
  int encInc = EncLookup[lastRawPos * 4 + newRawPos];
  updatedPos += encInc;
  lastRawPos = newRawPos;
}
void setup() 
{
  // Set up serial communication
  
  Serial.begin(115200);
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input
  pinMode(encPin1,INPUT);       // set encoder pin 1 to be an input
  pinMode(encPin2,INPUT);       // set encoder pin 2 to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  pinMode(dirPin2,OUTPUT);   // second dir pin for motor A
  pinMode(motorPin, OUTPUT);
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  digitalWrite(dirPin2,LOW);
  
  // Initialize position valiables
  lastRawPos = digitalRead (encPin1) * 2 + digitalRead (encPin2);

  attachInterrupt(digitalPinToInterrupt(encPin1), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPin2), readEncoder, CHANGE);
  Serial.setTimeout(100);
}


int adjust_kwall(int kind_cube){
  int current_kwall;
  if (kind_cube == 2)
  {
    current_kwall = 200;
  }
  else if (kind_cube == 0)
  {
    current_kwall = 500;
  }
  else
  {
    current_kwall = 1500;
  }
  return current_kwall;
}
void loop()
{
//  curr_time = millis()/64.0;
if (Serial.available() > 0) {
    Udata = Serial.readStringUntil('\n'); // 读取串行数据直到换行符
    Fdata = Serial.readStringUntil('\n');
    // Fdata = Serial.readStringUntil('\n');
    // Serial.println("Received: " + Fdata); // 回显接收到的数据，用于调试
    // 根据接收到的数据执行相应的操作
  }
  double theta_s = 0.75*(rp/rs)*updatedPos; 
  double rh = 0.075;   //[m] 
  bool Vib = false;
  xh = rh*(theta_s * 3.14159 / 180);
  dxh = (double)(xh - xh_prev) / 0.001;
  ax=(double)(dxh-dxh_prev)/0.001;
  xh_prev = xh;
  dxh_prev = dxh;
  Tp = rp/rs * rh * force; //
    if (xh > x_wall)
    {
        // Serial.println("kwall:"+ String(k_wall));
        k_wall = adjust_kwall(Udata.toInt());
        force = -k_wall * (xh - x_wall);
        //Serial.println("--------------into");
    }
    else
    {
        force = 0;
    }


  if(force > 0) { 
    digitalWrite(dirPin, LOW);
    digitalWrite(dirPin2,HIGH);
  } else {
    digitalWrite(dirPin, HIGH);
    digitalWrite(dirPin2,LOW);
  }
  double duty = sqrt(abs(Tp)/0.03);
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  int output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output); 
  // digitalWrite(motorPin, HIGH);
  // delay(3000);
  // digitalWrite(motorPin, LOW);
  // delay(3000); // 振动1秒
  if (Fdata.toInt())
  {
  // startTime = millis();
    // Serial.println("Received: " + Fdata);
    digitalWrite(motorPin, HIGH);
    delay(3000);
    digitalWrite(motorPin, LOW);}
  //   if (millis() - lastDataTime > timeout) {
  //   // 如果超过了超时时间没有接收到新的数据
  //   Fdata = "0";  // 可能Unity3D已经结束，重置Fdata
  // }
  // }
  
  
  
  // if(ax>50)
  // {
  //   Serial.println(-1);
  // }
  // else
  // {
  //   Serial.println(force, 4); // 输出当前手柄的位置，精确到小数点后四位
  // }
   
  Serial.println(force, 4);      
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
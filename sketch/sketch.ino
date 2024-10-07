#include <math.h>
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int dirPin2 = 9; // 2nd direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor *NOT USED*
int fsrPin = A3; // input pin for FSR sensor *NOT USED*
int encPin1 = 2; // encoder read pin 1
int encPin2 = 3; // encoder read pin 2
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
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;
double mu;
// Force output variables
double force = 0;           			// Force at the handle
double Tp = 0;              			// Torque of the motor pulley
double duty = 0;            			// Duty cylce (between 0 and 255)
unsigned int output = 0;    			// Output command to the motor

double Texture_Spatial_Frequency = 200; // Spatial frequency of texture rendering
double Texture_Damping = 0.1;             // Damping for texture rendering

String Udata;
String Fdata;
double adjustMotorResistance(int texture_kind,double dxh_filt, float fdata) {
    double force1;
    //Serial.println("-------------------------"+texture_kind);
    if (texture_kind == 2) 
    {
        Texture_Spatial_Frequency = 50;
        Texture_Damping = 0.1;
        mu = 0.1;
    } 
    else if (texture_kind == 0) 
    {
        Texture_Spatial_Frequency = 300;
        Texture_Damping = 4;
        mu = 1;
    } 
    else 
    {
        Texture_Spatial_Frequency = 1000;
        Texture_Damping =12;
        mu = 10;
    }
    // Serial.print(currentResistance);
    // Serial.print("--");
    if (cos(2*M_PI*Texture_Spatial_Frequency*xh)> 0)
    {
        force1 = -Texture_Damping * dxh_filt - mu * fdata;
    }
    else
    {
        force1 = 0;
    }
    return force1;
}

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


void loop()
{
//  curr_time = millis()/64.0;
 
if (Serial.available() > 0) {
    Udata = Serial.readStringUntil('\n'); // 读取串行数据直到换行符
    Fdata = Serial.readStringUntil('\n');
    // Pdata = Serial.readStringUntil('\n');
    //Serial.println("Received: " + Fdata); // 回显接收到的数据，用于调试
    // 根据接收到的数据执行相应的操作
  }
  double theta_s = 0.75*(rp/rs)*updatedPos; 
  double rh = 0.075;   //[m] 
  xh = rh*(theta_s * 3.14159 / 180);
  dxh = (double)(xh - xh_prev) / 0.001;
  dxh_filt = 0.9*dxh + 0.1*dxh_prev; 
  xh_prev2 = xh_prev;
  xh_prev = xh;
  dxh_prev = dxh;
  Tp = rp/rs * rh * force; //
  force=adjustMotorResistance(Udata.toInt(),dxh_filt, Fdata.toFloat()); 
  // force=adjustMotorResistance(1,dxh_filt, Fdata.toFloat()); 
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
  
  Serial.println(theta_s, 4); // 输出当前手柄的位置，精确到小数点后四位
  // Serial.println("hahahahha");
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

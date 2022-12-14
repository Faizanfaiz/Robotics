#include "stm32yyxx_ll_adc.h"
#include <SimpleFOC.h>

#define Pos 2
#define Tor 3 
#define Spe 4
#define Tap 7 

#define TP PA1
#define TG PA0
#define VP PB1
#define VG PA4
#define BP PB12
#define BG PB11

#define TCV PA7
#define VIGCV PB0
#define BOICV PA5 


#define Gate PC10

float Targetgain, Targetplus, TargetCV, Vigourgain, VigourCV, Vigourplus, Boinggain, BoingCV, Boingplus;

float Positionangle, PositionVigour, PositionBoing, Torqueangle, TorqueVigour, TorqueBoing, Speedangle, SpeedVigour, SpeedBoing;

float TGP, TGCV, TGG, VIP, VICV, VIG, BOP, BOCV, BOG;

float pvig, pbo, tvig, tbo, svig, sbo;

int var, val, ang, torang, sang;

float pi = 3.141592653589793238;
float rada, Trada, Srada;
int initial = 0;
int Checkstate = 0;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5048_I2C);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimitVolt(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doLimitVelocity(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

void setup() {

  analogReadResolution(12);
  pinMode(Pos, INPUT);
  pinMode(Tor, INPUT);
  pinMode(Spe, INPUT);
  pinMode(Tap, INPUT);
  pinMode(Gate, INPUT);
  
  sensor.init();
  
  driver.voltage_power_supply = 12;
  driver.init();
  
  motor.linkDriver(&driver);
  

  motor.velocity_limit = 8;
  // default voltage_power_supply
  motor.voltage_limit = 12;
  
  motor.controller = MotionControlType::angle;
  // init motor hardware
  motor.init();

  motor.linkSensor(&sensor);
  motor.initFOC();
  
  // add target command T
  command.add('T', doTarget, "target angle");
  command.add('L', doLimitVolt, "voltage limit");
  command.add('V', doLimitVelocity, "velocity limit");


  Serial.begin(115200);
  motor.move(initial);
  Serial.println("Motor ready!");
  Serial.println("Sensor ready!");
  
  Serial.println("Set target position [rad]");
  _delay(1000);
}


void loop()
{
  
    checkcasses();
}



void checkcasses()
{
  
if(digitalRead(Pos)==HIGH && digitalRead(Tor)==LOW && digitalRead(Spe)==LOW && digitalRead(Tap)==LOW)
{
  var=1;
}

if(digitalRead(Pos)==LOW && digitalRead(Tor)==HIGH && digitalRead(Spe)==LOW && digitalRead(Tap)==LOW)
{
  var=2;
}

if(digitalRead(Pos)==LOW && digitalRead(Tor)==LOW && digitalRead(Spe)==HIGH && digitalRead(Tap)==LOW)
{
  var=3;
}

if(digitalRead(Pos)==LOW && digitalRead(Tor)==LOW && digitalRead(Spe)==LOW && digitalRead(Tap)==HIGH)
{
  var=4;
}

switch (var) {
  case 1:
 Position();

    break;
  case 2:
 Torque();
    break;
    case 3:
 Speed();
    break;
    case 4:
 TapFunction();
    break;
  default:
    break;
}
  
  motor.loopFOC(); 
  motor.move();
}


void Position()
{
 /* if(Checkstate == 0)
  {
  sensor.init();
  
  driver.voltage_power_supply = 12;
  driver.init();
  
  motor.linkDriver(&driver);
  

  motor.velocity_limit = 8;
  // default voltage_power_supply
  motor.voltage_limit = 12;
  
  motor.controller = MotionControlType::angle;
  // init motor hardware
  motor.init();

  motor.linkSensor(&sensor);
  motor.initFOC();
  
  // add target command T
  command.add('T', doTarget, "target angle");
  command.add('L', doLimitVolt, "voltage limit");
  command.add('V', doLimitVelocity, "velocity limit");

  motor.move(initial);
  Serial.println("Motor ready!");
  Serial.println("Sensor ready!");
  
  Serial.println("Set target position [rad]");
  _delay(1000);
  Checkstate = 1;
  }
  */
  if(digitalRead(Gate)!=1)
  {
  motor.controller = MotionControlType::angle;
  
  readcal();

  Positionangle = TGP + (TGCV * TGG);
  ang = map(Positionangle, 0, 190,0 , 360);

  PositionVigour = VIP + (VICV * VIG);
  pvig = map(PositionVigour, 0, 190,0 , 20);

  PositionBoing = BOP + (BOCV * BOG);
  pbo = fmap(PositionBoing, 0, 190, 0.0 , 1.1);
  motor.P_angle.P = pvig;
  motor.PID_velocity.P = pbo;  
  
  motor.loopFOC(); 
  rada = ang*pi/180;
  motor.move(rada);
}
}



void Torque()
{
  if(digitalRead(Gate)!=1)
  {
    
  motor.controller = MotionControlType::angle_openloop;
  
  readcal();

  Torqueangle = TGP + (TGCV * TGG);
  torang = map(Torqueangle, 0, 190,0, 360);

  TorqueVigour = VIP + (VICV * VIG);
  tvig = map(TorqueVigour, 0, 190,0 , 8);

  TorqueBoing = BOP + (BOCV * BOG);
  tbo = fmap(TorqueBoing, 0, 190, 0.0 , 12);
  
  motor.velocity_limit = tvig;
  motor.voltage_limit = tbo; 

  motor.loopFOC(); 
  Trada = torang*pi/180;
  motor.move(Trada);
  
  
}

}


void Speed()
{

  Checkstate = 0;
  
  if(digitalRead(Gate)!=1)
  {
    
  motor.controller = MotionControlType::velocity_openloop;
  
  readcal();

  Speedangle = TGP + (TGCV * TGG);
  sang = map(Speedangle, 0, 190,-720, 720);

  SpeedVigour = VIP + (VICV * VIG);
  svig = map(SpeedVigour, 0, 190,0 , 12);

  SpeedBoing = BOP + (BOCV * BOG);
  sbo = fmap(SpeedBoing, 0, 190, 0.0 , 2);

  motor.voltage_limit = svig;
  motor.current_limit = sbo; 
  
  Serial.print("Tboing: ");
  Serial.print(sang);
  Serial.print("     TVig: ");
  Serial.print(svig);
  Serial.println();

  motor.loopFOC(); 
  Srada = sang*pi/180;

  motor.target= Srada;
  
}

}

void TapFunction()
{
  motor.controller = MotionControlType::angle;
  
  readcal();

  Positionangle = TGP + (TGCV * TGG);
  ang = map(Positionangle, 0, 190,0 , 360);

  PositionVigour = VIP + (VICV * VIG);
  pvig = map(PositionVigour, 0, 190,0 , 20);

  PositionBoing = BOP + (BOCV * BOG);
  pbo = fmap(PositionBoing, 0, 190, 0.0 , 1.1);
  motor.P_angle.P = pvig;
  motor.PID_velocity.P = pbo;  
  
  motor.loopFOC(); 
  rada = ang*pi/180;
  motor.move(0);

  if(digitalRead(Gate)==1)
  {
    motor.move(rada);
  }
}


void readcal()
{
  Targetplus = analogRead(TP);
  TargetCV = analogRead(TCV);
  Targetgain = analogRead(TG);

  Vigourplus = analogRead(VP);
  VigourCV = analogRead(VIGCV);
  Vigourgain = analogRead(VG);
  
  Boingplus = analogRead(BP);
  BoingCV = analogRead(BOICV);
  Boinggain = analogRead(BG);
  
    TGP = map( Targetplus, 4095, 0, 0, 100);
      
    TGCV = map( TargetCV, 0, 4095, 0, 10);
 
    TGG = map( Targetgain, 4095, 0, 0, 10);

    VIP = map( Vigourplus, 4095, 0, 0, 100);
      
    VICV = map( VigourCV, 0, 4095, 0, 10);
 
    VIG = map( Vigourgain, 4095, 0, 0, 10);

    BOP = map( Boingplus, 4095, 0, 0, 100);
      
    BOCV = map( BoingCV, 0, 4095, 0, 10);
 
    BOG = map( Boinggain, 4095, 0, 0, 10);
    
}

float fmap(float x, float a, float b, float c, float d)
{
      float f=x/(b-a)*(d-c)+c;
      return f;
}

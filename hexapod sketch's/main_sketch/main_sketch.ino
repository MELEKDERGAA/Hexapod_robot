#include<Servo.h>
#include<Wire.h>
#include<Adafruit_PWMServoDriver.h>
#include<math.h>
#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>
Adafruit_PWMServoDriver PWM=Adafruit_PWMServoDriver();//default 0x40//
Adafruit_PWMServoDriver PWM1=Adafruit_PWMServoDriver(0x41);
#define ServoMin 96//min of pulse length of MG995
#define ServoMax 470//max of pulse length of MG995
#define bodysidelen 14.1
#define bodyuplen 14
#define coxalen 63.38
#define femurlen 77.20
#define tibialen 0
float PosX,PosY,PosZ,RotX,RotY,RotZ;
char msg[10];
float bodyoffset[2][6]={
                      /*for the x-axe*/{
                                         /*leg_1*/bodysidelen-3.8,
                                         /*leg_2*/bodysidelen,
                                         /*leg_3*/bodysidelen-3.8,
                                         /*leg_4*/-bodysidelen-3.8,
                                         /*leg_5*/-bodysidelen,
                                         /*leg_6*/-bodysidelen-3.8
                                        },
                      /*for the y-axe*/{
                                         /*leg_1*/bodyuplen,
                                         /*leg_2*/0,
                                         /*leg_3*/-bodyuplen,
                                         /*leg_4*/-bodyuplen, 
                                         /*leg_5*/0,
                                         /*leg_6*/bodyuplen,     
                                        }
                     };
float legposition[6][3]{
                            //for x-axis               //for z-axis       //for y-axis
                  {cos(60/180*PI)*(coxalen + femurlen)   ,tibialen,    sin(60/180*PI)*(coxalen + femurlen)},
                  {coxalen+femurlen                      ,tibialen,                                      0},
                  {cos(60/180*PI)*(coxalen + femurlen)   ,tibialen,    sin(-60/180*PI)*(coxalen + femurlen)},
                  {-cos(60/180*PI)*(coxalen + femurlen)  ,tibialen,    sin(-60/180*PI)*(coxalen + femurlen)},
                  {-(coxalen+femurlen)                   ,tibialen,                                       0},
                  {-cos(60/180*PI)*(coxalen + femurlen)  ,tibialen,    sin(60/180*PI)*(coxalen + femurlen)}
                };
RF24 radio(0,0); // CE, CSN pins
const byte address[6] = "00001";

void setup() {
  PWM.begin();
  PWM1.begin();
  PWM.setPWMFreq(50);
  PWM1.setPWMFreq(50);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if (radio.available()) {
    String message;
    radio.read(&message, sizeof(message));
  }
 /* while (msg=="walk"){
    while(msg2=="forward"){
      movement(hexalegs);    
    }
    while(msg2=="rotation"){
      //rotate(hexlegs);
    }
  }*/
}

void ikcalc(float coxa,float femur,float tibia,int i){
  //BodyIK
float TotalY=	legposition[i][3]+ bodyoffset[2][i] + PosY;
float TotalX= legposition[i][1] + bodyoffset[1][i] + PosX;
float DistBodyCenterFeet=	sqrt(TotalY + TotalX);
float AngleBodyCenterX= PI/2 - atan2(TotalY, TotalX);
float RollZ=tan(RotZ * PI/180) * TotalX;
float PitchZ=	tan(RotX * PI/180) * TotalY;
float BodyIKX=	cos(AngleBodyCenterX+ (RotY * PI/180)) * DistBodyCenterFeet - TotalX;
float BodyIKY=(sin(AngleBodyCenterX+ (RotY  * PI/180)) * DistBodyCenterFeet) - TotalY;
float BodyIKZ=RollZ + PitchZ;
  //LegIK
float NewPosX=	legposition[i][1] + PosX +  BodyIKX;
float NewPosZ=legposition[i][2] + PosZ + BodyIKZ;
float NewPosY=legposition[i][3] + PosY + BodyIKY;
float CoxaFeetDist=	sqrt(pow(NewPosX,2)   + pow(NewPosY,2));
float IKSW=	sqrt(pow((CoxaFeetDist - coxalen ) ,2) +pow( NewPosZ,2));
float IKA1=	atan((CoxaFeetDist - coxalen)/NewPosZ);
float IKA2=	acos((pow(tibialen,2) - pow(femurlen,2) - pow(IKSW,2))/(-2 * IKSW * femurlen));
float TAngle=	acos((pow(IKSW,2) - pow(tibialen,2) - pow(femurlen,2))/(-2 * femurlen * tibialen));
float IKTibiaAngle=	90 - TAngle * 180/PI;
float IKFemurAngle=	90 - (IKA1 + IKA2) * 180/PI;
float IKCoxaAngle= 90 - atan2(NewPosY, NewPosX) * 180/PI;
switch(i){
  case 1:coxa=IKCoxaAngle - 60;
        femur=IKFemurAngle;
        tibia=IKTibiaAngle;break;
  case 2:coxa=IKCoxaAngle;
         femur=IKFemurAngle;
         tibia=IKTibiaAngle;break;
  case 3:coxa=IKCoxaAngle + 60;
        femur=IKFemurAngle;
        tibia=IKTibiaAngle;break;
  case 4:coxa=IKCoxaAngle - 240;
        femur=IKFemurAngle;
        tibia=IKTibiaAngle;break;
  case 5:coxa=IKCoxaAngle - 180;
        femur=IKFemurAngle;
        tibia=IKTibiaAngle;break;
  case 6:coxa=IKCoxaAngle - 120;
        femur=IKFemurAngle;
        tibia=IKTibiaAngle;break;
}
}
void legforward(Adafruit_PWMServoDriver servo,Adafruit_PWMServoDriver servo1,int LegNumber){
  float coxa,femur,tibia;
  float final_PosX,final_PosY,final_PosZ;
  final_PosX=PosX;
  final_PosY=PosY;
  final_PosZ=PosZ;
  PosX= (PosX!=0)? PosX/2:0;
  PosY= (PosY!=0)? PosY/2:0;
  PosZ= (PosZ!=0)? PosZ/2:0;
  ikcalc(coxa,femur,tibia,LegNumber);
  switch (LegNumber){
    case 1:servo.setPWM(2,0,map(femur,0,180,ServoMin,ServoMax));
           servo.setPWM(3,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo.setPWM(1,0,map(coxa,0,180,ServoMin,ServoMax));
           servo.setPWM(4,0,map(90,0,180,ServoMin,ServoMax));
           PosX=final_PosX;
           PosY=final_PosY;
           PosZ=final_PosZ;
           ikcalc(coxa,femur,tibia,LegNumber);
           servo.setPWM(2,0,map(femur,0,180,ServoMin,ServoMax));
           servo.setPWM(3,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo.setPWM(1,0,map(coxa,0,180,ServoMin,ServoMax));
           break;
    case 2:servo.setPWM(5,0,map(femur,0,180,ServoMin,ServoMax));
           servo.setPWM(6,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo.setPWM(4,0,map(coxa,0,180,ServoMin,ServoMax));
           servo.setPWM(7,0,map(90,0,180,ServoMin,ServoMax));
           PosX=final_PosX;
           PosY=final_PosY;
           PosZ=final_PosZ;
           ikcalc(coxa,femur,tibia,LegNumber);
           servo.setPWM(5,0,map(femur,0,180,ServoMin,ServoMax));
           servo.setPWM(6,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo.setPWM(4,0,map(coxa,0,180,ServoMin,ServoMax));
           break;
    case 3:servo.setPWM(8,0,map(femur,0,180,ServoMin,ServoMax));
           servo.setPWM(9,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo.setPWM(7,0,map(coxa,0,180,ServoMin,ServoMax));
           servo1.setPWM(1,0,map(90,0,180,ServoMin,ServoMax));
           PosX=final_PosX;
           PosY=final_PosY;
           PosZ=final_PosZ;
           ikcalc(coxa,femur,tibia,LegNumber);
           servo.setPWM(8,0,map(femur,0,180,ServoMin,ServoMax));
           servo.setPWM(9,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo.setPWM(7,0,map(coxa,0,180,ServoMin,ServoMax));
           break;
    case 4:servo1.setPWM(2,0,map(femur,0,180,ServoMin,ServoMax));
           servo1.setPWM(3,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo1.setPWM(1,0,map(coxa,0,180,ServoMin,ServoMax));
           servo1.setPWM(4,0,map(90,0,180,ServoMin,ServoMax));
           PosX=final_PosX;
           PosY=final_PosY;
           PosZ=final_PosZ;
           ikcalc(coxa,femur,tibia,LegNumber);
           servo1.setPWM(2,0,map(femur,0,180,ServoMin,ServoMax));
           servo1.setPWM(3,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo1.setPWM(1,0,map(coxa,0,180,ServoMin,ServoMax));
           break;
    case 5:servo1.setPWM(5,0,map(femur,0,180,ServoMin,ServoMax));
           servo1.setPWM(6,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo1.setPWM(4,0,map(coxa,0,180,ServoMin,ServoMax));
           servo1.setPWM(7,0,map(90,0,180,ServoMin,ServoMax));
           PosX=final_PosX;
           PosY=final_PosY;
           PosZ=final_PosZ;
           ikcalc(coxa,femur,tibia,LegNumber);
           servo1.setPWM(5,0,map(femur,0,180,ServoMin,ServoMax));
           servo1.setPWM(6,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo1.setPWM(4,0,map(coxa,0,180,ServoMin,ServoMax));
           break;
    case 6:servo1.setPWM(8,0,map(femur,0,180,ServoMin,ServoMax));
           servo1.setPWM(9,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo1.setPWM(7,0,map(coxa,0,180,ServoMin,ServoMax));
           servo.setPWM(1,0,map(90,0,180,ServoMin,ServoMax));
           PosX=final_PosX;
           PosY=final_PosY;
           PosZ=final_PosZ;
           ikcalc(coxa,femur,tibia,LegNumber);
           servo1.setPWM(8,0,map(femur,0,180,ServoMin,ServoMax));
           servo1.setPWM(9,0,map(tibia,0,180,ServoMin,ServoMax));
           delay(500);
           servo1.setPWM(7,0,map(coxa,0,180,ServoMin,ServoMax));
           break;

  }}
  void movement(){
    //step one of movement
    legforward(PWM,PWM1,1);
    legforward(PWM,PWM1,3);
    legforward(PWM,PWM1,5);
    //step two of movement
    legforward(PWM,PWM1,2);
    legforward(PWM,PWM1,4);
    legforward(PWM,PWM1,6);
  }

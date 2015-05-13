
#include <PinChangeInt.h>
#define PI 3.14159
#include <Wire.h>
#include <HMC5883L.h>
//#include <MsTimer2.h>
#define periodo 0.2 //periodo de amostragem em segundos
HMC5883L compass;
#define limite 255
#define SP 1
#define Kp 10
#define Ki 0.1//0.972
//#define Kp 1.886
//#define Ki -1.052
//#define Kp 4.89
//#define Ki 1.141



int HallAp=3;//input do Hall A no pino 3
int HallBp=2;//input do Hall B no pino 2
int HallCp=4;//input do Hall C no pino 4

int Hall_Estado=1;//define o estado da posicao  
int HallA=0;// Estado do Hall A
int HallB=0;// Estado do Hall B
int HallC=0;// Estado do Hall C
int flagang=0;
int vb=0,vc=0,va=0;//Contador para verificar quantas vezes o Hall B ou C mudaram de estado
int ff=1;
int fe=0;
float omega=0.0;//velocidade angular do motor
float temp=0.0;
unsigned long timeB=0;//tempo em que hallB fica em um unico estado (180 graus)
unsigned long timeC=0;//tempo em que hallC fica em um unico estado (180 graus)
unsigned long TimeB=0;//tempo que HallB sobe
unsigned long TimeC=0;//tempo que HallB desce
unsigned long timeBant=0;
unsigned long timeCant=0;
unsigned long TimeH=0;
unsigned long TimeG=0;
unsigned long timeA=0;
unsigned long timeAant=0;
unsigned long TL;
int novoloop=1;
unsigned long TISR;
unsigned long TISR2;
int flagomg=0;
int EN1=7;
int EN2=6; // pinos dos enables das fases
int EN3=8;
int IN1=11;//input da fase A  
int IN2=10;//inpout da fase B
int IN3=9;//input da fase C
float Omg;
int pot=A0;//potenciometro para controle de velocidade no pino A0
int pot_val=0;//valor lido do potenciometro
int dir=0; //variavel para indicar direcao do motor
int flagA=0;
float deltaT;
float deltaT2;
float pwm=0.0;
int m;
int i;
float numStep=10;
int ramp=1;
int count=0;
float erro;
float yi_0=0;
float yi_1=0;
float yp=0;
float soma=0;
int qtd=0;
float angl;
float ang;
  float vect[10];
void setup()

{
  Serial.begin(9600);
  pinMode(HallAp,INPUT);//   
  pinMode(HallBp,INPUT);//   Sensores Halls como entradas
  pinMode(HallCp,INPUT); //  

  pinMode(IN1,OUTPUT);//
  pinMode(IN2,OUTPUT);//
  pinMode(IN3,OUTPUT);//     Entradas do driver como saidas

  pinMode(EN1,OUTPUT);//
  pinMode(EN2,OUTPUT);//   Enables das fases como saidas
  pinMode(EN3,OUTPUT);//

  // Aumentar a frequencia do pwm nos pinos 9,10 e 11 (IN1,IN2,IN3)
  // SREG|=B10000000;

  /* Set PWM frequency on pins 9,10, and 11
   // this bit of code comes from 
   http://usethearduino.blogspot.com/2008/11/changing-pwm-frequency-on-arduino.html
   */
  // Set PWM for pins 9,10 to 32 kHz
  //First clear all three prescaler bits:
  int prescalerVal = 0x07; //create a variable called prescalerVal and set it equal to the binary number "00000111"                                                                                                     
  TCCR1B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"
  //  //Now set the appropriate prescaler bits:
  int prescalerVal2 = 1; //set prescalerVal equal to binary number "00000001"
  TCCR1B =prescalerVal2 ; //OR the value in TCCR0B with binary number "00000001"  
  //  // Set PWM for pins 3,11 to 32 kHz (Only pin 11 is used in this program)
  //  //First clear all three prescaler bits:
  TCCR2B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"
  //  //Now set the appropriate prescaler bits: 

  TCCR2B =prescalerVal2 ; //OR the value in TCCR0B with binary number "00000001"
  // 

  PCintPort::attachInterrupt(2, AtualizaB, CHANGE);
  //PCintPort::attachInterrupt(3, AtualizaA, CHANGE);
  PCintPort::attachInterrupt(4, AtualizaC, CHANGE);


  Wire.begin();
  compass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L
  Hall_init();
  //MsTimer2::set(500, controleISR); // 500ms period
  //MsTimer2::start();

  TIMSK2=0x01;

}


ISR(TIMER2_OVF_vect)
{

  count++;
  if(count>=31370*periodo){
   
    flagomg=1;
    //controle
    erro=SP-angl;
//    if(erro<0) dir=1;
//    if(erro>=0) dir=-1;
    yp=Kp*erro;
    yi_0=Ki*erro+yi_1;
    pwm=(yp+yi_0);
    if(pwm>=0){
      pwm=pwm+2;
      dir=-1;
     }
    if(pwm<0){
    pwm=pwm-2;
    dir=1;
  }
  pwm=abs(pwm*255.0/12.0);
    if(pwm>=limite)
      pwm=limite;
//    if(pwm<=2.0*255.0/12.0)
//      pwm=2.0*255.0/12.0;
    yi_1=yi_0;
    count=0;
    move(pwm,dir,Hall_Estado);
  }
}

void Hall_init()
{
  if( digitalRead(HallAp)== HIGH) HallA=1;
  else HallA=0;

  if( digitalRead(HallBp)== HIGH)  HallB=1;
  else  HallB=0;

  if( digitalRead(HallCp)== HIGH) HallC=1;
  else HallC=0;

  Hall_Estado=(HallA*1 + HallB*2 + HallC*4);
}

void AtualizaA()
{
  m++;
  if(flagomg==1)
  {
    TISR2=micros();
    deltaT2=TISR2-TISR;
    Omg=(m*PI/8)/(deltaT2/1000000);
    TISR=micros();
    flagomg=0;
    m=0;
  }
  //  timeA=micros();
  //  deltaT=timeA-timeAant;
  //  if ((timeA !=0)&& (va>=2))
  //    omega=(PI/4)/(deltaT/1000000);

  if(va<2)
    va++;


  if( digitalRead(HallAp)== HIGH) HallA=1;
  else HallA=0;

  Hall_Estado=(HallA*1 + HallB*2 + HallC*4);
  //if(!vb && !vc && !va){
  //for(i=1;i<numStep;i++)
  //move(pwm*(i/numStep),dir,Hall_Estado);}
  //else
  move(pwm,dir,Hall_Estado);
  //timeAant=timeA; 

}

void AtualizaB()// Rotina de interrupcao para mudanca de estado de HallB
{ 
  m++;
  if(flagomg==1)
  {
    TISR2=micros();
    deltaT2=TISR2-TISR;
    Omg=(m*PI/8)/(deltaT2/1000000);
    TISR=micros();
    flagomg=0;
    m=0;
  }
  //  timeB=micros();
  //  deltaT=timeB-timeBant;
  //
  //  if ((timeB !=0)&& (vb>=2))
  //    omega=(PI/4)/(deltaT/1000000);

  if(vb<2) 
    vb++;

  if( digitalRead(HallBp)== HIGH) {
    HallB=1;

    TimeB=micros();
  }
  else {
    HallB=0;

    TimeC=micros();

  }
  Hall_Estado=(HallA*1 + HallB*2 + HallC*4);
  // 
  //if(!vb && !vc && !va){
  //for(i=1;i<numStep;i++)
  //move(pwm*(i/numStep),dir,Hall_Estado);}
  //else
  move(pwm,dir,Hall_Estado);

  // timeBant=timeB;

}

void AtualizaC()
{
  m++;
  if(flagomg==1)
  {
    TISR2=micros();
    deltaT2=TISR2-TISR;
    Omg=(m*PI/8)/(deltaT2/1000000);
    TISR=micros();
    flagomg=0;
    m=0;
  }
  //  timeC=micros();
  //  deltaT=timeC-timeCant;
  //  if ((timeC !=0)&& (vc>=2))
  //    omega=(PI/4)/(deltaT/1000000);


  if(vc<2)
    vc++;


  if( digitalRead(HallCp)== HIGH){
    HallC=1;
    TimeH=micros();
  }
  else{
    HallC=0;
    TimeG=micros();
  }

  Hall_Estado=(HallA*1 + HallB*2 + HallC*4);

  //if(!vb && !vc && !va){
  //for(i=1;i<numStep;i++)
  //move(pwm*(i/numStep),dir,Hall_Estado);}
  //else
  move(pwm,dir,Hall_Estado);

  //timeCant=timeC;


}

float vaivolta(float x)
{
  if(ff)  pwm+=1;
  if(pwm==x) ff=0;
  if(!ff) pwm-=1;
  if(pwm<=0) pwm=0; 
  return pwm;
}

void loop() 
{

  //  if(novoloop){
  //    TL=millis();
  //    novoloop=0;
  //  }


  move(pwm,dir,Hall_Estado);

  //teste_modelo();


  SimulaA(dir);

  if(pwm==0 || Omg<1){
    vc=0;
    vb=0;
    va=0;
    //omega=0.01;
  }

  angl=angulo();
  Escreve();

}


float angulo ()
{

  
  ang=getHeading();
  if(ang>2*PI) ang-=2*PI;
  if(ang<-2*PI) ang+=2*PI;

  if(qtd<10){
    qtd++;
    vect[qtd-1]=ang;
  }
  
  if(qtd>=10){
    for(i=0;i<=8;i++)
      vect[i]=vect[i+1];
    vect[9]=ang;
  }
  
  for(i=0;i<=9;i++)
    soma+=vect[i];
    
  angl=soma/10.0;
  
//  Serial.print(qtd);
//  Serial.print(" ");
//  Serial.print(soma);
//  Serial.print(" ");
//  Serial.println(angl);
  soma=0.0;
  return angl;
}

void Escreve()
{

  Serial.print (millis()/1000.0,1);
  Serial.print(" ");
  Serial.print(12.0*pwm/255.0,1);
  //Serial.print(" ");
  //Serial.print(pwm);
  /* Serial.print("     Hall_E:");
   Serial.print(Hall_Estado); 
   Serial.print("     pwm:");
   Serial.print(pwm);
   Serial.print("     omega:");
   Serial.print(omega);*/
  //  Serial.print(" ");
  //  Serial.print(Omg,1);
  Serial.print(" ");
  Serial.println(angl,1);

}

void SimulaA(int dir)
{
  //simular estado 5
  if (HallB==1 && HallC==1)
    HallA=0;
  else
    //simular estado 2
    if (HallB==0 && HallC==0)
      HallA=1;
    else
      if(dir==1){
        //simular mudanca estado 3->4
        if(!HallA && HallB && !HallC)
          if(((micros()-TimeG)/1000000-((PI/12)/omega)) >= 0 )
          {
            HallA=0;
          }
          else
            //simular mudanca estado 6->1
            if(HallC && !HallA && !HallB)
              if(((micros()-TimeC)/1000000-((PI/12)/omega)) >= 0 )
              {
                HallA=1;
              }

      }
  if(dir==-1){
    if(HallA && HallB && !HallC)
      if(((micros()-TimeG)/1000000-((PI/12)/omega)) >= 0 )
      {
        HallA=1;
      }
      else
        //simular mudanca estado 6->1
        if(HallC && HallA && !HallB)
          if(((micros()-TimeH)/1000000-((PI/12)/omega)) >= 0 )
          {
            HallA=0;
          }

  }
  Hall_Estado=(HallA*1 + HallB*2 + HallC*4);
  //  if(!vb && !vc && !va){
  //for(i=1;i<numStep;i++)
  //move(pwm*(i/numStep),dir,Hall_Estado);}
  //else
  move(pwm,dir,Hall_Estado);
}

void move(int pwm, int dir,int Hall_Estado)//FUNCAO QUE CONTROLA AS ENTRADAS DO MOTOR E O FAZ GIRAR!!!
{

  switch (Hall_Estado)
  {
  case 1:
    switch (dir)
    {
    case 1:            

      digitalWrite(EN1,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN2,LOW); 



      digitalWrite(IN2,LOW);
      digitalWrite(IN3,LOW);
      analogWrite(IN1,pwm); 

      break;
    case -1:
      digitalWrite(EN1,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN2,LOW); 

      digitalWrite(IN1,LOW);
      digitalWrite(IN2,LOW);
      analogWrite(IN3,pwm);

      break;
    case 0:   
      digitalWrite(EN1,LOW); 
      digitalWrite(EN3,LOW); 
      digitalWrite(EN2,LOW); 
      digitalWrite(IN3,LOW);
      digitalWrite(IN2,LOW);
      digitalWrite(IN1,LOW);

      break;
    }
    break;
  case 2:
    switch (dir)
    {
    case 1:

      digitalWrite(EN1,HIGH); 
      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,LOW); 

      digitalWrite(IN3,LOW);
      digitalWrite(IN1,LOW);
      analogWrite(IN2,pwm); 

      break;
    case -1:

      digitalWrite(EN1,HIGH); 
      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,LOW); 

      digitalWrite(IN2,LOW);
      digitalWrite(IN3,LOW);
      analogWrite(IN1,pwm); 


      break;
    case 0:  
      digitalWrite(EN1,LOW); 
      digitalWrite(EN3,LOW); 
      digitalWrite(EN2,LOW);  
      digitalWrite(IN3,LOW);
      digitalWrite(IN2,LOW);
      digitalWrite(IN1,LOW);
      break;
    }
    break;
  case 3:
    switch (dir)
    {
    case 1:

      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN1,LOW); 

      digitalWrite(IN3,LOW);
      digitalWrite(IN1,LOW);
      analogWrite(IN2,pwm); 

      break;
    case -1:
      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN1,LOW); 

      digitalWrite(IN1,LOW);
      digitalWrite(IN2,LOW);
      analogWrite(IN3,pwm); 

      break;
    case 0:   
      digitalWrite(EN1,LOW); 
      digitalWrite(EN3,LOW); 
      digitalWrite(EN2,LOW); 
      digitalWrite(IN3,LOW);
      digitalWrite(IN2,LOW);
      digitalWrite(IN1,LOW); 
      break;
    }
    break;
  case 4:
    switch (dir)
    {
    case 1:
      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN1,LOW);

      digitalWrite(IN2,LOW);
      digitalWrite(IN1,LOW);
      analogWrite(IN3,pwm); 

      break;
    case -1:
      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN1,LOW);

      digitalWrite(IN1,LOW);
      digitalWrite(IN3,LOW);
      analogWrite(IN2,pwm); 

      break;
    case 0:   
      digitalWrite(EN1,LOW); 
      digitalWrite(EN3,LOW); 
      digitalWrite(EN2,LOW); 
      digitalWrite(IN3,LOW);
      digitalWrite(IN2,LOW);
      digitalWrite(IN1,LOW);
      break;
    }
    break;
  case 5:
    switch (dir)
    {
    case 1:

      digitalWrite(EN1,HIGH); 
      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,LOW); 

      digitalWrite(IN3,LOW);
      analogWrite(IN1,pwm);
      digitalWrite(IN2,LOW); 



      break;
    case -1:

      digitalWrite(EN1,HIGH); 
      digitalWrite(EN2,HIGH); 
      digitalWrite(EN3,LOW); 

      digitalWrite(IN1,LOW);
      digitalWrite(IN3,LOW);
      analogWrite(IN2,pwm); 

      break;
    case 0:   
      digitalWrite(EN1,LOW); 
      digitalWrite(EN3,LOW); 
      digitalWrite(EN2,LOW); 
      digitalWrite(IN3,LOW);
      digitalWrite(IN2,LOW);
      digitalWrite(IN1,LOW);  
      break;
    }
    break;
  case 6:
    switch (dir)
    {
    case 1:

      digitalWrite(EN1,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN2,LOW); 

      digitalWrite(IN1,LOW);
      digitalWrite(IN2,LOW);
      analogWrite(IN3,pwm); 

      break;
    case -1:

      digitalWrite(EN1,HIGH); 
      digitalWrite(EN3,HIGH); 
      digitalWrite(EN2,LOW);


      digitalWrite(IN2,LOW);
      digitalWrite(IN3,LOW);
      analogWrite(IN1,pwm); 


      break;
    case 0:  
      digitalWrite(EN1,LOW); 
      digitalWrite(EN3,LOW); 
      digitalWrite(EN2,LOW);  
      digitalWrite(IN3,LOW);
      digitalWrite(IN2,LOW);
      digitalWrite(IN1,LOW); 
      break;
    }
    break;

  }
}

void lerpot()
{
  pot_val=analogRead(pot);
  if(pot_val>522) {
    dir=1;
    pwm=map(pot_val,522,1023,0,255);
  }
  if(pot_val<500){
    dir=-1;
    pwm=map(pot_val,0,500,255,0);
  } 
  if((pot_val>=500)&&(pot_val<=522)) {
    dir=0;
    pwm=0;
    vb=0;
    vc=0;
  }
}

int depois(unsigned long T,unsigned long X)
{
  if((millis()-X)>T*1000)
    return 1;
  else
    return 0;
}
int entre(unsigned long A, unsigned long B,unsigned long X)
{
  if(((millis()-X)>A*1000 && (millis()-X)<B*1000))
    return 1;  
  else
    return 0;
}


void setupHMC5883L(){
  //Setup the HMC5883L, and check for errors
  int error;  
  error = compass.SetScale(1.3); //Set the scale of the compass.
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
}

float getHeading(){
  //Get the reading from the HMC5883L and calculate the heading
  MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Correct for when signs are reversed.
  if(heading < -2*PI) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;

  return heading; 
}

void teste_modelo()
{


  if(millis()<3000){
    dir=0;
    pwm=0;

  }
  if(entre(3,8,TL)){
    dir=-1;
    pwm=(1.0/12.0)*255.0;

  }
  if(entre(8,13,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(13,18,TL)){
    dir=-1;
    pwm=(2.0/12.0)*255.0;

  }
  if(entre(18,23,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(23,28,TL)){
    dir=-1;
    pwm=(3.0/12.0)*255.0;
  }
  if(entre(28,33,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(33,38,TL)){
    dir=-1;
    pwm=(4.0/12.0)*255.0;
  }
  if(entre(38,43,TL)){
    dir=0;
    pwm=0;
  }
  if(entre(43,48,TL)){
    dir=-1;
    pwm=(5.0/12.0)*255.0;

  }
  if(entre(48,53,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(53,58,TL)){
    dir=-1;
    pwm=(6.0/12.0)*255.0;

  }
  if(entre(58,63,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(63,68,TL)){
    dir=-1;
    pwm=(7.0/12.0)*255.0;
  }
  if(entre(68,73,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(73,78,TL)){
    dir=-1;
    pwm=(8.0/12.0)*255.0;
  }
  if(entre(78,83,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(83,88,TL)){
    dir=-1;
    pwm=(9.0/12.0)*255.0;
  }
  if(entre(88,93,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(93,98,TL)){
    dir=-1;
    pwm=(10.0/12.0)*255.0;
  }
  if(entre(98,103,TL)){
    dir=0;
    pwm=0;

  }
  if(entre(103,108,TL)){
    dir=-1;
    pwm=(11.0/12.0)*255.0;
  }
  if(entre(108,113,TL)){
    dir=0;
    pwm=0;
  }
  if(entre(113,118,TL)){
    dir=-1;
    pwm=(12.0/12.0)*255.0;
  }

  if(millis()>118000){
    dir=0;
    pwm=0;
    //novoloop=1;
  }
}


//void resetTimer2()
//{
//       // set timer 2 prescale factor to 64
//#if defined(__AVR_ATmega8__)
//       TCCR2 |= (1<<CS22);
//#else
//      TCCR2B |= (1<<CS22);
//#endif
//     // configure timer 2 for phase correct pwm (8-bit)
//#if defined(__AVR_ATmega8__)
//     TCCR2 |= (1<<WGM20);
//#else
//     TCCR2A |= (1<<WGM20);
//#endif
//
//}
//
//void Timer2()
//{
//  
//  int prescalerVal = 0x07;
//  int prescalerVal2 = 1;
//   TCCR2B &= ~prescalerVal; //AND the value in TCCR0B with binary number "11111000"
//  //Now set the appropriate prescaler bits: 
//  TCCR2B |= prescalerVal2; //OR the value in TCCR0B with binary number "00000001"
//}






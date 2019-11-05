#include "mbed.h"
#include "stdio.h"
#include "string.h"

volatile float    Kp=39.9,// 77.4  7.31  3.43- 36.6 13 0.66 - 93 27.3 3.12
                   Ki=11.1,
                   Kd=2.11;//
 volatile long int P=0,
                   I=0,
                   D=0,
                   PID=0; 
                   
Serial pc(USBTX, USBRX);
AnalogIn adc19(p19);
char dato_recibido[3];
Timer k;

float celB=0;
DigitalIn A(p16);
DigitalIn B(p5); //DigitalIn B(p5);
PwmOut pwmLeft(p22);
PwmOut pwmRight(p23);
 //tx rx
 volatile int datak=0;
char kchar[9];
Ticker timer;
const float Crpm=0.2564;
float pwmL=0,pwmR=0;
InterruptIn Aevent(p16);
InterruptIn Bevent(p5);
Timer ff;

//const float Cp=0.3846;
const float Cp=0.3461;//0.3461;
AnalogOut dac18(p18);
volatile double long f=0;
volatile double long count2 =0;
volatile float count1=0,cont=0;
volatile unsigned long timep,ttime,etime;
 
float position;
float rpm;
float Bposition;
float Error=0;
float pError=0;
 volatile int state,statep,indexx;
Timer mm;
 float reciveAngle=0;
int QEM[16]={0,-1,0,1,1,0,-1,0,0,1,0,-1,-1,0,1,0};

 
Serial GSM(p13,p14);  // tx rx puertos del FRDM para el modem

char buffer[20];// TAMAÑO DEL BUFER
Timer t;   //VALOR DEL TIEMPO
int count;
int i = 0;
int c=0;
float dato;
char r[]=""; 
char Qr[]="qmAIzQGtSK";
DigitalOut LedRojo(LED1);
DigitalOut LedVerde(LED2);
DigitalOut LedAzul(LED3);
int excep=0;
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Achange(){ 
  //A=digitalRead(2);
  //B=digitalRead(3);
  if(  (!A)&&(!B)    ){state=0;}
  if(  (!A)&&(B)   ){state=1;}
  if(  (A)&&(B)  ){state=2;}
  if(  (A)&&(!B)   ){state=3;}

indexx=4*state+statep;  
if( (count1>1040)||(count1<=-1040)  ){count1=0;}
count1=count1+QEM[indexx];

cont++;
count2=count2+QEM[indexx];
statep=state;

  }
void Bchange(){

  if(  (!A)&&(!B)    ){state=0;}
  if(  (!A)&&(B)   ){state=1;}
  if(  (A)&&(B)  ){state=2;}
  if(  (A)&&(!B)   ){state=3;}

indexx=4*state+statep;  
if( (count1>1040)||(count1<=-1040)  ){count1=0;}
count1=count1+QEM[indexx];
count2=count2+QEM[indexx];

statep=state;
cont++;

  }

    void turnRight(){  pwmLeft.write(0.00f);
                     pwmRight.write(pwmR);
                   }
                 
void turnLeft(){pwmRight.write(0.00f);
                    pwmLeft.write(pwmR);}
                
void turnOFF(){pwmRight.write(0.00f);
                pwmLeft.write(0.00f);}           
void mPID(){
  
 
  
  
 // reciveAngle=dato;
 // pc.printf("\n Reci: %3.2f ", reciveAngle); 
   Error=abs(reciveAngle-position);
   //pwmControl=map(error,0,360,0,255); 
   //pc.printf("\n E: %3.2f ", Error); 
    if(Error<=1.5){turnOFF(); PID=0; pError=0;  Error=0;}

   P=Kp*Error;
   I+=Ki*Error*0.02;
   if(I>450){I=450;}
    //pc.printf("\n I: %6.2f ", I); 
   D=Kd*(Error-pError)/0.02;
   PID=P+I+D; 
   
   pError=Error;
   
   if(PID>53000){PID=52423;}
   
   if(PID<2){PID=0;}
  
   //pc.printf("\n PID: %d ", PID);                                                 ///////////////////////////////////////////////////7
   if(Error<=1.5){turnOFF(); PID=0; pError=0;  Error=0; I=0;}
   // pc.printf("\n %d ", PID); 
 pwmR=map(PID,9500.0,52423.0,0.38f,1.0f);

   ///
  if(reciveAngle>position){ turnRight();}
   else{ turnLeft();}
   celB=reciveAngle;
  
}      

void show(){ 
       mm.start();             
         Bposition=position*0.05;
          position=(count1*Cp);//-Bposition;
          f =(count2/0.02)/4;
           int a = position;
           int b = PID/50;
           int c = Error*100;
   pc.printf("$ %d %d %d;", a,b,c);
         
           //f=(f/234 )*60 ;
           
          // pc.printf("\n\r %3.2f  ", (f/234 )*60); 
         // pc.printf("\n P:%2.2f ", position); 
          //pc.printf("\n E: %3.2f ", Error); 
          count2=0;
          cont=0;
}



 
int readBuffer(char *buffer,int count)   //esta funcion lee un bufer de datos
{
    int i=0; 
    t.start();    //CUENTA EL TIEMPO DE CONEXION E INICIA
    while(1) {
        while (GSM.readable()) {
            char c = GSM.getc();
           // if (c == '\r' || c == '\n') c = '$';//si se envia fin de linea o de caracter inserta $
            buffer[i++] = c;//mete al bufer el caracter leido
            if(i > count)break;//sale del loop si ya detecto terminacion
        }
        if(i > count)break;
        if(t.read() > 0.001) {  //MAS DE UN SEGUNDO DE ESPERA SE SALE Y REINICA EL RELOJ Y SE SALE lmao
            t.stop();
            t.reset();
            break;
        }
    }
     return 0;
}
 
void cleanBuffer(char *buffer, int count)  //esta funcion limpia el bufer
{
    for(int i=0; i < count; i++) {
        buffer[i] = '\0';
    }
}
 
   
int main(void){

       pwmLeft.period(0.001);
pwmRight.period(0.001);
//pwm period
 Aevent.mode(PullUp);
 Bevent.mode(PullUp);
       
        timer.attach_us(&show,20000.00);// periodo de muestreo
  Aevent.fall(&Achange);
 Bevent.fall(&Bchange);
  Aevent.rise(&Achange);
 Bevent.rise(&Bchange);
       
       
   
       
       LedVerde=1;
       GSM.baud(115200);
       pc.baud(115200);

       GSM.format(8,Serial::None,1); 
               
       while(1){ 
       if (GSM.readable()) {
          readBuffer(buffer,10);
         
          //dato = (buffer[0]-48)*100 + (buffer[1]-48)*10 + (buffer[2]-48)*1 + buffer[3]*0;
           dato =atoi(buffer);
           //pc.printf("\n ::: %2f",dato); 
          reciveAngle=dato;
         // pc.printf("buffer= %s\n\r ",buffer);  //imprime el bufer
          //pc.printf("buffer= %c  %c\n\r ",buffer[0],buffer[1]);//imprime el cero y el uno
          
       
          
          
          
          } 
          
          cleanBuffer(buffer,10);    
          for(int j = 0; j < 4; j++) buffer[j] = '0';  
          mPID();      
}


}
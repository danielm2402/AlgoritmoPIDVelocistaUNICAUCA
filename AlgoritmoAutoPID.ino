#include <QTRSensors.h>

//Mapeo de pines
const int PWMA = 3;
const int AIN2 = 5; //7;
const int AIN1 = 4; //8;
const int BIN1 = 8;
const int BIN2 = 7;
const int PWMB = 9;
const int STBY = 6;

/* Constantes del PID */
float Kp = 1.5;         /* + grande + oscilaciones, responde mejor a curvas cerradas */
float Kd = 20;        /* + grande - oscilaciones */
float Ki = 0;         /*  */
const long muestreo = 1;                                            // Tiempo de muestreo en milisegundos

float Error_an = 0;

#define NUM_SENSORS             6    
#define NUM_SAMPLES_PER_SENSOR  5    
#define EMITTER_PIN            11  
#define LED                    13     
#define  pulsador  12  

byte estado = 1.1;

float Error = 0; 
float ErrorAcu = 0;
float Vd, Vi; 
float VelocidadM = 255;

QTRSensorsAnalog qtra((unsigned char[]) {1,2,3,4,5,6}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

unsigned long anteriormillis = 0;                                   // 

void calibracion(){
  
  estado = digitalRead(pulsador);                                   // se lee estado pulsador
  
  while (estado == 1){
    digitalWrite(LED, HIGH);
    estado = digitalRead(pulsador);
  }
  
  if (estado == 0){                                               // el pulsador se activa con 0 entonces apaga los led led
  digitalWrite(LED, LOW);
  }
 
  delay(2000); 
  digitalWrite(LED, HIGH);

  // inicio calibracion
    
  for (int i=0; i<70; i++){
    digitalWrite(LED, HIGH); 
    delay(20);
    qtra.calibrate();
    digitalWrite(LED, LOW);  
    delay(20);
  }
  delay(3000);

  // fin calibracion

  digitalWrite(LED, LOW);                                       // Apaga el led para indicar que se termino la calibracion.
  delay(300);
  digitalWrite(LED, HIGH);
  delay(300);
  digitalWrite(LED, LOW);                                      // Se encienden y apagan LEDS .
  delay(300);               
  digitalWrite(LED, HIGH);     
  delay(300);
  digitalWrite(LED, LOW);     
  delay(300);
  
  estado = digitalRead(pulsador);
  
  while (estado == HIGH){
    digitalWrite(LED, HIGH);  
    estado = digitalRead(pulsador);    
  }
  
  if (estado == LOW){                                         //   se esepera a que se presione pulsador para poner a funcionar robot
    digitalWrite(LED, LOW);
    delay(2000);                                              /* PARA LA PARTIDA */
  }
}

void setup(){
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  //analogWrite(PWMA, 100);
  //analogWrite(PWMB, 100);
  digitalWrite(STBY, HIGH);
  
  //erial.begin(9600);
  
      // Motor B hacia adelante
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);

      // Motor A haciaa adelante
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2, HIGH);
      
  calibracion();
}

void loop(){  

  /* ********* Autotunning ********* */
  if(abs(Error)<55)
    Kp = 1.6;    
  else
    Kp = 10;
  /* ****************************** */
  
  unsigned long actualmillis = millis();

  if(actualmillis - anteriormillis >= muestreo){              // condicion para comprobar el tiempo de muestreo
    
    anteriormillis = actualmillis;
    
    Error = ((((int)(qtra.readLine(sensorValues)))-2500)/10);   // lectura del sensor y calculo del error normalizado
    ErrorAcu = Error*muestreo + ErrorAcu;
    Vd = VelocidadM - ((Kp*Error)+(Kd*((Error-Error_an)/muestreo))+(Ki*ErrorAcu));             // calculo del esfuerzo de control para motor der
    Vd = constrain(Vd,(0),+VelocidadM);               // saturación del esfuerzo de control
    Vi = VelocidadM + ((Kp*Error)+(Kd*((Error-Error_an)/muestreo))+(Ki*ErrorAcu));             // calculo del esfuerzo de control para motor izq
    Vi = constrain(Vi,(0),+VelocidadM);               // saturación del esfuerzo de control

    
    analogWrite(PWMB, (int)Vd);      
    analogWrite(PWMA, (int)Vi);
    Error_an = Error;    
  }

  /*
    if(Vd >= 0){
      // Motor B hacia adelante
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      // esfuerzo de control para cada motor
      analogWrite(PWMB, Vd);
    }
    else{
      // Motor B hacia atras
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      // esfuerzo de control para cada motor
      analogWrite(PWMB, -Vd);      
    }

    if(Vi >= 0){
      // Motor A hacia adelante
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2, HIGH);
      // esfuerzo de control para cada motor
      analogWrite(PWMA, Vi);
    }
    else{
      // Motor A hacia atras
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      // esfuerzo de control para cada motor
      analogWrite(PWMA, -Vi);    
    }
    */  
}

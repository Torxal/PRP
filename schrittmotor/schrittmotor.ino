/*
  Source file: schittmotor 
  Program: arduino editor
  Project: Project Rapid Prototype / Projekt Maschinenbau
  Author: Nikita Vetter
  Date: 15th January 2020
  Version: 1.0
  Programming language: C++ with Arduino IDE 
*/

#include <Servo.h>
int stepCounter;
int steps = 2000;
// Prototypen
class IRSensor;
class Stepper;
class Servo_;
void prozess();


// =========================== Schrittmotorenklasse ==================================
class Stepper{
  private: 
    double PIN_ENABLE, PIN_STEP, PIN_ENDSCHALTER;
    double lage;
  public: 
    double geschwindigkeit; 
    double PIN_DIR; //STEP_DIR = HIGH => Im Uhrzeigersinn (Mutter fährt nach oben)
    void fahre(double pos ,double drehzahl){ //dir = 1 (High) Im Uhrzeigersinn nach oben, Weg in mm 
        int dir; 
        Serial.begin(9600); 
        double steps; 
        //Setze Drehrichtung; 
        if(pos > lage){
         digitalWrite(PIN_DIR,HIGH); // im Uhrzeigersinn 
           steps = (pos-lage)*200/8;
         }
        else{
         digitalWrite(PIN_DIR,LOW); // Gegen den uhrzeigersinnUhrzeigersinn
           steps = (lage-pos)*200/8;
         }
         delay(1000); 
        double timeout = drehzahl;
        for(int stepCounter = 0; stepCounter < steps; stepCounter++)
           {
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds(500*5/drehzahl);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds(500*5/drehzahl);
           } 
           Serial.print(lage); 
    };  //Ende von fahre
    // Reset() nullt den Schrittmotor bei der Initialisierung 
    void reset(int dir){
      //Abfrage der Rückwärtsrichtung
           if(dir == 1){
             digitalWrite(dir,HIGH); // im Uhrzeigersinn
             }
           else{
             digitalWrite(dir,LOW); // im Uhrzeigersinn
           }
           digitalWrite(PIN_ENDSCHALTER,INPUT);
           Serial.print("Schrittmotor nullen\n");
           Serial.print(digitalRead(PIN_ENDSCHALTER));
           Serial.print(PIN_ENDSCHALTER);
           
           while(digitalRead(PIN_ENDSCHALTER) == HIGH){
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds(1000);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds(1000);
               
           }    
           lage = 0;
           Serial.print("Schrittmotor genullt");
           
      } 
    Stepper(double PIN_ENABLE_, double PIN_STEP_ ,double PIN_DIR_,int dir_reset, int PIN_ENDSCHALTER_){
        PIN_ENABLE = PIN_ENABLE_; 
        PIN_STEP   = PIN_STEP_;
        PIN_DIR    = PIN_DIR_;
        PIN_ENDSCHALTER = PIN_ENDSCHALTER_;
        pinMode(PIN_ENABLE, OUTPUT); //...Enable
        pinMode(PIN_STEP, OUTPUT); //  ...Step
        pinMode(PIN_DIR, OUTPUT); //   ...DIR
        digitalWrite(PIN_ENDSCHALTER,INPUT);
        Serial.print("Schrittmotor initialisiert");
        //Prüfen ob Endschalter vorhanden. Falls vorhanden => Schrittmotor nullen
        if(PIN_ENDSCHALTER >= 0){
          reset(dir_reset); 
        }
      };    

};
/* 
 ======================== Klasse für den IR-Sensor =============================    
*/
class IRSensor{
    private: 
      int PIN;
      double hoehe; 
     public: 
      IRSensor(int PIN){
        pinMode(PIN, INPUT); // IR 
      }; 
      void vermessen(Stepper stepper){
        hoehe = 0;
        for(int i; i<(350/10);i++){
            stepper.fahre(i*35, 1);
            if(((12.88)*analogRead(PIN)-0.42) < 0.05){
                hoehe = i*35; 
              }
          }
      };
      
  };
  /* 
 ======================== Klasse für den Servomotor =============================    
==> Probleme mit der Klasse. Erst ein Mal durch void spruehen ersetzt. 
*/ 

/*class ServoClass {
  private: 
    double winkel;
    int PIN; 
  public: 
  Servo servo; 
    ServoClass(int PIN_){
      PIN = PIN_;
        servo.attach(9, 800, 2100);
      };
    void spruehen(int delay_){
        //Zwischen 800 (0Grad und 2100  (90 Grad) möglich 
        servo.attach(9, 800, 2100);
        servo.writeMicroseconds(1800);
        //delay(delay_*1000);
        delay(1000);
        servo.writeMicroseconds(800);
        delay(1000);
      };
  };
  */
Servo servo;
void spruehen(int PIN){
    servo.attach(PIN, 800, 2100);
    // 2100 = 90Grad 
    // 800  = 0Grad
      servo.writeMicroseconds(1800);
        //delay(delay_*1000);
        delay(1000);
        servo.writeMicroseconds(800);
        delay(1000);
  }
  
/* Hier müssen alle Schrittmoren, der Stepper und der IR Sensor initialisiert werden 
Stepper (1: PIN ENABLE, 2: PIN STEP ,3: PIN DIR,4: Richtung für das Nullen des Schrittmotors, 5: PIN ENDSCHALTER (Falls nicht vorhanden = -1 setzen))
Servo(PIN => Für analogen Output)
IR-SENSOR(PIN => Für analogen Input)
*/

//Methode für den Gesamtprozess 
void prozess(){
    // (...) 1: Wegschieber 1 schiebt Druck auf Platte 
    // (...) 2: Verschließen 
    // (...) 3: Vermessen
    // (...) 4: Drehteller + Sprühen + Warten
    // (...) 5: Wegschieber 2 schiebt Druck von Platte
  };
//Stepper stepper_lack(6,5,4,1,12) ;
void setup()
{
  Serial.begin(9600); 
 // servo.attach(9, 800, 2100);
}
//Stepper stepper_lack(4,5,6,1,3) ;

void loop()
{
  //stepper_lack.fahre(350, 0.01667);
  spruehen(9);
 // servo.write(600); //Position 1 ansteuern mit dem Winkel 0°
 //   spruehen();
   //delay(1000); //Das Programm stoppt für 3 Sekunden
   //servo.writeMicroseconds(1500); //Position 1 ansteuern mit dem Winkel 0°
    
   // delay(1000); //Das Programm stoppt für 3 Sekunden
  // servo.writeMicroseconds(2100);
    
delay(1000);
  delay(1000);
}

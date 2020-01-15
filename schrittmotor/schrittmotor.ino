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
        double steps; 
        //Setze Drehrichtung; 
        if(pos > lage){
          dir = 1;
         digitalWrite(dir,HIGH); // im Uhrzeigersinn 
           steps = (pos-lage)*200*8;
         }
        else{
          dir = 0;
         digitalWrite(dir,LOW); // Gegen den uhrzeigersinnUhrzeigersinn
           steps = (lage-pos)*200*8;
         }
        double timeout = drehzahl;
        for(int stepCounter = 0; stepCounter < steps; stepCounter++)
           {
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds((1/(200*drehzahl)*1000000)/2);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds((1/(200*drehzahl)*1000000)/2);
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
           while(digitalRead(PIN_ENDSCHALTER)== false)
           {
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds(1000);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds(1000);
           }    
           lage = 0;
           Serial.print("Schrittmotor genullt");
           
      } 
    Stepper(double PIN_ENABLE_, double PIN_STEP_ ,double PIN_DIR_,int dir_reset, int PIN_ENDSCHALTER){
        PIN_ENABLE = PIN_ENABLE_; 
        PIN_STEP   = PIN_STEP_;
        PIN_DIR    = PIN_DIR_; 
        pinMode(PIN_ENABLE, OUTPUT); //...Enable
        pinMode(PIN_STEP, OUTPUT); //  ...Step
        pinMode(PIN_DIR, OUTPUT); //   ...DIR
        pinMode(PIN_DIR, OUTPUT); //   ...Endschalter
        digitalWrite(PIN_ENDSCHALTER,INPUT);
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
*/
class Servo_ {
  private: 
    double winkel;
    int PIN; 
    Servo servo; 
  public: 
    Servo_(int PIN){
        servo.attach(PIN);
      };
    void spruehen(int delay_){
        servo.write(70);
        delay(delay_*1000);
        servo.write(0);
      };
  };

/* Hier müssen alle Schrittmoren, der Stepper und der IR Sensor initialisiert werden 
Stepper (1: PIN ENABLE, 2: PIN STEP ,3: PIN DIR,4: Richtung für das Nullen des Schrittmotors, 5: PIN ENDSCHALTER (Falls nicht vorhanden = -1 setzen))
Servo(PIN => Für analogen Output)
IR-SENSOR(PIN => Für analogen Input)
*/
Stepper stepper_lack(4,5,6,0,1) ;
Servo_ servo(9); 

//Methode für den Gesamtprozess 
void prozess(){
    // (...) 1: Wegschieber 1 schiebt Druck auf Platte 
    // (...) 2: Verschließen 
    // (...) 3: Vermessen
    // (...) 4: Drehteller + Sprühen + Warten
    // (...) 5: Wegschieber 2 schiebt Druck von Platte
  };

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  stepper_lack.fahre(350, 6);
  delay(1000);
}

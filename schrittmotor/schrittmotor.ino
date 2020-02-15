/*
  Source file: HAUPTPROGRAMM.ino
  Program: arduino editor
  Project: Project Rapid Prototype
  Author: Nikita Vetter
  Date: 15th January 2020
  Version: 1.1
  Programming language: C++ with Arduino IDE 
*/

#include <Servo.h>

int stepCounter;
int steps = 2000;
// Prototypklassen
class IRSensor;
class Stepper;
class Servo_;
// Protypfunktionen
void prozess();


/* =========================== KLASSE STEPPER  ==================================
*/
class Stepper{
  // PRIVATE VARIABLEN 
  private: 
    int PIN_ENABLE, PIN_STEP, PIN_ENDSCHALTER, PIN_DIR,  DREHRICHTUNG_ZU_NULL; //Initialisierungsparameter (PINS,  MICROSTEPS -> Welche Auflösung wird verwendet (1, 1/2, 1/3), DREHZAHL_STANDARD -> Drehungen pro Sekunnde
    double POSITION=0, DREHUNGEN_JE_MM_GRAD, GESCHWINDIGKEIT_STANDARD, MICROSTEPS;                                             // Aktuelle Position (Position),  Wie viel Drehungen sind 1 mm oder Grad , Standardgeschwindigkeit mm/s(GESCHWINDIGKEIT_STANDARD)
    
  public: 
    // Funktion zur Berechnung der erfoderlichen Steps für einen bestimmten WEG (mm oder grad) 
    double STEPS_PRO_WEG(double WEG){
        return DREHUNGEN_JE_MM_GRAD * 200 * WEG * (1/MICROSTEPS); 
      }
    // Funktion zu Berechnung Geschwindigkeit durch Berechnung der Wartezeit pro Step
    double ZEIT_PRO_STEP(){
        1/(GESCHWINDIGKEIT_STANDARD * DREHUNGEN_JE_MM_GRAD *(1/MICROSTEPS) *200);
      }
    // ----- Methode für Relatives anfahren ------ (Fahren in eine Richtung, daher relativ), Weg in mm, Richtung 1 (HIGH) oder 0 (LOW)
    void fahre_relativ(double WEG, int RICHTUNG){
        digitalWrite(PIN_DIR, RICHTUNG);                //Drehrichtung festlegen HIGH = 1, LOW = 0
        double erf_weg = STEPS_PRO_WEG(WEG);
        if(PIN_ENDSCHALTER < 0){                       // Prüfen ob ENDSCHALTER vorhanden   
          for(int step=0; step<erf_weg; step++){
            if(digitalRead(PIN_ENDSCHALTER) != HIGH){ //  Prüfe durchgehend ob Schalter vorhanden
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds(ZEIT_PRO_STEP()/2);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds(ZEIT_PRO_STEP()/2);
            }//end if
            else{
              POSITION = 0;
              }
          }//end for
        }// end if
        else{
          // Kein Enschalter vorhanden
          for(int step=0; step<erf_weg; step++){
          while(digitalRead(PIN_ENDSCHALTER) != HIGH){ //  Prüfe durchgehend ob Schalter vorhanden
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds(ZEIT_PRO_STEP()/2);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds(ZEIT_PRO_STEP()/2);
            }//end while
          }//end for
         }
      }
  // ----- Methode für Positionsanfahren ------ (Anfahren einer Koordinate, daher absolut) 
    void fahre_absolut(double SOLLPOSITION){ //dir = 1 (High) Im Uhrzeigersinn nach oben, Weg in mm 
        double WEG;
        int DIR;
        //Setze Drehrichtung; 
        if(SOLLPOSITION > POSITION){
          DIR = HIGH;
         WEG = (SOLLPOSITION-POSITION);
         }
        else{
           DIR = LOW;
         WEG = (POSITION-SOLLPOSITION);
         }
         fahre_relativ(WEG, DIR);
         POSITION = SOLLPOSITION; //Position speichern
         
       
    };  //Ende von fahre
    
    // Reset() nullt den Schrittmotor bei der Initialisierung 
    void reset(){
      //Abfrage der Rückwärtsrichtung
           digitalWrite(PIN_DIR,DREHRICHTUNG_ZU_NULL); // im Uhrzeigersinn  
           
           while(digitalRead(PIN_ENDSCHALTER) == HIGH){
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds(ZEIT_PRO_STEP()/2);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds(ZEIT_PRO_STEP()/2);
           }    
           POSITION = 0;
           Serial.print("Schrittmotor genullt");
           
      } 
      
    // ------- Konstruktorklasse ----------
    Stepper(int PIN_ENABLE_, int PIN_STEP_ ,int PIN_DIR_,int PIN_ENDSCHALTER_, int DREHRICHTUNG_ZU_NULL_,double DREHUNGEN_JE_MM_GRAD, double GESCHWINDIGKEIT_STANDARD ){
        //Festlegen aller Pins und deren Initialisierung
        PIN_ENABLE = PIN_ENABLE_; 
        PIN_STEP   = PIN_STEP_;
        PIN_DIR    = PIN_DIR_;
        PIN_ENDSCHALTER = PIN_ENDSCHALTER_;
        pinMode(PIN_ENABLE, OUTPUT); 
        pinMode(PIN_STEP, OUTPUT); 
        pinMode(PIN_DIR, OUTPUT); 
        digitalWrite(PIN_ENDSCHALTER,INPUT);
        //Drehrichtung welche negativ sein soll oder die zum Endschalter führt 
        DREHRICHTUNG_ZU_NULL = DREHRICHTUNG_ZU_NULL_; 
        
        //NULLPUNKT finden => Prüfen ob Endschalter vorhanden. Falls vorhanden => Schrittmotor nullen
        if(PIN_ENDSCHALTER >= 0){
          reset(); 
        }

        //Abschluss der Initialisierung
        Serial.print("Schrittmotor initialisiert");
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
           // stepper.fahre(i*35, 1);
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
Stepper (1: PIN ENABLE, 2: PIN STEP ,3: PIN DIR,4: PIN_ENDSCHALTER, 5: DREHRICHTUNG_ZU_NULL, 6: DREHUNGEN_JE_MM_GRAD, 7: GESCHWINDIGKEIT_STANDARD)
Servo(PIN => Für analogen Output)
IR-SENSOR(PIN => Für analogen Input)
*/
//Stepper stepper_lack(3,2,53,1,24) ;

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
 // servo.attach(9, 800, 2100);
}
//Stepper stepper_lack(4,5,6,1,3) ;

void loop()
{
//  stepper_lack.fahre(50, 0.3);
  //stepper_lack.fahre(0, 1);
  //spruehen(9);
     
}

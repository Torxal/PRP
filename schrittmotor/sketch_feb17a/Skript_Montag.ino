/*
  Source file: HAUPTPROGRAMM.ino
  Program: arduino editor
  Project: Project Rapid Prototype
  Author: Nikita Vetter; Hannes Bohne
  Date: 15th January 2020
  Version: 1.3 (21.02.2020)
  Programming language: C++ with Arduino IDE
*/

#include <Servo.h>
// Prototypklassen
class IRSensor;
class Stepper;
class Servo_;
class Endschalter;
class LED;
class Lichtschranke;
/*
  ======================== Initialisierung aller Zeiger =============================
*/
Stepper *schieber_1;
Stepper *schieber_2;
Stepper *dose;
Stepper *drehteller;
Stepper *glocke;
IRSensor *IR_Sensor;
Servo_ *servo;
/* =========================== KLASSE STEPPER  ==================================
*/
class Stepper {
    // PRIVATE VARIABLEN
  private:
    int PIN_ENABLE, PIN_STEP, PIN_ENDSCHALTER, PIN_DIR,  DREHRICHTUNG_ZU_NULL; //Initialisierungsparameter (PINS,  MICROSTEPS -> Welche Auflösung wird verwendet (fest auf 1/16 gesetzt), DREHZAHL_STANDARD -> Drehungen pro Sekunnde
    double DREHUNGEN_JE_MM_GRAD, GESCHWINDIGKEIT_STANDARD, MICROSTEPS; // Aktuelle Position (Position),  Wie viel Drehungen sind 1 mm oder Grad , Standardgeschwindigkeit mm/s(GESCHWINDIGKEIT_STANDARD)


  public:
  double POSITION;
    // Funktion zur Berechnung der erfoderlichen Steps für einen bestimmten WEG (mm oder grad)
    double STEPS_PRO_WEG(double WEG) {
      return DREHUNGEN_JE_MM_GRAD * 200 * WEG * (1 / MICROSTEPS);
    }

    // Funktion zu Berechnung Geschwindigkeit durch Berechnung der Wartezeit pro Step (in us), WENN Geschwindigkeit <=0, dann wird mit der Standardgeschwindigkeit gerechnet
    double ZEIT_PRO_STEP(double GESCHWINDIGKEIT) {
      double ZEIT;
      if (GESCHWINDIGKEIT > 0) {
        ZEIT = (1000000 / (GESCHWINDIGKEIT * DREHUNGEN_JE_MM_GRAD * (1 / MICROSTEPS) * 200));
      }
      else {
        ZEIT = (1000000 / (GESCHWINDIGKEIT_STANDARD * DREHUNGEN_JE_MM_GRAD * (1 / MICROSTEPS) * 200));
      }
      return ZEIT;
    }

   

    // ----- Methode für Relatives anfahren ------ (Fahren in eine Richtung, daher relativ), Weg in mm, Richtung 1 (HIGH) oder 0 (LOW), Geschwindigkeit in mm/s
    void fahre_relativ(double WEG, int RICHTUNG, double GESCHWINDIGKEIT) {

      digitalWrite(PIN_ENABLE, LOW);
      //Neue Position speichern
      if (RICHTUNG == DREHRICHTUNG_ZU_NULL) {
        POSITION -= WEG;
      }
      else {
        POSITION += WEG;
      }

      digitalWrite(PIN_DIR, RICHTUNG);                //Drehrichtung festlegen HIGH = 1, LOW = 0
      double erf_weg = STEPS_PRO_WEG(WEG);

      double Zeit_ms = 0, Zeit_us = 0;  //Pausenzeiten für entsprechende Drehgeschwindigkeit in ms und us zerlegen
      if (ZEIT_PRO_STEP(GESCHWINDIGKEIT) > 1000) {
        Zeit_ms = (int) (ZEIT_PRO_STEP(GESCHWINDIGKEIT) / 2) / 1000;
        Zeit_us = (int) (ZEIT_PRO_STEP(GESCHWINDIGKEIT) / 2) % 1000;
      }
      else {
        Zeit_us = ZEIT_PRO_STEP(GESCHWINDIGKEIT);
        Zeit_ms = 0;
      }

      if (PIN_ENDSCHALTER > 0 && RICHTUNG == DREHRICHTUNG_ZU_NULL) { // Prüfen ob ENDSCHALTER vorhanden und Richtung Endschalter gefahren wird
        //Enschalter vorhanden
        for (int step = 0; step < erf_weg; step++) {
          if ((digitalRead(PIN_ENDSCHALTER) == 1)) {
            digitalWrite(PIN_STEP, HIGH);
            delay(Zeit_ms);
            delayMicroseconds(Zeit_us);
            digitalWrite(PIN_STEP, LOW);
            delay(Zeit_ms);
            delayMicroseconds(Zeit_us);
            
          }//end if
          else POSITION = 0;
        }//end for

      } else {
        for (long int step = 0; step < erf_weg; step++) {
          digitalWrite(PIN_STEP, HIGH);
          delay(Zeit_ms);
          delayMicroseconds(Zeit_us);
          digitalWrite(PIN_STEP, LOW);
          delay(Zeit_ms);
          delayMicroseconds(Zeit_us);
        }//end for
      }
    }


    // ----- Methode für Positionsanfahren ------ (Anfahren einer Koordinate, daher absolut)
    void fahre_absolut(double SOLLPOSITION, double GESCHWINDIGKEIT) { //dir = 1 (High) Im Uhrzeigersinn nach oben, Weg in mm
      double WEG;
      int DIR;
      //Setze Drehrichtung;
      if (SOLLPOSITION > POSITION) {
        //DIR = HIGH;
        if(DREHRICHTUNG_ZU_NULL == 1){
            DIR = LOW; 
          }
         else{
          DIR = HIGH;
          }
        WEG = (SOLLPOSITION - POSITION);
      }
      else {
        if(DREHRICHTUNG_ZU_NULL == 0){
            DIR = LOW; 
          }
         else{
          DIR = HIGH;
          }
        WEG = (POSITION - SOLLPOSITION);
      }
      fahre_relativ(WEG, DIR, GESCHWINDIGKEIT);
    }  //Ende von fahre

    // ----- Methode für Enable ------ (Motortreiber einschalten)
    void enable() {
      digitalWrite(PIN_ENABLE, LOW);
    }

    // ----- Methode für Disable ------ (Motortreiber ausschalten)
    void disable() {
      digitalWrite(PIN_ENABLE, HIGH);
    }

    // ----- Methode für aktuelle Position ------
    double Position() {
      return POSITION;
    }

    // Reset() nullt den Schrittmotor bei der Initialisierung
    void reset() {
      //Abfrage der Rückwärtsrichtung
      digitalWrite(PIN_ENABLE, LOW);
      digitalWrite(PIN_DIR, DREHRICHTUNG_ZU_NULL); // im Uhrzeigersinn

      double Zeit_ms = 0, Zeit_us = 0;
      if (ZEIT_PRO_STEP(0) > 1000) {
        Zeit_ms = (int) (ZEIT_PRO_STEP(0) / 2) / 1000;
        Zeit_us = (int) (ZEIT_PRO_STEP(0) / 2) % 1000;
      }
      else {
        Zeit_us = ZEIT_PRO_STEP(0);
        Zeit_ms = 0;
      }

      while (digitalRead(PIN_ENDSCHALTER) == 1) {
        digitalWrite(PIN_STEP, HIGH);
        delay(Zeit_ms);
        delayMicroseconds(Zeit_us);
        digitalWrite(PIN_STEP, LOW);
        delay(Zeit_ms);
        delayMicroseconds(Zeit_us);
      }

      delay(100);
      //toggelt die Drehrichtung
      digitalWrite(PIN_DIR, !digitalRead(PIN_DIR));

      while (digitalRead(PIN_ENDSCHALTER) != 1) {
        digitalWrite(PIN_STEP, HIGH);
        delay(1);
        digitalWrite(PIN_STEP, LOW);
        delay(1);
      }

      for (int step = 0; step < 3600; step++) {
        digitalWrite(PIN_STEP, HIGH);
        delay(1);
        digitalWrite(PIN_STEP, LOW);
        delay(1);
      }

      digitalWrite(PIN_DIR, DREHRICHTUNG_ZU_NULL);

      while (digitalRead(PIN_ENDSCHALTER) == 1) {
        digitalWrite(PIN_STEP, HIGH);
        //delayMicroseconds(ZEIT_PRO_STEP());
        delay(1);
        digitalWrite(PIN_STEP, LOW);
        //delayMicroseconds(ZEIT_PRO_STEP());
        delay(1);
      }

      POSITION = 0;

    }

    // ------- Konstruktorklasse ----------
    Stepper(int PIN_ENABLE_, int PIN_STEP_ , int PIN_DIR_, int PIN_ENDSCHALTER_, int DREHRICHTUNG_ZU_NULL_, double DREHUNGEN_JE_MM_GRAD_, double GESCHWINDIGKEIT_STANDARD_ ) {
    //Serial.begin(9600);
      double MICROSTEPS_ = 0.0625;
      //Festlegen aller Pins und deren Initialisierung
      PIN_ENABLE = PIN_ENABLE_;
      PIN_STEP   = PIN_STEP_;
      PIN_DIR    = PIN_DIR_;
      PIN_ENDSCHALTER = PIN_ENDSCHALTER_;
      pinMode(PIN_ENABLE, OUTPUT);
      digitalWrite(PIN_ENABLE, HIGH);
      pinMode(PIN_STEP, OUTPUT);
      digitalWrite(PIN_ENABLE, LOW);
      pinMode(PIN_DIR, OUTPUT);
      digitalWrite(PIN_ENABLE, LOW);
      pinMode(PIN_ENDSCHALTER, INPUT_PULLUP);
      //Drehrichtung welche negativ sein soll oder die zum Endschalter führt
      DREHRICHTUNG_ZU_NULL = DREHRICHTUNG_ZU_NULL_;
      DREHUNGEN_JE_MM_GRAD = DREHUNGEN_JE_MM_GRAD_;
      GESCHWINDIGKEIT_STANDARD = GESCHWINDIGKEIT_STANDARD_ ;
      MICROSTEPS = MICROSTEPS_;
      POSITION = 0;
      //NULLPUNKT finden => Prüfen ob Endschalter vorhanden. Falls vorhanden => Schrittmotor nullen
      if (PIN_ENDSCHALTER >= 0) {
        reset();
      }
       //Serial.print("Test") ;
      //Abschluss der Initialisierung
      //Serial.print("Schrittmotor initialisiert");
    }
};

/*
  ======================== Klasse für IR-Sensoren =============================
*/
class IRSensor {
  private:
    int PIN;
    double hoehe_sensor;
  public:
    IRSensor(int PIN_) {
      PIN = PIN_;
      pinMode(PIN, INPUT); // IR
      hoehe_sensor = 145; 
    }
    double messe() {
      return (12.88) * analogRead(PIN) - 0.42;
    }
    int SichtAufObjekt(){
      // Falls Objekt sichtbar wird der Eingang über 2000 geschätzt 
      if( ((12.88) * analogRead(PIN) - 0.42) < 2000){
        return 0;
        }
      else{
        return 1; 
        }
      }
     int hoehe_bestimmen(int maxhoehe,int aufloesung_IR, int aufloesung_teller){
      int x = 0;
      int hoehe_objekt; 
      //Schritte in Höhe
      dose->fahre_absolut(125,20);
      for(int i= 145; (maxhoehe+145) > i ; i=i+aufloesung_IR){
        Serial.print(i); 
        dose->fahre_absolut(i, 20);
        dose->disable();
          //Weg Drehteller
         for(int zaehler = 0; zaehler<200; zaehler= zaehler+aufloesung_teller){
          Serial.print(zaehler); 
              drehteller->enable();
              drehteller->fahre_relativ(aufloesung_teller,0,0);
              drehteller->disable();
              delay(1000);
              Serial.print("Objekt in Sicht: "); 
              Serial.print(SichtAufObjekt());
              Serial.print('\n'); 
              if(SichtAufObjekt() == 1){
                hoehe_objekt = i-145;
                }
            } 
            
        }
        Serial.print("Höhe des Objektes: "); 
        Serial.print(hoehe_objekt );
        Serial.print('\n'); 
        dose->fahre_absolut(30,20);
        return hoehe_objekt;
    }
    
};

/*
  ======================== Klasse für LED =============================
*/
class LED {
  private:
    int PIN;
  public:
    //Konstruktor
    LED(int PIN_) {
      PIN = PIN_;
      pinMode(PIN, OUTPUT);
      digitalWrite(PIN, LOW);
    }

    //LED ein
    void ein() {
      digitalWrite(PIN, HIGH);
    }

    //LED aus
    void aus() {
      digitalWrite(PIN, LOW);
    }
};

/*
  ======================== Klasse für Endschalter =============================
*/
class Endschalter {
  private:
    int PIN;
  public:
    //Konstruktor
    Endschalter(int PIN_) {
      PIN = PIN_;
      pinMode(PIN, INPUT_PULLUP);
    }

    //Endschalter Status abfragen
    int status() {
      return digitalRead(PIN);
    }
};

/*
  ======================== Klasse für Lichtschranken =============================
*/
class Lichtschranke {
  private:
    int PIN;
  public:
    //Konstruktor
    Lichtschranke(int PIN_) {
      PIN = PIN_;
      pinMode(PIN, INPUT);
    }

    //Positionieren
    int status() {
      return digitalRead(PIN);
    }
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

/*
  ======================== Klasse für Servo =============================
*/
class Servo_ {
  private:
    int PIN;
    Servo servo;
  public:
    //Konstruktor
    Servo_(int PIN_) {
      PIN = PIN_;
      servo.attach(PIN);
    }

    void spruehen_start() {
      servo.write(180); //180°
    }

    void spruehen_stopp() {
      servo.write(50); //0°
    }
    void lackieren(int hoehe_teil, int aufloesung){
        for(int i= 0; (hoehe_teil+0) > i ; i=i+aufloesung){
        Serial.print(i); 
        dose->fahre_absolut(i, 0);
        dose->disable();
          //Weg Drehteller
          spruehen_start(); 
              drehteller->enable();
              drehteller->fahre_relativ(200,0,0);
              drehteller->disable(); 
              spruehen_stopp();
        }
      }

};

/*
  ======================== inverse Kennlinie Geschwindigkeit Schieber 1 =============================
*/
double Kennlinie_v_inv(double pos_grad) {
  return -1 / ((4 * (-5445 + pos_grad)) / (15 * sqrt(982125 + (43560 * pos_grad) - (4 * sq(pos_grad)))));
}


/*
  ======================== Kennlinie Position Schieber 1 =============================
*/
double Kennlinie_pos_mm(double pos_grad) {
  return 3 * sqrt(59049 - sq(242 - 2 * (pos_grad * 8 / 360)));
}





////////////////////////////////////// HIER NOCH ZEIGER

//LED(PIN => Für Output)
LED LED_Warnung(28);
LED LED_IR(29);

//Lichtschranke(PIN => Für Output)
Lichtschranke Lichtschranke(24);

//Endschalter(PIN => Für digitalen Input)
Endschalter Druckbett(17);

/*
  ======================== Setup =============================
*/
void setup()
{
  //Enable aller Treiber deaktivieren, Kein Strom über Stepper
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  pinMode(44, OUTPUT);
  digitalWrite(44, HIGH);
  pinMode(43, OUTPUT);
  digitalWrite(43, HIGH);
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  //delay(5000);
  Serial.begin(9600);

  //Stepper (1: PIN ENABLE, 2: PIN STEP ,3: PIN DIR,4: PIN_ENDSCHALTER, 5: DREHRICHTUNG_ZU_NULL, 6: DREHUNGEN_JE_MM_GRAD, 7: GESCHWINDIGKEIT_STANDARD)
  //Stepper stepper_schieber_1(40, 46, 51, 18, 1, 0.005, 20); //Ein Schritt (1mm WEG) entspricht 1.8° Drehung des Steppers
  //schieber_1 = &stepper__schieber_1;

  //Stepper stepper_schieber_2(15, 34, 36, 20, 1, 0.125, 1);
  //schieber_2 = &stepper_schieber_2;
  /* Stepper stepper_glocke(14, 35, 38, 19, 1, 0.032, 25);
   glocke = &stepper_glocke; //182 für komplettes herunterfahren
   glocke->fahre_absolut(182,25);
   */
  Stepper stepper_drehteller(43, 48, 53, -1, 1, 0.005, 20); //Ein Schritt (1mm WEG) entspricht 1.8° Drehung des Steppers
  drehteller = &stepper_drehteller;
  drehteller->disable();

  Stepper stepper_dose(44, 50, 52, 21, 1, 0.125, 8);
  dose = &stepper_dose;
  dose->disable();
  
  //IR-SENSOR(PIN => Für analogen Input)
  IRSensor irsensor(A7);
  IR_Sensor = &irsensor;
  
  //Servo(PIN => Für Output)
  Servo_ servo_(16);
  servo = &servo_;
}

/*
  ======================== Main - Loop =============================
*/
class Teil{
  private: int hoehe = 0; //Diskrete Höhe. Daher int
  
  };

void loop(){

  //(*dose).fahre_relativ(10, 0, 16);
  //delay(2000);
  //Serial.print((*IR_Sensor).vermessen());
  //Serial.print("\n");
  // (*servo).spruehen_start();
  //delay(2000);
  //(*servo).spruehen_stopp();

  //Sprühprozess beschrieben:  - PIN STARTSCHALTER 


/*void loop()
  {

  //Motoren abschalten


  stepper_schieber_1.disable();
  stepper_schieber_2.disable();
  stepper_dose.disable();
  stepper_drehteller.disable();
  servo.spruehen_stopp();


  //Glocke schließen

  //stepper_glocke.fahre(200,0);
  //stepper_glocke.disable();


  //Auf Trigger warten

  int trigger = 0;
  int stop_time = 0;

  while (Druckbett.status() == 1) { //Endschalter nicht gedrückt
    delay(10);
  }
  trigger = 1;
  //Endschalter gedrückt
  for (int time = 0; time < 300; time++) { //in 10ms Sequenzen
    delay(10);
    if (Druckbett.status() == 1 && trigger == 1) { //Endschalter nicht gedrückt
      trigger = 2;
    }
    if (Druckbett.status() == 0 && trigger == 2) { //Endschalter gedrückt
      trigger = 3;
    }
    if (Druckbett.status() == 1 && trigger == 3) { //Endschalter nicht gedrückt
      trigger = 4;
    }
    if (Druckbett.status() == 0 && trigger == 4) { //Endschalter gedrückt
      trigger = 5;
      stop_time = time;
    }
    if (Druckbett.status() == 0 && trigger > 4) { //Endschalter gedrückt
      if (trigger == 5) {
        stop_time = time;
      }
      trigger++;
      if ((time - stop_time) == 10 && (trigger - 6) == 10) {
        time = 300;
      }
    }
  }


  //Heben der Glocke

  //Druckbett muss diese Zeit warten
  stepper_glocke.reset();


  //Ausfahren Schieber 1 mit angepasster Geschwindigkeit

  double endstellung = 400; //mm
  double offset_Druckbett_drehteller = 0;
  double v_Druckbett = 10; //mm/s
  double pos_mm, pos_grad;

  pos_grad = stepper_schieber_1.Position() * 1.8;
  pos_mm = Kennlinie_pos_mm(pos_grad);

  while (pos_mm < endstellung) {
    //Geschwindigkeit aus Position berechnen:
    double phi_p = 0;
    phi_p = Kennlinie_v_inv(pos_grad) * v_Druckbett;

    //Verfahren mit Geschwindigkeit
    stepper_schieber_1.fahre_relativ(10, 0, phi_p);

    //Positionen aktualisieren
    pos_grad = stepper_schieber_1.Position() * 1.8;
    pos_mm = Kennlinie_pos_mm(pos_grad);
  }


  //Lichtschranke aktiviern und Bauteil positionieren

  LED_Warnung.ein();
  LED_IR.ein();

  while(Lichtschranke.status() == 1){
    //Geschwindigkeit aus Position berechnen:
    double phi_p = 0;
    phi_p = Kennlinie_v_inv(pos_grad) * v_Druckbett;

    //Verfahren mit Geschwindigkeit
    stepper_schieber_1.fahre_relativ(10, 0, phi_p);

    //Positionen aktualisieren
    pos_grad = stepper_schieber_1.Position() * 1.8;
    pos_mm = Kennlinie_pos_mm(pos_grad);
  }

  double bauteil_start = pos_mm;

  while(Lichtschranke.status() == 0){
    //Geschwindigkeit aus Position berechnen:
    double phi_p = 0;
    phi_p = Kennlinie_v_inv(pos_grad) * v_Druckbett;

    //Verfahren mit Geschwindigkeit
    stepper_schieber_1.fahre_relativ(10, 0, phi_p);

    //Positionen aktualisieren
    pos_grad = stepper_schieber_1.Position() * 1.8;
    pos_mm = Kennlinie_pos_mm(pos_grad);
  }

  LED_IR.aus();
  LED_Warnung.aus();

  double bauteil_laenge = pos_mm - bauteil_start;

  double ziel = 105 - (bauteil_laenge/2) + offset_Druckbett_drehteller + endstellung;

  while (pos_mm < endstellung) {
    //Geschwindigkeit aus Position berechnen:
    double phi_p = 0;
    phi_p = Kennlinie_v_inv(pos_grad) * v_Druckbett;

    //Verfahren mit Geschwindigkeit
    stepper_schieber_1.fahre_relativ(10, 0, phi_p);

    //Positionen aktualisierenla
    pos_grad = stepper_schieber_1.Position() * 1.8;
    pos_mm = Kennlinie_pos_mm(pos_grad);
  }


  //Schieber zurück

  while (pos_mm < endstellung) {
    //Geschwindigkeit aus Position berechnen:
    double phi_p = 0;
    phi_p = Kennlinie_v_inv(pos_grad) * v_Druckbett;

    //Verfahren mit Geschwindigkeit
    stepper_schieber_1.fahre_relativ(10, 0, phi_p);

    //Positionen aktualisieren
    pos_grad = stepper_schieber_1.Position() * 1.8;
    pos_mm = Kennlinie_pos_mm(pos_grad);
  }

  stepper_schieber_1.disable();



  //Druck starten


  //Glocke schließen

  //stepper_glocke.fahre(200,0);
  //stepper_glocke.disable();




s

  //sprühen
*/
  ////NIKITA DEIN PART
  //double umdrehungen = 10; //Anzahl der Umdrehungen die der Drehteller zurücklegen soll
  //double geschw = 2; //Als Beispiel.. wären dann 2*1.8° pro sekunde. also Eine Umdrehung des Tellers in 30 Sekunden
  //int richtung = 1; //1 oder 0
  int hoehe = IR_Sensor->hoehe_bestimmen(100, 20, 50);
  //delay(5000);
  servo->lackieren(hoehe, 30);
  
 /* for(int u=0; u < umdrehungen; u++){
    stepper_drehteller.fahre_relativ(200, 0, geschw);
  }
  */
   //servo->spruehen_start();
  //delay(500);
// servo->spruehen_stopp();
 // delay(500);
  //servo->lackieren(80, 40);

}
  ////NIKITA DEIN PART


  //Zum Schluss:
  /*
  stepper_dose.disable();
  stepper_drehteller.disable();
  
*/




  /*
  //Glocke heben

  stepper_glocke.reset();



  //Schieber 2 drückt Bauteil raus


  stepper_schieber_2.fahre_absolut(238,0);
  stepper_schieber_2.reset();
  stepper_schieber_2.disable();
*/
//  }

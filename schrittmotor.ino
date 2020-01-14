


// Schrittmotorenklasse 
class Stepper{
  private: 
    double PIN_ENABLE, PIN_STEP;
    double lage;
  public: 
    double geschwindigkeit; 
    double PIN_DIR; //STEP_DIR = HIGH => Im Uhrzeigersinn (Mutter fährt nach oben)
    Stepper(double PIN_ENABLE_, double PIN_STEP_ ,double PIN_DIR_){
        PIN_ENABLE = PIN_ENABLE_; 
        PIN_STEP   = PIN_STEP_;
        PIN_DIR    = PIN_DIR_; 
        pinMode(PIN_ENABLE, OUTPUT); // Enable
        pinMode(PIN_STEP, OUTPUT); // Step
        pinMode(PIN_DIR, OUTPUT); // DIR
        digitalWrite(5,LOW);
         
      };     
     void fahre(double weg ,double drehzahl, int dir){ //dir = 1 (High) Im Uhrzeigersinn nach oben, Weg in mm 
        //Setze Drehrichtung; 
        if(dir == 1){
         digitalWrite(dir,HIGH); // im Uhrzeigersinn
         lage = lage + weg; 
         }
        else{
         digitalWrite(dir,LOW); // im Uhrzeigersinn
         lage = lage - weg; 
         }
        double steps = weg*200/8;
        double timeout = drehzahl;
        for(int stepCounter = 0; stepCounter < steps; stepCounter++)
           {
               digitalWrite(PIN_STEP,HIGH);
               delayMicroseconds((1/(200*drehzahl)*1000000)/2);
               digitalWrite(PIN_STEP,LOW);
               delayMicroseconds((1/(200*drehzahl)*1000000)/2);
           } 
           Serial.print(lage); 
     }
};

//Globale Objekterstellung
Stepper stepper_lack(4,5,6) ;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  digitalWrite(4,HIGH); // im Uhrzeigersinn
  stepper_lack.fahre(350, 6, 1);
  delay(1000);
  delay(1000);
}

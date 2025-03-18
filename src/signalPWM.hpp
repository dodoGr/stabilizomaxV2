#ifndef signalPWM_hpp
#define signalPWM_hpp

#include "base.hpp"

// Définition des broches ESP32 pour le signal PWM
#define bobineA 15
#define bobineB 2  //verifier les broches
#define bobineC 0  //verifier les broches
#define bobineD 4  //verifier les broches

#define frequence 5000

//calculés suivant le besoin
#define rapportCycliqueA 250
#define rapportCycliqueB 250
#define rapportCycliqueC 200
#define rapportCycliqueD 200

#define potTest1 36
#define potTest2 39

void signalPWM(int pin, int dutyCycle) {
    
    pinMode(pin, OUTPUT);
    analogWrite(pin, dutyCycle);
}



/*
pinMode(potTest, INPUT);

//a remplacer par la valeur calculée pour alimenter chaque bobine
int cycle = analogRead(potTest);  
cycle = map(cycle, 0, 4095, 0, 255); 
*/

#endif // SIGNALPWM_HPP
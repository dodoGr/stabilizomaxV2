#ifndef signalPWM_hpp
#define signalPWM_hpp

#include "base.hpp"

//pour les tests
int cycle1;
int cycle2;

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
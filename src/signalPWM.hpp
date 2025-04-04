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

#endif // SIGNALPWM_HPP
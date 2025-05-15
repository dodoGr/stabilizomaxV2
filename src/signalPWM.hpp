#ifndef signalPWM_hpp
#define signalPWM_hpp

#include "base.hpp"

void signalPWM(int pin, int dutyCycle) {
    
    analogWrite(pin, dutyCycle);
    //Serial.println(dutyCycle);
}

void envoieA(void* arg){
    digitalWrite(bobineA, LOW);
    delayMicroseconds(100);
    digitalWrite(bobineA, HIGH);
}

void envoieB(void* arg){
    digitalWrite(bobineB, LOW);
    delayMicroseconds(100);
    digitalWrite(bobineB, HIGH);
}

void envoieC(void* arg){
    digitalWrite(bobineC, LOW);
    delayMicroseconds(100);
    digitalWrite(bobineC, HIGH);
}

void envoieD(void* arg){
    digitalWrite(bobineD, LOW);
    delayMicroseconds(100);
    digitalWrite(bobineD, HIGH);
}

#endif // SIGNALPWM_HPP
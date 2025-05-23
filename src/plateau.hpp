#ifndef plateau_hpp
#define plateau_hpp

#include "base.hpp"

/*
int convertirX(int X) {
    return map(X, Xmin, Xmax, XminPlateau, XmaxPlateau);
}
int convertirY(int Y) {
    return map(Y, Ymin, Ymax, YminPlateau, YmaxPlateau);
}
*/ 

double lireX() {
    pinMode(Xri, INPUT);
    pinMode(Xle, OUTPUT);
    digitalWrite(Xle, LOW);

    pinMode(Yup, INPUT);
    pinMode(Ylo, OUTPUT);
    digitalWrite(Ylo, HIGH);
    
    X = analogRead(Xri);
    if (X < Xmin && X > 0) {
        Xmin = X;
    }
    if (X > Xmax) {
        Xmax = X;
    }

    X = constrain(X, 0, maxPlateauX);

    // On garde la dernière position pour les rebonds
    if(X != 0){
        sautZeroX = X;
    }
    // Si la bille est à zéro plus de 5 valeurs consécutives, alors c'est qu'il n'y a pas de bille sur le plateau 
    // Sinon, on ne prend pas en compte la position actuelle
    if(X == 0){
        X = sautZeroX;
    }

    /*
    Serial.print("Xmin = ");
    Serial.print(Xmin);
    Serial.print("\tXmax = ");
    Serial.println(Xmax);
    */
    return X;
}

double lireY() {
    pinMode(Yup, INPUT);
    pinMode(Ylo, OUTPUT);
    digitalWrite(Ylo, LOW);

    pinMode(Xri, INPUT);
    pinMode(Xle, OUTPUT);
    digitalWrite(Xle, HIGH);

    Y = analogRead(Yup);
    if (Y < Ymin && Y > 0) {
        Ymin = Y;
    }
    if (Y > Ymax) {
        Ymax = Y;
    }

    Y = constrain(Y, 0, maxPlateauY);

    // On garde la dernière position pour les rebonds
    if(Y != 0){
        sautZeroY = Y;
    }
    // Si la bille est à zéro, on ne prend pas en compte la position actuelle
    if(Y == 0){
        Y = sautZeroY;
    }

    /*
    Serial.print("Ymin = ");
    Serial.print(Ymin);
    Serial.print("\tYmax = ");
    Serial.println(Ymax);
    */
    return Y;
}

void afficherInfoXY() {
    Serial.print("Tableau X : ");
    for (int i = 0; i < TAILLE_TAB; i++) {
        Serial.print(tabX[i]);
        Serial.print("  ");
    }
    Serial.println();
//
    Serial.print("Tableau Y : ");
    for (int i = 0; i < TAILLE_TAB; i++) {
        Serial.print(tabY[i]);
        Serial.print("  ");
    }
    Serial.println();
    Serial.println("--------------------------------------------------");
    //Serial.print("Lissage X = ");
    //Serial.print(lissageX);
    //Serial.print("\tsautZeroX = ");
    //Serial.println(sautZeroX);
    //
    //Serial.print("Lissage Y = ");
    //Serial.print(lissageY);
    //Serial.print("\tsautZeroY = ");
    //Serial.println(sautZeroY);
    //Serial.println("--------------------------------------------------");
    //Serial.print("Signal PWM A = ");Serial.print(rapportCycliqueA);           Serial.print("\tmodif A = ");Serial.println(forceA);       
    //Serial.print("Signal PWM B = ");Serial.print(rapportCycliqueB);           Serial.print("\tmodif B = ");Serial.println(forceB);                        
    //Serial.print("Signal PWM C = ");Serial.print(rapportCycliqueC);           Serial.print("\tmodif C = ");Serial.println(forceC);       
    //Serial.print("Signal PWM D = ");Serial.print(rapportCycliqueD);           Serial.print("\tmodif D = ");Serial.println(forceD);     
    //Serial.println("--------------------------------------------------");
    //Serial.print("forceA = ");Serial.print(forceA);             Serial.print("\tdelayA = ");Serial.println(delayA);
    //Serial.print("forceB = ");Serial.print(forceB);             Serial.print("\tdelayB = ");Serial.println(delayB);
    //Serial.print("forceC = ");Serial.print(forceC);             Serial.print("\tdelayC = ");Serial.println(delayC);
    //Serial.print("forceD = ");Serial.print(forceD);             Serial.print("\tdelayD = ");Serial.println(delayD);
    //Serial.println("--------------------------------------------------");
    //Serial.print("Vitesse X = ");Serial.print(vitesseX);        Serial.print("\tVitesse Y = ");Serial.println(vitesseY);
    //Serial.println("--------------------------------------------------");

}

void acceleration() {
    int accelerationX = 0;
    int accelerationY = 0;

    accelerationX = (tabVitesseX[1] - tabVitesseX[0]);
    accelerationY = (tabVitesseY[1] - tabVitesseY[0]);

    Serial.print("Acceleration X = ");
    Serial.println(accelerationX);
    Serial.print("\tAcceleration Y = ");
    Serial.println(accelerationY);
}
    
#endif // PLATEAU_HPP
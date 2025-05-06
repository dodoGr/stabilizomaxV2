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
    /*
    Serial.print("Ymin = ");
    Serial.print(Ymin);
    Serial.print("\tYmax = ");
    Serial.println(Ymax);
    */
    return Y;
}

/*
void recupTabPlateau() {
    for (int i = TAILLE_TAB - 1; i > 0; i--) {
        tabX[i] = tabX[i - 1];
        tabY[i] = tabY[i - 1];
    }
    tabX[0] = lireX();
    tabY[0] = lireY();
}
*/

void afficherTableauXY() {
    Serial.print("Tableau X : ");
    for (int i = 0; i < TAILLE_TAB; i++) {
        Serial.print(tabX[i]);
        Serial.print("  ");
    }
    Serial.println();

    Serial.print("Tableau Y : ");
    for (int i = 0; i < TAILLE_TAB; i++) {
        Serial.print(tabY[i]);
        Serial.print("  ");
    }
    Serial.println();
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
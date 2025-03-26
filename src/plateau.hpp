#ifndef plateau_hpp
#define plateau_hpp

#include "base.hpp"

int convertirX(int X) {
    return map(X, Xmin, Xmax, XminPlateau, XmaxPlateau);
}

int convertirY(int Y) {
    return map(Y, Ymin, Ymax, YminPlateau, YmaxPlateau);
}

int lireX() {
    pinMode(Xri, INPUT);
    pinMode(Xle, OUTPUT);
    digitalWrite(Xle, LOW);

    pinMode(Yup, INPUT);
    pinMode(Ylo, OUTPUT);
    digitalWrite(Ylo, HIGH);

    int X = analogRead(Xri);
    if (X < Xmin && X > 0) {
        Xmin = X;
    }
    if (X > Xmax) {
        Xmax = X;
    }
    return convertirX(X);
}

int lireY() {
    pinMode(Yup, INPUT);
    pinMode(Ylo, OUTPUT);
    digitalWrite(Ylo, LOW);

    pinMode(Xri, INPUT);
    pinMode(Xle, OUTPUT);
    digitalWrite(Xle, HIGH);

    int Y = analogRead(Yup);
    if (Y < Ymin && Y > 0) {
        Ymin = Y;
    }
    if (Y > Ymax) {
        Ymax = Y;
    }
    return convertirY(Y);
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

void afficherTableau() {
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

void vitesse() {
    int vitesseX = 0;
    int vitesseY = 0;

    vitesseX = (tabX[0] - tabX[1]);
    vitesseY = (tabY[0] - tabY[1]);

    for (int i = TAILLE_TAB - 1; i > 0; i--) {
        tabVitesseX[i] = tabVitesseX[i - 1];
        tabVitesseY[i] = tabVitesseY[i - 1];
    }
    Serial.print("Vitesse X = ");
    Serial.print(vitesseX);
    Serial.print("\tVitesse Y = ");
    Serial.print(vitesseY);
    Serial.print("\t en mm/");
    Serial.print(temps/1000);
    Serial.println("s");

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
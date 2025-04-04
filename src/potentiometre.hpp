#ifndef potentiometre_hpp
#define potentiometre_hpp

#include "base.hpp"

int convertirBD(int BD) {
    return map(BD, BDminmesure, BDmaxmesure, BDmin*resolutionPot, BDmax*resolutionPot);
}

int convertirAC(int AC) {
    return map(AC, ACminmesure, ACmaxmesure, ACmin*resolutionPot, ACmax*resolutionPot);
}

int lireBD() {
    pinMode(BDpot, INPUT);

    BD = analogRead(BDpot);
    if (BD < BDminmesure && BD > 0) {
        BDminmesure = BD;
    }
    if (BD > BDmaxmesure) {
        BDmaxmesure = BD;
    }

    return convertirBD(BD);
}

int lireAC() {
    pinMode(ACpot, INPUT);

    AC = analogRead(ACpot);
    if (AC < ACminmesure && AC > 0) {
        ACminmesure = AC;
    }
    if (AC > ACmaxmesure) {
        ACmaxmesure = AC;
    }

    return convertirAC(AC);
}

void afficherTableauPOT() {
    Serial.print("pot AC : ");
    for (int i = 0; i < TAILLE_TAB; i++) {
        Serial.print(tabAC[i]);
        Serial.print("  ");
    }
    Serial.println();

    Serial.print("pot BD : ");
    for (int i = 0; i < TAILLE_TAB; i++) {
        Serial.print(tabBD[i]);
        Serial.print("  ");
    }
    Serial.println();
}

/*
//recupérer et afficher les valeurs des potentiomètres
void afficherBD() {
    int BD = analogRead(BDpot);
    Serial.print("BD : ");
    Serial.println(BD);
}

void afficherAC() {
    int AC = analogRead(ACpot);
    Serial.print("AC : ");
    Serial.println(AC);
}
*/


#endif // POTENTIOMETRE_HPP
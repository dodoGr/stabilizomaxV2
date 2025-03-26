#ifndef potentiometre_hpp
#define potentiometre_hpp

#include "base.hpp"

int lireBD() {
    pinMode(BDpot, INPUT);

    int BD = analogRead(BDpot);
    if (BD < BDminmesure && BD > 0) {
        BDminmesure = BD;
    }
    if (BD > BDmaxmesure) {
        BDmaxmesure = BD;
    }

    return analogRead(BDpot);
}

int lireAC() {
    pinMode(ACpot, INPUT);

    int AC = analogRead(ACpot);
    if (AC < ACminmesure && AC > 0) {
        ACminmesure = AC;
    }
    if (AC > ACmaxmesure) {
        ACmaxmesure = AC;
    }

    return analogRead(ACpot);
}

int convertirBD(int BD) {
    return map(BD, BDminmesure, BDmaxmesure, BDmin*resolutionPot, BDmax*resolutionPot);
}

int convertirAC(int AC) {
    return map(AC, ACminmesure, ACmaxmesure, ACmin*resolutionPot, ACmax*resolutionPot);
}

void afficherBD() {
    int BD = lireBD();
    Serial.print("BD : ");
    Serial.println(convertirBD(BD));
}

void afficherAC() {
    int AC = lireAC();
    Serial.print("AC : ");
    Serial.println(convertirAC(AC));
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
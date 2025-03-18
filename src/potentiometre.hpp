#ifndef potentiometre_hpp
#define potentiometre_hpp

#include "base.hpp"

// Définition des broches ESP32 pour le potentiomètre
#define BDpot 34  // entrée analogique => inclinaison autour de BD
#define ACpot 35  // entrée analogique => inclinaison autour de AC

//inclinaison autour de BD = 14° -> -7° à 7°
//inclinaison autour de AC = 12° -> -6° à 6°
//definition de la plage d'angles
#define BDmin -7
#define BDmax 7
#define ACmin -6
#define ACmax 6

//definition de la résolution des valeurs que l'on veut
#define resolutionPot 3

int BDminmesure = 4095;
int BDmaxmesure = 0;
int ACminmesure = 4095;
int ACmaxmesure = 0;

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
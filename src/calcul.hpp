#ifndef calcul_hpp
#define calcul_hpp

#include "base.hpp"

void vitesse(){
 
    vitesseX = (tabX[0] - tabX[1]) / 0.008;
    vitesseY = (tabY[0] - tabY[1]) / 0.008;
    
    for (int i = TAILLE_TAB - 1; i > 0; i--) {
        tabVitesseX[i] = tabVitesseX[i - 1];
        tabVitesseY[i] = tabVitesseY[i - 1];
        }
        /*
        Serial.print("Vitesse X = ");
        Serial.print(vitesseX);
        Serial.print("\tVitesse Y = ");
        Serial.print(vitesseY);
        Serial.print("\t en mm/");
        Serial.print(temps/1000);
        Serial.println("s");
        */
    
    }
    
/*
void acceleration(){
    int accelerationX = 0;
    int accelerationY = 0;
    
    accelerationX = (tabVitesseX[1] - tabVitesseX[0]);
    accelerationY = (tabVitesseY[1] - tabVitesseY[0]);
    
    Serial.print("Acceleration X = ");
    Serial.println(accelerationX);
    Serial.print("\tAcceleration Y = ");
    Serial.println(accelerationY);
}
*/

float erreur = 0;
float derivee = 0;

// Fonction PID
float computePID(float cible, float positionActuelle, float &erreur_prec, float &integrale, float Kp, float Ki, float Kd) {
    
    /* Recuperer intervalle de temps */
    ecartTemps = tempsCalcul - tempsPrecedentCalcul; // Calculer l'écart de temps

    erreur = cible - positionActuelle; // erreur actuelle
    integrale += erreur * ecartTemps; // somme des erreurs 
    derivee = (erreur - erreur_prec) / ecartTemps;
    erreur_prec = erreur;

    // Si l'erreur est trop grande, on ne prend pas en compte l'intégrale
    if (abs(erreur) < 100) {
        integrale = integrale + erreur * ecartTemps;
    }
    
    integrale = constrain(integrale, -1000, 1000); // Limiter l'intégrale pour éviter l'overshoot

    return Kp * erreur + Ki * integrale + Kd * derivee;
}

// Fonction principale de calcul PID pour contrôler les 4 électroaimants
void calculPID() {
    float distanceCentre = sqrt(pow(cibleX - lissageX, 2) + pow(cibleY - lissageY, 2));

    float commandeX = computePID(cibleX, lissageX, ancienneErreurX, integraleX, Kp_pos, Ki_pos, Kd_pos); 
    float commandeY = computePID(cibleY, lissageY, ancienneErreurY, integraleY, Kp_pos, Ki_pos, Kd_pos);
    
    float commandeVitX =  computePID(cibleVitX, vitesseX, ancienneerreurVitX, integraleVitX, Kp_vit, Ki_vit, Kd_vit);
    float commandeVitY = computePID(cibleVitY, vitesseY, ancienneerreurVitY, integraleVitY, Kp_vit, Ki_vit, Kd_vit);
    
    float commandeAC = computePID(cibleAC, AC, ancienneerreurAC, integraleAC, Kp_inclin, Ki_inclin, Kd_inclin);
    float commandeBD = computePID(cibleBD, BD, ancienneerreurBD, integraleBD, Kp_inclin, Ki_inclin, Kd_inclin);

    //coeff de proportionallité pour que X et Y aient le même poids
    float coeff = 304/228;

    //influence des bobines suivant X
    /*
    forceA = - commandeX;
    forceB = - commandeX;
    forceC = + commandeX;
    forceD = + commandeX;
    */

    //influence des bobines suivant Y
    /*
    forceA = + commandeY;
    forceB = - commandeY;
    forceC = - commandeY;
    forceD = + commandeY;
    */

   /*
    //influence des bobines suivant X et Y
    forceA = - commandeX + coeff * commandeY;
    forceB = - commandeX - coeff * commandeY;
    forceC = + commandeX - coeff * commandeY;
    forceD = + commandeX + coeff * commandeY;
    
    //ajout de la vitesse
    forceA = forceA - (+ coeffVit * commandeVitX - coeff * coeffVit * commandeVitY);
    forceB = forceB - (+ coeffVit * commandeVitX + coeff * coeffVit * commandeVitY);
    forceC = forceC - (- coeffVit * commandeVitX + coeff * coeffVit * commandeVitY);
    forceD = forceD - (- coeffVit * commandeVitX - coeff * coeffVit * commandeVitY);
    
    //ajout de l'inclinaison
    forceA = forceA + (- commandeAC - commandeBD) * facteurInclinaison;
    forceB = forceB + (- commandeAC - commandeBD) * facteurInclinaison;
    forceC = forceC + (- commandeAC - commandeBD) * facteurInclinaison;
    forceD = forceD + (- commandeAC - commandeBD) * facteurInclinaison;
    */
    
    /*                  position                                            vitesse                                                 inclinaison                 */
    forceA = (- commandeX + coeff * commandeY) - (+ coeffVit * commandeVitX - coeff * coeffVit * commandeVitY) + (- commandeAC - commandeBD) * facteurInclinaison;
    forceB = (- commandeX - coeff * commandeY) - (+ coeffVit * commandeVitX + coeff * coeffVit * commandeVitY) + (- commandeAC - commandeBD) * facteurInclinaison;
    forceC = (+ commandeX - coeff * commandeY) - (- coeffVit * commandeVitX + coeff * coeffVit * commandeVitY) + (- commandeAC - commandeBD) * facteurInclinaison;
    forceD = (+ commandeX + coeff * commandeY) - (- coeffVit * commandeVitX - coeff * coeffVit * commandeVitY) + (- commandeAC - commandeBD) * facteurInclinaison;


    // Échelle plus douce pour atténuer la brutalité des variations
    float facteurEchelle = 0.8;
    
    delayA = constrain(6000 - forceA * facteurEchelle, 4000, 7500) - sliderA;
    delayB = constrain(6000 - forceB * facteurEchelle, 4000, 7500) - sliderB;
    delayC = constrain(6000 - forceC * facteurEchelle, 4000, 7500) - sliderC;
    delayD = constrain(6000 - forceD * facteurEchelle, 4000, 7500) - sliderD;

}

#endif // CALCUL_HPP
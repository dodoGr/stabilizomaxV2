#ifndef calcul_hpp
#define calcul_hpp

#include "base.hpp"

void vitesse(){
 
    vitesseX = (tabX[0] - tabX[1]);
    vitesseY = (tabY[0] - tabY[1]);
    
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

// Fonction PID
float computePID(float cible, float positionActuelle, float &erreur_prec, float &integrale, float Kp, float Ki, float Kd) {
    
    /* Recuperer intervalle de temps */
    ecartTemps = tempsCalcul - tempsPrecedentCalcul; // Calculer l'écart de temps
    float erreur = 0;
    float derivee = 0;

    erreur = cible - positionActuelle; // erreur actuelle
    integrale += erreur * ecartTemps; // somme des erreurs 
    derivee = (erreur - erreur_prec) / ecartTemps;
    erreur_prec = erreur;

    return Kp * erreur + Ki * integrale + Kd * derivee;
}

// Fonction principale de calcul PID pour contrôler les 4 électroaimants
void calculPID() {

    float commandeX = computePID(cibleX, lissageX, ancienneErreurX, integraleX, Kp_pos, Ki_pos, Kd_pos); 
    float commandeY = computePID(cibleY, lissageY, ancienneErreurY, integraleY, Kp_pos, Ki_pos, Kd_pos);
    
    //float commandeVitX =  computePID(cibleVitX, vitesseX, ancienneerreurVitX, integraleVitX, Kp_vit, Ki_vit, Kd_vit);
    //float commandeVitY = computePID(cibleVitY, vitesseY, ancienneerreurVitY, integraleVitY, Kp_vit, Ki_vit, Kd_vit);
    
    //float commandeAC = computePID(cibleAC, AC, ancienneerreurAC, integraleAC, Kp_pos, Ki_pos, Kd_pos);
    //float commandeBD = computePID(cibleBD, BD, ancienneerreurBD, integraleBD, Kp_pos, Ki_pos, Kd_pos);

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

    //influence des bobines suivant X et Y
    forceA = - coeff * commandeX + commandeY;
    forceB = - coeff * commandeX - commandeY;
    forceC = + coeff * commandeX + commandeY;
    forceD = + coeff * commandeX - commandeY;
    /*
    */
  
    // Échelle plus douce pour atténuer la brutalité des variations
    float facteurEchelle = 0.1;
    
    // Conversion en PWM avec réponse plus progressive et inversée (20 = max puissance)
    rapportCycliqueA = constrain(0 - forceA * facteurEchelle, 0, 255)-5;
    rapportCycliqueB = constrain(0 - forceB * facteurEchelle, 0, 255);
    rapportCycliqueC = constrain(0 - forceC * facteurEchelle, 0, 255);
    rapportCycliqueD = constrain(0 - forceD * facteurEchelle, 0, 255)-3;

}

#endif // CALCUL_HPP
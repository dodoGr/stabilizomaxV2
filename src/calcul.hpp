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
// Fonction PID complète
float computePID(float cible, float positionActuelle, float &erreur_prec, float &integrale, float Kp, float Ki, float Kd) { 
    float erreur = cible - positionActuelle; // erreur actuelle
    integrale += erreur * temps / 1000; // somme des erreurs 
    float derivee = (erreur - erreur_prec)*1000 / temps;
    erreur_prec = erreur;

    Serial.print("proportionnel = ");
    Serial.print(erreur*Kd);
    Serial.print("\tintegrale = ");
    Serial.print(integrale*Kd);
    Serial.print("\tderivée = ");
    Serial.println(derivee*Kd);  

    return Kp * erreur + Ki * integrale + Kd * derivee;
}

// Fonction principale de calcul PID pour contrôler les 4 électroaimants
void calculPID() {

    // PID position + vitesse pour ajuster la force à appliquer
    float commandeX = computePID(cibleX, X, ancienneErreurX, integraleX, Kp_pos, Ki_pos, Kd_pos); 
    float commandeVitX =  computePID(cibleVitX, vitesseX, ancienneerreurVitX, integraleVitX, Kp_vit, Ki_vit, Kd_vit);
    float commandeY = computePID(cibleY, Y, ancienneErreurY, integraleY, Kp_pos, Ki_pos, Kd_pos);
    float commandeVitY = computePID(cibleVitY, vitesseY, ancienneerreurVitY, integraleVitY, Kp_vit, Ki_vit, Kd_vit);
    float commandeAC = computePID(cibleAC, AC, ancienneerreurAC, integraleAC, Kp_pos, Ki_pos, Kd_pos);
    float commandeBD = computePID(cibleBD, BD, ancienneerreurBD, integraleBD, Kp_pos, Ki_pos, Kd_pos);

    // Commande totale par électroaimant (influence des 4 axes)
    float forceA = (  commandeX + commandeVitX - commandeY - commandeVitY - commandeBD*0.3);
    float forceB = (  commandeX + commandeVitX + commandeY + commandeVitY + commandeAC*0.3);
    float forceC = (- commandeX - commandeVitX + commandeY + commandeVitY + commandeBD*0.3);
    float forceD = (- commandeX - commandeVitX - commandeY - commandeVitY - commandeAC*0.3);    

    // Échelle plus douce pour atténuer la brutalité des variations
    float facteurEchelle = 0.10;

    // Conversion en PWM avec réponse plus progressive et inversée (50 = max puissance)
    rapportCycliqueA = constrain(150 - forceA * facteurEchelle, 50, 255);
    rapportCycliqueB = constrain(150 - forceB * facteurEchelle, 50, 255);
    rapportCycliqueC = constrain(150 - forceC * facteurEchelle, 50, 255);
    rapportCycliqueD = constrain(150 - forceD * facteurEchelle, 50, 255);

    Serial.print("Inclinaison A = ");Serial.print(rapportCycliqueA);           Serial.print("\tforce A = ");Serial.println(forceA);       
    Serial.print("Inclinaison B = ");Serial.print(rapportCycliqueB);           Serial.print("\tforce B = ");Serial.println(forceB);                        
    Serial.print("Inclinaison C = ");Serial.print(rapportCycliqueC);           Serial.print("\tforce C = ");Serial.println(forceC);       
    Serial.print("Inclinaison D = ");Serial.print(rapportCycliqueD);           Serial.print("\tforce D = ");Serial.println(forceD);       
    
    Serial.print("commande X = ");Serial.print(commandeX);    Serial.print("\tcommande AC = ");Serial.print(commandeAC);    Serial.print("\tcommande vitesse X = ");Serial.println(commandeVitX);
    Serial.print("commande Y = ");Serial.print(commandeY);    Serial.print("\tcommande BD = ");Serial.print(commandeBD);    Serial.print("\tcommande vitesse Y = ");Serial.println(commandeVitY);

   
    
    
}


/*
void test_equilibrage(){
    //( les bobines fonctionnent à l'inverse, de 0 à 255 en sachant que 0 est la valeur la plus importante et 255 la plus faible )
    
    //si le plateau penche vers la Bobine C alors : 
    if(ACpot > 0){ 
        //si le plateau descend
        if ((tabAC[0]-tabAC[1]) < 0){     
            //on veut redresser sur la bobine C donc on baisse la valeur de C (puissance +) et augmente celle de A (puissance -)
            if (rapportCycliqueC > 0 - (tabAC[0]-tabAC[1])){
                rapportCycliqueC = rapportCycliqueC + (tabAC[0]-tabAC[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueC = 0;
            }
            if (rapportCycliqueA < 255 + (tabAC[0]-tabAC[1])){
                rapportCycliqueA = rapportCycliqueA + (tabAC[0]-tabAC[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueA = 255;
            }
        }
        //si le plateau remonte
        else if((tabAC[0]-tabAC[1]) > 0){     
            //on veut redresser sur la bobine C donc on baisse la valeur de C (puissance +) et augmente celle de A (puissance -)
            if (rapportCycliqueC > 0 + (tabAC[0]-tabAC[1])){
                rapportCycliqueC = rapportCycliqueC + (tabAC[0]-tabAC[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueC = 0;
            }
            if (rapportCycliqueA < 255 - (tabAC[0]-tabAC[1])){
                rapportCycliqueA = rapportCycliqueA - (tabAC[0]-tabAC[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueA = 255;
            }
        }
    }
    
    /////////////////////////////////////////////////////////
    
    //si le plateau penche vers la Bobine A alors :
    if(ACpot < 0){ 
        //si le plateau descend
        if ((tabAC[0]-tabAC[1]) > 0){     
            //on veut redresser sur la bobine A donc on baisse la valeur de A (puissance +) et augmente celle de C (puissance -)
            if (rapportCycliqueA > 0 - (tabAC[0]-tabAC[1])){
                rapportCycliqueA = rapportCycliqueA + (tabAC[0]-tabAC[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueA = 0;
            }
            if (rapportCycliqueC < 255 + (tabAC[0]-tabAC[1])){
                rapportCycliqueC = rapportCycliqueC + (tabAC[0]-tabAC[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueC = 255;
            }
        }
        //si le plateau remonte
        else if((tabAC[0]-tabAC[1]) < 0){     
            //on veut redresser sur la bobine A donc on baisse la valeur de A (puissance +) et augmente celle de C (puissance -)
            if (rapportCycliqueA > 0 + (tabAC[0]-tabAC[1])){
                rapportCycliqueA = rapportCycliqueA + (tabAC[0]-tabAC[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueA = 0;
            }
            if (rapportCycliqueC < 255 - (tabAC[0]-tabAC[1])){
                rapportCycliqueC = rapportCycliqueC - (tabAC[0]-tabAC[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueC = 255;
            }
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    
    //si le plateau penche vers la Bobine D alors : 
    if(BDpot > 0){ 
        //si le plateau descend
        if ((tabBD[0]-tabBD[1]) < 0){     
            //on veut redresser sur la bobine D donc on baisse la valeur de D (puissance +) et augmente celle de B (puissance -)
            if (rapportCycliqueD > 0 - (tabBD[0]-tabBD[1])){
                rapportCycliqueD = rapportCycliqueD + (tabBD[0]-tabBD[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueD = 0;
            }
            if (rapportCycliqueB < 255 + (tabBD[0]-tabBD[1])){
                rapportCycliqueB = rapportCycliqueB + (tabBD[0]-tabBD[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueB = 255;
            }
        }
        //si le plateau remonte
        else if((tabBD[0]-tabBD[1]) > 0){     
            //on veut redresser sur la bobine C donc on baisse la valeur de C (puissance +) et augmente celle de A (puissance -)
            if (rapportCycliqueD > 0 + (tabBD[0]-tabBD[1])){
                rapportCycliqueD = rapportCycliqueD + (tabBD[0]-tabBD[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueD = 0;
            }
            if (rapportCycliqueB < 255 - (tabBD[0]-tabBD[1])){
                rapportCycliqueB = rapportCycliqueB - (tabBD[0]-tabBD[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueB = 255;
            }
        }
    }
    
    /////////////////////////////////////////////////////////

    //si le plateau penche vers la Bobine A alors : 
    if(BDpot < 0){ 
        //si le plateau descend
        if ((tabBD[0]-tabBD[1]) > 0){     
            //on veut redresser sur la bobine A donc on baisse la valeur de A (puissance +) et augmente celle de C (puissance -)
            if (rapportCycliqueB > 0 - (tabBD[0]-tabBD[1])){
                rapportCycliqueB = rapportCycliqueB + (tabBD[0]-tabBD[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueB = 0;
            }
            if (rapportCycliqueD < 255 + (tabBD[0]-tabBD[1])){
                rapportCycliqueD = rapportCycliqueD + (tabBD[0]-tabBD[1]); //amplifier le mouvement
            }
            else{
                rapportCycliqueD = 255;
            }
        }
        //si le plateau remonte
        else if((tabBD[0]-tabBD[1]) < 0){     
            //on veut redresser sur la bobine A donc on baisse la valeur de A (puissance +) et augmente celle de C (puissance -)
            if (rapportCycliqueB > 0 + (tabBD[0]-tabBD[1])){
                rapportCycliqueB = rapportCycliqueB + (tabBD[0]-tabBD[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueB = 0;
            }
            if (rapportCycliqueD < 255 - (tabBD[0]-tabBD[1])){
                rapportCycliqueD = rapportCycliqueD - (tabBD[0]-tabBD[1]); //amortir le mouvement
            }
            else{
                rapportCycliqueD = 255;
            }
        }
    }
}
*/

#endif // CALCUL_HPP
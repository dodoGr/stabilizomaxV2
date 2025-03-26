#ifndef calcul_hpp
#define calcul_hpp

#include "base.hpp"

/*
void vitesse(){
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

#endif // CALCUL_HPP
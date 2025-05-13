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
    if (X ==0 || Y == 0) { // Si la bille n'est pas la
        //erreur = erreur;
        //integrale = integrale;
        //derivee = derivee;
        erreur = 0;
        integrale = integrale;
        derivee = 0;
    }
    else{
        erreur = cible - positionActuelle; // erreur actuelle
        integrale += erreur * ecartTemps; // somme des erreurs 
        derivee = (erreur - erreur_prec) / ecartTemps;
        erreur_prec = erreur;
    }
    /*
    Serial.print("proportionnel = ");
    Serial.print(erreur*Kd);
    Serial.print("\tintegrale = ");
    Serial.print(integrale*Kd);
    Serial.print("\tderivée = ");
    Serial.println(derivee*Kd);  
    */

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


    forceA = - coeff * commandeX + commandeY;
    forceB = - coeff * commandeX - commandeY;
    forceC = + coeff * commandeX + commandeY;
    forceD = + coeff * commandeX - commandeY;

    
    // Échelle plus douce pour atténuer la brutalité des variations
    float facteurEchelle = 0.1;
    
    // Conversion en PWM avec réponse plus progressive et inversée (20 = max puissance)
    rapportCycliqueA = constrain(150 - forceA * facteurEchelle, 50, 255);
    rapportCycliqueB = constrain(150 - forceB * facteurEchelle, 50, 255);
    rapportCycliqueC = constrain(150 - forceC * facteurEchelle, 50, 255);
    rapportCycliqueD = constrain(150 - forceD * facteurEchelle, 50, 255);

    /*
    Serial.print("Signal PWM A = ");Serial.print(rapportCycliqueA);           Serial.print("\tmodif A = ");Serial.println(forceA);       
    Serial.print("Signal PWM B = ");Serial.print(rapportCycliqueB);           Serial.print("\tmodif B = ");Serial.println(forceB);                        
    Serial.print("Signal PWM C = ");Serial.print(rapportCycliqueC);           Serial.print("\tmodif C = ");Serial.println(forceC);       
    Serial.print("Signal PWM D = ");Serial.print(rapportCycliqueD);           Serial.print("\tmodif D = ");Serial.println(forceD);     
    Serial.println("--------------------------------------------------");  
    // 
    Serial.print("commande X = ");Serial.println(commandeX);  
    Serial.print("commande Y = ");Serial.println(commandeY);  
    */

}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


// Structure pour représenter l'état de la bille
struct BilleState {
    double x;
    double y;
};

// Structure pour représenter l'état de la plaque (commandes PWM pour les actuateurs)
struct PlaqueState {
    int control_1;
    int control_2;
    int control_3;
    int control_4;
};

// Structure pour les coefficients du PID
struct PIDCoefficients {
    double kp; // Gain proportionnel
    double ki; // Gain intégral
    double kd; // Gain dérivé
};

class PIDController {
private:
    PIDCoefficients coefficients;
    double integral;
    double previous_error;

public:
    PIDController(PIDCoefficients coeffs) : coefficients(coeffs), integral(0.0), previous_error(0.0) {}

    double compute(double setpoint, double current_value, double dt) {
        double error = setpoint - current_value;
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        double output = coefficients.kp * error + coefficients.ki * integral + coefficients.kd * derivative;
        previous_error = error;
        return output;
    }

    void reset() {
        integral = 0.0;
        previous_error = 0.0;
    }
};

class BillePlaqueController {
private:
    PIDController pid_x;
    PIDController pid_y;
    BilleState bille_state;
    PlaqueState plaque_state;
    double dt; // Intervalle de temps entre les mises à jour

public:
    BillePlaqueController(PIDCoefficients pid_x_coeffs, PIDCoefficients pid_y_coeffs, double time_step)
        : pid_x(pid_x_coeffs), pid_y(pid_y_coeffs), dt(time_step) {
        // Initialiser les broches pour les actuateurs en mode OUTPUT
        pinMode(bobineA, OUTPUT);
        pinMode(bobineB, OUTPUT);
        pinMode(bobineC, OUTPUT);
        pinMode(bobineD, OUTPUT);
    }

    // Lire la position de la bille à partir des capteurs analogiques
    BilleState getBillePosition() {
        // TODO: Implémenter la lecture des données des capteurs de position de la bille
        // Si les capteurs sont analogiques:
        int raw_x = analogRead(X);
        int raw_y = analogRead(Y);

        // TODO: Convertir les valeurs brutes de l'ADC (0-1023) en unités de position significatives
        // Cela dépendra de votre capteur et de la géométrie de votre plaque.
        bille_state.x = static_cast<double>(raw_x); // Exemple simple: utiliser la valeur brute
        bille_state.y = static_cast<double>(raw_y); // Exemple simple: utiliser la valeur brute

        Serial.print("Position de la bille (brute): X=");
        Serial.print(bille_state.x);
        Serial.print(", Y=");
        Serial.println(bille_state.y);

        return bille_state;
    }

    // Appliquer les commandes PWM aux actuateurs
    void setActuatorCommands(int control_1, int control_2, int control_3, int control_4) {
        // Assurer que les commandes PWM sont dans la plage 0-255 (pour analogWrite sur la plupart des Arduinos)
        control_1 = constrain(control_1, 0, 255);
        control_2 = constrain(control_2, 0, 255);
        control_3 = constrain(control_3, 0, 255);
        control_4 = constrain(control_4, 0, 255);

        analogWrite(bobineA, control_1);
        analogWrite(bobineB, control_2);
        analogWrite(bobineC, control_3);
        analogWrite(bobineD, control_4);

        plaque_state.control_1 = control_1;
        plaque_state.control_2 = control_2;
        plaque_state.control_3 = control_3;
        plaque_state.control_4 = control_4;

        Serial.print("Commandes PWM: C1=");
        Serial.print(plaque_state.control_1);
        Serial.print(", C2=");
        Serial.print(plaque_state.control_2);
        Serial.print(", C3=");
        Serial.print(plaque_state.control_3);
        Serial.print(", C4=");
        Serial.println(plaque_state.control_4);
    }

    // Calculer les commandes pour les actuateurs en fonction des corrections d'angle PID
    void computeActuatorCommands(double control_x, double control_y) {
        // TODO: Déterminer la relation entre les corrections d'angle (control_x, control_y)
        // et les commandes PWM nécessaires pour chaque actuateur.
        // Ceci dépend de la géométrie de votre plaque et de la disposition des actuateurs.

        // Exemple très simplifié (nécessite une modélisation physique ou une calibration)
        // Ces valeurs de contrôle PID devront être mises à l'échelle pour correspondre à la plage PWM (0-255).
        int command_1 = static_cast<int>(127 + control_x + control_y * 0.5);
        int command_2 = static_cast<int>(127 - control_x + control_y * 0.5);
        int command_3 = static_cast<int>(127 - control_x - control_y * 0.5);
        int command_4 = static_cast<int>(127 + control_x - control_y * 0.5);

        setActuatorCommands(command_1, command_2, command_3, command_4);
    }

    void control(double target_x, double target_y) {
        BilleState current_state = getBillePosition();
        double current_x = current_state.x;
        double current_y = current_state.y;

        // Calculer les corrections pour les "angles" nécessaires via les PID
        double control_x = pid_x.compute(target_x, current_x, dt);
        double control_y = pid_y.compute(target_y, current_y, dt);

        // Traduire ces corrections en commandes pour les actuateurs
        computeActuatorCommands(control_y, control_x); // Note: L'angle Y contrôle X et vice versa (potentiellement adapté)
    }

    void resetControllers() {
        pid_x.reset();
        pid_y.reset();
    }
};	

#endif // CALCUL_HPP
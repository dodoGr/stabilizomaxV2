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
    
    /* Recuperer intervalle de temps */
    ecartTemps = tempsCalcul - tempsPrecedentCalcul; // Calculer l'écart de temps
    float erreur = 0;
    float derivee = 0;
    if (X ==0 || Y == 0) { // Si la bille est en mouvement
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

    // PID position + vitesse pour ajuster la force à appliquer
    float commandeX = computePID(cibleX, lissageX, ancienneErreurX, integraleX, Kp_pos, Ki_pos, Kd_pos); 
    //float commandeVitX =  computePID(cibleVitX, vitesseX, ancienneerreurVitX, integraleVitX, Kp_vit, Ki_vit, Kd_vit);
    float commandeY = computePID(cibleY, lissageY, ancienneErreurY, integraleY, Kp_pos, Ki_pos, Kd_pos);
    //float commandeVitY = computePID(cibleVitY, vitesseY, ancienneerreurVitY, integraleVitY, Kp_vit, Ki_vit, Kd_vit);
    //float commandeAC = computePID(cibleAC, AC, ancienneerreurAC, integraleAC, Kp_pos, Ki_pos, Kd_pos);
    //float commandeBD = computePID(cibleBD, BD, ancienneerreurBD, integraleBD, Kp_pos, Ki_pos, Kd_pos);

    //coeff de proportionallité pour que X et Y aient le même poids
    float coeff = 304/228;

    //commande partielle sur X pour chaque électroaimant (influence des 2 axes)
    if(lissageX > cibleX){
        forceA = - commandeX;
        forceB = - commandeX;
        forceC = + commandeX;
        forceD = + commandeX;
    }
    else if(lissageX < cibleX){
        forceA = + commandeX;
        forceB = + commandeX;
        forceC = - commandeX;
        forceD = - commandeX;
    }
    
    /*
    //commande partielle sur Y pour chaque électroaimant (influence des 2 axes)
    if(lissageY > cibleY){
        forceA = commandeY;
        forceB = commandeY;
        forceC = commandeY;
        forceD = commandeY;
    }
    else if(lissageY < cibleY){
        forceA = -commandeY;
        forceB = -commandeY;
        forceC = -commandeY;
        forceD = -commandeY;
    }
    */


    /*
    // Commande totale par électroaimant (influence des 4 axes)
    //coté B
    if (lissageX > cibleX && lissageY > cibleY){
        forceA = (- coeff * commandeX + commandeY);
        forceB = (- coeff * commandeX - commandeY);
        forceC = (+ coeff * commandeX - commandeY);
        forceD = (+ coeff * commandeX + commandeY);
    }
    //coté D
    else if (lissageX < cibleX && lissageY < cibleY){
        forceA = (+ coeff * commandeX - commandeY);
        forceB = (+ coeff * commandeX + commandeY);
        forceC = (- coeff * commandeX + commandeY);
        forceD = (- coeff * commandeX - commandeY);
    } 
    //coté C
    else if (lissageX > cibleX && lissageY < cibleY){
        forceA = (- coeff * commandeX - commandeY);
        forceB = (- coeff * commandeX + commandeY);
        forceC = (+ coeff * commandeX + commandeY);
        forceD = (+ coeff * commandeX - commandeY);
    } 
    //coté A
    else if (lissageX < cibleX && lissageY > cibleY){
        forceA = (+ coeff * commandeX + commandeY);
        forceB = (+ coeff * commandeX - commandeY);
        forceC = (- coeff * commandeX - commandeY);
        forceD = (- coeff * commandeX + commandeY);
    }
    */
    
    // Échelle plus douce pour atténuer la brutalité des variations
    float facteurEchelle = 0.1;
    
    // Conversion en PWM avec réponse plus progressive et inversée (20 = max puissance)
    rapportCycliqueA = constrain(150 + forceA * facteurEchelle, 20, 255);
    rapportCycliqueB = constrain(150 + forceB * facteurEchelle, 40, 255);
    rapportCycliqueC = constrain(150 + forceC * facteurEchelle, 100, 255);
    rapportCycliqueD = constrain(150 + forceD * facteurEchelle, 100, 255);
    
    /*
    rapportCycliqueA = 0;
    rapportCycliqueB = 255;
    rapportCycliqueC = 255;
    rapportCycliqueD = 0;
    */

    /*
    Serial.print("Signal PWM A = ");Serial.print(rapportCycliqueA);           Serial.print("\tmodif A = ");Serial.println(forceA);       
    Serial.print("Signal PWM B = ");Serial.print(rapportCycliqueB);           Serial.print("\tmodif B = ");Serial.println(forceB);                        
    Serial.print("Signal PWM C = ");Serial.print(rapportCycliqueC);           Serial.print("\tmodif C = ");Serial.println(forceC);       
    Serial.print("Signal PWM D = ");Serial.print(rapportCycliqueD);           Serial.print("\tmodif D = ");Serial.println(forceD);       
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
	
	
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
	
// code trouvé par meo
/*

// Structure pour les valeurs PID
struct PIDController {
    float kP;        // Coefficient proportionnel
    float kI;        // Coefficient intégral
    float kD;        // Coefficient dérivé
    float setpoint;  // Valeur cible
    float prevError; // Erreur précédente
    float integral;  // Somme des erreurs (pour terme I)
    float lastTime;  // Temps de la dernière mise à jour
    float maxOutput; // Sortie maximale pour limiter la force
    
    PIDController() : kP(0), kI(0), kD(0), setpoint(0), prevError(0), integral(0), lastTime(0), maxOutput(100) {}
};

// Valeurs mesurées
float ballX = 0;     // Position X actuelle de la bille
float ballY = 0;     // Position Y actuelle de la bille
float targetX = 0;   // Position X cible
float targetY = 0;   // Position Y cible

// Variables pour suivre la vitesse de la bille
float prevBallX = 0;
float prevBallY = 0;
float ballSpeedX = 0;
float ballSpeedY = 0;
unsigned long lastSpeedCalcTime = 0;

// Contrôleurs PID pour les axes X et Y
PIDController pidX;  // PID pour l'axe X (Est-Ouest)
PIDController pidY;  // PID pour l'axe Y (Nord-Sud)

// Constantes pour la plaque
const float PLATE_WIDTH = 100.0;   // Largeur de la plaque en mm
const float PLATE_HEIGHT = 100.0;  // Hauteur de la plaque en mm

// Zone de garde pour éviter que la bille ne sorte
const float MARGIN = 10.0;         // Marge en mm depuis le bord

// Limites pour les sorties PWM (rappel: plus le PWM est élevé, plus la puissance est faible)
const int PWM_MIN = 0;      // Puissance maximale
const int PWM_MAX = 255;    // Puissance minimale
const int PWM_NEUTRAL = 128; // Position neutre

// Variables pour le temps
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500; // Intervalle d'affichage en ms

// Mode de contrôle progressif
bool progressiveMode = true;    // Commencer en mode progressif
float maxForce = 30.0;          // Force maximale initiale (0-100)
unsigned long progressiveTimer = 0;
const unsigned long PROGRESSIVE_INTERVAL = 3000; // Augmenter la force toutes les 3 secondes

// Fonction pour initialiser les contrôleurs PID
void setupPID() {
    // Coefficients beaucoup plus bas pour commencer
    // Réglage des coefficients PID pour l'axe X
    pidX.kP = 0.5;  // Réduit considérablement
    pidX.kI = 0.01; // Réduit considérablement
    pidX.kD = 0.5;  // Pour amortir les oscillations
    pidX.maxOutput = maxForce;
    
    // Réglage des coefficients PID pour l'axe Y
    pidY.kP = 0.5;  // Réduit considérablement
    pidY.kI = 0.01; // Réduit considérablement
    pidY.kD = 0.5;  // Pour amortir les oscillations
    pidY.maxOutput = maxForce;
    
    // Mise à zéro des intégrales
    pidX.integral = 0;
    pidY.integral = 0;
    
    // Initialisation du temps
    pidX.lastTime = millis();
    pidY.lastTime = millis();
}

// Fonction pour lire la position de la bille depuis le pavé tactile
void readBallPosition() {
    // À remplacer par votre code pour lire la position de la bille
    // Cette fonction devrait mettre à jour ballX et ballY
    
    // Pour l'exemple, on pourrait lire des entrées analogiques
    // ballX = map(analogRead(A0), 0, 1023, 0, PLATE_WIDTH);
    // ballY = map(analogRead(A1), 0, 1023, 0, PLATE_HEIGHT);
    
    // Calculer la vitesse de la bille (pour la composante D du PID)
    unsigned long currentTime = millis();
    if (currentTime - lastSpeedCalcTime > 50) { // Calcul toutes les 50ms
        float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0;
        ballSpeedX = (ballX - prevBallX) / deltaTime;
        ballSpeedY = (ballY - prevBallY) / deltaTime;
        
        prevBallX = ballX;
        prevBallY = ballY;
        lastSpeedCalcTime = currentTime;
    }
}

// Sécurité pour maintenir la bille sur le plateau
void applySafetyLimits() {
    // Si la bille est proche du bord ou si elle se déplace trop vite vers le bord
    if ((ballX < MARGIN && ballSpeedX < 0) || 
        (ballX > PLATE_WIDTH - MARGIN && ballSpeedX > 0) ||
        (ballY < MARGIN && ballSpeedY < 0) || 
        (ballY > PLATE_HEIGHT - MARGIN && ballSpeedY > 0)) {
        
        // Réduire l'intégrale du PID pour diminuer la force
        pidX.integral *= 0.5;
        pidY.integral *= 0.5;
        
        // Ajuster la cible pour ramener la bille vers le centre
        if (ballX < MARGIN) targetX = max(targetX, MARGIN * 2);
        if (ballX > PLATE_WIDTH - MARGIN) targetX = min(targetX, PLATE_WIDTH - MARGIN * 2);
        if (ballY < MARGIN) targetY = max(targetY, MARGIN * 2);
        if (ballY > PLATE_HEIGHT - MARGIN) targetY = min(targetY, PLATE_HEIGHT - MARGIN * 2);
        
        pidX.setpoint = targetX;
        pidY.setpoint = targetY;
    }
}

// Calcul de la sortie PID pour un axe
float computePID(PIDController &pid, float currentValue, float velocity) {
    // Calcul du temps écoulé
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - pid.lastTime) / 1000.0; // Conversion en secondes
    pid.lastTime = currentTime;
    
    // Protection contre les délais trop grands (premier appel ou débordement)
    if (deltaTime > 1.0) {
        deltaTime = 0.01; // Valeur par défaut raisonnable
    }
    
    // Calcul de l'erreur
    float error = pid.setpoint - currentValue;
    
    // Terme proportionnel
    float p = pid.kP * error;
    
    // Terme intégral avec anti-windup plus conservateur
    pid.integral += error * deltaTime;
    pid.integral = constrain(pid.integral, -20, 20); // Anti-windup plus strict
    float i = pid.kI * pid.integral;
    
    // Terme dérivé - utilisant la vitesse mesurée de la bille pour plus de stabilité
    // Un terme négatif pour amortir le mouvement
    float d = -pid.kD * velocity;
    
    // Mise à jour de l'erreur précédente
    pid.prevError = error;
    
    // Calcul de la sortie PID
    float output = p + i + d;
    
    // Limiter la sortie à la force maximale autorisée
    output = constrain(output, -pid.maxOutput, pid.maxOutput);
    
    return output;
}

// Calcul des valeurs PWM pour les électroaimants antagonistes avec une approche plus douce
void calculateAntagonistPWM(float pidOutput, int &pwmNegative, int &pwmPositive) {
    // Normaliser la sortie PID entre -maxForce et maxForce
    float normalizedOutput = constrain(pidOutput, -maxForce, maxForce);
    
    // Convertir la sortie en pourcentage de la plage
    float percentage = normalizedOutput / maxForce; // -1.0 à 1.0
    
    // Approche plus progressive pour les PWM
    if (percentage > 0) {
        // Pour une sortie positive: augmenter pwmPositive proportionnellement et diminuer pwmNegative
        pwmNegative = PWM_NEUTRAL + int((1.0 - percentage) * (PWM_MAX - PWM_NEUTRAL));
        pwmPositive = PWM_NEUTRAL - int(percentage * (PWM_NEUTRAL - PWM_MIN));
    } else {
        // Pour une sortie négative: augmenter pwmNegative proportionnellement et diminuer pwmPositive
        percentage = -percentage; // Rendre positif pour les calculs
        pwmPositive = PWM_NEUTRAL + int((1.0 - percentage) * (PWM_MAX - PWM_NEUTRAL));
        pwmNegative = PWM_NEUTRAL - int(percentage * (PWM_NEUTRAL - PWM_MIN));
    }
    
    // Vérifier les limites des valeurs PWM
    pwmPositive = constrain(pwmPositive, PWM_MIN, PWM_MAX);
    pwmNegative = constrain(pwmNegative, PWM_MIN, PWM_MAX);
}

void setup() {
    // Initialisation de la communication série
    Serial.begin(115200);
    
    // Configuration des broches PWM
    pinMode(bobineC, OUTPUT);
    pinMode(bobineB, OUTPUT);
    pinMode(bobineD, OUTPUT);
    pinMode(bobineA, OUTPUT);
    
    // Initialisation des valeurs PWM (puissance neutre au démarrage)
    analogWrite(bobineC, PWM_NEUTRAL);
    analogWrite(bobineB, PWM_NEUTRAL);
    analogWrite(bobineD, PWM_NEUTRAL);
    analogWrite(bobineA, PWM_NEUTRAL);
    
    // Initialisation des contrôleurs PID
    setupPID();
    
    // Point cible initial au centre de la plaque
    targetX = PLATE_WIDTH / 2;
    targetY = PLATE_HEIGHT / 2;
    
    // Mise à jour des valeurs cibles pour les PID
    pidX.setpoint = targetX;
    pidY.setpoint = targetY;
    
    // Initialiser le timer pour le mode progressif
    progressiveTimer = millis();
    
    Serial.println("Système initialisé en mode progressif.");
    Serial.println("La force maximale augmentera progressivement pour trouver le bon niveau.");
    Serial.println("Envoyez 'x,y' pour définir une nouvelle cible (ex: '25,75')");
    Serial.println("Envoyez 'p:X,i:Y,d:Z' pour ajuster les coefficients PID (ex: 'p:0.5,i:0.01,d:0.5')");
    Serial.println("Envoyez 'max:N' pour définir la force maximale (ex: 'max:50')");
    Serial.println("Envoyez 'prog:off' pour désactiver le mode progressif");
}

void loop() {
    // Lecture de la position actuelle de la bille
    readBallPosition();
    
    // Appliquer les limites de sécurité pour maintenir la bille sur le plateau
    applySafetyLimits();
    
    // Calcul des sorties PID en utilisant la position et la vitesse
    float pidOutputX = computePID(pidX, ballX, ballSpeedX);
    float pidOutputY = computePID(pidY, ballY, ballSpeedY);
    
    // Calcul des valeurs PWM pour les paires d'électroaimants antagonistes
    int pwmWest, pwmEast, pwmNorth, pwmSouth;
    
    // Pour l'axe X: Est (positif) vs Ouest (négatif)
    calculateAntagonistPWM(pidOutputX, pwmWest, pwmEast);
    
    // Pour l'axe Y: Sud (positif) vs Nord (négatif)
    calculateAntagonistPWM(pidOutputY, pwmNorth, pwmSouth);
    
    // Calcul des valeurs PWM pour chaque électroaimant individuel
    // en combinant les effets des deux axes avec une moyenne pondérée
    int pwmNW = (pwmNorth + pwmWest) / 2;  // Nord-Ouest
    int pwmNE = (pwmNorth + pwmEast) / 2;  // Nord-Est
    int pwmSW = (pwmSouth + pwmWest) / 2;  // Sud-Ouest
    int pwmSE = (pwmSouth + pwmEast) / 2;  // Sud-Est
    
    // Application des valeurs PWM aux électroaimants
    analogWrite(bobineC, pwmNW);
    analogWrite(bobineB, pwmNE);
    analogWrite(bobineD, pwmSW);
    analogWrite(bobineA, pwmSE);
    
    // Mode progressif: augmenter graduellement la force maximale
    if (progressiveMode) {
        unsigned long currentTime = millis();
        if (currentTime - progressiveTimer >= PROGRESSIVE_INTERVAL) {
            progressiveTimer = currentTime;
            
            // Augmenter progressivement la force maximale
            if (maxForce < 100) {
                maxForce += 5;
                pidX.maxOutput = maxForce;
                pidY.maxOutput = maxForce;
                
                Serial.print("Force maximale augmentée à: ");
                Serial.println(maxForce);
            }
        }
    }
    
    // Affichage des informations pour le débogage (limité en fréquence)
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
        lastPrintTime = currentTime;
        
        Serial.print("Position: (");
        Serial.print(ballX);
        Serial.print(", ");
        Serial.print(ballY);
        Serial.print(") - Cible: (");
        Serial.print(targetX);
        Serial.print(", ");
        Serial.print(targetY);
        Serial.println(")");
        
        Serial.print("Vitesse: (");
        Serial.print(ballSpeedX);
        Serial.print(", ");
        Serial.print(ballSpeedY);
        Serial.println(")");
        
        Serial.print("PID - X: ");
        Serial.print(pidOutputX);
        Serial.print(", Y: ");
        Serial.println(pidOutputY);
        
        Serial.print("PWM - NW: ");
        Serial.print(pwmNW);
        Serial.print(", NE: ");
        Serial.print(pwmNE);
        Serial.print(", SW: ");
        Serial.print(pwmSW);
        Serial.print(", SE: ");
        Serial.println(pwmSE);
        
        Serial.print("Coefficients - P:");
        Serial.print(pidX.kP);
        Serial.print(" I:");
        Serial.print(pidX.kI);
        Serial.print(" D:");
        Serial.print(pidX.kD);
        Serial.print(" - Force max: ");
        Serial.println(maxForce);
    }
    
    // Vérification des commandes entrées via le port série
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        
        // Format pour nouvelle cible: "x,y"
        if (input.indexOf(',') != -1 && !input.startsWith("p:") && !input.startsWith("max:") && !input.startsWith("prog:")) {
            int commaIndex = input.indexOf(',');
            float newX = input.substring(0, commaIndex).toFloat();
            float newY = input.substring(commaIndex + 1).toFloat();
            
            // Vérification des limites
            if (newX >= MARGIN && newX <= PLATE_WIDTH - MARGIN && 
                newY >= MARGIN && newY <= PLATE_HEIGHT - MARGIN) {
                targetX = newX;
                targetY = newY;
                pidX.setpoint = targetX;
                pidY.setpoint = targetY;
                
                // Réinitialisation des intégrales pour éviter l'accumulation d'erreurs
                pidX.integral = 0;
                pidY.integral = 0;
                
                Serial.print("Nouvelle cible définie: (");
                Serial.print(targetX);
                Serial.print(", ");
                Serial.print(targetY);
                Serial.println(")");
            } else {
                Serial.println("Coordonnées trop proches des bords! Utilisez des valeurs entre 10 et 90.");
            }
        }
        // Format pour ajuster les coefficients PID: "p:0.5,i:0.01,d:0.5"
        else if (input.startsWith("p:")) {
            float p = 0, i = 0, d = 0;
            bool foundP = false, foundI = false, foundD = false;
            
            // Extraire P
            int pStart = input.indexOf("p:") + 2;
            int pEnd = input.indexOf(",", pStart);
            if (pEnd == -1) pEnd = input.length();
            p = input.substring(pStart, pEnd).toFloat();
            foundP = true;
            
            // Extraire I
            int iStart = input.indexOf("i:");
            if (iStart != -1) {
                iStart += 2;
                int iEnd = input.indexOf(",", iStart);
                if (iEnd == -1) iEnd = input.length();
                i = input.substring(iStart, iEnd).toFloat();
                foundI = true;
            }
            
            // Extraire D
            int dStart = input.indexOf("d:");
            if (dStart != -1) {
                dStart += 2;
                int dEnd = input.indexOf(",", dStart);
                if (dEnd == -1) dEnd = input.length();
                d = input.substring(dStart, dEnd).toFloat();
                foundD = true;
            }
            
            // Mise à jour des coefficients
            if (foundP) pidX.kP = pidY.kP = p;
            if (foundI) pidX.kI = pidY.kI = i;
            if (foundD) pidX.kD = pidY.kD = d;
            
            Serial.print("Nouveaux coefficients PID - P: ");
            Serial.print(pidX.kP);
            Serial.print(", I: ");
            Serial.print(pidX.kI);
            Serial.print(", D: ");
            Serial.println(pidX.kD);
        }
        // Format pour ajuster la force maximale: "max:50"
        else if (input.startsWith("max:")) {
            int maxStart = input.indexOf("max:") + 4;
            float newMax = input.substring(maxStart).toFloat();
            
            if (newMax >= 0 && newMax <= 100) {
                maxForce = newMax;
                pidX.maxOutput = maxForce;
                pidY.maxOutput = maxForce;
                
                Serial.print("Force maximale définie à: ");
                Serial.println(maxForce);
            } else {
                Serial.println("Force maximale doit être entre 0 et 100");
            }
        }
        // Format pour activer/désactiver le mode progressif: "prog:on" ou "prog:off"
        else if (input.startsWith("prog:")) {
            String mode = input.substring(5);
            if (mode == "on") {
                progressiveMode = true;
                progressiveTimer = millis();
                Serial.println("Mode progressif activé");
            } else if (mode == "off") {
                progressiveMode = false;
                Serial.println("Mode progressif désactivé");
            }
        }
    }
    
    // Petit délai pour éviter une boucle trop rapide
    delay(10);
}
*/



#endif // CALCUL_HPP
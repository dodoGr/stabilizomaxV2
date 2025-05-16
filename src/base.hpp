#ifndef base_hpp
#define base_hpp

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include <Ticker.h>

#include <stdio.h>
#include <cmath>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

// definition du tableau
#define TAILLE_TAB 20 
// definition du temps
unsigned long tempsSynchro = 10;//temps en ms

//////////////////////////////////////////////////////////

//protéger les variables globales
SemaphoreHandle_t xMutex;

void initBase(){
    xMutex = xSemaphoreCreateMutex();
}

//////////////////////////////////////////////////////////
//                       plateau                        //
//////////////////////////////////////////////////////////


// Définition des broches ESP32 pour le plateau
#define Xri 32  // Doit être une entrée analogique (ADC)       //marron
#define Xle 25  // Doit être une sortie numérique              //rouge
#define Yup 33  // Doit être une entrée analogique (ADC)       //bleu
#define Ylo 26  // Doit être une sortie numérique              //blanc

#define XminPlateau 0
#define XmaxPlateau 304
#define YminPlateau 0
#define YmaxPlateau 228

double X = 0;
double Y = 0;

int Xmin = 3000;
int Xmax = 0;
int Ymin = 3000;
int Ymax = 0;

int tabX[TAILLE_TAB] = {0};
int tabY[TAILLE_TAB] = {0};
int tabVitesseX[TAILLE_TAB] = {0};
int tabVitesseY[TAILLE_TAB] = {0};

volatile bool calculVitesse = false;
volatile bool calculAcceleration = false;

int vitesseX = 0;
int vitesseY = 0;
int coeffVit = 0.1;

int sautZeroX = 0;
int sautZeroY = 0;


//////////////////////////////////////////////////////////
//                  Potentiometre                       //
//////////////////////////////////////////////////////////


// Définition des broches ESP32 pour le potentiomètre
#define BDpot 35  // entrée analogique => inclinaison autour de BD
#define ACpot 34  // entrée analogique => inclinaison autour de AC

//inclinaison autour de BD = 14° -> -7° à 7°
//inclinaison autour de AC = 12° -> -6° à 6°
//definition de la plage d'angles
#define BDmin -7
#define BDmax 7
#define ACmin -6
#define ACmax 6

int BD = 0; //valeur du potentiomètre
int AC = 0; //valeur du potentiomètre

//definition de la résolution des valeurs que l'on veut
#define resolutionPot 3

int BDminmesure = 4095;
int BDmaxmesure = 0;
int ACminmesure = 4095;
int ACmaxmesure = 0;

int tabAC[TAILLE_TAB] = {0};
int tabBD[TAILLE_TAB] = {0};


//////////////////////////////////////////////////////////
//                      Bobines                         //
//////////////////////////////////////////////////////////


// Définition des broches ESP32 pour le signal PWM
#define bobineA 5       //autres broches fonctionnelles 15
#define bobineB 18      //autres broches fonctionnelles 2  
#define bobineC 19      //autres broches fonctionnelles 0  
#define bobineD 23      //autres broches fonctionnelles 4  

void outBobines(){
    pinMode(bobineA, OUTPUT);
    pinMode(bobineB, OUTPUT);
    pinMode(bobineC, OUTPUT);
    pinMode(bobineD, OUTPUT);
}

#define frequence 5000

//calculés suivant le besoin
int rapportCycliqueA = 0;
int rapportCycliqueB = 0;
int rapportCycliqueC = 0;
int rapportCycliqueD = 0;

//////////////////////////////////////////////////////////
//                         PID                          //
//////////////////////////////////////////////////////////

unsigned long tempsCalcul = millis(); // Temps écoulé depuis le démarrage de l'Arduino
static unsigned long tempsPrecedentCalcul = 0; // Temps de la dernière mise à jour
unsigned long ecartTemps = 0; // Écart de temps entre les calculs

float Kp_pos = 2.4,      Kp_vit = 2.6;       //coefficient proportionnel (vitesse de réponse)
float Ki_pos = 0.05,    Ki_vit = 0.05;     //coefficient intégral      (précision)
float Kd_pos = 20,       Kd_vit = 20;      //coefficient dérivé        (stabilité)

float ancienneErreurX = 0; //erreur précédente sur X
float ancienneErreurY = 0; //erreur précédente sur Y
float ancienneerreurVitX = 0; //erreur précédente sur la vitesse X
float ancienneerreurVitY = 0; //erreur précédente sur la vitesse Y
float ancienneerreurAC = 0; //erreur précédente sur AC
float ancienneerreurBD = 0; //erreur précédente sur BD

float integraleX = 0; //erreur intégrale sur X 
float integraleY = 0; //erreur intégrale sur Y 
float integraleAC = 0; //erreur intégrale sur AC 
float integraleBD = 0; //erreur intégrale sur BD 

float integraleVitX = 0; //erreur intégrale sur la vitesse X
float integraleVitY = 0; //erreur intégrale sur la vitesse Y

//position souhaitée
int cibleX =  747;      //986; //1127;       //856
int cibleY =  306;      //476; //742;        //391

//inclinaison souhaitée
int cibleAC = 0;    
int cibleBD = 0;

float cibleVitX = 0; //vitesse cible sur X
float cibleVitY = 0; //vitesse cible sur Y


float forceA = 0.00;
float forceB = 0.00;
float forceC = 0.00;
float forceD = 0.00;

int signe(float x) {
    return (x > 0) - (x < 0); // Renvoie +1, -1 ou 0
}

float forceAccoup = 0.5; //force d'accoup
float seuilDistanceCentre = 100; //seuil de distance pour le passage à l'accoup

//////////////////////////////////////////////////////////
//                    Passage à 0                       //
//////////////////////////////////////////////////////////

#define DetectionPassageZero 21

//////////////////////////////////////////////////////////
//                       timers                         //
//////////////////////////////////////////////////////////

esp_timer_handle_t timerA, timerB, timerC, timerD; //timer pour le passage à zéro

// delay en µs
int delayA = 5000;
int delayB = 5000;
int delayC = 5000;
int delayD = 5000;

//////////////////////////////////////////////////////////

void recupTab(int valA[], int valB[], int fonctA, int fonctB) {
    for (int i = TAILLE_TAB - 1; i > 0; i--) {
        valA[i] = valA[i - 1];
        valB[i] = valB[i - 1];
    }
    valA[0] = fonctA;
    valB[0] = fonctB;
}

//////////////////////////////////////////////////////////
//                       lissage                        //
//////////////////////////////////////////////////////////

//nos valeurs varient trop donc pas de précision, il faudrait lisser les valeurs tout en prenant en compte le deplacement de la bille
int lissageX = 0;
int lissageY = 0;

int echantillonPos = 7; //nombre d'échantillons à lisser
int echantillonVit = 6; //nombre d'échantillons à lisser

float alpha_pos = 0.5; // Coefficient de lissage
float alpha_vit = 0.7; // Coefficient de lissage

void lissageVal(){

    //lissage basique
    /*
    for (int i = 0; i < echantillonPos; i++) {
        lissageX += tabX[i];
        lissageY += tabY[i];
    }
    lissageX = lissageX / (echantillonPos+1);
    lissageY = lissageY / (echantillonPos+1);
    */
    
    //lissage par filtre exponentiel

    lissageX = (1 - alpha_pos) * lissageX + alpha_pos * tabX[0];
    lissageY = (1 - alpha_pos) * lissageY + alpha_pos * tabY[0];
    /*
    */
}

void lissageVitesse(){
    /*
    for (int i = 0; i < echantillonVit; i++) {
        vitesseX += tabVitesseX[i];
        vitesseY += tabVitesseY[i];
    }
    vitesseX = vitesseX / (echantillonVit+1);
    vitesseY = vitesseY / (echantillonVit+1);
    */

    //lissage par filtre exponentiel

    vitesseX = (1 - alpha_vit) * vitesseX + alpha_vit * tabVitesseX[0];
    vitesseY = (1 - alpha_vit) * vitesseY + alpha_vit * tabVitesseY[0];

}

#endif // base_hpp
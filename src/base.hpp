#ifndef base_hpp
#define base_hpp

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include <Ticker.h>

// definition du tableau
#define TAILLE_TAB 20 
// definition du temps
#define temps 250 //temps en ms

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

int X = 0;
int Y = 0;

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

#define frequence 5000

//calculés suivant le besoin
int rapportCycliqueA = 230;
int rapportCycliqueB = 230;
int rapportCycliqueC = 230;
int rapportCycliqueD = 230;

//////////////////////////////////////////////////////////
//                         PID                          //
//////////////////////////////////////////////////////////

float Kp_pos = 0.03, Kp_vit = 0.03; //coefficient proportionnel (vitesse de réponse)
float Ki_pos = 0.001, Ki_vit = 0.0005; //coefficient intégral (précision)
float Kd_pos = 0.03, Kd_vit = 0.03; //coefficient dérivé        (stabilité)

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
int cibleX = 200;
int cibleY = 150;

//inclinaison souhaitée
int cibleAC = 0;    
int cibleBD = 0;

float cibleVitX = 0; //vitesse cible sur X
float cibleVitY = 0; //vitesse cible sur Y

//////////////////////////////////////////////////////////


void recupTab(int valA[], int valB[], int fonctA, int fonctB) {
    for (int i = TAILLE_TAB - 1; i > 0; i--) {
        valA[i] = valA[i - 1];
        valB[i] = valB[i - 1];
    }
    valA[0] = fonctA;
    valB[0] = fonctB;
}



#endif // base_hpp
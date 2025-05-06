#include "plateau.hpp"
#include "potentiometre.hpp"
#include "signalPWM.hpp"
#include "calcul.hpp"

#define tempsTest 500

/*
void Passage0(void* arg){
  while (1)
  {
    tempsSynchro = PassageAzero(); //appel de la fonction passage à zéro
  }
}
*/


volatile unsigned long lastInterruptTime = 0;
volatile unsigned long elapsedTime = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

unsigned long getTempsSynchro() {
  unsigned long temp;
  portENTER_CRITICAL(&mux);
  temp = elapsedTime;
  elapsedTime = 0;
  portEXIT_CRITICAL(&mux);
  return temp;
}

void IRAM_ATTR onZeroCrossing() {

  unsigned long currentTime = millis();
  
  portENTER_CRITICAL_ISR(&mux);
  tempsSynchro = currentTime - lastInterruptTime;
  lastInterruptTime = currentTime;
  portEXIT_CRITICAL_ISR(&mux);

  //envoyer le pwm ici pour qu'il soit synchro avec le passage à zéro/*
  /*
  */
  signalPWM(bobineA, rapportCycliqueA);
  signalPWM(bobineB, rapportCycliqueB);
  signalPWM(bobineC, rapportCycliqueC);
  signalPWM(bobineD, rapportCycliqueD);
  
  //debug
  /*
  Serial.println("Passage à zéro détecté !");
  Serial.print("tempsSynchro = ");
  Serial.println(tempsSynchro);
  */
}



void recupDonnees(void* arg)
{
  while(1){

  // plateau
    recupTab(tabX, tabY, lireX(),lireY());
    lissageVal();
    //afficherTableauXY();

  //potentiometre
    recupTab(tabAC, tabBD, lireAC(),lireBD());
    //afficherTableauPOT();
    
    //Serial.print("\nla tache fonctionne dans l'objet recupDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(tempsSynchro));
  };
}

void envoiDonnees(void* arg)
{
  while(1){

  //signalPWM
    signalPWM(bobineA, rapportCycliqueA);
    signalPWM(bobineB, rapportCycliqueB);
    signalPWM(bobineC, rapportCycliqueC);
    signalPWM(bobineD, rapportCycliqueD);

    //Serial.print("\nla tache fonctionne dans l'objet envoiDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(tempsSynchro));
  };
}

void calculDonnees(void* arg)
{
  /*
  PIDCoefficients pid_x_coeffs = {50, 5, 25}; // Coefficients PID pour le contrôle de la position X
  PIDCoefficients pid_y_coeffs = {50, 5, 25}; // Coefficients PID pour le contrôle de la position Y
  PlaqueState plaque_state = {bobineA, bobineB, bobineC, bobineD}; // État de la plaque (commandes PWM pour les actuateurs)
  BilleState bille_state = {lireX(), lireY()}; // État de la bille (position X et Y)
  
  BillePlaqueController controller(pid_x_coeffs, pid_y_coeffs, temps/1000.0);
  */

  while(1){
    //vitesse();
    /*  
    acceleration(); => a revoir (pas terminé
  )
    */ 
   
    calculPID();

    //controller.getBillePosition();
    //controller.control(cibleX, cibleY);
    
    //Serial.print("\nla tache fonctionne dans l'objet calculDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(tempsSynchro));

  };
}

void setup()
{
  Serial.begin(115200);

  outBobines();

  for (int i = 0; i < TAILLE_TAB; i++)
  {
    tabX[i] = 0;
    tabY[i] = 0;
  }

  //xTaskCreate(Passage0, "MaTachePassageAZero", 4096, NULL, 5, NULL);
  
  pinMode(DetectionPassageZero, INPUT);
  attachInterrupt(digitalPinToInterrupt(DetectionPassageZero), onZeroCrossing, RISING);


  xTaskCreate(recupDonnees, "MaTacheReception", 4096, NULL, 5, NULL);
  xTaskCreate(calculDonnees, "MaTacheCalcul", 4096, NULL, 5, NULL);
  //xTaskCreate(envoiDonnees, "MaTacheEnvoi", 4096, NULL, 5, NULL);  => du coup plus obligé de lappeler si on envoie dans le zero crossing ???

}

void loop()
{
  /*
  //A
  rapportCycliqueA = 50;
  rapportCycliqueB = 255;
  rapportCycliqueC = 255;
  rapportCycliqueD = 255;
  delay(tempsSynchro);
  
  //B
  rapportCycliqueA = 255;
  rapportCycliqueB = 50;
  rapportCycliqueC = 255;
  rapportCycliqueD = 255;
  delay(tempsSynchro);
  
  //C
  rapportCycliqueA = 255;
  rapportCycliqueB = 255;
  rapportCycliqueC = 50;
  rapportCycliqueD = 255;
  delay(tempsSynchro);
  
  //D
  rapportCycliqueA = 255;
  rapportCycliqueB = 255;
  rapportCycliqueC = 255;
  rapportCycliqueD = 50;
  delay(tempsSynchro);
  */

  delay(1000);
  //Serial.println(tempsSynchro);

}

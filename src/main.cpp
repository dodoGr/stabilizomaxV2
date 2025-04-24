#include "plateau.hpp"
#include "potentiometre.hpp"
#include "signalPWM.hpp"
#include "calcul.hpp"

#define tempsTest 500

void recupDonnees(void* arg)
{
  while(1){

  // plateau
    recupTab(tabX, tabY, lireX(),lireY());
    afficherTableauXY();

  //potentiometre
    recupTab(tabAC, tabBD, lireAC(),lireBD());
    //afficherTableauPOT();
    
    Serial.print("\nla tache fonctionne dans l'objet recupDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(temps));
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

    Serial.print("\nla tache fonctionne dans l'objet envoiDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(temps));
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
    
    Serial.print("\nla tache fonctionne dans l'objet calculDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(temps));

  };
}

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < TAILLE_TAB; i++)
  {
    tabX[i] = 0;
    tabY[i] = 0;
  }

  xTaskCreate(recupDonnees, "MaTacheReception", 4096, NULL, 5, NULL);
  xTaskCreate(envoiDonnees, "MaTacheEnvoi", 4096, NULL, 5, NULL);
  xTaskCreate(calculDonnees, "MaTacheCalcul", 4096, NULL, 5, NULL);

}

void loop()
{
  /*
  //A
  rapportCycliqueA = 50;
  rapportCycliqueB = 255;
  rapportCycliqueC = 255;
  rapportCycliqueD = 255;
  delay(tempsTest);
  
  //B
  rapportCycliqueA = 255;
  rapportCycliqueB = 50;
  rapportCycliqueC = 255;
  rapportCycliqueD = 255;
  delay(tempsTest);
  
  //C
  rapportCycliqueA = 255;
  rapportCycliqueB = 255;
  rapportCycliqueC = 50;
  rapportCycliqueD = 255;
  delay(tempsTest);
  
  //D
  rapportCycliqueA = 255;
  rapportCycliqueB = 255;
  rapportCycliqueC = 255;
  rapportCycliqueD = 50;
  delay(tempsTest);
  */
  
  //delay(temps);

}

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
  while(1){
    vitesse();
    /*  
    acceleration(); => a revoir (pas terminé)
    */ 
    //test_equilibrage();

    calculPID();
    Serial.print("\nla tache fonctionne dans l'objet calculDonnees\n");
    /*
    Serial.println(rapportCycliqueA);
    Serial.println(rapportCycliqueB);
    Serial.println(rapportCycliqueC);
    Serial.println(rapportCycliqueD);
    */
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

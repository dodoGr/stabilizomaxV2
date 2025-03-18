#include "plateau.hpp"
#include "potentiometre.hpp"
#include "signalPWM.hpp"

void recupDonnees(void* arg)
{
  while(1){

  // plateau
    recupTab();
    afficherTableau();

  //potentiometre
    afficherBD();
    afficherAC();
    
    Serial.print("\nla tache fonctionne dans l'objet recupDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(temps));
  };
}

void envoiDonnees(void* arg)
{
  while(1){

    /*
    //signalPWM
    signalPWM(bobineC, rapportCycliqueC);
    signalPWM(bobineD, rapportCycliqueD);
    */

    //pout les tests
    cycle1 = analogRead(potTest1);
    cycle1 = map(cycle1, 0, 4095, 0, 255);
    cycle2 = analogRead(potTest2);
    cycle2 = map(cycle2, 0, 4095, 0, 255);
    //

    signalPWM(bobineA, cycle1);
    signalPWM(bobineB, cycle2);

    Serial.print("\nla tache fonctionne dans l'objet envoiDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(temps));
  };
}

void calculDonnees(void* arg)
{
  while(1){
    /*
    vitesse();
    //acceleration(); => a revoir (pas terminé)
    */

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

  delay(temps);

}

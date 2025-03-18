#include "plateau.hpp"
#include "potentiometre.hpp"
#include "signalPWM.hpp"

void interruptTimer() {
  calculVitesse = true;
  calculAcceleration = true;
}

void run(void* arg)
{
  while(1){
      Serial.print("la tache fonctionne dans l'objet plateau\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
  };
}

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < TAILLE_TAB; i++) {
        tabX[i] = 0;
        tabY[i] = 0;
    }
    recupTab(); 
   
    //initSite();
    
    xTaskCreate(run, "MyTask", 4096, NULL, 5, NULL);
}

int cycle1; 
int cycle2 = analogRead(potTest2);

void loop() {

  
  //plateau
  lireX();
  lireY();
  /*
  recupTab();
  //afficherTableau();
  vitesse();
  //acceleration(); => a revoir (pas terminé)
  
  //potentiometre
  afficherBD();
  afficherAC();
  
  //signalPWM
  signalPWM(bobineC, rapportCycliqueC);
  signalPWM(bobineD, rapportCycliqueD);
  */
 cycle1 = analogRead(potTest1);
 cycle1 = map(cycle1, 0, 4095, 0, 255);

  
  cycle2 = map(cycle2, 0, 4095, 0, 255);

 signalPWM(bobineA, cycle1);
 signalPWM(bobineB, cycle2);
  
  delay(temps);

  //site web
  //server.handleClient();

}


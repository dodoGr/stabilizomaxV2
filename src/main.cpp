#include "plateau.hpp"
#include "potentiometre.hpp"
#include "signalPWM.hpp"
#include "calcul.hpp"

#define tempsTest 500

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
  signalPWM(bobineA, rapportCycliqueA);
  signalPWM(bobineB, rapportCycliqueB);
  signalPWM(bobineC, rapportCycliqueC);
  signalPWM(bobineD, rapportCycliqueD);
  */

  delayA = map(rapportCycliqueA, 0, 255, 4000, 7500); 
  delayB = map(rapportCycliqueB, 0, 255, 4000, 7500); 
  delayC = map(rapportCycliqueC, 0, 255, 4000, 7500); 
  delayD = map(rapportCycliqueD, 0, 255, 4000, 7500); 

  esp_timer_stop(timerA);
  esp_timer_start_once(timerA, delayA);
  esp_timer_stop(timerB);
  esp_timer_start_once(timerB, delayB);
  esp_timer_stop(timerC);
  esp_timer_start_once(timerC, delayC);
  esp_timer_stop(timerD);
  esp_timer_start_once(timerD, delayD);

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
    //afficherInfoXY();

  //potentiometre
    //recupTab(tabAC, tabBD, lireAC(),lireBD());
    //afficherTableauPOT();
    
    //Serial.print("\nla tache fonctionne dans l'objet recupDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(tempsSynchro/2));
  };
}

void envoiDonnees(void* arg)
{
  while(1){

  //signalPWM
    //signalPWM(bobineA, rapportCycliqueA);
    //signalPWM(bobineB, rapportCycliqueB);
    //signalPWM(bobineC, rapportCycliqueC);
    //signalPWM(bobineD, rapportCycliqueD);

    esp_timer_stop(timerA);
    esp_timer_start_once(timerA, delayA);
    esp_timer_stop(timerB);
    esp_timer_start_once(timerB, delayB);
    esp_timer_stop(timerC);
    esp_timer_start_once(timerC, delayC);
    esp_timer_stop(timerD);
    esp_timer_start_once(timerD, delayD);

    //Serial.print("\nla tache fonctionne dans l'objet envoiDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(tempsSynchro));
  };
}

void calculDonnees(void* arg){
  while(1){
    calculPID();

    //Serial.print("\nla tache fonctionne dans l'objet calculDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(tempsSynchro));
  };
}


void initTimers()
{
  esp_timer_create_args_t timerA_args = {
      .callback = &envoieA,
      .name = "TimerA",
  };
  esp_timer_create(&timerA_args, &timerA);

  esp_timer_create_args_t timerB_args = {
      .callback = &envoieB,
      .name = "TimerB",
  };
  esp_timer_create(&timerB_args, &timerB);

  esp_timer_create_args_t timerC_args = {
      .callback = &envoieC,
      .name = "TimerC",
  };
  esp_timer_create(&timerC_args, &timerC);

  esp_timer_create_args_t timerD_args = {
      .callback = &envoieD,
      .name = "TimerD",
  };
  esp_timer_create(&timerD_args, &timerD);
}


void setup()
{
  Serial.begin(115200);

  outBobines();
  initTimers();
  
  for (int i = 0; i < TAILLE_TAB; i++)
  {
    tabX[i] = 0;
    tabY[i] = 0;
  }
  
  pinMode(DetectionPassageZero, INPUT);
  attachInterrupt(digitalPinToInterrupt(DetectionPassageZero), onZeroCrossing, RISING && tempsSynchro > 2); 

  xTaskCreate(recupDonnees, "MaTacheReception", 4096, NULL, 5, NULL);
  xTaskCreate(calculDonnees, "MaTacheCalcul", 4096, NULL, 5, NULL);
  //xTaskCreate(envoiDonnees, "MaTacheEnvoi", 4096, NULL, 5, NULL);  //=> plus obligé de lappeler si on envoie dans le zero crossing

  esp_timer_start_once(timerA, delayA);
  esp_timer_start_once(timerB, delayB);
  esp_timer_start_once(timerC, delayC);
  esp_timer_start_once(timerD, delayD);
}

void loop()
{
   
  //Serial.println(tempsSynchro);
  delay(1000);
  

}

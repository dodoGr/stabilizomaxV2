#include "plateau.hpp"
#include "potentiometre.hpp"
#include "signalPWM.hpp"
#include "calcul.hpp"
#include "transmission.hpp"

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


    if(Flag_ActivEchantPos){
      lissageVal();}
    if(Flag_ActivEchantVit){
      lissageVitesse();}

    filtreExpPosition();
    filtreExpVitesse();
    //afficherInfoXY();

  //potentiometre
    recupTab(tabAC, tabBD, lireAC(),lireBD());
    //afficherTableauPOT();
    
    //Serial.print("\nla tache fonctionne dans l'objet recupDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(4));
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
    vTaskDelay(pdMS_TO_TICKS(8));
  };
}

void calculDonnees(void* arg){
  while(1){
    calculPID();
    vitesse();

    //Serial.print("\nla tache fonctionne dans l'objet calculDonnees\n");
    vTaskDelay(pdMS_TO_TICKS(8));
  };
}

void VisuKoeffs(void* arg)
{
  while(1){
    Serial.print("Kp = ");
    Serial.print(Kp_pos);
    Serial.print("\tKi = ");
    Serial.print(Ki_pos);
    Serial.print("\tKd = ");
    Serial.println(Kd_pos);
    vTaskDelay(pdMS_TO_TICKS(1000));
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

void lecturePIDSerie() {
  if (Serial.available()) {
    String ligne = Serial.readStringUntil('\n');
    if (ligne.startsWith("PID:")) {
      float kp_p, ki_p, kd_p;
      float kp_v, ki_v, kd_v;
      float kp_i, ki_i, kd_i;
      float alpha_p, alpha_v;
      int lissage_pos, lissage_vit;
      int nb_vals_pos, nb_vals_vit;
      float forceSupA, forceSupB, forceSupC, forceSupD;
      float inclin;
      
      int n = sscanf(ligne.c_str(), "PID:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f",
      &kp_p, &ki_p, &kd_p,
      &kp_v, &ki_v, &kd_v,
      &kp_i, &ki_i, &kd_i,
      &alpha_p, &alpha_v,
      &lissage_pos, &lissage_vit,
      &nb_vals_pos, &nb_vals_vit,
      &forceSupA, &forceSupB, &forceSupC, &forceSupD,
      &inclin);

      if (n == 20) {
        Kp_pos = constrain(kp_p, 0.0, 10.0);
        Ki_pos = constrain(ki_p, 0.0, 1.0);
        Kd_pos = constrain(kd_p, 0.0, 300.0);
      
        Kp_vit = constrain(kp_v, 0.0, 10.0);
        Ki_vit = constrain(ki_v, 0.0, 1.0);
        Kd_vit = constrain(kd_v, 0.0, 300.0);

        Kp_inclin = constrain(kp_i, 0.0, 10.0);
        Ki_inclin = constrain(ki_i, 0.0, 1.0);
        Kd_inclin = constrain(kd_i, 0.0, 300.0);
      
        alpha_pos = constrain(alpha_p, 0.0, 1.0);
        alpha_vit = constrain(alpha_v, 0.0, 1.0);
      
        Flag_ActivEchantPos = lissage_pos;
        Flag_ActivEchantVit = lissage_vit;
      
        echantillonPos = constrain(nb_vals_pos, 1, 10);
        echantillonVit = constrain(nb_vals_vit, 1, 10);
      
        sliderA = forceSupA;
        sliderB = forceSupB;
        sliderC = forceSupC;
        sliderD = forceSupD;

        facteurInclinaison = inclin;
      }

        else {
        Serial.println("ERREUR: Format invalide");
      }
    }
  }
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
  //xTaskCreate(VisuKoeffs, "MaTacheKoeffs", 4096, NULL, 5, NULL);
  //xTaskCreate(envoiDonnees, "MaTacheEnvoi", 4096, NULL, 5, NULL);  //=> plus obligé de lappeler si on envoie dans le zero crossing

  esp_timer_start_once(timerA, delayA);
  esp_timer_start_once(timerB, delayB);
  esp_timer_start_once(timerC, delayC);
  esp_timer_start_once(timerD, delayD);


  //transmission 

  nvs_flash_init();
    esp_log_level_set("wifi", ESP_LOG_INFO);

    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        printf("Erreur NVS init : 0x%x\n", ret);
    }

    wifi_init();
    esp_random();
    xTaskCreate(udp_receive_task, "udp_receive_task", 4096, NULL, 5, NULL);

    xTaskCreate(&udp_client_task, "udp_client_task", 4096, NULL, 5, NULL);

}

void loop()
{
  lecturePIDSerie();

  //Serial.println(tempsSynchro);
  delay(10);
  

}


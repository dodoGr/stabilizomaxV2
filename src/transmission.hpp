#ifndef transmission_hpp
#define transmission_hpp

#include "base.hpp"

//initialise et connecte l'esp32 au wifi
void wifi_init() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);

    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    // Tentative de connexion
    printf("Tentative de connexion au WiFi \"%s\"...\n", WIFI_SSID);
    esp_err_t err = esp_wifi_connect();

    if (err != ESP_OK) {
        printf("Erreur de connexion WiFi : 0x%x (%s)\n", err, esp_err_to_name(err));
    }

    // Attente de l'adresse IP
    esp_netif_ip_info_t ip_info;
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // attendre 1 seconde
        if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info) == ESP_OK &&
            ip_info.ip.addr != 0) {

            // Afficher l'adresse IP
            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
            printf("Connecté au WiFi. Adresse IP : %s\n", ip_str);
            break;
        } else {
            printf("Connexion en cours...\n");
        }

        // Si la connexion échoue encore après quelques tentatives, attendons avant de réessayer
        if (err != ESP_OK) {
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Pause de 5 secondes avant de tenter de se reconnecter
            esp_wifi_connect();
        }
    }
}

void udp_receive_task(void *pvParameters) {
    char rx_buffer[5];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    char *Ref1 = "ON\0";
    char *Ref2 = "OFF\0";
    char *Ref3 = "A\0";
    char *Ref4 = "B\0";
    char *Ref5 = "C\0";
    char *Ref6 = "D\0";

   

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(UDP_LOCAL_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            printf("Unable to create socket: errno %d\n", errno);
            break;
        }
        printf("Socket created\n");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            printf("Socket unable to bind: errno %d\n", errno);
            break;
        }
        printf("Socket bound, port %d\n", UDP_LOCAL_PORT);

        while (1) {
            printf("Waiting for data\n");
            struct sockaddr_in source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0) {
                printf("recvfrom failed: errno %d\n", errno);
                break;
            } 
            
            else {
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                rx_buffer[len] = 0; 
                printf("Received %d bytes from %s: %s\n", len, addr_str, rx_buffer);


                for(int i = 0; i < 2; i++) {

                    if(rx_buffer[i] == Ref1[i] && i == 1) {
                        printf("premiere coammnde recu\n");
                        gpio_set_level(LED_GPIO, 1);  // Allumer la LED
                        commandeTransmission = 1;
                    }
                }
                

                for(int i = 0; i < 3; i++) {

                     if(rx_buffer[i] == Ref2[i] && i == 2) {
                        printf("deuxieme coammnde recu\n");
                        gpio_set_level(LED_GPIO, 0);  // Allumer la LED
                        commandeTransmission = 2;
                    }
                }

                for(int i = 0; i < 1; i++) {

                     if(rx_buffer[i] == Ref3[i] && i == 0) {
                        printf("troisieme coammnde recu\n");
                        commandeTransmission = 3;
                    }
                }

                for(int i = 0; i < 1; i++) {

                     if(rx_buffer[i] == Ref4[i] && i == 0) {
                        printf("quatrieme coammnde recu\n");
                        commandeTransmission = 4;
                    }
                }

                for(int i = 0; i < 1; i++) {

                     if(rx_buffer[i] == Ref5[i] && i == 0) {
                        printf("cinquieme coammnde recu\n");
                        commandeTransmission = 5;
                    }
                }

                for(int i = 0; i < 1; i++) {

                     if(rx_buffer[i] == Ref6[i] && i == 0) {
                        printf("sixieme coammnde recu\n");
                        commandeTransmission = 6;
                    }
                }

                if (rx_buffer[0] == 'x') {
                    x_value = atoi(&rx_buffer[1]); // Convertir la partie numérique en entier
                    printf("Valeur de x reçue : %d\n", x_value);
                    commandeTransmission = 7;
                } else if (rx_buffer[0] == 'y') {
                    y_value = atoi(&rx_buffer[1]); // Convertir la partie numérique en entier
                    printf("Valeur de y reçue : %d\n", y_value);
                    commandeTransmission = 7;
                    
                } else {
                    printf("Message non reconnu : %s\n", rx_buffer);
                }
                
            }

        }

        if (sock != -1) {
            printf("Shutting down socket and restarting...\n");
            shutdown(sock, 0);
            close(sock);
        }

            
        
    }
}

void udp_client_task(void *pvParameters) {
    struct sockaddr_in server_addr; // creer une structure pour stocker l'addresse du serveur UDP
    server_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP); // convertit l'adresse IP du serveur en un format lisible par le socket 
    server_addr.sin_family = AF_INET; // utilise le protocole IPV4
    server_addr.sin_port = htons(UDP_SERVER_PORT); // deffinit le port de destination (ici 12345)

    int sock = socket(AF_INET, SOCK_DGRAM, 0); // creer un socket UDP
    if (sock < 0) {
        printf("Erreur socket UDP\n"); // verifie si la création a échouer, il affiche un message et supprime la tache si c'est la cas 
        vTaskDelete(NULL);
    }


while (1) {
    int randomX = esp_random() % 100; // Nombre aléatoire entre 0 et 99
    int randomY = esp_random() % 100;

    char positionX[10];  // Buffer pour stocker x 
    char positionY[10];  // Buffer pour stocker y

    sprintf(positionX, "x%d", randomX);  // Construit la chaîne x
    sprintf(positionY, "y%d", randomY);  // Construit la chaîne y

    // Envoi de X
    sendto(sock, positionX, strlen(positionX), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
    printf("Message envoyé : %s à : %s\n", positionX, inet_ntoa(server_addr.sin_addr));
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Envoi de Y
    sendto(sock, positionY, strlen(positionY), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
    printf("Message envoyé : %s\n", positionY);
    printf("\n");

    printf("valeur de x recu : %d\n", x_value);
    printf("valeur de y recu : %d\n", y_value);
    printf("etat de la commandeTransmission : %d\n", commandeTransmission);

    vTaskDelay(pdMS_TO_TICKS(500));
}


    close(sock);  // Fermer le socket (en théorie, cela ne sera jamais atteint car la boucle est infinie)
    vTaskDelete(NULL);
}


#endif // transmission_hpp
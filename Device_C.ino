/*Senaryoda verilen C cihazının kodlarıdır. C cihazı çift yönlü olarak haberleşmektedir.C cihazı Yayıncı modunda  çalışmaktadır. Default olarak mac adresi girilmemiştir
Ortamdaki diğer esp now özelliği barındıran cihazlara bilgi göndermeye hazırdır.Senaryoya başlarken hiçbir mac adresini bilmez.

C CİHAZININ KODLARI B CİHAZI İLE NEREDEYSE AYNIDIR BU YÜZDEN AÇIKLAMALAR B CİHAZININ KODUNDADIR*/
#include <esp_now.h>
#include <WiFi.h>

#define BUTTON_PIN_1 27  
#define LED_PIN_1    12  

int led_state_1 = LOW;   
int button_state_1;       
int button_state_A = 0;

// ****************A-MAC:0x30, 0xC6, 0xF7, 0x05, 0x70, 0xEC******


uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x05, 0x70, 0xEC};


// float veri1 = 32; TEST VERİSİ C İCİN KULLANILMADI

int gidenledC_A;



float gelenveri1;
int gelenledA_C;
int gelendeletebuton;


String success;


typedef struct struct_message {
  int device_no; // kart numarası
  float veri_x;

  int LedA1;
  int LedA2;

  int ButonA1;
  int ButonA2;
  int ButonA3;

  int ButonB1;

  int ButonC1;

} struct_message;


struct_message gidecekveri;


struct_message gelenveri;

esp_now_peer_info_t peerInfo;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);



  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&gelenveri, incomingData, sizeof(gelenveri));

  gelenledA_C = gelenveri.ButonA2;
  gelendeletebuton = gelenveri.ButonA3;


  if (gelenledA_C == 1) {
    digitalWrite(LED_PIN_1, HIGH);
  }
  else {
    digitalWrite(LED_PIN_1, LOW);

  }

  char macStr2[18];
  snprintf(macStr2, sizeof(macStr2), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr2);
  Serial.print("Last Packet Recv Data: "); Serial.println(*incomingData);
  Serial.println("");

  Serial.print("Bytes received: ");
  Serial.println(len);



}

void setup() {
  
  Serial.begin(115200);
  pinMode(BUTTON_PIN_1, INPUT); 
  pinMode(LED_PIN_1, OUTPUT);          

  button_state_1 = digitalRead(BUTTON_PIN_1);

 
  delay(5000);
  WiFi.mode(WIFI_STA);
  WiFi.macAddress();
  WiFi.disconnect();

  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  
  esp_now_register_send_cb(OnDataSent);

  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
 
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  getReadings();

  
  gidecekveri.LedA2 = gidenledC_A;
  gidecekveri.ButonC1 = button_state_1;
  gidecekveri.device_no = 3;


  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gidecekveri, sizeof(gidecekveri));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  updateDisplay();

  if (gelendeletebuton == 1) {

  ESP.restart();

  }

  delay(500);

}

void getReadings() {
  if (gelendeletebuton == 0){
    gidenledC_A = HIGH;
    }
    
  else {gidenledC_A = LOW;}
  
  gidenledC_A = HIGH;
  button_state_1 = digitalRead(BUTTON_PIN_1);
}

void updateDisplay() {

  Serial.print("buton-A-dangelen: ");
  Serial.print(gelenledA_C);
  Serial.println();
}

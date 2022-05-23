/*   
Senaryoda verilen A cihazının kodlarıdır.Çift yönlü olarak çalışmaktadır.Birden fazla cihaza veri gönderir ve birden fazla cihazdan veri alır.
A cihazı hiçbir mac adresi bilmeden senaryoya başlar.A cihazına veri gönderen cihazların mac adreslerini A cihazı peer olarak ekler ve veri 
göndereceği cihazların mac adresini öğrenmiş olur. A cihazının haberleşmeye başlaması için ilk olarak tetiklenmesi gerekir. Bu tetiklenme de A cihazının veri almasıyla
gerçekleşir.
*/

#include <esp_now.h>
#include <WiFi.h>

#define NUMSLAVES 20 //A cihzına Maksimum eklenebilecek cihaz sayısı
esp_now_peer_info_t slaves[NUMSLAVES] = {}; //A cihazına bağlanacak cihazların bilgilerine ulaşmak için esp_now_peer_info_t yapısı kullanılacaktır.
esp_now_peer_num_t devices_num; //A ya bağlı olan cihazların sayısını görmek için kullanacağımız yapı.
int SlaveCnt = 0;
#define PRINTSCANRESULTS 0

#define BUTTON_PIN_1 26  // BUTON bağlanan GPIO
#define LED_PIN_1    12  // LED bağlanan GPIO

#define BUTTON_PIN_2 27  // BUTON bağlanan GPIO
#define LED_PIN_2    13  // LED bağlanan GPIO

#define BUTTON_PIN_3 14  // BUTON bağlanan GPIO
#define LED_PIN_3    32  // LED bağlanan GPIO

#define LED_PIN_4    33  // LED bağlanan GPIO


int led_state_1 = LOW;    // LED'in mevcut durumu
int led_state_2 = LOW;    // LED'in mevcut durumu
int led_state_3 = LOW;    // LED'in mevcut durumu
int led_state_4 = LOW;    // LED'in mevcut durumu

int button_state_1;       // BUTON1 mevcut durum
int button_state_2;       // BUTON2 mevcut durum
int button_state_3;       // BUTON3 mevcut durum


float veri1 = 10; // A cihazına ait veridir.1 arttırılarak diğer cihazlara anlık olarak gönderilecektir.


float gelenveri1; // diğer cihazdan gelen veri1 değişkenini saklamak için oluşturulan gelenveri1 değişkenidir.


String success; // Veri gönderme başarılıysa depolanacak değişken

//Veri göndermek için gereken yapı oluşturulmuştur.
//Haberleşecek tüm cihazların yapı içindeki değişkenleri aynı olmalıdır
typedef struct struct_message {
  /*Bu yapıdaki değişkenleri birer taşıma yolu olarak düşünebilirsiniz.Yapıdaki Değişkenler(yollar) her cihazda aynı isimdedir.
   Cihazın kendi içinde tanımladığınız değişkenleri bu ortak yollarda taşıyabilirsiniz */
  
  int device_no; // cihaz numarası: A, birden fazla cihazdan veri almaktadır.Verilerin çakışmaması için cihazlara numara verilmelidir.
  float veri_x;  // veri1 değişkenini taşımak için oluşturulmuştur. Tüm cihazlara test amaçlı veri1 koyulmuş ve sürekli 1 arttırılmıştır. Haberleşme gözlenmiştir.

  int LedA1;  // A daki 1 numaralı Led için.
  int LedA2;  // A daki 2 numaralı Led için

  int ButonA1; 
  int ButonA2;
  int ButonA3;

  int ButonB1;

  int ButonC1;

} struct_message;

// gönderilecek veriler için struct_message yapısı tipinde gidecekveri yapısı oluşturulmuştur.
struct_message gidecekveri;

// alınacak veriler için struct_message yapısı tipinde gidecekveri yapısı oluşturulmuştur.
struct_message gelenveri;



// Veri gönderildiği an bu fonksiyon işleyecektir.Bu fonksiyon içindeki mac adres gönderim yapıldıktan sonra karşı taraftan gelen karşı tarafın mac adresidir.
// Mac adres yankısı olarak düşünebilirsiniz.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");

 Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TESLİMAT BAŞARILI" : "TESLİMAT BAŞARISIZ");
  if (status == 0) {
    success = "TESLİMAT BAŞARILI :)";
  }
  else {
    success = "TESLİMAT BAŞARISIZ :(";
  }
}

// Veri alındığında bu fonksiyon işleyecektir. Bu fonksiyon içerisinde bulunan mac değişkeni veri gönderen cihazın mac değişkenidir.
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&gelenveri, incomingData, sizeof(gelenveri));
  
  char macStr2[18]; //veri gönderen cihazın yani mac değişkeninin yazılacağı değişken
  snprintf(macStr2, sizeof(macStr2), "%02x:%02x:%02x:%02x:%02x:%02x",  // veri gönderen cihazın mac bilgisi alınmış ve char tipindeki macStr2 değişkenine yazılmıştır.
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr2);
  Serial.print("Last Packet Recv Data: "); Serial.println(*incomingData);
  Serial.println("");

  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  /*veri gönderen cihazın mac adresini bir şekilde stringe çevirmemiz gerekiyor başka şekilde peer listemize istediğimiz mac adresli cihazı ekleyemiyoruz
  bu yüzden mac adresimizi önce char olarak yazdırdık sonra stringe çevirdik sonrasonda da slaves yapımızın içindeki peer_addr değişkenine kaydettik*/
  String macstring;
  int gelenmac[6];
  macstring = String(macStr2);
  if ( 6 == sscanf(macstring.c_str(), "%x:%x:%x:%x:%x:%x",  &gelenmac[0], &gelenmac[1], &gelenmac[2], &gelenmac[3], &gelenmac[4], &gelenmac[5] ) ) {
    for (int ii = 0; ii < 6; ++ii ) {
       slaves[SlaveCnt].peer_addr[ii] = gelenmac[ii];
   //  slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
    }
  }
  slaves[SlaveCnt].channel = 0; // pick a channel
  slaves[SlaveCnt].encrypt = 0; // no encryption
  SlaveCnt++;

  Serial.print("Bytes received: ");
  Serial.println(len);

  if (gelenveri.device_no == 2) {

    gelenveri1 = gelenveri.veri_x;

    led_state_1 = gelenveri.LedA1;
    digitalWrite(LED_PIN_1, led_state_1);

    led_state_3 = gelenveri.ButonB1;
    digitalWrite(LED_PIN_3, led_state_3);


  }

  if (gelenveri.device_no == 3) {

    led_state_2 = gelenveri.LedA2;
    digitalWrite(LED_PIN_2, led_state_2);

    led_state_4 = gelenveri.ButonC1;
    digitalWrite(LED_PIN_4, led_state_4);

    
  }


  updateDisplay();

}
void setup() {

  Serial.begin(115200);

  pinMode(BUTTON_PIN_1, INPUT); // Buton giriş olarak ayarlandı.
  pinMode(LED_PIN_1, OUTPUT);          //  LED çıkış olarak ayarlandı

  button_state_1 = digitalRead(BUTTON_PIN_1);

  pinMode(BUTTON_PIN_2, INPUT); // Buton giriş olarak ayarlandı.
  pinMode(LED_PIN_2, OUTPUT);          //  LED çıkış olarak ayarlandı

  button_state_2 = digitalRead(BUTTON_PIN_2);

  pinMode(BUTTON_PIN_3, INPUT); // Buton giriş olarak ayarlandı.  
  pinMode(LED_PIN_3, OUTPUT);          //  LED çıkış olarak ayarlandı

  button_state_3 = digitalRead(BUTTON_PIN_3);


  pinMode(LED_PIN_4, OUTPUT);          //  LED çıkış olarak ayarlandı




  // Cihaz ilk olarak istasyon moduna alındı.Mac adresi sorgulandı.Wifi devre dışı bırakıldı.
  WiFi.mode(WIFI_STA);
  WiFi.macAddress();
  WiFi.disconnect();

  // ESP-NOW özelliği başlatıldı.
  InitESPNow();

  esp_now_register_send_cb(OnDataSent); // Veri gönderim işlemi sağlandığında ondatasent fonksyionunun çalışacağı belirtildi

  esp_now_register_recv_cb(OnDataRecv); // Veri alma işlemi sağlandığında ondatarecv fonksiyonunun çalışacağı belirtildi
}

void loop() {
  if (SlaveCnt > 0) { //Eklenen cihaz sayısı 0 dan fazla olduğunda çalışacak olan loop döngü şartı.

    manageSlave();

    sendData();

  }

  else {

  }
//  delay(500);
}

void getReadings() { //A cihazından diğer cihazlara gönderilecek olanv verilerin güncellemesi ve kontrolü burada sağlanır.

  veri1++;
 
  button_state_1 = digitalRead(BUTTON_PIN_1);

  Serial.print("SendingButon1: ");
  Serial.println(button_state_1);

  button_state_2 = digitalRead(BUTTON_PIN_2);

  Serial.print("SendingButon2: ");
  Serial.println(button_state_2);

  button_state_3 = digitalRead(BUTTON_PIN_3);
  if(button_state_3 == 1){
  led_state_1=LOW; 
  digitalWrite(LED_PIN_1,LOW);
  led_state_2=LOW; 
  digitalWrite(LED_PIN_2,LOW);}
 
  Serial.print("SendingButon3: ");
  Serial.println(button_state_3);

}

void updateDisplay() {

  // Seri Monitörde Okumaları Görüntüleme
  Serial.println("INCOMING READINGS");
  Serial.print("gelenveri1: ");
  Serial.print(gelenveri.veri_x);
  devices_num.total_num = ESP_NOW_MAX_TOTAL_PEER_NUM;
  Serial.print("baglicihaz: ");
  Serial.print(devices_num.total_num);
  
  
  Serial.println();
}

void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(slaves[i].peer_addr);
      if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
        //   sendData();

      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&slaves[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}


void sendData() {

  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
          //  if (i == 0) { // print only for first slave
    Serial.print("Sending: ");
    Serial.println(veri1);

         //   }
    getReadings();

    gidecekveri.ButonA1 = button_state_1;
    gidecekveri.ButonA2 = button_state_2;
    gidecekveri.ButonA3 = button_state_3;

    gidecekveri.veri_x = veri1;

    esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &gidecekveri, sizeof(gidecekveri));
    Serial.println(button_state_1);

    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
    delay(100);
  }

}

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

/*Senaryoda verilen B cihazının kodlarıdır. B cihazı çift yönlü olarak haberleşmektedir.B cihazı Yayıncı modunda  çalışmaktadır. Default olarak mac adresi girilmemiştir
Ortamdaki diğer esp now özelliği barındıran cihazlara bilgi göndermeye hazırdır.Senaryoya başlarken hiçbir mac adresini bilmez.*/

#include <esp_now.h>
#include <WiFi.h>

#define BUTTON_PIN_1 35  // BUTON bağlanan GPIO
#define LED_PIN_1    27  // LED bağlanan GPIO

esp_now_peer_num_t devices_num; // Bağlı cihazların sayısını görmek için kullanacağımız yapı

int led_state_1 = LOW;    // LED State
int button_state_1;       // BUTON State

// Alıcı MAC adresi buraya yazlır
uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x05, 0x70, 0xEC}; //Burayı default olarak bırakırsak cihazımız yayıncı modunda çalışacaktır.

// B cihazına ait diğer cihazlara gönderilecek olan test verisi.
float veri1 = 80;


// diğer cihazlardan gelen test verisinin B cihazında saklanacağı değişken.
float gelenveri1;

int gelenledA_B; // A cihazında gelen B cihazındaki Led yakma bilgisinin saklanacağı B cihazına ait değişken.
int gelendeletebuton; // A cihazından gelen cihaz silme bilgisin saklanacağı değişken.

int gidenledB_A; //B cihazından A cihazına gidecek olan değişken
// Variable to store if sending data was successful
String success;

//Veri göndermek için gereken yapı oluşturulmuştur.
//Haberleşecek tüm cihazların yapı içindeki değişkenleri aynı olmalıdır
typedef struct struct_message {
  /*Bu yapıdaki değişkenleri birer taşıma yolu olarak düşünebilirsiniz.Yapıdaki Değişkenler(yollar) her cihazda aynı isimdedir.
   Cihazın kendi içinde tanımladığınız değişkenleri bu ortak yollarda taşıyabilirsiniz */
  int device_no; // cihaz numarası: A, birden fazla cihazdan veri almaktadır.Verilerin çakışmaması için cihazlara numara verilmelidir.
  float veri_x;

  int LedA1; // A daki 1 numaralı Led için.
  int LedA2; // A daki 2 numaralı Led için

  int ButonA1; //A daki 1 numaralı buton değerini taşıyacak olan yol olarak düşünebiliriz.
  int ButonA2; //A daki 2 numaralı buton değerini taşıyacak olan yol olarak düşünebiliriz.
  int ButonA3; //A daki 3 numaralı buton değerini taşıyacak olan yol olarak düşünebiliriz.

  int ButonB1; //B deki 1 numaralı buton değerini taşıyacak olan yol olarak düşünebiliriz.
 
  int ButonC1; //C deki 1 numaralı buton değerini taşıyacak olan yol olarak düşünebiliriz.

} struct_message;

// gönderilecek veriler için struct_message yapısı tipinde gidecekveri yapısı oluşturulmuştur.
struct_message gidecekveri;

// alınacak veriler için struct_message yapısı tipinde gidecekveri yapısı oluşturulmuştur.
struct_message gelenveri;

esp_now_peer_info_t peerInfo; //Bu yapı esp now kütüphanesi içerisinde hazır olan yapıdır.Yapı içerisinde çeşitli parametreler bulunmaktadır.

// Veri gönderimi yapıldığında bu fonksiyon işleyecektir.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  char macStr[18]; //Veri gönderilen mac adresini char tipine dönüştürdük.Bunu yapmamızın sebebi ileriki zamanlarda kolaylıkla mac adresini farklı görevlerde kullanabilmek.
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);

  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TESLİMAT BAŞARILI" : "TESLİMAT BAŞARISIZ");  //Gönderim işleminin gerçekleşip gerçekleşmediği sorgulanıyor.
  if (status == 0) {
    success = "TESLİMAT BAŞARILI :)";
  }
  else {
    success = "TESLİMAT BAŞARISIZ :(";
  }
}

// Veri alımında çalışacak olan fonksiyon.
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&gelenveri, incomingData, sizeof(gelenveri));
  /*Bu kısımda B cihazına ait olan değişkenlere struck message yapısı ile taşınan veriler aktarılmıştır*/
  gelenveri1 = gelenveri.veri_x;  
  gelenledA_B = gelenveri.ButonA1;
  gelendeletebuton = gelenveri.ButonA3;
 /*Bu kısımda alınan verilere göre B cihazındaki GPIO lar kontrol edilmiştir*/
  if (gelenledA_B == 1) {
    digitalWrite(LED_PIN_1, HIGH);
  }
  else {
   digitalWrite(LED_PIN_1, LOW);

  }
  //Veri gönderen cihazın mac adresi char tipindeki macStr2 değişkenine yazılmıştır.
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
  pinMode(BUTTON_PIN_1, INPUT);  //Buton giriş olarak ayarlandı.
  pinMode(LED_PIN_1, OUTPUT);    // LED çıkış olarak ayarlandı

  button_state_1 = digitalRead(BUTTON_PIN_1);

  
  delay(5000);
  // elektrik verildikten 5 saniye sonra cihaz istasyon moduna alınıyor,mac adresi sorgulanıyor ve wifi devre dışı bırakılıyor.
  WiFi.mode(WIFI_STA);
  WiFi.macAddress();
  WiFi.disconnect();

  // ESP NOW özelliği başlatılıyor.
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

 
  //  Veri gönderim işlemi yapıldığında ondatasent fonksiyonu çalışacak.
  esp_now_register_send_cb(OnDataSent);

  // Peer cihaz ayarlaması.
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Peer cihazın mac adresinin PeerInfo yapısı içerisine yazılması.
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Veri alma işlemi gerçekleştiğinde ondatarecv fonksiyonu çalışacak
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  getReadings();

  // Gönderilecek olan verilerin hazırlanması.
  gidecekveri.veri_x = veri1;
  gidecekveri.LedA1 = gidenledB_A;
  gidecekveri.ButonB1 = button_state_1;
  gidecekveri.device_no = 2;

  
  // Veri gönderim işlemi ve verinin gönderilip gönderilmediğine ilişkin çıktı bu kısımda alınır.
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gidecekveri, sizeof(gidecekveri));

 if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  updateDisplay();

  if (gelendeletebuton == 1) { //A cihazından Buton3 e basıldığında cihaz silme işlemi bu fonksiyonda gerçekleşir. ESPNOW devre dışı bırakılır ve 10 saniye sonra tekrar başlatılır.

   ESP.restart();

}

  delay(500);

}

void getReadings() {  //B cihazından gönderilecek olan veriler bu fonksiyon içerisinde güncelleniyor.
  veri1++;
  if (gelendeletebuton == 0){
    gidenledB_A = HIGH;
    }
    
  else {gidenledB_A = LOW;}
//  gidenledB_A = HIGH;
  button_state_1 = digitalRead(BUTTON_PIN_1);
 }

void updateDisplay() { //Bu fonksiyonda gelen veriler serial ekranda gösterilir.

  
  Serial.println("INCOMING READINGS");
  Serial.print("veri1: ");
  Serial.print(gelenveri.veri_x);
  Serial.println();
  devices_num.total_num = ESP_NOW_MAX_TOTAL_PEER_NUM; // Bağlı cihaz sayısını göreceğimiz değişken
  Serial.print("baglicihaz: ");
  Serial.print(devices_num.total_num);  
}

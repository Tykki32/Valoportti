#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <esp_now.h>

/* ================= I2C ================= */

#define ATTINY_kpl   2

#define CMD_PA4_TIME    1          //Koska PA4 signaali tuli
#define CMD_ADC_START 2    //Aloita ADC mittaus
#define CMD_ADC_STOP 3     //Lopeta ADC mittaus
#define CMD_ADC_RESULT 4   //ADC keskiarvo
#define CMD_ADC_VALUE 5    //nyt ADC arvo
#define CMD_ADC_LOWER 6    //Lower limit asetus
#define CMD_ADC_UPPER 7    //Upper limit asetus
#define CMD_RESULT    8     //Mittauksen tulos
#define CMD_START     9     //Tapahtuman mittauksen aloitus
#define CMD_STOP      10     //Tapahtuman mittauksen lopetus
#define CMD_I2C_TEST 11     //Testaa I2C toimivuus
#define CMD_RUNTIME 12     //Palauta runtime
#define CMD_SPEED   13      //Adc laskujen keskinopeus
#define CMD_RESPONSE 14    //vastaa +1 arvolla
#define CMD_PRINT 15        //printtaa arvoja
#define CMD_REPORT_TRUE 16        //Ilmoita kun tapahtuma
#define CMD_REPORT_FALSE 17        //Älä ilmoita kun tapahtuma
#define CMD_ID 19             //Palauta device ID
#define CMD_TEMP_START 20           //Aloita lämpötila mittaus
#define CMD_TEMP_STOP 21           //Palauta lämpötila mittaus

/* ================= PINIT ================= */

#define I2C_SDA 19
#define I2C_SCL 20
#define SYNC_PIN 21

/* =============== ESP-NOW =============== */

#define TYPE_DATA 0
#define TYPE_ACK  1
#define ACK_TIMEOUT_MS 200

/* ================= BLE ================= */

#define BLE_SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CHAR_RX_UUID        "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CHAR_TX_UUID        "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic *txChar;
String rxBuffer;

uint16_t ATTINY_ADDR[ATTINY_kpl] = {0x41, 0x42};
uint8_t peerMac[] = {0x84, 0xCC, 0xA8, 0xB5, 0xDE, 0x46}; // ESP8266

typedef struct {
  uint16_t value;
  uint16_t seq;
  uint8_t  type;
} packet_t;

packet_t outgoing;
packet_t incoming;
volatile bool ackReceived = false;
uint16_t lastSeq = 0;
uint16_t lastSync = 0;
uint64_t syncTimes[1];



/* ================= FUNKTIOT ================= */

void i2cReset() {
  Wire.end();
  delay(100);  // pieni tauko, jotta ajuri ehtii resetoida
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(5);
  while(Wire.available()) Wire.read(); 
  Serial.println("I2C reset completed");
}

// Lähettää uint64_t arvon 7-bittisinä tavuina
void i2cWrite7(uint8_t addr, uint8_t cmd, uint64_t data=0, uint8_t bytes=0) {
  uint8_t err;
  for (uint8_t i = 0; i < 5; i++) {
    Wire.beginTransmission(addr);
    Wire.write(cmd);
    for (int8_t b = bytes - 1; b >= 0; b--) {
      uint8_t chunk = (data >> (b * 7)) & 0x7F;  // ota 7 bittiä
      Wire.write(chunk);
    }
    err = Wire.endTransmission();
    if (err == 0) return; // onnistui
    delayMicroseconds(100);
  }
  Serial.print("I2C write error, cmd: ");
  Serial.println(cmd);
  Serial.print("I2C write error, data: ");
  Serial.println(data);
  Serial.print("I2C write error, err: ");
  Serial.println(err);
  i2cReset();
}

// Lukee uint64_t arvon 7-bittisinä tavuina
uint64_t i2cRead7(uint8_t addr, uint8_t bytes) {
  uint8_t tuli = Wire.requestFrom(addr, bytes);
  if (tuli != bytes) {
    Serial.print("I2C read error, tuli bittejä: ");
    Serial.println(tuli);
    i2cReset();
    return 0;
  }

  uint64_t result = 0;
  for (uint8_t i = 0; i < bytes; i++) {
    uint8_t chunk = Wire.read() & 0x7F;  // 7 bittiä
    result = (result << 7) | chunk;
  }
  return result;
}
void sendBLE(const String &s) {
  txChar->setValue(s.c_str());
  txChar->notify();
}

void sendSyncPulse() {
  digitalWrite(SYNC_PIN, HIGH);
  delayMicroseconds(80);
  digitalWrite(SYNC_PIN, LOW);
}

uint64_t readPa4Time(uint8_t osoite) {
  i2cWrite7(osoite, CMD_PA4_TIME);
  uint64_t t = 0;
  t=i2cRead7(osoite, 8);
  String out = String(osoite) + " PA4 aika: " + String(t);
  //Serial.println(out);
  sendBLE(out);
  return t;
}
uint16_t testADC(uint16_t osoite){
  i2cWrite7(osoite, CMD_ADC_START);
  delay(2000);
  uint32_t t;
  i2cWrite7(osoite, CMD_ADC_RESULT);
  t=(i2cRead7(osoite, 8)*1000) >> 16;
  String out = "ADC: " + String(t/1000) + " = " + String(t*550/1024) + " uV";
  Serial.println(out);
  sendBLE(out);
  return t;
}
void runADC(uint16_t osoite){
  uint64_t luku = 0;
  while(true){
    char c = Serial.read();
    if(c=='\n') break;
    i2cWrite7(osoite, CMD_ADC_VALUE);
    uint64_t t=0;
    t=i2cRead7(osoite, 2);
    if(t != 0){
      String out = "ADC: " + String(t) + " = " + String(t*550/1024) + " mV";
      Serial.println(out);
      sendBLE(out);
      luku = t;
    }else{
      delayMicroseconds(100);
    }
  }
}
bool testI2C(uint16_t osoite){
  i2cWrite7(osoite, CMD_I2C_TEST);
  uint8_t t=0;
  t = i2cRead7(osoite, 1);
  if(t==17){
    Serial.println("I2C ok");
    sendBLE("I2C ok");
    return true;
  }else{
    Serial.println("I2C ei ok");
    sendBLE("I2C ei ok");
    return false;
  }
}
void runTime(uint16_t osoite){
  i2cWrite7(osoite, CMD_RUNTIME);
  uint64_t t=0;
  t=i2cRead7(osoite, 8);
  t /= 20000000;
  String out = "Päällä: " + String(t) + " sekuntia";
  Serial.println(out);
  sendBLE(out);
}
void sendESP(uint8_t cmd, uint8_t data){
  bool ok = sendWithAck((cmd << 8) | data);
  /*if (ok) {
    Serial.println("ESP-NOW lähetys OK");
  } else {
    Serial.println("ESP-NOW ACK timeout");
  }*/
}
/* ================= KOMENTOKÄSITTELY ================= */

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd == "sync"){
    sendSyncPulse();
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      readPa4Time(ATTINY_ADDR[a]);
      }}
  else if (cmd == "start sync"){
    while (true){
      char c = Serial.read();
      if(c=='\n') break;
      sendSyncPulse();
      for(uint8_t a = 0;a<ATTINY_kpl;a++){
        Serial.print(ATTINY_ADDR[a]);
        Serial.print(" PA aika: ");
        Serial.println(readPa4Time(ATTINY_ADDR[a]));
        }
      delay(3);
    }
    }
  else if (cmd == "test adc"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      testADC(ATTINY_ADDR[a]);}}
  else if (cmd == "one adc"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_VALUE);
      uint8_t t=0;
      t = i2cRead7(ATTINY_ADDR[a], 2);
      Serial.print(ATTINY_ADDR[a]);
      Serial.print(" ADC: ");
      Serial.println(t);
    }
    }
  else if (cmd ==  "test"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      testI2C(ATTINY_ADDR[a]);}}
  else if (cmd ==  "runtime"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      runTime(ATTINY_ADDR[a]);}}
  else if (cmd.startsWith("send esp ")){
    int firstSpace = cmd.indexOf(' ', 9);  // ensimmäinen väli luvun jälkeen
    int arvo1=0;
    int arvo2=0;
    if (firstSpace == -1) {
      arvo1 = cmd.substring(8).toInt();
    }else{
      arvo1 = cmd.substring(9, firstSpace).toInt();
      arvo2 = cmd.substring(firstSpace + 1).toInt();
    }
    sendESP(arvo1, arvo2);}
  else if(cmd.startsWith("run adc ")){
    int osoite = cmd.substring(7).toInt();
    runADC(ATTINY_ADDR[osoite]);}
  else if (cmd.startsWith("send num ")){
    int arvo = cmd.substring(8).toInt();
      
    //if (arvo < 0) arvo = 0;
    //if (arvo > 255) arvo = 255;
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_RESPONSE, arvo, 8);
      uint64_t t=0;
      t=i2cRead7(ATTINY_ADDR[a], 8);
      Serial.print(ATTINY_ADDR[a]);
      Serial.print(" Lähetin: ");
      Serial.println(String(arvo));
      String out = "Vastasi: " + String(t);
      Serial.println(out);}}
  else if (cmd == "start"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_START);}}
  else if (cmd == "stop"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_STOP);}}
  else if (cmd == "result"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_RESULT);
      uint64_t t= 0;
      t = i2cRead7(ATTINY_ADDR[a],8);
      uint64_t virhe = 72057594037927935;
      if(t==virhe){
        Serial.print(ATTINY_ADDR[a]);
        Serial.println(" mittaus ei valmis");
      }
      else{
        Serial.print(ATTINY_ADDR[a]);
        Serial.print(" raja ylitetty: ");
        Serial.print(String(t));
        Serial.println(" kelloa alusta");
        
        //i2cWrite7(ATTINY_ADDR, CMD_START);

    }}}
  else if (cmd == "set yla"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_START);
    }
    delay(1000);
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      uint64_t t;
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_RESULT);
      t=i2cRead7(ATTINY_ADDR[a], 8);
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_UPPER, t, 8);
      t=t*1000 >> 16;
      String out = String(ATTINY_ADDR[a]) + " ADC: " + String(t/1000) + " = " + String(t*550/1024) + " uV";
      Serial.println(out);}}
  else if (cmd == "set ala"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_START);
    }
    delay(1000);
    uint64_t t;
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_RESULT);
      t=i2cRead7(ATTINY_ADDR[a], 8);
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_LOWER, t, 8);
      t=t*1000 >> 16;
      String out = String(ATTINY_ADDR[a]) + " ADC: " + String(t/1000) + " = " + String(t*550/1024) + " uV";
      Serial.println(out);}}
  else if (cmd == "speed"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_SPEED);
      uint64_t t = i2cRead7(ATTINY_ADDR[a],8);
      Serial.print(ATTINY_ADDR[a]);
      Serial.print(" Keskinopeus: ");
      Serial.println(t);
    }}
  else if (cmd.startsWith("printtaa kaikki ")){
    uint8_t osoite = cmd.substring(15).toInt();
    uint64_t t;
    Serial.println(ATTINY_ADDR[osoite]);
    for(uint8_t a=1;a<6;a++){
      i2cWrite7(ATTINY_ADDR[osoite], CMD_PRINT, a, 3);
      t = i2cRead7(ATTINY_ADDR[osoite], 8);
      Serial.println(t);
    }
    uint16_t a = 10;
    i2cWrite7(ATTINY_ADDR[osoite], CMD_PRINT, a, 3);
    t = i2cRead7(ATTINY_ADDR[osoite], 8);
    Serial.println(t);
    while(t>0){
      a++;
      i2cWrite7(ATTINY_ADDR[osoite], CMD_PRINT, a, 3);
      t = i2cRead7(ATTINY_ADDR[osoite], 8);
      Serial.println(t);
    }
    }
  else if(cmd == "aloita testi"){
    uint64_t t=0;
    uint64_t a2;
    uint64_t a1;
    uint64_t b2;
    uint64_t b1;
    uint64_t c2;
    uint64_t c1;
    int64_t d0;
    int64_t d1;
    while(true){
      t=0;
      while(t<20){    //kunnes valoa
        i2cWrite7(ATTINY_ADDR[0], CMD_ADC_VALUE);
        t = i2cRead7(ATTINY_ADDR[0], 2);
        delay(100);
      }
      delay(1000);
      for(uint8_t a = 0;a<ATTINY_kpl;a++){    //aseta yläraja uusiksi
        i2cWrite7(ATTINY_ADDR[a], CMD_ADC_START);
      }
      delay(2000);
      for(uint8_t a = 0;a<ATTINY_kpl;a++){
        uint64_t t;
        i2cWrite7(ATTINY_ADDR[a], CMD_ADC_RESULT);
        t=i2cRead7(ATTINY_ADDR[a], 8);
        i2cWrite7(ATTINY_ADDR[a], CMD_ADC_UPPER, t, 8);
      }

      delay(500);
      i2cWrite7(ATTINY_ADDR[0], CMD_REPORT_TRUE);
      for(uint8_t a = 0;a<ATTINY_kpl;a++){
        i2cWrite7(ATTINY_ADDR[a], CMD_START);}

      delay(500);
      Serial.println("Valmis");
      while (digitalRead(4) == HIGH) {
        delayMicroseconds(1); // pieni viive estää watchdog-resetin
      }
      
      sendSyncPulse();
      delay(2);
      a1=readPa4Time(ATTINY_ADDR[0]);
      a2=readPa4Time(ATTINY_ADDR[1]);
      
      //delay(1);
      sendSyncPulse();
      b1=readPa4Time(ATTINY_ADDR[0]);
      b2=readPa4Time(ATTINY_ADDR[1]);
      
      i2cWrite7(ATTINY_ADDR[0], CMD_RESULT);
      c1 = i2cRead7(ATTINY_ADDR[0],8);
      i2cWrite7(ATTINY_ADDR[1], CMD_RESULT);
      c2 = i2cRead7(ATTINY_ADDR[1],8);

      //d0 = (c0-a0)/20;
      //d1 = (c1-a1)/20;
      //Serial.println((b1-a1)/20*((c2-a2)/(b2-a2)-(c1-a1)/(b1-a1)));
      Serial.println(a1);
      Serial.println(a2);
      Serial.println(b1);
      Serial.println(b2);
      Serial.println(c1);
      Serial.println(c2);

      /*
      uint64_t virhe = 72057594037927935;
      for(uint8_t a = 0;a<ATTINY_kpl;a++){
        i2cWrite7(ATTINY_ADDR[a], CMD_RESULT);
        t = i2cRead7(ATTINY_ADDR[a],8);
        if(t==virhe){
          Serial.print(ATTINY_ADDR[a]);
          Serial.println(" mittaus ei valmis");
        }
        else{
          Serial.print(ATTINY_ADDR[a]);
          Serial.print(" raja ylitetty: ");
          Serial.print(String(t));
          Serial.println(" kelloa alusta");
        }
      }*/
    }
    }
    
  else if (cmd.startsWith("printtaa ")){
    int arvo = cmd.substring(8).toInt();
    i2cWrite7(ATTINY_ADDR[0], CMD_PRINT, arvo, 3);
    uint64_t t = i2cRead7(ATTINY_ADDR[0], 8);
    Serial.println(t);
    }
  else if (cmd.startsWith("report ")){
    int osoite = cmd.substring(6).toInt();
    i2cWrite7(ATTINY_ADDR[osoite], CMD_REPORT_TRUE);
    }
  else if (cmd == "koska valmis"){
    i2cWrite7(ATTINY_ADDR[0], CMD_ADC_START);
    uint64_t aika = millis();
    uint64_t t=2550866976440000;
    while(t>(255086697644000)){
      i2cWrite7(ATTINY_ADDR[0], CMD_PRINT, 5, 3);
      Serial.println(i2cRead7(ATTINY_ADDR[0], 8));
      i2cWrite7(ATTINY_ADDR[0], CMD_ADC_RESULT);
      t=i2cRead7(ATTINY_ADDR[0], 8);
      delay(1000);
      Serial.print((millis()-aika)/1000);
      Serial.print(" s, idx: ");
    }
      
    }
  else if (cmd == "id"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      uint64_t t;
      i2cWrite7(ATTINY_ADDR[a], CMD_ID);
      t=i2cRead7(ATTINY_ADDR[a], 8);
      Serial.println(t);
    }
    }
  else if (cmd == "temp"){
    while(true){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_TEMP_START);
    }
    delay(2000);
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      uint64_t t;
      i2cWrite7(ATTINY_ADDR[a], CMD_TEMP_STOP);
      t=i2cRead7(ATTINY_ADDR[a], 8);
      //Serial.print(ATTINY_ADDR[a]);
      //Serial.print(" lämpötila: ");
      Serial.print(t/1000.-273.15);
      Serial.print("   ");
      //Serial.println(" astetta");
    }
    Serial.println("");
    }}
}

/* ================= BLE CALLBACK ================= */

class RXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    rxBuffer = c->getValue().c_str();
    handleCommand(rxBuffer);
  }
};

/* =================ESP-NOW vastaanotto==================*/

void onReceive(const esp_now_recv_info *info,const uint8_t *data,int len) {
  if (len != sizeof(packet_t)) return;
  memcpy(&incoming, data, sizeof(incoming));

  if (incoming.type == TYPE_ACK && incoming.seq == lastSeq) {
    Serial.println("ESP-NOW ACK vastaanotettu");
    ackReceived = true;
  }

  if (incoming.type == TYPE_DATA) {
    uint16_t receivedValue = incoming.value;
    Serial.print("ESP-NOW DATA vastaanotettu: ");
    Serial.println(receivedValue);

    packet_t ack;
    ack.value = 0;
    ack.seq   = incoming.seq;
    ack.type  = TYPE_ACK;

    esp_now_send(info->src_addr,(uint8_t*)&ack,sizeof(ack));
  }
}


/* =================ESP-NOW lähetys==================*/

bool sendWithAck(uint16_t value) {
  outgoing.value = value;
  outgoing.seq = ++lastSeq;
  outgoing.type = TYPE_DATA;

  ackReceived = false;
  esp_now_send(peerMac, (uint8_t*)&outgoing, sizeof(outgoing));

  unsigned long start = millis();
  while (!ackReceived) {
    if (millis() - start > ACK_TIMEOUT_MS) {
      return false;
    }
    delay(1);
  }
  return true;
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init epäonnistui");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  esp_now_register_recv_cb(onReceive);

  BLEDevice::init("Valoportti");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(BLE_SERVICE_UUID);

  txChar = service->createCharacteristic(
    BLE_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic *rxChar = service->createCharacteristic(
    BLE_CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  rxChar->setCallbacks(new RXCallbacks());

  service->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->start();

  Serial.println("ESP32-S3 Valoportti käynnissä");
  //Serial.print("ESP Board MAC Address:  ");
  //Serial.println(WiFi.macAddress());
  for(uint8_t a = 0;a<ATTINY_kpl;a++){
    while(!testI2C(ATTINY_ADDR[a])){
      Serial.print("Attiny ");
      Serial.print(ATTINY_ADDR[a]);
      Serial.println(" epäinnistui");
      delay(500);
    }
  }
  pinMode(4, INPUT_PULLUP);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}

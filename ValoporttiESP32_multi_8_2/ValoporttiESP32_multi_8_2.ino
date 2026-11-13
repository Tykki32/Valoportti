/* =============================================================================
 * ValoporttiESP32_multi_8_2  -  ESP32-S3 master, 1..4 x ATtiny816 (Valoportti816-firmware)
 * =============================================================================
 * Pohjana ValoporttiESP32_multi_8_1. Muutokset (ks. myös Valoportti816/README.md §7 ja §8):
 *   - Anturit etsitään käynnistyksessä (osoitteet 0x41..0x44); puuttuva anturi ei jumita käynnistystä.
 *     Uuden ATtinyn UID: kytke vain se väylälle ja kirjoita "id40" (tuntematon UID -> osoite 0x40); lisää UID ATtiny-sketsin setup():n switchiin.
 *     Komennot käyvät läpi vain löytyneet anturit (ATTINY_kpl / ATTINY_ADDR). "scan" etsii uudelleen.
 *   - Uudet komentomäärittelyt (VREF, STATUS, EDGE_NEXT, EDGE_DATA, CLOCK, ADC_CFG, TIME_ADJ, EDGE_PARAM, CAPTURE).
 *   - ADC-arvot skaalataan oikein: ADC_VALUE on N muunnoksen summa, ADC_RESULT on Q16 yhtä muunnosta kohti,
 *     referenssi luetaan STATUSista (ei oleteta 0,55 V).
 *   - "kalibroi": valitsee jokaiselle anturille pienimmän referenssin >= 1,1 V, jolla taso <= 85 % asteikosta.
 *   - "aloita testi": kalibrointi, START, PA4-synkronointi ennen ja jälkeen ohituksen, odotus ohituksen loppuun
 *     (EI I2C-liikennettä kesken ohituksen), kaikkien 8 reunan luku ja aikojen muunto yhteiselle aikajanalle.
 *   - "kaappaa <anturi> <k>": kaappaa yhden reunan kokonaisena (CMD_CAPTURE) ja tulostaa lukemat CSV:nä.
 *   - Aikakatkaisut, keskeytys Serial-rivinvaihdolla, uint16_t uint8_t:n tilalla ADC-luvuissa.
 * Aikaleimat ovat ATtinyn 32-bittisiä kiertäviä sykliarvoja: erotukset aina (uint32_t)(b - a).
 * ============================================================================= */
#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>

/* ================= I2C ================= */

#define ATTINY_MAX   4

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
#define CMD_SPEED   13      //Adc laskujen keskinopeus (EI toteutettu ATtinyssa)
#define CMD_RESPONSE 14    //vastaa +1 arvolla (EI toteutettu ATtinyssa)
#define CMD_PRINT 15        //printtaa arvoja
#define CMD_REPORT_TRUE 16        //Ilmoita kun tapahtuma
#define CMD_REPORT_FALSE 17        //Älä ilmoita kun tapahtuma
#define CMD_VREF 18           //ADC-referenssi: 0=0,55 V 1=1,1 V 2=2,5 V 3=4,34 V 4=1,5 V 5=VDD
#define CMD_ID 19             //Palauta device ID
#define CMD_TEMP_START 20           //Aloita lämpötila mittaus
#define CMD_TEMP_STOP 21           //Palauta lämpötila mittaus
#define CMD_STATUS 22         //Tila: bitit 8-15 N, 16-19 ref, 24-27 jonon pituus, 28-29 kaappaus (2 = valmis)
#define CMD_EDGE_NEXT 25      //Poista jonon vanhin reuna
#define CMD_EDGE_DATA 26      //Raakalukemat (8 kpl / 16 tavua)
#define CMD_CLOCK 27          //Ulkoinen kello (kHz), 0 = sisäinen
#define CMD_ADC_CFG 28        //Kertymäkoodi | samplen << 3 | (esijakaja+1) << 8
#define CMD_TIME_ADJ 30       //Aikakorjaus (sykliä)
#define CMD_EDGE_PARAM 31     //Tunnistusparametrit
#define CMD_CAPTURE 32        //Kaappaa yksi reuna (payload = ohitettavien reunojen määrä)

/* ================= PINIT ================= */

#define I2C_SDA 19
#define I2C_SCL 20
#define SYNC_PIN 21
#define REPORT_PIN 4          // ATtinyn PA1 (ilmoitus kun reuna alkaa; LOW-pulssi)

/* =============== ESP-NOW =============== */

#define TYPE_DATA 0
#define TYPE_ACK  1
#define ACK_TIMEOUT_MS 200

/* ================= BLE ================= */

#define BLE_SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CHAR_RX_UUID        "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CHAR_TX_UUID        "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

/* ================= ASETUKSET ================= */

#define ATTINY_FCPU      20000000UL   // ATtinyn F_CPU (Arduino-käännöksen asetus); vain nimellinen, todellinen taajuus mitataan synkronoinnilla
#define LIGHT_MIN        10           // "valoa" = vähintään 10 ADC-yksikköä / muunnos
#define WARMUP_MS        60           // CMD_START -> tunnistus valmis (lämmittely ~11-22 ms)
#define PASS_TIMEOUT_MS  3000         // ohituksen jälkeen: kauanko odotetaan että jonoon tulee 8 reunaa
#define REPORT_TIMEOUT_MS 60000       // kauanko odotetaan PA1-ilmoitusta (kappaleen saapumista)

uint16_t passWaitMs = 300;            // kauanko ohituksen alusta odotetaan ennen I2C-lukuja (komento "passwait")

BLECharacteristic *txChar;
String rxBuffer;

const uint8_t ATTINY_CAND[ATTINY_MAX] = {0x41, 0x42, 0x43, 0x44};   // mahdolliset osoitteet
uint8_t  ATTINY_kpl = 0;                                          // löytyneiden määrä
uint16_t ATTINY_ADDR[ATTINY_MAX];                                 // löytyneiden osoitteet
uint8_t peerMac[] = {0x84, 0xCC, 0xA8, 0xB5, 0xDE, 0x46}; // ESP8266

const uint16_t VREF_MV[6] = {550, 1100, 2500, 4340, 1500, 3300};   // 5 = VDD (oletus 3,3 V)
#define VIRHE56 72057594037927935ULL                              // 2^56-1 = "ei valmis" (kahdeksan tavua 0x7F)

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

struct EdgeRec { uint32_t t; int16_t H; uint8_t flags; };        // t = ekaAika + vikaAika (ATtinyn syklit, mod 2^32)

/* ================= ETEENPÄIN-MÄÄRITTELYT ================= */

bool sendWithAck(uint16_t value);
void handleCommand(String cmd);

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
  uint8_t err = 0;
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
  Serial.println((unsigned long)data);
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

// Hiljainen I2C-koetin (ei i2cResetiä): onko osoitteessa Valoportti-ATtiny
bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(CMD_I2C_TEST);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  return (Wire.read() & 0x7F) == 17;
}

// Etsii anturit osoitteista 0x41..0x44 ja täyttää ATTINY_ADDR / ATTINY_kpl
void scanSensors() {
  ATTINY_kpl = 0;
  for (uint8_t c = 0; c < ATTINY_MAX; c++) {
    bool ok = false;
    for (uint8_t t = 0; t < 3 && !ok; t++) { ok = probe(ATTINY_CAND[c]); if (!ok) delay(5); }
    if (ok) ATTINY_ADDR[ATTINY_kpl++] = ATTINY_CAND[c];
  }
  Serial.print("Antureita löytyi: ");
  Serial.print(ATTINY_kpl);
  Serial.print(" (");
  for (uint8_t a = 0; a < ATTINY_kpl; a++) { Serial.print("0x"); Serial.print(ATTINY_ADDR[a], HEX); Serial.print(a + 1 < ATTINY_kpl ? " " : ""); }
  Serial.println(")");
}

uint32_t getStatus(uint8_t addr) {
  i2cWrite7(addr, CMD_STATUS);
  return (uint32_t)i2cRead7(addr, 8);
}
uint8_t statusN(uint32_t st) { uint8_t n = (st >> 8) & 0xFF; return n ? n : 1; }
uint8_t statusVref(uint32_t st) { uint8_t v = (st >> 16) & 0xF; return v > 5 ? 1 : v; }
uint8_t statusQueue(uint32_t st) { return (st >> 24) & 0xF; }

// ADC_RESULT (Q16, yksi muunnos) -> millivoltit annetulla referenssillä
uint32_t q16ToMv(uint32_t q16, uint8_t vref) {
  return (uint32_t)(((uint64_t)q16 * VREF_MV[vref]) / (65536ULL * 1023ULL));
}

uint32_t printVal(uint8_t addr, uint8_t sel) {          // CMD_PRINT-valitsin (< 128)
  i2cWrite7(addr, CMD_PRINT, sel, 1);
  return (uint32_t)i2cRead7(addr, 8);
}

uint64_t readPa4Time(uint8_t osoite) {
  i2cWrite7(osoite, CMD_PA4_TIME);
  uint64_t t = 0;
  t=i2cRead7(osoite, 8);
  String out = String(osoite) + " PA4 aika: " + String((unsigned long)t);
  //Serial.println(out);
  sendBLE(out);
  return t;
}
uint16_t testADC(uint16_t osoite){
  i2cWrite7(osoite, CMD_ADC_START);
  delay(2000);
  uint32_t st = getStatus(osoite);
  uint8_t vref = statusVref(st);
  i2cWrite7(osoite, CMD_ADC_RESULT);
  uint64_t q = i2cRead7(osoite, 8);
  if (q == VIRHE56) { Serial.println("ADC: ei valmis"); return 0; }
  uint32_t mv = q16ToMv((uint32_t)q, vref);
  String out = String(osoite) + " ADC: " + String((unsigned long)(q >> 16)) + " yks (1023 = täysi) = " + String((unsigned long)mv) + " mV (ref " + String(VREF_MV[vref]) + " mV)";
  Serial.println(out);
  sendBLE(out);
  return (uint16_t)(q >> 16);
}
void runADC(uint16_t osoite){
  uint8_t N = statusN(getStatus(osoite));
  uint8_t vref = statusVref(getStatus(osoite));
  while(true){
    char c = Serial.read();
    if(c=='\n') break;
    i2cWrite7(osoite, CMD_ADC_VALUE);
    uint64_t t=0;
    t=i2cRead7(osoite, 2);
    if(t != 0){
      uint32_t perConv = (uint32_t)(t / N);
      String out = "ADC: " + String((unsigned long)t) + " (N=" + String(N) + ") = " + String((unsigned long)(perConv * VREF_MV[vref] / 1023)) + " mV";
      Serial.println(out);
      sendBLE(out);
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
  // aika on 32-bittinen (kiertää 2^32 syklin välein) ja nollautuu ADC-asetuksen muutoksesta
  String out = "Aikaperusta: " + String((unsigned long)(t / ATTINY_FCPU)) + " s (nimellinen " + String((unsigned long)(ATTINY_FCPU / 1000000UL)) + " MHz)";
  Serial.println(out);
  sendBLE(out);
}
void sendESP(uint8_t cmd, uint8_t data){
  bool ok = sendWithAck((cmd << 8) | data);
  (void)ok;
}

/* ================= KALIBROINTI ================= */

// Valitsee referenssin: pienin >= 1,1 V jolla taso <= 85 % asteikosta. Palauttaa false jos mittaus ei onnistu.
bool calibrateSensor(uint8_t addr, uint8_t *chosen, float *level) {
  const uint8_t order[4] = {1, 4, 2, 3};            // 1,1 V, 1,5 V, 2,5 V, 4,34 V
  for (uint8_t k = 0; k < 4; k++) {
    i2cWrite7(addr, CMD_VREF, order[k], 1);
    delay(10);                                       // referenssi ja ADC asettuvat; aikaperusta alkaa alusta
    i2cWrite7(addr, CMD_ADC_START);
    uint64_t q = VIRHE56;
    for (uint8_t t = 0; t < 60 && q == VIRHE56; t++) {
      delay(10);
      i2cWrite7(addr, CMD_ADC_RESULT);
      q = i2cRead7(addr, 8);
    }
    if (q == VIRHE56) {
      Serial.print("kalibrointi: anturi 0x"); Serial.print(addr, HEX); Serial.println(" ei vastaa ADC-tulosta");
      return false;
    }
    float L = (float)q / 65536.0f;                   // ADC-yksikköä / muunnos (0..1023)
    Serial.print("  0x"); Serial.print(addr, HEX);
    Serial.print(" ref "); Serial.print(VREF_MV[order[k]]); Serial.print(" mV: taso ");
    Serial.print(L, 1); Serial.print(" / 1023 (");
    Serial.print(100.0f * L / 1023.0f, 0); Serial.println(" %)");
    if (L <= 0.85f * 1023.0f || k == 3) {
      *chosen = order[k]; *level = L;
      if (L > 0.85f * 1023.0f) Serial.println("  VAROITUS: taso > 85 % suurimmallakin referenssillä");
      return true;
    }
  }
  return false;
}

bool calibrateAll() {
  Serial.println("Kalibrointi (ledit päällä):");
  for (uint8_t a = 0; a < ATTINY_kpl; a++) {
    uint8_t v = 1; float L = 0;
    if (!calibrateSensor(ATTINY_ADDR[a], &v, &L)) return false;
    i2cWrite7(ATTINY_ADDR[a], CMD_VREF, v, 1);       // lopullinen valinta (viimeksi mitattu ref voi olla eri)
    delay(10);
    Serial.print("  -> 0x"); Serial.print(ATTINY_ADDR[a], HEX);
    Serial.print(": ref "); Serial.print(VREF_MV[v]); Serial.print(" mV, taso ");
    Serial.print(100.0f * L / 1023.0f, 0); Serial.println(" %");
  }
  return true;
}

/* ================= REUNOJEN LUKU ================= */

// Odottaa PA1-ilmoitusta (LOW). Palauttaa false aikakatkaisussa tai Serial-keskeytyksessä.
bool waitReport(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (digitalRead(REPORT_PIN) == HIGH) {
    if (millis() - t0 > timeoutMs) return false;
    if (Serial.available()) { Serial.read(); return false; }
    delayMicroseconds(5);                            // ei watchdog-resetiä
  }
  return true;
}

// Odottaa että kaikissa antureissa on jonossa vähintään `need` reunaa (tai aikakatkaisu)
void waitQueues(uint8_t need, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    bool all = true;
    for (uint8_t a = 0; a < ATTINY_kpl; a++) if (statusQueue(getStatus(ATTINY_ADDR[a])) < need) all = false;
    if (all) return;
    delay(20);
  }
}

// Lukee anturin kaikki jonossa olevat reunat (ja poistaa ne jonosta)
uint8_t readEdges(uint8_t addr, EdgeRec *out, uint8_t maxn) {
  uint8_t n = 0;
  while (n < maxn) {
    uint32_t st = getStatus(addr);
    if (!statusQueue(st)) break;
    uint32_t eka  = printVal(addr, 5);                  // buf[0]:n aika (syklit)
    i2cWrite7(addr, CMD_RESULT);
    uint32_t vika = (uint32_t)i2cRead7(addr, 8);        // 50 %-ylitys suhteessa ekaAika:an
    uint32_t hf   = printVal(addr, 7);                  // H + 32768 | liput << 16
    out[n].t = eka + vika;
    out[n].H = (int16_t)((int32_t)(hf & 0xFFFF) - 32768);
    out[n].flags = (hf >> 16) & 0xFF;
    n++;
    i2cWrite7(addr, CMD_EDGE_NEXT);
  }
  return n;
}

// Kaapatun reunan raakalukemat CSV:nä: indeksi, aika (syklit), lukema (N muunnoksen summa)
void readCapture(uint8_t a) {
  uint32_t st = getStatus(a);
  if (((st >> 28) & 3) != 2) { Serial.println("ei kaapattu"); return; }
  uint8_t  N    = statusN(st);
  uint8_t  vref = statusVref(st);
  uint32_t eka  = printVal(a, 5);
  uint32_t P    = printVal(a, 6);
  uint8_t  len  = (uint8_t)printVal(a, 1);
  i2cWrite7(a, CMD_RESULT);
  uint32_t vika = (uint32_t)i2cRead7(a, 8);
  uint32_t lo   = printVal(a, 60);
  uint32_t hi   = printVal(a, 61);
  uint32_t hf   = printVal(a, 7);
  Serial.printf("# N=%u ref=%umV P=%lu eka=%lu vika=%lu lo=%lu hi=%lu H=%d liput=%u len=%u\n", N, VREF_MV[vref],
                (unsigned long)P, (unsigned long)eka, (unsigned long)vika, (unsigned long)lo, (unsigned long)hi,
                (int)((int32_t)(hf & 0xFFFF) - 32768), (unsigned)((hf >> 16) & 0xFF), len);
  Serial.println("idx,aika_syklia,lukema");
  for (uint8_t i = 0; i < len; i += 8) {
    i2cWrite7(a, CMD_EDGE_DATA, i, 1);
    if (Wire.requestFrom(a, (uint8_t)16) != 16) { i2cReset(); return; }
    for (uint8_t j = 0; j < 8 && (uint8_t)(i + j) < len; j++) {
      uint16_t v = Wire.read(); v |= (uint16_t)Wire.read() << 8;
      Serial.printf("%u,%lu,%u\n", (unsigned)(i + j), (unsigned long)(eka + (uint32_t)(i + j) * P), (unsigned)v);
    }
  }
}

void runCapture(uint8_t idx, uint8_t skip) {
  if (idx >= ATTINY_kpl) { Serial.println("anturia ei ole"); return; }
  uint8_t a = ATTINY_ADDR[idx];
  uint8_t v = 1; float L = 0;
  Serial.println("Kalibrointi:");
  if (!calibrateSensor(a, &v, &L)) return;
  i2cWrite7(a, CMD_VREF, v, 1);
  delay(10);
  i2cWrite7(a, CMD_REPORT_TRUE);
  i2cWrite7(a, CMD_CAPTURE, skip, 1);
  delay(WARMUP_MS);
  Serial.println("Valmis - ohita kappale (keskeytys: rivinvaihto)");
  if (!waitReport(REPORT_TIMEOUT_MS)) { Serial.println("ei ilmoitusta"); i2cWrite7(a, CMD_STOP); return; }
  delay(passWaitMs);                                 // ei I2C-liikennettä kesken ohituksen
  uint32_t t0 = millis();
  while (((getStatus(a) >> 28) & 3) != 2) {
    if (millis() - t0 > PASS_TIMEOUT_MS) { Serial.println("ei kaapattu (reunaa ei tullut)"); return; }
    delay(20);
  }
  readCapture(a);
}

/* ================= MITTAUSTESTI ================= */

void runTest() {
  if (!ATTINY_kpl) { Serial.println("ei antureita (komento: scan)"); return; }
  Serial.println("Odotetaan valoa anturissa 1 (keskeytys: rivinvaihto)");
  while (true) {
    uint8_t N = statusN(getStatus(ATTINY_ADDR[0]));
    i2cWrite7(ATTINY_ADDR[0], CMD_ADC_VALUE);
    uint32_t v = (uint32_t)i2cRead7(ATTINY_ADDR[0], 2);
    if (v / N >= LIGHT_MIN) break;
    if (Serial.available()) { Serial.read(); Serial.println("keskeytetty"); return; }
    delay(100);
  }
  delay(1000);
  if (!calibrateAll()) return;

  i2cWrite7(ATTINY_ADDR[0], CMD_REPORT_TRUE);
  for (uint8_t a = 0; a < ATTINY_kpl; a++) i2cWrite7(ATTINY_ADDR[a], CMD_START);
  delay(WARMUP_MS);

  // synkronointipulssi A ennen ohitusta (I2C hiljaa)
  uint32_t pa[ATTINY_MAX], pb[ATTINY_MAX];
  int64_t tA = esp_timer_get_time();
  sendSyncPulse();
  delay(2);
  for (uint8_t a = 0; a < ATTINY_kpl; a++) { pa[a] = (uint32_t)readPa4Time(ATTINY_ADDR[a]); }

  Serial.println("Valmis - ohita kappale (keskeytys: rivinvaihto)");
  if (!waitReport(REPORT_TIMEOUT_MS)) {
    Serial.println("ei ilmoitusta, keskeytetty");
    for (uint8_t a = 0; a < ATTINY_kpl; a++) i2cWrite7(ATTINY_ADDR[a], CMD_STOP);
    return;
  }
  delay(passWaitMs);                                 // ohitus loppuu; ei I2C-liikennettä sen aikana
  waitQueues(8, PASS_TIMEOUT_MS);

  // synkronointipulssi B ohituksen jälkeen
  int64_t tB = esp_timer_get_time();
  sendSyncPulse();
  delay(2);
  for (uint8_t a = 0; a < ATTINY_kpl; a++) { pb[a] = (uint32_t)readPa4Time(ATTINY_ADDR[a]); }

  EdgeRec e[ATTINY_MAX][8];
  uint8_t ne[ATTINY_MAX];
  double f[ATTINY_MAX];                              // ATtinyn todellinen taajuus [sykliä / us] (ESP:n kellosta)
  double dtUs = (double)(tB - tA);
  for (uint8_t a = 0; a < ATTINY_kpl; a++) {
    ne[a] = readEdges(ATTINY_ADDR[a], e[a], 8);
    f[a] = (pa[a] == 1 || pb[a] == 1) ? 0.0 : (double)(uint32_t)(pb[a] - pa[a]) / dtUs;
  }

  Serial.println("--- tulokset (aika us synkronointipulssista A, ATtinyn kello kalibroitu ESP:n kellolla) ---");
  Serial.println("anturi,k,aika_us,H,liput,syklit,f_MHz");
  for (uint8_t a = 0; a < ATTINY_kpl; a++) {
    if (f[a] <= 0) { Serial.print("anturi 0x"); Serial.print(ATTINY_ADDR[a], HEX); Serial.println(": PA4-aikaleima puuttuu (pulssi ei tullut perille)"); continue; }
    for (uint8_t k = 0; k < ne[a]; k++) {
      double us = (double)(uint32_t)(e[a][k].t - pa[a]) / f[a];
      Serial.printf("0x%02X,%u,%.2f,%d,%u,%lu,%.4f\n", (unsigned)ATTINY_ADDR[a], (unsigned)k, us, (int)e[a][k].H,
                    (unsigned)e[a][k].flags, (unsigned long)e[a][k].t, f[a]);
    }
    if (ne[a] != 8) { Serial.print("  huom: anturi 0x"); Serial.print(ATTINY_ADDR[a], HEX); Serial.print(" antoi "); Serial.print(ne[a]); Serial.println("/8 reunaa"); }
  }
  for (uint8_t a = 0; a < ATTINY_kpl; a++) {
    uint32_t st = getStatus(ATTINY_ADDR[a]);
    if (st & 0x0C) { Serial.print("  VAROITUS: anturi 0x"); Serial.print(ATTINY_ADDR[a], HEX); Serial.println(" menetti lukemia (STATUS 0x04/0x08)"); }
  }
}

/* ================= KOMENTOKÄSITTELY ================= */

void printStatus(uint8_t addr) {
  uint32_t st = getStatus(addr);
  uint8_t vref = statusVref(st);
  Serial.printf("0x%02X: tila=0x%02X N=%u ref=%umV esijakaja=%u jono=%u kaappaus=%u pino_vapaa=%lu tavua\n", (unsigned)addr,
                (unsigned)(st & 0xFF), (unsigned)statusN(st), (unsigned)VREF_MV[vref], (unsigned)((st >> 20) & 7),
                (unsigned)statusQueue(st), (unsigned)((st >> 28) & 3), (unsigned long)printVal(addr, 9));
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd == "scan") { scanSensors(); return; }
  if (cmd == "id40") {                               // uuden (UID tuntematon -> osoite 0x40) ATtinyn UID: kytke vain se yksi väylälle
    i2cWrite7(0x40, CMD_ID);
    Serial.println((unsigned long)i2cRead7(0x40, 8));
    return;
  }
  if (cmd == "send esp" || cmd.startsWith("send esp ")) {
    int firstSpace = cmd.indexOf(' ', 9);  // ensimmäinen väli luvun jälkeen
    int arvo1=0;
    int arvo2=0;
    if (firstSpace == -1) {
      arvo1 = cmd.substring(8).toInt();
    }else{
      arvo1 = cmd.substring(9, firstSpace).toInt();
      arvo2 = cmd.substring(firstSpace + 1).toInt();
    }
    sendESP(arvo1, arvo2);
    return;
  }
  if (cmd.startsWith("passwait ")) { passWaitMs = cmd.substring(9).toInt(); Serial.print("passwait "); Serial.println(passWaitMs); return; }
  if (ATTINY_kpl == 0) { Serial.println("ei antureita (komento: scan)"); return; }

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
        Serial.println((unsigned long)readPa4Time(ATTINY_ADDR[a]));
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
      uint16_t t=0;
      t = i2cRead7(ATTINY_ADDR[a], 2);
      Serial.print(ATTINY_ADDR[a]);
      Serial.print(" ADC: ");
      Serial.print(t);
      Serial.print(" (N muunnoksen summa, N=");
      Serial.print(statusN(getStatus(ATTINY_ADDR[a])));
      Serial.println(")");
    }
    }
  else if (cmd ==  "test"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      testI2C(ATTINY_ADDR[a]);}}
  else if (cmd ==  "runtime"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      runTime(ATTINY_ADDR[a]);}}
  else if(cmd.startsWith("run adc ")){
    int osoite = cmd.substring(7).toInt();
    if (osoite >= 0 && osoite < ATTINY_kpl) runADC(ATTINY_ADDR[osoite]);}
  else if (cmd.startsWith("send num ")){
    Serial.println("CMD_RESPONSE (14) ei ole toteutettu ATtinyssa (ei ollut alkuperäisessäkään) - ei lähetetä");
  }
  else if (cmd == "start"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_START);}}
  else if (cmd == "stop"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_STOP);}}
  else if (cmd == "result"){
    // jonon kaikki reunat: lukee ja poistaa ne jonosta (ekaAika + vikaAika = reunan hetki, syklit)
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      EdgeRec e[8];
      uint8_t n = readEdges(ATTINY_ADDR[a], e, 8);
      if (n == 0) {
        Serial.print(ATTINY_ADDR[a]);
        Serial.println(" mittaus ei valmis");
      }
      for (uint8_t k = 0; k < n; k++) {
        Serial.print(ATTINY_ADDR[a]);
        Serial.print(" reuna ");
        Serial.print(k);
        Serial.print(": ");
        Serial.print((unsigned long)e[k].t);
        Serial.print(" kelloa alusta, H=");
        Serial.print(e[k].H);
        Serial.print(" liput=");
        Serial.println(e[k].flags);
      }
    }}
  else if (cmd == "set yla" || cmd == "set ala"){
    // (vanha) ylä-/alarajan asetus: ATtiny vastaanottaa arvon mutta ei käytä sitä (tunnistus on suhteellinen).
    // Komento mittaa tason ja tulostaa sen.
    uint8_t cmdId = (cmd == "set yla") ? CMD_ADC_UPPER : CMD_ADC_LOWER;
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_START);
    }
    delay(1000);
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      uint64_t t;
      i2cWrite7(ATTINY_ADDR[a], CMD_ADC_RESULT);
      t=i2cRead7(ATTINY_ADDR[a], 8);
      if (t == VIRHE56) { Serial.println("ADC: ei valmis"); continue; }
      i2cWrite7(ATTINY_ADDR[a], cmdId, t, 8);
      uint8_t vref = statusVref(getStatus(ATTINY_ADDR[a]));
      String out = String(ATTINY_ADDR[a]) + " ADC: " + String((unsigned long)(t >> 16)) + " = " + String((unsigned long)q16ToMv((uint32_t)t, vref)) + " mV";
      Serial.println(out);}}
  else if (cmd == "speed"){
    Serial.println("CMD_SPEED (13) ei ole toteutettu ATtinyssa (ei ollut alkuperäisessäkään)");
  }
  else if (cmd.startsWith("printtaa kaikki ")){
    uint8_t osoite = cmd.substring(15).toInt();
    if (osoite >= ATTINY_kpl) { Serial.println("anturia ei ole"); return; }
    uint64_t t;
    Serial.println(ATTINY_ADDR[osoite]);
    for(uint8_t a=1;a<6;a++){
      i2cWrite7(ATTINY_ADDR[osoite], CMD_PRINT, a, 3);
      t = i2cRead7(ATTINY_ADDR[osoite], 8);
      Serial.println((unsigned long)t);
    }
    uint16_t a = 10;
    i2cWrite7(ATTINY_ADDR[osoite], CMD_PRINT, a, 3);
    t = i2cRead7(ATTINY_ADDR[osoite], 8);
    Serial.println((unsigned long)t);
    while(t>0 && a < 70){                           // lista päättyy nollaan (enintään 50 lukemaa)
      a++;
      i2cWrite7(ATTINY_ADDR[osoite], CMD_PRINT, a, 3);
      t = i2cRead7(ATTINY_ADDR[osoite], 8);
      Serial.println((unsigned long)t);
    }
    }
  else if(cmd == "aloita testi"){
    runTest();
    }
  else if (cmd == "kalibroi"){
    calibrateAll();
    }
  else if (cmd == "status"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++) printStatus(ATTINY_ADDR[a]);
    }
  else if (cmd.startsWith("kaappaa ")){
    int sp = cmd.indexOf(' ', 8);
    int idx = (sp == -1) ? cmd.substring(8).toInt() : cmd.substring(8, sp).toInt();
    int k   = (sp == -1) ? 0 : cmd.substring(sp + 1).toInt();
    runCapture(idx, k);
    }
  else if (cmd.startsWith("vref ")){
    int sp = cmd.indexOf(' ', 5);
    if (sp != -1) {
      int idx = cmd.substring(5, sp).toInt();
      int v = cmd.substring(sp + 1).toInt();
      if (idx >= 0 && idx < ATTINY_kpl && v >= 0 && v <= 5) i2cWrite7(ATTINY_ADDR[idx], CMD_VREF, v, 1);
    }
    }
  else if (cmd.startsWith("cfg ")){
    // cfg <anturi> <kertymäkoodi 0..3|7=auto> <samplen> <esijakaja 0=auto|1..7>
    int idx = 0, n = 7, sl = 0, pr = 0;
    sscanf(cmd.c_str() + 4, "%d %d %d %d", &idx, &n, &sl, &pr);
    if (idx >= 0 && idx < ATTINY_kpl) i2cWrite7(ATTINY_ADDR[idx], CMD_ADC_CFG, (uint64_t)((n & 7) | ((sl & 31) << 3) | ((pr & 7) << 8)), 2);
    }
  else if (cmd.startsWith("printtaa ")){
    int arvo = cmd.substring(8).toInt();
    i2cWrite7(ATTINY_ADDR[0], CMD_PRINT, arvo, 3);
    uint64_t t = i2cRead7(ATTINY_ADDR[0], 8);
    Serial.println((unsigned long)t);
    }
  else if (cmd.startsWith("report ")){
    int osoite = cmd.substring(6).toInt();
    if (osoite >= 0 && osoite < ATTINY_kpl) i2cWrite7(ATTINY_ADDR[osoite], CMD_REPORT_TRUE);
    }
  else if (cmd == "koska valmis"){
    i2cWrite7(ATTINY_ADDR[0], CMD_ADC_START);
    uint64_t aika = millis();
    uint64_t t=VIRHE56;
    while(t == VIRHE56){
      delay(100);
      i2cWrite7(ATTINY_ADDR[0], CMD_ADC_RESULT);
      t=i2cRead7(ATTINY_ADDR[0], 8);
      Serial.print((unsigned long)((millis()-aika)));
      Serial.println(" ms");
      if (Serial.available()) { Serial.read(); break; }
    }
    }
  else if (cmd == "id"){
    for(uint8_t a = 0;a<ATTINY_kpl;a++){
      uint64_t t;
      i2cWrite7(ATTINY_ADDR[a], CMD_ID);
      t=i2cRead7(ATTINY_ADDR[a], 8);
      Serial.println((unsigned long)t);
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
      Serial.print(t/1000.-273.15);
      Serial.print("   ");
    }
    Serial.println("");
    if (Serial.available()) { Serial.read(); break; }
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

  Serial.println("ESP32-S3 Valoportti käynnissä (8_2)");
  pinMode(REPORT_PIN, INPUT_PULLUP);
  scanSensors();
  if (ATTINY_kpl == 0) Serial.println("VAROITUS: yhtään anturia ei löytynyt - tarkista johdotus ja kirjoita 'scan'");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}

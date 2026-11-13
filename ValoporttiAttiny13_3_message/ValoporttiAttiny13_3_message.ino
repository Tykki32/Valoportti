/* =============================================================================
 * VALOPORTTI  (valoportti = valokenno / "light gate")  -  ATtiny816, I2C-orja
 * =============================================================================
 * KOMMENTTIEN LUKUOHJE
 *   Tähän tiedostoon on lisätty VAIN kommentteja; itse koodia ei ole muutettu.
 *   Lisätyt kommentit on merkitty, jotta ne erottuvat alkuperäisistä:
 *     HUOM:    havaittu mahdollinen virhe / riski / epäselvyys (EI korjattu)
 *     OLETUS:  päätelty koodista, ei varmistettu laitteelta tai tekijältä
 *
 * MIKÄ TÄMÄ ON
 *   - Piiri on ATtiny816. Tiedostonimessä lukee "Attiny13", mutta käännöstiedostoissa
 *     on .t816 / megaTinyCore v2.6.11 / 20 MHz sisäinen kello / millis() pois (mNONE).
 *   - Mittaa valoportin analogisignaalia (PA7 = AIN7, ADC0). Kun kappale ohittaa
 *     portin, signaali nousee tai laskee ("reuna"). Koodi tallentaa reunan näytteet,
 *     sovittaa niihin käyrän ja laskee tarkan ajanhetken jolloin signaali ylitti
 *     50 % tason (alle yhden näytteen tarkkuudella).
 *   - Laite on I2C-orja. Isäntä (master) kalibroi laitteen, virittää mittauksen ja
 *     lukee tulokset. Samalla väylällä voi olla useita laitteita: osoite (0x40/0x41/
 *     0x42) valitaan sirun sarjanumeron (UID) mukaan setup():ssa.
 *
 * AJAT JA KELLO
 *   - Kaikki ajat ovat 20 MHz kellosyklejä (1 sykli = 50 ns, 1 µs = 20 sykliä).
 *   - 64-bittinen aikaleima = (tca_overflows << 16) | TCA0.CNT, ks. get_timestamp_cycles_safe().
 *   - ADC-näytejakso = TCD0.CMPBCLR = 550 sykliä = 27,5 µs (n. 36 kHz).
 *
 * LAITTEISTON JAKO
 *   PA7   analogitulo (valokenno), ADC0 kanava AIN7
 *   PA4   digitaalitulo -> EVSYS-kanava 0 -> TCB0 capture -> pa4_time (tarkka reunan aikaleima).
 *         OLETUS: PA4 on kytketty samaan synkronointiverkkoon kuin PA1 (koodi lukee vain PA4:ää,
 *         PA1:tä ei lueta koskaan).
 *   PA1   YHTEINEN synkronointilinja (käyttäjän vahvistama: kytketty ESP32:een ja muihin ATtiny816-
 *         laitteisiin). Lepotilassa tulo + pull-up (HIGH). ESP32:n lähettämä synkronointipulssi saapuu
 *         kaikkiin laitteisiin yhtä aikaa, joten kukin tallentaa sen oman kellonsa mukaan (pa4_time) ja
 *         ESP32 voi laskea kellojen erot. Kun raportointi on päällä, laite vetää linjan itse LOW 80 µs
 *         kun mitattava reuna ylittää 50 % (sendReport), eli muut näkevät tapahtuman (OLETUS).
 *   TCA0  vapaasti pyörivä 16-bit ajastin + ylivuotolaskuri = aikaleimojen perusta
 *   TCD0  näytteenottokello: CMPBCLR-event -> EVSYS-kanava 1 -> käynnistää ADC0-muunnoksen
 *   TCB0  capture-ajastin PA4-reunan aikaleimaukseen
 *   ADC0  10-bit, kaksi näytettä summataan yhdeksi lukemaksi; tulos-keskeytys = tilakone
 *
 * TIEDONSIIRTO I2C:LLÄ  (luvut koodataan 7 bittiä/tavu, MSB ensin, ks. write7Bit)
 *   Kirjoitus (master -> laite): [komento] [payload-tavut...]            -> receiveEvent()
 *   Luku (laite -> master):      master kirjoittaa ensin [komento](+payload), sitten lukee
 *                                vastauksen                              -> requestEvent()
 *   "Ei valmis" -vastaus = -1 = kaikki vastaustavut 0x7F.
 *   Komentotaulukko on CMD_*-määrittelyjen yläpuolella.
 *
 * TYÖNKULKU (OLETUS, päätelty koodista)
 *   1. Kalibrointi: CMD_ADC_START -> odota n. 113 ms -> CMD_ADC_RESULT (keskiarvo x 65536)
 *      -> kirjoita arvo CMD_ADC_LOWER:lla (pienempi ADC-taso) tai CMD_ADC_UPPER:lla
 *      (suurempi taso), kumpikin omalla mittauksellaan. Laite laskee rajat 10 % / 50 % / 90 %
 *      (setAlaYla).
 *   2. (valinnainen) CMD_REPORT_TRUE: laite antaa PA1-pulssin kun reuna ylittää 50 %.
 *   3. CMD_START: laite virittyy (vaihe 1) ja odottaa reunaa.
 *   4. Pollaa CMD_RESULT kunnes vastaus != -1 (vaihe 4) ja lue ekaAika (CMD_PRINT, valitsin 5).
 *      50 %-ylityksen absoluuttinen aika = ekaAika + vikaAika (OLETUS, ks. vikaAika).
 *   5. Tarvittaessa lue tallennetut näytteet CMD_PRINT:llä (debug / jälkianalyysi).
 * ============================================================================= */

#include <Wire.h>

// Wire.h (megaTinyCore): I2C-orja. onReceive/onRequest-käsittelijät ajetaan TWI-KESKEYTYKSESSÄ.
// util/delay.h: _delay_us(), käytössä lämpötilamittauksen käynnistyksessä ja lopetuksessa.
// avr/wdt.h: wdt_reset(). Vahtikoiraa ei käynnistetä missään tässä koodissa (OLETUS: se on
//   mahdollisesti sulakeasetuksilla), joten wdt_reset()-kutsut ovat muuten turhia.
#include <util/delay.h>
#include <avr/wdt.h>
#include <stdint.h>

/* ================= VAKIOT ================= */
// Oletusosoite jos laitteen UID ei ole setup():n tunnistuslistalla. Silloin myös lämpötilan
// kalibrointi (Temp_kerroin, Temp_offset) jää 0:ksi, ks. HUOM setup():ssa.
#define I2C_ADDR_DEFAULT   0x40

/* ================= KOMENNOT ================= */
// Komentotaulukko.  S = master kirjoittaa (receiveEvent),  L = master lukee vastauksen (requestEvent).
// Luku: master lähettää ensin komentotavun (+ mahdollinen payload) ja lukee vasta sitten.
//
//  #   komento           S/L  payload (S)  /  vastaus (L)
//  1   CMD_PA4_TIME      L    8 tavua: viimeisimmän PA4-reunan aikaleima (syklit); 1 = reunaa ei ole tullut
//  2   CMD_ADC_START     S    -   aloita keskiarvoistava ADC-mittaus (vaihe 5, 4096 lukemaa)
//  3   CMD_ADC_STOP      S    -   keskeytä ja nollaa (vaihe 0)
//  4   CMD_ADC_RESULT    L    8 tavua: keskiarvo x 65536 (Q16) kun vaihe 6, muuten -1. Nollaa tilan luettaessa
//  5   CMD_ADC_VALUE     L    2 tavua: viimeisin ADC-lukema (kahden näytteen summa, 0..2046)
//  6   CMD_ADC_LOWER     S    alarajan kalibrointiarvo (Q16) -> lower
//  7   CMD_ADC_UPPER     S    ylärajan kalibrointiarvo (Q16) -> upper
//  8   CMD_RESULT        L    8 tavua: vikaAika kun vaihe 4, muuten -1
//  9   CMD_START         S    -   viritä reunamittaus (vaihe 1)
// 10   CMD_STOP          S    -   keskeytä ja nollaa (vaihe 0)
// 11   CMD_I2C_TEST      L    1 tavu: aina 17 (yhteystesti)
// 12   CMD_RUNTIME       L    8 tavua: laitteen oma aikaleima nyt (syklit)
// 13   CMD_SPEED         -    EI TOTEUTETTU (ei käsittelyä receiveEvent- eikä requestEvent-funktiossa)
// 14   CMD_RESPONSE      -    EI TOTEUTETTU
// 15   CMD_PRINT         L    debug, 8 tavua. Valitsin lähetetään samassa kirjoituksessa komennon perässä:
//                             1 = last_ADC_idx (näytteitä)  2 = lowerLimit  3 = upperLimit
//                             4 = vikaAika  5 = ekaAika  >=10 = lista[valitsin - 10]
// 16   CMD_REPORT_TRUE   S    -   salli PA1-synkronointipulssi (kertakäyttöinen)
// 17   CMD_REPORT_FALSE  S    -   estä pulssi, PA1 takaisin tuloksi
// 18   CMD_VREF          S    ADC-referenssi 0..4 -> VREF.CTRLA (0=0,55 V 1=1,1 V 2=2,5 V 3=4,34 V 4=1,5 V)
// 19   CMD_ID            L    8 tavua: UID:n 4 viimeistä tavua (SERNUM6..9)
// 20   CMD_TEMP_START    S    -   aloita sisäisen lämpötila-anturin keskiarvoistus (vaihe 5)
// 21   CMD_TEMP_STOP     L    8 tavua: lämpötilatulos (yksikkö epäselvä); palauttaa ADC:n valoporttitilaan
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
#define CMD_VREF 18           //Aseta Vref
#define CMD_ID 19             //Palauta device ID
#define CMD_TEMP_START 20           //Aloita lämpötila mittaus
#define CMD_TEMP_STOP 21           //Palauta lämpötila mittaus

/* ================= DATA ================= */
// Tämän laitteen I2C-osoite. Valitaan setup():ssa UID:n perusteella.
uint8_t I2C_ADDR;        //I2C osoite

// Viimeisin I2C-komento (1. tavu master-kirjoituksesta). requestEvent() päättää tämän perusteella mitä vastataan.
volatile uint8_t last_cmd = 0;
// Viimeisimmän komennon payload purettuna (7 bittiä/tavu, MSB ensin). Nollautuu AINA uuden kirjoituksen alussa,
// myös kun master lähettää pelkän komentotavun (lukupyyntö). Ks. receiveEvent().
uint64_t t_last_message = 0;
// t_last_message typistettynä 16 bittiin. Käytössä vain CMD_PRINT:n valitsimena ja CMD_VREF:ssä
// (CMD_ADC_LOWER/UPPER käyttävät t_last_message:a suoraan).
uint16_t last_message=0;
// true = master on pyytänyt (CMD_REPORT_TRUE) PA1-synkronointipulssia kun reuna ylittää 50 %.
// Kertakäyttöinen: loop() nollaa tämän kun pulssi on annettu.
volatile bool report = false;

// Yhden reunan tallennetut näytteet. Arvo = ADC-lukema - lowerLimit (alle alarajan: 0).
// Indeksi last_ADC_idx. 70 alkiota = 140 tavua (varmistettu .map-tiedostosta).
// HUOM: indeksiä ei rajata kirjoitettaessa, ks. ADC-keskeytys.
volatile uint16_t lista[70];

// Viimeisin ADC-lukema. Yksi lukema = KAHDEN 10-bittisen näytteen SUMMA (ADC0.CTRLB = 1), alue 0..2046.
volatile uint16_t last_ADC;
// TILAKONE (muuttuu I2C-komennoilla ja ADC-keskeytyksessä):
//   0 = lepo, ei mittausta
//   1 = viritetty (CMD_START): odotetaan reunan alkua
//   2 = nouseva reuna tallennuksessa (aloitettiin alatasolta)
//   3 = laskeva reuna tallennuksessa (aloitettiin ylätasolta)
//   4 = reuna valmis, tulos (vikaAika) luettavissa CMD_RESULT:lla
//   5 = keskiarvoistava mittaus käynnissä (CMD_ADC_START tai CMD_TEMP_START)
//   6 = keskiarvo valmis, luettavissa CMD_ADC_RESULT:lla (tai CMD_TEMP_STOP:lla)
volatile uint8_t vaihe;
// 90 %:n taso (ADC-lukeman yksiköissä, kokonaisluku). Nouseva reuna päättyy kun lukema >= tämä.
volatile uint16_t upperLimit;
// 10 %:n taso. Laskeva reuna päättyy kun lukema <= tämä. Vähennetään myös tallennetuista näytteistä.
volatile uint16_t lowerLimit;
// Seuraava vapaa paikka lista[]:ssa = tallennettujen näytteiden määrä. HUOM: uint8_t, ei ylärajatarkistusta.
volatile uint8_t last_ADC_idx;
// MITTAUKSEN TULOS: reunan 50 %-ylityksen ajankohta SYKLEINÄ SUHTEESSA ekaAika:an
// (= estimate_center_time() * TCD0.CMPBCLR). HUOM: nimi "vikaAika" (viimeinen aika) voi johtaa harhaan:
// arvo ei ole absoluuttinen aikaleima. Vanha tapa (kommentoitu ADC-keskeytyksessä) tallensi aikaleiman suoraan.
// OLETUS: master laskee absoluuttisen ajan = ekaAika + vikaAika.
volatile uint64_t vikaAika;
// 50 %:n taso kokonaislukuna (ADC-lukeman yksiköt), vertailuihin ADC-keskeytyksessä.
volatile uint16_t mid;
// Sama 50 %:n taso desimaalitarkkuudella (ei katkaistu kokonaisluvuksi). Käytetään käyrän sovituksessa.
float midTarkka;
// Ensimmäisen tallennetun näytteen (lista[0]) absoluuttinen aikaleima, syklit, tämän laitteen kello.
// Näyte n on ajassa ekaAika + n * 550 sykliä.
volatile uint64_t ekaAika;
// Kaksikäyttöinen: (1) keskiarvoistuksessa ADC-lukemien summa, (2) valmistuttua keskiarvo x 65536 (Q16).
// Nimi ja sumX/sumY/sumXX ovat jäänteitä aiemmasta suorasovitusmenetelmästä.
volatile uint32_t sumXY;
// Lukemalaskuri keskiarvoistuksessa (vaihe 5).
volatile uint16_t sumX;
// Alarajan kalibrointiarvo Q16-muodossa (= ADC-keskiarvo x 65536), master asettaa CMD_ADC_LOWER:lla.
// Pienempi ADC-taso. HUOM: setAlaYla() olettaa upper > lower (uint32, muuten alivuoto).
uint32_t lower;
// Ylärajan kalibrointiarvo Q16-muodossa, master asettaa CMD_ADC_UPPER:lla. Suurempi ADC-taso.
uint32_t upper;
// Ei käytössä (vain nollataan).
uint16_t sumY;
// Ei käytössä (vain nollataan).
uint32_t sumXX;
// true kun tämän reunan 50 %-ylitykseen on jo reagoitu (PA1-pulssi) -> vain yksi pulssi per reuna.
// Nollataan kun uusi reuna alkaa (vaihe 1 -> 2 tai 3).
volatile bool sendedSync;
// Keskiarvoistuksen lukemamäärä - 1: 0x0FFF -> 4096 lukemaa = 4096 x 27,5 µs = n. 113 ms.
// HUOM: kommentoitu vaihtoehto 0x1FFFF ei mahdu uint16_t:hen (sumX on myös uint16_t).
volatile uint16_t keskiarvoitus = 0x0FFF; //0x1FFFF
// Ei käytössä.
volatile bool kellotetaanADC = false;
// ADC-referenssin valinta, arvo suoraan VREF.CTRLA:han (bitit 6:4 = ADC0REFSEL, bitit 2:0 = DAC0REFSEL).
// 0 = 0,55 V (oletus). Muuttuu CMD_VREF:llä.
uint8_t Vref = 0;
// Lämpötilan kalibrointi, laitekohtainen (asetetaan UID:n mukaan setup():ssa).
// Tulos CMD_TEMP_STOP:ssa = (keskiarvo / Temp_kerroin) + Temp_offset. Yksikkö epäselvä.
int16_t Temp_offset;
uint8_t Temp_kerroin;

// Ei käytössä.
union Osissa64{
  uint64_t luku;
  uint8_t lista[8];
};

// Aikaleimat ovat 64-bittisiä syklilukuja (20 MHz). Ks. get_timestamp_cycles_safe().
/* Aikaleimat */
// TCA0:n ylivuotojen lukumäärä (kasvaa 65536 syklin = 3,2768 ms välein). Aikaleiman ylemmät bitit.
volatile uint32_t tca_overflows = 0;
// Viimeisimmän PA4-reunan aikaleima (syklit). Alkuarvo 1 = reunaa ei ole vielä tullut.
// Päivittyy TCB0_INT_vect:ssa, luetaan CMD_PA4_TIME:lla. Uusi reuna ylikirjoittaa edellisen.
// Käyttö: ESP32:n synkronointipulssi tallentuu kaikissa laitteissa samalla fyysisellä hetkellä mutta kunkin
// oman kellon mukaan; ESP32 lukee arvot ja laskee laitteiden kellojen erot.
// HUOM (OLETUS: PA4 on samassa linjassa kuin PA1): myös minkä tahansa laitteen oma sendReport()-pulssi
//      laukaisee capturen kaikissa PA4:ään kytketyissä laitteissa ja ylikirjoittaa ESP32:n pulssin aikaleiman.
//      Lue pa4_time ennen kuin joku antaa reunapulssin.
volatile uint64_t pa4_time = 1;
// Ei käytössä (tämä ja pa4_ovf_snapshot).
volatile uint16_t pa4_capture = 0;
volatile uint32_t pa4_ovf_snapshot = 0;
// Aikaleima jolloin PA1-pulssi päätetään (loop() vertaa tähän). 0xFFFF... = ei pulssia käynnissä.
volatile uint64_t palautaSync = 0xFFFFFFFFFFFFFFFF;

// Lukee sirun sarjanumeron (SIGROW.SERNUM0..9, 10 tavua) alimmat 4 tavua (SERNUM6..9) 32-bittiseksi.
// Tunnistaa laitteen: valitsee I2C-osoitteen ja lämpötilakalibroinnin (setup()) ja palautetaan CMD_ID:llä.
// Kolme tavua lisää (SERNUM3..5) on kommentoitu pois.
uint32_t read_uid() {
  uint32_t tulos = 0;
  //tulos = (tulos << 8) | SIGROW.SERNUM3;
  //tulos = (tulos << 8) | SIGROW.SERNUM4;
  //tulos = (tulos << 8) | SIGROW.SERNUM5;
  tulos = (tulos << 8) | SIGROW.SERNUM6;
  tulos = (tulos << 8) | SIGROW.SERNUM7;
  tulos = (tulos << 8) | SIGROW.SERNUM8;
  tulos = (tulos << 8) | SIGROW.SERNUM9;
  return tulos;
  }
// TCD0 = ADC:n NÄYTTEENOTTOKELLO. One-ramp-tila: laskuri juoksee 0..CMPBCLR ja alkaa alusta.
// CMPBCLR-vertailu generoi tapahtuman (event), joka käynnistää ADC-muunnoksen (ks. evsys_init).
// Jakso = n. 550 sykliä = 27,5 µs. Koodi käyttää arvoa 550 aikaskaalana (TCD0.CMPBCLR) kun näyteindeksi
// muutetaan sykleiksi.
// HUOM: One-ramp-tilassa jakso voi olla CMPBCLR + 1 = 551 sykliä (tarkista datasheetistä). Silloin
//       vikaAika:n mittakaava olisi n. 0,18 % liian pieni.
void tcd0_init(){
  TCD0.CTRLA=0;
  TCD0.CTRLB = 0; //One ramp
  TCD0.CMPBCLR = 550;
  //TCD0.CTRLA |= TCD_CLKSEL_SYSCLK_gc | TCD_ENABLE_bm;
  // CTRLA = 1: vain ENABLE. Kellolähde CLKSEL = 0 (OSC20M; OLETUS: sama 20 MHz kuin järjestelmäkello,
  // jolloin TCD0:n syklit = TCA0:n syklit), esijakajat 1.
  TCD0.CTRLA =  1;

  }

// ADC0: 10-bit, tulo PA7/AIN7, kaksi näytettä summataan yhdeksi lukemaksi, aloitus TCD0-tapahtumasta
// (ei vapaasti pyörivä), tulos-keskeytys päällä (ISR(ADC0_RESRDY_vect) alla).
void adc_init(){
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc; //Ei pullup ja Digital input buffer disabled

  // Referenssijännite. Oletus Vref = 0 -> 0,55 V (ei vielä muutettu CMD_VREF:llä).
  VREF.CTRLA = Vref;                          //Vref arvo ADC0
  // Pakotetaan ADC-referenssi aina päälle (ei käynnistysviivettä muunnoksiin).
  VREF.CTRLB = VREF_ADC0REFEN_bm;             //ADC Vref aina päällä

  ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;           //Single ended pinnin valinta
  // SAMPNUM = 1 -> 2 näytettä KERTYY SUMMAKSI (ei jaeta keskiarvoksi). Siksi last_ADC on 0..2046.
  ADC0.CTRLB  = 1;                            //Keskiarvoitus 2^n
  // CTRLC: bitti 6 = SAMPCAP, bitit 5:4 = REFSEL (0 = sisäinen ref, 1 = VDD), bitit 2:0 = esijakaja.
  // HUOM: SAMPCAP:ia ei päivitetä kun Vref vaihdetaan CMD_VREF:llä (kommentin mukaan 1 kun Vref > 1 V).
  ADC0.CTRLC  = 0 << 6;                       //capasitanssi. 0 kun Vref<1V, 1 kun Vref>1V
  ADC0.CTRLC |= 0 << 4;                       //0 Vref, 1 VDD
  // HUOM: ADC-kello = 20 MHz / 8 = 2,5 MHz. Datasheetin 10-bit-ADC:n raja on n. 1,5 MHz (n. 115 ksps);
  //       tarkista, onko tämä tarkoituksella spesifikaation ulkopuolella (DIV16 = 1,25 MHz olisi rajoissa).
  // Yksi lukema (2 muunnosta) kestää n. 10,4 µs, mikä mahtuu 27,5 µs näytejaksoon.
  ADC0.CTRLC |= ADC_PRESC_DIV8_gc;            //ADC_clk = CLK/DIV
  // STARTEI: muunnos käynnistyy tapahtumasta (EVSYS-kanava 1 = TCD0 CMPBCLR), ei ohjelmallisesti.
  ADC0.EVCTRL = ADC_STARTEI_bm;

  // Tyhjennetään mahdollinen vanha valmis-lippu ja sallitaan valmis-keskeytys.
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  ADC0.INTCTRL = ADC_RESRDY_bm;               //Result Ready Interrupt Enable
  ADC0.CTRLA  = ADC_ENABLE_bm;                //ADC enabled
  }

// ===== ADC-VALMIS-KESKEYTYS = MITTAUKSEN TILAKONE =====
// Ajetaan jokaisen ADC-lukeman jälkeen (n. 27,5 µs välein). Toimii tilamuuttujan "vaihe" mukaan
// (vaiheissa 0, 4 ja 6 vain last_ADC päivittyy):
//   vaihe 1  viritetty, odottaa reunan alkua; päättelee suunnan -> vaihe 2 (nouseva) tai 3 (laskeva)
//   vaihe 2  nouseva reuna: tallentaa näytteet lista[]:an kunnes lukema >= upperLimit (90 %) -> vaihe 4
//   vaihe 3  laskeva reuna: tallentaa näytteet kunnes lukema <= lowerLimit (10 %) -> vaihe 4
//   vaihe 5  keskiarvoistus (kalibrointi tai lämpötila): summaa 4096 lukemaa -> vaihe 6
// Reunan valmistuttua tästä keskeytyksestä kutsutaan raskasta estimate_center_time()-laskentaa.
// Suunta päätellään vain ensimmäisestä "välitason" lukemasta: alle mid = nouseva, mid..upperLimit = laskeva.
ISR(ADC0_RESRDY_vect){
  // Luetaan tulos. OLETUS: RES:n lukeminen kuittaa RESRDY-lipun.
  last_ADC = ADC0.RES;            //Luetaan ADC arvo
  // Kuittaus. Tekijän kysymykseen "Turha??": OLETUS että RES:n luku jo kuittaa lipun, joten tämä on turha mutta
  // harmiton. Se takaa joka tapauksessa, että vaihe 1:n odotussilmukka (while !RESRDY) odottaa UUTTA tulosta.
  ADC0.INTFLAGS = ADC_RESRDY_bm;              //kuitataan että luettu. Turha??
  // --- vaihe 2: nouseva reuna käynnissä (aloitettiin alatasolta) ---
  if(vaihe==2){                                       //mittaus alkanut alhaalta
    // Vielä alle 90 %: jatketaan tallennusta.
    if(last_ADC<upperLimit){                                //vielä ei ole ylittänyt ylärajaa
      uint16_t v = 0;
      // Ensimmäinen lukema joka on 50 %:ssa tai yli: anna PA1-synkronointipulssi (jos CMD_REPORT_TRUE) vain kerran
      // per reuna (sendedSync). Huom: vaiheessa 1 tallennettua ensimmäistä näytettä (lista[0]) ei tarkisteta tässä,
      // joten pulssi voi viivästyä yhden näytteen (27,5 µs).
      if((last_ADC>=mid) && !sendedSync){
        if(report){
          sendReport();
        }
        sendedSync=true;
      }
      // Tallennetaan "lukema - lowerLimit" (alle rajan: 0). estimate_center_time() lisää lowerLimit:n takaisin.
      if(last_ADC>lowerLimit){
        v = last_ADC - lowerLimit;
      }
      // HUOM: last_ADC_idx:lle ei ole ylärajatarkistusta (lista[] on 70 alkiota, indeksi uint8_t). Jos reuna ei
      //       koskaan päätä (signaali ei saavuta 90 %:a tai suunta tulkittiin väärin vaiheessa 1), tallennus
      //       kirjoittaa taulukon yli muuhun RAM-muistiin.
      lista[last_ADC_idx]=v;
      last_ADC_idx++;
    }else{                                                  //ylitti ylärajan
      //vikaAika = get_timestamp_cycles_safe();
      // 90 % ylitetty -> reuna valmis. Sovitetaan käyrä ja lasketaan 50 %-ylitys (näyteindeksi, murtoluku) x
      // näytejakso (TCD0.CMPBCLR syklejä) = vikaAika. HUOM: tulos on SUHTEESSA ekaAika:an (ei absoluuttinen).
      // Raskas float-laskenta tehdään tässä keskeytyksessä, ks. HUOM estimate_center_time():n kohdalla.
      vikaAika= (estimate_center_time()*TCD0.CMPBCLR);
      // Tulos valmis (luettavissa CMD_RESULT:lla). Tilassa 4 näytteitä ei enää tallenneta.
      vaihe = 4;
    }
  }else if(vaihe == 3){                                     //mittaus alkanut ylhäältä
    // --- vaihe 3: laskeva reuna käynnissä (aloitettiin ylätasolta) ---
    // Peilikuva vaiheesta 2: tallennus kunnes lukema <= lowerLimit (10 %); PA1-pulssi kun lukema <= mid.
    if(last_ADC>lowerLimit){                                //vielä ei alittanut alarajaa
      uint16_t v = 0;
      // (Tässä on bittioperaattori & eikä &&; vaikutus on sama.)
      if((last_ADC<=mid) & !sendedSync){
        if(report){
          sendReport();
        }
        sendedSync=true;
      }
      // HUOM: ehto on tässä aina tosi (ulompi if varmistaa jo last_ADC>lowerLimit): turha mutta harmiton.
      if(last_ADC>lowerLimit){
        v = last_ADC - lowerLimit;
      }
      lista[last_ADC_idx]=v;
      last_ADC_idx++;
    }else{                                                  //alitti alarajan
      //vikaAika = get_timestamp_cycles_safe();
      vikaAika= (estimate_center_time()*TCD0.CMPBCLR);
      vaihe = 4;
    }
  }else if(vaihe == 1){                                           //ei vielä alkanut mittaus
    // --- vaihe 1: viritetty, reunaa ei vielä havaittu ---
    // Suunta päätellään ensimmäisestä lukemasta, joka on rajojen (10 %..90 %) välissä:
    //   lukema < mid (ja > lowerLimit)  -> tuli alhaalta -> NOUSEVA reuna (vaihe 2)
    //   lukema >= mid (ja < upperLimit) -> tuli ylhäältä -> LASKEVA reuna (vaihe 3)
    //   lukema <= lowerLimit tai >= upperLimit -> ollaan vielä lepotasolla, odotetaan.
    // HUOM: jos reuna on niin nopea, että yksi lukema hyppää 10 %:sta yli 50 %:n, nouseva reuna tulkitaan laskevaksi
    //       (vaihe 3). Silloin lukema ei koskaan alita lowerLimit:iä ja tallennus jatkuu (ks. HUOM lista[]-indeksi).
    if(last_ADC<mid){                                       //Onko alhaalla
      if(last_ADC>lowerLimit){                              //tullut alhaalta
        //odotetaan että uusi mittaus valmis
        // Nousevan reunan alku. Odotetaan SEURAAVAA tulosta ISR:n sisällä (aktiivinen odotus, n. 27 µs), jotta
        // aikaleima ekaAika saadaan tarkasti heti tuloksen valmistuttua. Tämä toinen lukema on lista[0]; ensimmäinen
        // (jolla reuna havaittiin) hylätään.
        // HUOM: odotus pitää CPU:n ISR:ssä (I2C-, TCA0- ja TCB0-keskeytykset viivästyvät) eikä siinä ole aikarajaa:
        //       jos ADC-tapahtumat loppuvat (TCD0 pysähtyy), koodi jää tähän ikuisesti.
        wdt_reset();
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
            // Tyhjä silmukka, odotetaan
        }
        last_ADC = ADC0.RES;            //Luetaan ADC arvo
        // Aikaleima otetaan heti tuloksen valmistuttua: ekaAika = lista[0]:n ajankohta (absoluuttinen).
        ekaAika = get_timestamp_cycles_safe();

        uint16_t v = 0;
        if(last_ADC>lowerLimit){
          v = last_ADC - lowerLimit;
        }
        lista[last_ADC_idx]=v;
        last_ADC_idx++;
        // Siirry nousevan reunan tallennukseen. sendedSync nollataan, jotta tälle reunalle voi antaa PA1-pulssin.
        vaihe = 2;
        sendedSync=false;
      }
    }else if((last_ADC<upperLimit)){                        //tullut ylhäältä
      // Laskevan reunan alku: lukema on 50 %..90 % välissä -> tuli ylhäältä. Sama kuin yllä (odota seuraava
      // tulos, ekaAika, lista[0]), mutta siirrytään vaiheeseen 3.
      //odotetaan että uusi mittaus valmis
      wdt_reset();
      while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
          // Tyhjä silmukka, odotetaan
      }
      last_ADC = ADC0.RES;            //Luetaan ADC arvo
      ekaAika = get_timestamp_cycles_safe();


      uint16_t v = 0;
      if(last_ADC>lowerLimit){
        v = last_ADC - lowerLimit;
      }
      lista[last_ADC_idx]=v;
      last_ADC_idx++;
      // Siirry laskevan reunan tallennukseen.
      vaihe=3;
      sendedSync=false;
    }
    }
  else if(vaihe == 5){                                     //mitataan keskiarvoa
    // --- vaihe 5: keskiarvoistus (kalibrointi: CMD_ADC_START, lämpötila: CMD_TEMP_START) ---
    // Summataan ADC-lukemat sumXY:hyn (kukin lukema on 2 näytteen summa).
    sumXY += last_ADC;
    // Kun 4096 lukemaa on kerätty (sumX ehtii 0..4095):
    if(sumX >= keskiarvoitus){                                          
      // Keskiarvo Q16-muodossa: summa x 65536 / 4096. Kalibrointiarvossa on siis 16 desimaalibittiä; setAlaYla()
      // jakaa 65536:lla takaisin ADC-yksiköihin. Tulos korvaa summan sumXY:ssä.
      sumXY = (uint64_t)((uint64_t)sumXY << 16)/((uint64_t)(keskiarvoitus)+(uint64_t)1);
      // Valmis: luettavissa CMD_ADC_RESULT:lla (tai CMD_TEMP_STOP:lla).
      vaihe=6; 
    }
    sumX++;
  }
  }

// PA4 = tulo. ISC = INTDISABLE: pinnin oma keskeytys pois, mutta digitaalinen tulopuskuri PÄÄLLÄ (tarvitaan,
// jotta tapahtumajärjestelmä näkee pinnin). Sisäistä pull-uppia ei ole: OLETUS että signaali ajetaan ulkoa.
void pa4_event_init() {
  PORTA.DIRCLR = PIN4_bm;
  PORTA.PIN4CTRL = PORT_ISC_INTDISABLE_gc;
  PORTA.INTFLAGS = PIN4_bm;
  }

// Tapahtumajärjestelmä (EVSYS), kaksi asynkronista kanavaa:
//   kanava 0: generaattori 0x0E (PORTA PIN4)      -> käyttäjä 0 = TCB0 (capture)
//   kanava 1: generaattori 0x04 (TCD0 CMPBCLR)    -> käyttäjä 1 = ADC0 (muunnoksen käynnistys)
// ASYNCUSERn-arvo valitsee kanavan: 0 = pois, 1 = SYNCCH0, 2 = SYNCCH1, 3 = ASYNCCH0, 4 = ASYNCCH1.
// (OLETUS: käyttäjä 0 = TCB0, käyttäjä 1 = ADC0; jälkimmäinen on tekijän omassa kommentissa.)
void evsys_init(void) {
  // Generator: PORTA PIN4 → ASYNC channel 0
  EVSYS.ASYNCCH0 = 0x0E;
  EVSYS.ASYNCUSER0 = 0x3;


  EVSYS.ASYNCCH1 = 0x04; //TCD0_CMPBCLR
  EVSYS.ASYNCUSER1 = 0x4; //multiplexer ADC0, Asyncch1
  }


// TCB0 = PA4-reunan TARKKA aikaleimaus. Input Capture Event -tila: kun PA4:n tapahtuma tulee, laskurin arvo
// kopioituu CCMP:hen ja syntyy keskeytys. Kello = TCA0:n kello (20 MHz), vapaasti pyörivä 16-bit laskuri.
// Reunan suunta määräytyy EVCTRL.EDGE-bitistä (nyt 0, ei asetettu).
void tcb0_init() {
  TCB0.CTRLA = 0; // stop
  TCB0.CTRLB = TCB_CNTMODE_CAPT_gc; // Capture mode
  TCB0.EVCTRL = TCB_CAPTEI_bm;      // Enable event input
  TCB0.INTCTRL = TCB_CAPT_bm;       // Capture interrupt
  TCB0.INTFLAGS = TCB_CAPT_bm;
  TCB0.CNT = 0;
  // CLKSEL = TCA0:n kello, SYNCUPD = TCB0:n päivitys synkronoidaan TCA0:n kanssa, ENABLE.
  // Koodi ei oleta että TCB0.CNT == TCA0.CNT; se käyttää vain erotusta (ks. TCB0_INT_vect).
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_SYNCUPD_bm | TCB_ENABLE_bm;

  }

// PA4-reuna on tapahtunut hetkellä, jolloin TCB0 oli arvossa CCMP. Keskeytykseen reagoidaan vasta hetken
// myöhemmin, joten lasketaan kuluneet syklit delta = CNT - CCMP (16-bit, kiertää oikein) ja vähennetään ne
// nykyisestä absoluuttisesta aikaleimasta: pa4_time = reunan todellinen aikaleima (TCA-aikaskaalassa).
// Pätee kun keskeytyksen viive < 65536 sykliä (3,3 ms).
// HUOM: ADC-keskeytyksen float-laskenta (estimate_center_time) voi viivästyttää tätä keskeytystä; jos viive
//       ylittää 3,3 ms, pa4_time heittää 65536 syklin monikerran.
ISR(TCB0_INT_vect) {
  // Clear interrupt
  TCB0.INTFLAGS = TCB_CAPT_bm;

  uint16_t cap = TCB0.CCMP;
  uint16_t aika1 = TCB0.CNT;
  uint64_t aika2 = get_timestamp_cycles_safe();
  uint16_t delta = (uint16_t)(aika1 - cap);
  pa4_time = aika2 - (uint64_t)delta;

  
  }

// TCA0 = aikaleimojen perusta: vapaasti pyörivä 16-bit laskuri 0..0xFFFF, esijakaja 1 (20 MHz).
// Ylivuoto 65536 syklin (3,2768 ms) välein -> TCA0_OVF_vect kasvattaa tca_overflows:ia.
void tca0_init(){
  TCA0.SINGLE.CTRLA = 0; // pysäytä
  TCA0.SINGLE.CTRLD = 0; // SINGLE-mode
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
  TCA0.SINGLE.PER = 0xFFFF;

  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

  
  }




// Laskee TCA0:n ylivuodot (aikaleiman ylemmät bitit).
// HUOM: jos tämän keskeytyksen palvelu viivästyy yli yhden jakson (3,3 ms) kahden ylivuodon yli, vain yksi
//       ylivuoto lasketaan ja aikaleima jää jälkeen 65536 sykliä. Pitkät ISR:t (ks. estimate_center_time)
//       ovat tässä riski.
ISR(TCA0_OVF_vect){
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;   //kuittaus
  tca_overflows++;
  }

// Palauttaa 64-bittisen aikaleiman (20 MHz syklejä) = (tca_overflows << 16) | TCA0.CNT.
// Kutsuttavissa myös keskeytyksestä (SREG tallennetaan ja palautetaan).
static inline uint64_t get_timestamp_cycles_safe(void){ //tarkista struct järjestys
  // Muistiasettelu (AVR, little-endian, ei täytetavuja): tavut 0..1 = cnt, 2..5 = high, 6..7 = reserved,
  // eli full = cnt | (high << 16) | (reserved << 48).
  // Union joka yhdistää tca_overflows (32b) ja cnt (16b) 64-bittiseksi
  typedef union {
      struct {
          uint16_t cnt;    // alimmainen 16 bittiä
          uint32_t high;   // seuraavat 32 bittiä
          uint16_t reserved; // ylimääräinen 16 bittiä, jotta union on 64 bittiä
      };
      uint64_t full;
  } timestamp_u;

  // HUOM: ts jää alustamatta, joten "reserved" (full:n bitit 48..63) on määrittelemätön eikä sitä aseteta
  //       missään. Kaikkien aikaleimojen ylin 16 bittiä voi siis sisältää roskaa. Vaikuttaa mm. CMD_RUNTIME,
  //       pa4_time, palautaSync-vertailuun loop():ssa ja ekaAika:an. write7Bit(.., 8) lähettää bitit 0..55, joten
  //       roska pääsee mukaan 8 ylimpään bittiin (48..55). Alla kommentoitu vanha versio ((high << 16) | cnt) ei
  //       kärsinyt tästä. Huom: repossa olevat .lst/.hex on käännetty tuosta vanhasta versiosta.
  timestamp_u ts;

  // Keskeytykset pois, jotta tca_overflows ja TCA0.CNT luetaan yhtenä kokonaisuutena.
  uint8_t sreg = SREG;
  cli();  // Keskeytykset pois

  ts.high = tca_overflows;           // asetetaan yläosa
  ts.cnt  = TCA0.SINGLE.CNT;         // alapuoli

  // Overflow-tarkistus
  // Ylivuoto on ehtinyt tapahtua, mutta ISR ei ole voinut vielä kasvattaa laskuria (keskeytykset ovat pois):
  // korjataan itse (high++) ja luetaan CNT uudelleen (nyt ylivuodon jälkeinen arvo). Näin aikaleima ei hyppää taaksepäin.
  if(TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm){
      ts.cnt = TCA0.SINGLE.CNT;
      ts.high++;
  }

  SREG = sreg;  // palautetaan keskeytykset
  return ts.full;


  // Vanha toteutus (kommentoituna): sama logiikka ilman unionia, palautti (high << 16) | cnt.
  /*
    uint64_t high;
    uint16_t cnt;
    uint8_t sreg = SREG;
    cli();
    high = tca_overflows;
    cnt  = TCA0.SINGLE.CNT;
    if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) {
      cnt = TCA0.SINGLE.CNT;
      high ++;
    }
    SREG = sreg;
    return ((high << 16) | cnt);*/
  }

// Kirjoittaa luvun I2C-vastaukseen 7 bittiä/tavu, MSB ensin, "bytes" tavua (8 tavua = 56 bittiä; ylimmät
// bitit katoavat). Tavun bitti 7 on aina 0. Master purkaa: arvo = (arvo << 7) | (tavu & 0x7F).
// Esim. write7Bit(-1, 8) lähettää 8 x 0x7F = "ei valmis".
// Kutsutaan requestEvent():stä (TWI-keskeytys).
void write7Bit(uint64_t val, uint8_t bytes) {

  for (int8_t i = bytes - 1; i >= 0; i--) {
    uint8_t chunk = (val >> (i * 7)) & 0x7F;
    Wire.write(chunk);
  }
  
  }

// PA1-synkronointipulssi ulos: linja LOW nyt, vapautus 80 µs (80 * 20 sykliä) päästä (loop() vapauttaa).
// PA1 on yhteinen linja ESP32:n ja muiden ATtiny816-laitteiden kanssa (käyttäjän vahvistama), joten pulssin
// näkevät kaikki yhtä aikaa. OLETUS: ne ottavat sen aikaleiman PA4-capturella (pa4_time). Kutsutaan
// ADC-keskeytyksestä kun reuna ylittää 50 %
// ja report == true. PA1 on tällöin lähtönä, koska CMD_REPORT_TRUE asetti sen (OUT = HIGH, DIR = lähtö).
void sendReport(){
  PORTA.OUTCLR = PIN1_bm;   // LOW
  palautaSync = get_timestamp_cycles_safe() + 80*20; //80us eteenpäin
  }
// Nollaa mittauslaskurit ja tallennusindeksin (EI vaihetta, rajoja, ekaAika:a tai vikaAika:a).
// Käytännössä merkitystä vain sumXY:llä, sumX:llä ja last_ADC_idx:llä; sumY ja sumXX ovat jäänteitä.
void nollaa(){
  sumY = 0;
  sumXY = 0;
  sumX = 0;
  sumXX = 0;
  last_ADC_idx = 0; 
  }


// Laskee reunan 50 %-ylityksen ajankohdan NÄYTEINDEKSINÄ (murtoluku) suhteessa lista[0]:aan.
// Menetelmä:
//   1. Pienimmän neliösumman sovitus polynomilla  y(x) = a + b*x + c*x^2 + e*x^4  (ei x^3-termiä;
//      kommentti "1,2,4 aste"). x = näyteindeksi - keskiindeksi (keskitys parantaa numeerista vakautta).
//   2. Normaaliyhtälöt (4x4) ratkaistaan Gauss-Jordan-eliminaatiolla -> kertoimet a, b, c, e.
//   3. Ratkaistaan y(x) = midTarkka (50 %-taso) Newtonin menetelmällä: alkuarvaus lineaarisesta osasta,
//      sitten 2 iteraatiota.
//   4. Palautetaan x + keskiindeksi. Kutsuja kertoo tuloksen näytejaksolla (TCD0.CMPBCLR) -> syklit.
// Toimii samoin nousevalle ja laskevalle reunalle (b:n etumerkki vaihtuu).
// HUOM: laskenta on ohjelmallista float-laskentaa 8-bittisellä AVR:llä (ei FPU:ta) ja ajetaan ADC-keskeytyksessä.
//       Suoritusaikaa ei ole mitattu; OLETUS (karkea arvio): useita millisekunteja kun näytteitä on kymmeniä.
//       Sinä aikana I2C-, TCA0- ja TCB0-keskeytykset odottavat (ks. HUOM TCA0_OVF:n ja TCB0:n kohdalla).
float estimate_center_time(){ //1,2,4 aste
    // n = tallennettujen näytteiden määrä.
    uint16_t n = last_ADC_idx;
    // Keskiindeksi. x = i - x_mean keskittää datan nollan ympärille.
    float x_mean = 0.5f * (n - 1);

    // Sk = summa(x^k). HUOM: S7 ja T3 lasketaan mutta niitä ei käytetä (kuuluisivat x^3-termiin, joka on jätetty pois).
    // Momenttisummat
    float S0 = 0.0f, S1 = 0.0f, S2 = 0.0f, S3 = 0.0f;
    float S4 = 0.0f, S5 = 0.0f, S6 = 0.0f, S7 = 0.0f, S8 = 0.0f;

    // Dataan liittyvät summat
    float T0 = 0.0f, T1 = 0.0f, T2 = 0.0f, T3 = 0.0f, T4 = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        float x  = (float)i - x_mean;
        float x2 = x * x;
        float x3 = x2 * x;
        float x4 = x2 * x2;
        float x5 = x3 * x2;
        float x6 = x3 * x3;
        float x7 = x4 * x3;
        float x8 = x4 * x4;

        // Palautetaan alkuperäinen ADC-lukema (tallennuksessa vähennettiin lowerLimit). Alarajalle leikatut (0:ksi
        // tallennetut) näytteet palautuvat arvona lowerLimit.
        float y = lista[i]+lowerLimit;

        S0 += 1.0f;
        S1 += x;
        S2 += x2;
        S3 += x3;
        S4 += x4;
        S5 += x5;
        S6 += x6;
        S7 += x7;
        S8 += x8;

        T0 += y;
        T1 += x  * y;
        T2 += x2 * y;
        T3 += x3 * y;
        T4 += x4 * y;
    }

    // Normaaliyhtälöt kantafunktioille [1, x, x^2, x^4]: rivin i alkio j = summa(f_i * f_j), oikea puoli = summa(f_i * y).
    // Vasen osa: S0..S8 (x:n potenssien summat), viimeinen sarake: T0, T1, T2, T4.
    // Normaaliyhtälöiden matriisi (4x4) ja oikea puoli
    float A[4][5] = {
        { S0, S1, S2, S4, T0 },
        { S1, S2, S3, S5, T1 },
        { S2, S3, S4, S6, T2 },
        { S4, S5, S6, S8, T4 }
    };

    // Ratkaisu paikallaan: lopuksi A[i][4] = ratkaistu kerroin. Ei osittaispivotointia (OLETUS: normaalimatriisin
    // diagonaali ei ole nolla). HUOM: matriisin alkiot vaihtelevat suuruusluokasta 1 aina x^8-summiin (luokka 1e12
    //       kun n on iso), joten float32:n tarkkuus voi heikentää ratkaisua.
    // Gauss–Jordan-eliminaatio
    for (int i = 0; i < 4; i++)
    {
        float inv = 1.0f / A[i][i];
        for (int j = i; j < 5; j++)
            A[i][j] *= inv;

        for (int k = 0; k < 4; k++)
        {
            if (k == i) continue;
            float f = A[k][i];
            for (int j = i; j < 5; j++)
                A[k][j] -= f * A[i][j];
        }
    }

    // Sovitetun polynomin kertoimet: y(x) = a + b*x + c*x^2 + e*x^4.
    float a = A[0][4];
    float b = A[1][4];
    float c = A[2][4];
    float e = A[3][4];

    // Alkuarvaus: pelkän lineaarisen osan (a + b*x = midTarkka) ratkaisu. Kommentin "y(x)=0.5" tilalla käytetään
    // todellisuudessa midTarkka:a (50 %-taso ADC-yksiköissä).
    // Keskipisteen ratkaisu: y(x)=0.5
    float x = (midTarkka - a) / b;   // alkuarvaus

    for (int it = 0; it < 2; it++)
    {
        float f  = a + b*x + c*x*x + e*x*x*x*x - midTarkka;
        // HUOM: derivaatta f'(x) = b + 2*c*x + 4*e*x^3, mutta koodissa x^3-termin kerroin on 3.0f (pitäisi olla 4.0f).
        //       Newton toimii silti (lähes-Newton), mutta konvergoi hitaammin; vain 2 iteraatiota -> pieni jäännösvirhe
        //       mahdollinen kun e*x^3 on merkittävä. Ei korjattu.
        float df = b + 2.0f*c*x + 3.0f*e*x*x*x;
        x -= f / df;
    }

    // x on keskitetty indeksi; lisätään keskiindeksi takaisin -> indeksi suhteessa lista[0]:aan.
    return x + x_mean;
  }

// Laskee mittausrajat kalibrointiarvoista (Q16, kun jaetaan 65536:lla saadaan ADC-lukeman yksikkö):
//   lowerLimit = lower + 10 % * (upper - lower)     (10 %-taso)
//   upperLimit = lower + 90 % * (upper - lower)     (90 %-taso)
//   mid        = (lower + upper) / 2                (50 %-taso; kokonaisluku ja desimaalitarkka midTarkka)
// Kutsutaan aina kun lower tai upper asetetaan. Rajat ovat järkeviä vasta kun molemmat on asetettu.
// HUOM: laskenta uint32-aritmetiikalla olettaa upper > lower; jos ei, (upper - lower) kiertää ympäri.
void setAlaYla(){
  lowerLimit=((upper-lower)/10+lower) / 65536;
  upperLimit=((upper-lower)*9/10+lower) / 65536;
  mid = ((lower+upper)/2) / 65536;
  midTarkka = ((lower+upper)/2) / 65536.f;
  }

// I2C-VASTAANOTTO (master kirjoittaa). Ajetaan TWI-keskeytyksessä. Muoto: [komento] [payload-tavut...].
// Payload puretaan 7 bittiä/tavu, MSB ensin, ja käsitellään heti. Käsittelee vain kirjoituskomennot;
// lukukomentoihin (CMD_PA4_TIME, CMD_RESULT, ...) vastataan vasta requestEvent():ssä.
void receiveEvent(int){
  if (Wire.available()){
    last_cmd = Wire.read();
  }
  // HUOM: nollautuu myös kun master lähettää pelkän komentotavun. Siksi esim. CMD_PRINT:n valitsin pitää
  //       lähettää samassa kirjoituksessa komennon kanssa ennen lukua.
  t_last_message = 0;
  while (Wire.available()) {
    t_last_message = (t_last_message << 7) | (Wire.read() & 0x7F);
  }
  // Typistys 16 bittiin (uint16_t last_message).
  last_message=t_last_message;
  switch (last_cmd) {
    // Kalibrointiarvo (Q16) talteen ja rajat uusiksi.
    case CMD_ADC_LOWER:{
      lower = t_last_message;
      setAlaYla();
      break;
      }
    case CMD_ADC_UPPER:{
      upper = t_last_message;
      setAlaYla();
      break;
      }
    // Viritä reunamittaus: nollaa laskurit ja tallennusindeksin (vanhat lista[]-tulokset mitätöityvät), vaihe 1.
    case CMD_START:{
      nollaa(); 
      vaihe = 1;
      break;
      }
    case CMD_STOP:{
      nollaa(); 
      vaihe = 0;
      break;
      }
    // Aloita keskiarvoistava mittaus (kalibrointi): 4096 lukemaa n. 113 ms, lopuksi vaihe 6.
    case CMD_ADC_START:{
      nollaa();
      vaihe = 5;
      break;
      }
    case CMD_ADC_STOP:{
      nollaa(); 
      vaihe = 0;
      break;
      }
    // Salli synkronointipulssi: PA1 lähdöksi ja HIGH:ksi (OUTSET ennen DIRSET, ettei linjaan tule LOW-piikkiä).
    // Kertakäyttöinen: loop() palauttaa PA1:n tuloksi ja nollaa report:in pulssin jälkeen.
    // HUOM: ATtiny ajaa PA1:n push-pull-tilassa HIGH:ksi niin kauan kuin report on päällä (ei open-drain).
    //       Jaetulla linjalla (ESP32 + muut ATtinyt) syntyy oikosulkuvirta, jos joku muu vetää linjan LOW
    //       samalla hetkellä: esim. ESP32:n synkronointipulssi, toisen laitteen sendReport() tai toinen laite
    //       jolla report on myös päällä. Turvallista vain jos kerrallaan vain yksi laite on report-tilassa ja
    //       ESP32 ei pulssita sillä välin, tai jos linjassa on sarjavastukset (ei tiedossa).
    case CMD_REPORT_TRUE:{
      report=true;
      PORTA.OUTSET = PIN1_bm;
      PORTA.DIRSET = PIN1_bm;
      break;
      }
    // Estä pulssi: PA1 takaisin tuloksi + pull-up (linja vapaa).
    case CMD_REPORT_FALSE:{
      report=false;
      PORTA.DIRCLR = PIN1_bm;
      PORTA.PIN1CTRL = PORT_PULLUPEN_bm;
      break;
      }
    // Valitsee ADC-referenssin. Payload (0..4) kopioidaan VREF.CTRLA:n molempiin nibbleihin (ADC0REFSEL ja DAC0REFSEL).
    // HUOM: kalibrointi (lower/upper) on tehtävä uudelleen referenssin vaihdon jälkeen; SAMPCAP-bittiä ei päivitetä.
    case CMD_VREF:{
      Vref = last_message << 4 | last_message;
      VREF.CTRLA = Vref;                          //Vref arvo ADC0
      break;
      }
    // Lämpötilamittaus: referenssi 1,1 V (VREF.CTRLA = 0x11), ADC-tuloksi sisäinen lämpötila-anturi (MUXPOS 0x1E),
    // odotetaan 1 ms että anturi asettuu, käynnistetään keskiarvoistus (vaihe 5). Tulos CMD_TEMP_STOP:lla.
    // HUOM: aktiivinen odotus _delay_us(1000) tehdään TWI-keskeytyksessä.
    case CMD_TEMP_START:{
      VREF.CTRLA = 1 << 4 | 1;
      ADC0.MUXPOS = 0x1E;
      _delay_us(1000);
      nollaa();
      vaihe = 5;
      break;
      }
    }
  }

// I2C-VASTAUS (master lukee). Ajetaan TWI-keskeytyksessä. Vastaus riippuu viimeisimmästä komennosta (last_cmd).
// Kaikki luvut lähetetään write7Bit():llä. Komennot joilla ei ole case:a (esim. CMD_SPEED) eivät vastaa mitään.
void requestEvent(){
  switch (last_cmd){
    case CMD_I2C_TEST:{
      // Yhteystesti: palauttaa aina 17.
      write7Bit(17,1);
      break;
      }
    case CMD_PA4_TIME:{
      // Viimeisimmän PA4-reunan aikaleima (syklit). 1 = reunaa ei ole tullut.
      write7Bit(pa4_time,8);
      break;
      }
    case CMD_RUNTIME:{
      // Laitteen oma aikaleima nyt (syklit). OLETUS: master vertaa eri laitteiden arvoja kellojen erojen arviointiin.
      uint64_t t = get_timestamp_cycles_safe();
      write7Bit(t,8);
      break;
      }
    case CMD_ADC_RESULT:{
      // Keskiarvo valmis vasta vaiheessa 6; muuten vastataan -1 (kaikki tavut 0x7F = "ei valmis").
      if(vaihe!=6){
        write7Bit(-1,8);
        break;
      }
      // Keskiarvo x 65536 (Q16). Luku KUITTAA tuloksen: nollaa laskurit ja palaa lepoon, eli tuloksen voi lukea vain kerran.
      write7Bit(sumXY,8);
      nollaa(); 
      vaihe = 0;
      break;
      }
    case CMD_RESULT:{
      // Reuna valmis vasta vaiheessa 4; muuten -1 (ei valmis).
      if(vaihe!=4){   //Ei valmis
        write7Bit(-1,8);
        break;
        }
      // HUOM: tilaa ei palauteta lepoon: vaihe pysyy 4:ssä ja last_ADC_idx/lista[] säilyvät, joten tuloksen voi lukea
      //       uudelleen ja näytteet voi lukea CMD_PRINT:llä kunnes tulee CMD_START/CMD_STOP. Nämä nollaukset (sumY,
      //       sumXY, sumX, sumXX) ovat jäänteitä vanhasta menetelmästä.
      sumY=0;
      sumXY=0;
      sumX = 0;
      sumXX = 0;
      //t=(uint32_t)(estimate_center_time()*TCD0.CMPBCLR);
      // 50 %-ylityksen aika sykleinä suhteessa ekaAika:an (ks. vikaAika).
      write7Bit(vikaAika,8);
      break;
      }
    case CMD_ADC_VALUE:{
      // Viimeisin ADC-lukema (2 tavua = 14 bittiä; lukema on 0..2046).
      write7Bit(last_ADC,2);
      break;
      }
    case CMD_PRINT:{
      // Debug-luku. Valitsin = last_message (edellisen kirjoituksen payload): 1 = näytemäärä, 2 = lowerLimit,
      // 3 = upperLimit, 4 = vikaAika, 5 = ekaAika, >=10 = lista[valitsin - 10].
      // HUOM: lista-indeksiä ei rajata: valitsin >= 80 lukee taulukon yli. Valitsimilla 0 ja 6..9 ei vastata mitään.
      if(last_message==1){
        write7Bit(last_ADC_idx,8);
      }else if(last_message==2){
        write7Bit(lowerLimit,8);
      }else if(last_message==3){
        write7Bit(upperLimit,8);
      }else if(last_message==4){
        write7Bit(vikaAika,8);
      }else if(last_message==5){
        write7Bit(ekaAika,8);
      }
      else if(last_message>=10){
        write7Bit(lista[last_message-10],8);
      }
      break;
      }
    case CMD_ID:{
      // UID:n alimmat 4 tavua. Tällä tunnistetaan uusi laite: lisää sen arvo setup():n switch-rakenteeseen.
      write7Bit(read_uid(), 8);

      break;
      }
    
    case CMD_TEMP_STOP:{      
      // Lämpötilan muunnos: sumXY on keskiarvo x 65536 (kahden näytteen summana). >> ADC0.CTRLB (=1) jakaa kahdella
      // -> yhden näytteen keskiarvo x 65536. Sitten / Temp_kerroin ja + Temp_offset (laitekohtaiset kalibrointiarvot).
      // HUOM: ei tarkisteta että vaihe == 6: jos luetaan ennen valmistumista tai ilman CMD_TEMP_START:ia, tulos on
      //       roskaa (ja sumXY tuhoutuu). HUOM: tuntemattomalla UID:lla Temp_kerroin == 0 -> jako nollalla (AVR ei kaadu,
      //       mutta tulos on roskaa). Tuloksen yksikkö on epäselvä.
      sumXY >>= ADC0.CTRLB;
      sumXY /= Temp_kerroin;
      sumXY += Temp_offset;

      write7Bit(sumXY, 8);
      
      // Palautetaan ADC valoporttimittaukseen: valittu referenssi takaisin ja tuloksi PA7/AIN7. Odotus antaa
      // multiplekserin asettua. Lopuksi laskurit nollataan ja palataan lepoon (vaihe 0).
      VREF.CTRLA = Vref;
      ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;           //Single ended pinnin valinta
      _delay_us(100);
      nollaa();
      vaihe = 0;
      break;
      }
    default:
      break;
  }
  }


// Alustus:
//   1. Tunnistetaan laite UID:n perusteella -> I2C-osoite ja lämpötilakalibrointi.
//   2. I2C-orja käyntiin, takaisinkutsut keskeytyksiin.
//   3. PA7 (analogitulo) ja PA1 (synkronointilinja) tuloiksi.
//   4. Keskeytykset pois, alustetaan ajastimet/EVSYS/ADC, keskeytykset päälle.
void setup(){
  switch (read_uid()){
    // case-arvot = read_uid():n palauttama 32-bittinen luku (desimaalina). Rivin kommentin pitkä luku on koko
    // 7-tavuinen UID (SERNUM3..9); sen alin 32 bittiä on täsmälleen case-arvo (tarkistettu). Ylimmät 3 tavua
    // (0x324A45) ovat molemmilla laitteilla samat.
    // Laite A: osoite 0x41, lämpötilakalibrointi kerroin 114 / offset -5315.
    case 2235643953:{   //14155411284375601
      I2C_ADDR = 0x41;
      Temp_kerroin=114;
      Temp_offset=-5315;
      break;
      }
    // Laite B: osoite 0x42, lämpötilakalibrointi kerroin 115 / offset 4774.
    case 3314495277:{    //14155412363226925
      I2C_ADDR = 0x42;
      Temp_kerroin=115;
      Temp_offset=4774;
      break;
      }
    // Tuntematon laite: oletusosoite 0x40. HUOM: Temp_kerroin ja Temp_offset jäävät 0:ksi -> CMD_TEMP_STOP jakaisi
    // nollalla. Uuden laitteen lisäys: lue UID CMD_ID:llä ja lisää oma case yllä.
    default:{
      I2C_ADDR = I2C_ADDR_DEFAULT;
      break;
      }
    }
  // I2C-orja käyntiin valitulla osoitteella; kirjoitus -> receiveEvent, luku -> requestEvent (molemmat TWI-keskeytyksessä).
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  // PA7 tuloksi (analogi). PA1 tuloksi + sisäinen pull-up: synkronointilinja lepotilassa HIGH, vapaa.
  PORTA.DIRCLR = PIN7_bm;
  PORTA.DIRCLR = PIN1_bm;            // PA1 tuloksi (High-Z)
  PORTA.PIN1CTRL = PORT_PULLUPEN_bm; // sisäinen pull-up päälle

  // Periferiat alustetaan keskeytykset pois päältä. Järjestys: TCA0 (aikaleimat), TCD0 (näytekello), PA4, EVSYS,
  // ADC, TCB0. Heti sei():n jälkeen näytteenotto ja keskeytykset ovat käynnissä.
  cli();

  tca0_init();
  tcd0_init();
  pa4_event_init();
  evsys_init();
  adc_init();
  tcb0_init();


  sei();
  }

// Pääsilmukka tekee vain yhden asian: vapauttaa PA1-synkronointipulssin kun 80 µs on kulunut. Muu toiminta
// (ADC, I2C, PA4-aikaleimaus, ylivuotolaskuri) tapahtuu keskeytyksissä.
// HUOM: pulssin todellinen pituus = 80 µs + silmukan viive; jos ADC-ISR (float-laskenta) pitää CPU:n varattuna,
//       vapautus viivästyy. Vertailu käyttää 64-bit aikaleimaa (ks. HUOM "reserved" get_timestamp_cycles_safe():ssa).
void loop(){
  if(get_timestamp_cycles_safe()>=palautaSync){
    // Pulssi ohi: PA1 takaisin tuloksi + pull-up (linja vapautuu), report nollataan (kertakäyttöinen) ja
    // palautaSync asetetaan "ei pulssia" -tilaan.
    PORTA.DIRCLR = PIN1_bm;            // takaisin tuloksi
    PORTA.PIN1CTRL = PORT_PULLUPEN_bm; // pull-up uudelleen päälle
    report = false;
    palautaSync = 0xFFFFFFFFFFFFFFFF;
  }
  }

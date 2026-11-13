# Valoportti816 – ATtiny816-valoportti, tarkka reunan keskiaika

Korvaa `ValoporttiAttiny13_3_message`-sketsin (alkuperäinen on koskematta). **I2C-protokolla on sama** (komennot 1–21,
7 bittiä / tavu, MSB ensin) + uudet komennot 22–31. Uusi koodi mittaa jokaiselle reunalle sen **50 %-ylityshetken**
(hetki, jolloin signaali on puolivälissä askelta eli kappaleen reuna on valonsäteen keskellä) ja antaa ajan
CLK_PER-sykleinä.

> **Testaamaton laitteella.** Koodi kääntyy (avr-gcc 7.3 + iotn816.h) ja estimaattori on testattu isännällä ja
> AVR-simulaattorilla, mutta ADC/EVSYS/TCB-ajoitusta, I2C:tä ja EXTCLK-vaihtoa ei ole ajettu oikealla ATtiny816:lla.
> Katso [Tarkistuslista](#tarkistuslista-ensimmäiseen-käynnistykseen).

Tiedostot: `Valoportti816.ino` (laite, I2C, ADC-aikaperusta), `edge_core.h/.cpp` (laiteriippumaton reunantunnistus ja
sovitus; testattavissa `g++`:lla).
`test/test_core.cpp` on isäntätesti (portaikkosignaali, kohina, ajautuva taso): `cd test && g++ -O2 -o test_core test_core.cpp ../edge_core.cpp && ./test_core 83 8 2.1 170 400`
(argumentit: Ts µs, N, σ/muunnos, askel/muunnos, ajojen määrä).

## 1. Miten aika syntyy

* TCA0 käy vapaasti jaksolla **P** (CLK_PER-sykliä). Jokainen ylivuoto laukaisee ADC:n EVSYS:n kautta (`STARTEI`).
  ADC kerryttää **N ≤ 8** muunnosta yhdeksi lukemaksi (`SAMPNUM`), joten laukaisu → tulos on laitteistolla ajastettu:
  lukeman *j* näytehetki = `(j+1)·P + delta_eff`. Ei keskeytysviiveitä eikä ohjelmallista ajoitusta.
* `delta_eff` (laukaisusta vaikuttavaan näytehetkeen) lasketaan datasheetin ADC-ajoituksesta
  (aloitusviive PRESC/2+2, (N−1)/2 muunnosta, (2+SAMPLEN) CLK_ADC-sykliä). `INITDLY = 0`, `ASDV = 0`.
  Vakio on sama kaikille reunoille ja laitteille, jotka käyttävät samaa asetusta – se ei vaikuta laitteiden väliseen
  suhteelliseen aikaan; absoluuttinen offset hienosäädetään `CMD_TIME_ADJ`:llä.
* Aikaleimat (`CMD_RUNTIME`, `CMD_PA4_TIME`, reunat) ovat samalla aikaperustalla: `t = ylivuotolaskuri·P + TCA0.CNT`.
  PA4-pulssin tallentaa TCB0 (capture, TCB käynnistyy TCA0:n ylivuodossa), joten CCMP = TCA0.CNT hetkellä, jolloin
  pulssi tuli – keskeytyksen viive ei vaikuta.
* TCA0:n ylivuoto on nostettu korkeimpaan prioriteettiin (`CPUINT.LVL1VEC`).
* Ajat ovat **32-bittisiä ja kiertäviä** (2³² sykliä ≈ 7 min @ 10 MHz); laske erotukset modulo 2³². Vastaus on
  8 × 7 bittiä, ylimmät bitit nollia.
* Reunan hetki = `ekaAika + vikaAika`: `CMD_PRINT` 5 → puskurin alun (buf[0]) aika, `CMD_RESULT` (tai `CMD_PRINT` 4) →
  ylitys suhteessa siihen. Yhteensä: `t_reuna = ekaAika + vikaAika` (sykliä).

### Näytejakso eri asetuksilla (laskettu `adc_derive()`-kaavoilla, ei mitattu)

| CPU | ref | CLK_ADC | N | P | delta_eff |
|---|---|---|---|---|---|
| 10 MHz | 1,1 V | 1250 kHz | 8 | 84,4 µs | 38,6 µs |
| 10 MHz | 0,55 V | 156 kHz | 2 | 170 µs | 57,8 µs |
| 16 MHz | 1,1 V | 1000 kHz | 8 | 105 µs | 48,1 µs |
| 20 MHz | 1,1 V | 1250 kHz | 8 | 84 µs | 38,5 µs |
| 20 MHz | 0,55 V | 156 kHz | 2 | 170 µs | 57,7 µs |

Datasheetin ADC-rajat (taulukko 36-26): 0,55 V ref → CLK_ADC 100–260 kHz; ≥ 1,1 V → 200–1500 kHz. Vanha koodi ajoi ADC:ta
2,5 MHz:llä, mikä on näiden rajojen ulkopuolella. CPU 20 MHz vaatii VDD ≥ 4,5 V (3,3 V:lla 10 MHz).

## 2. Tarkkuus (ennuste, ei mittaus)

Reunan aikavirheen teoreettinen alaraja (Cramér–Rao, sileä askel): σ_t = (σ/H)·√(Ts·Te/1,2). Rajoitteilla joista
sovittiin (kohina muutama ADC-yksikkö, askel ≈ 1/6 asteikosta, reuna 0,8–2,6 ms, ei vahvistinta, ei slottia, ei
reunojen keskiarvoistusta) alaraja on **n. 1,5–2,1 µs**. Estimaattori (kokonaisluku, ei float) on **1,35–1,6 × alaraja**:

| ADC | virhe RMS | 95 % | huom. |
|---|---|---|---|
| N = 8, Ts = 83 µs, σ = 2,1 / muunnos | 2,4 µs | 4,8 µs | alaraja 1,5 µs |
| N = 8, Ts = 83 µs, σ = 3,0 | 3,3 µs | 6,8 µs | alaraja 2,1 µs |
| N = 8, Ts = 83 µs, σ = 3,0, askel 120 | 4,7 µs | 9,4 µs | |
| N = 1 (ei kertymää) | 6,8 µs | 13,7 µs | |

(3200 reunaa / rivi, satunnaiset reunan kestot 0,8–2,6 ms, ajautuva taso; tunnistus 100 %, vääriä 0.)
**"Selvästi alle 1 µs" absoluuttinen tarkkuus ei ole saavutettavissa näillä rajoitteilla** – se vaatisi suuremman askeleen
tai pienemmän kohinan (esim. valotehon nosto, vahvistin, tai reunan nopeampi aikaskaala). Mikä auttaa ilman uusia
komponentteja: (a) ESP32:ssa tehtävä koko reunan sovitus raakadatasta (`CMD_EDGE_DATA`) → arviolta 1,3 × alaraja,
(b) ref-jännitteen valinta alla, (c) ADC täydellä käyttösuhteella (jo oletus, ≈ 98 %).

## 3. I2C-protokolla

Kirjoitus: `[komento][payload: 7 bittiä / tavu, MSB ensin]`. Luku: komennon jälkeen `requestFrom(addr, n)`, jossa n = 8
(ellei toisin mainita); arvo 7 bittiä / tavu, MSB ensin. **−1 = kahdeksan tavua 0x7F** ("ei valmis").

| # | Nimi | Kirj./Luku | Merkitys |
|---|---|---|---|
| 1 | PA4_TIME | L | viimeisimmän PA4-pulssin aika (sykliä); 1 = ei pulssia |
| 2 / 3 | ADC_START / STOP | S | tason mittaus (ks. kalibrointi): tasoseuranta käyntiin / pois |
| 4 | ADC_RESULT | L | taso/muunnos × 65536 (Q16), −1 kunnes seuranta on lämmennyt; kuittaa mittauksen |
| 5 | ADC_VALUE | L (2 tavua) | viimeisin lukema (summa N muunnoksesta, ≤ 8184) |
| 6 / 7 | ADC_LOWER / UPPER | S | (vanha) tallennetaan, ei käytetä – tunnistus on suhteellinen |
| 8 | RESULT | L | jonon vanhimman reunan ylitys suhteessa ekaAika:an (sykliä), −1 jos jono tyhjä |
| 9 | START | S | tyhjennä jono, viritä tunnistus (lämmittely ≈ 11 ms @ P = 84 µs) |
| 10 | STOP | S | lopeta tunnistus |
| 11 | I2C_TEST | L (1 tavu) | 17 |
| 12 | RUNTIME | L | laitteen aikaleima nyt (sykliä) |
| 15 | PRINT | S+L | valitsin = payload: 1 = reunapuskurin pituus, 2 = kohinan keskipoikkeama × 8 (lukemayksikköä), 4 = vikaAika, 5 = ekaAika, 6 = P (sykliä), 7 = askel H + 32768 \| liput << 16, ≥ 10 = raaka lukema nro (valitsin − 10) |
| 16 / 17 | REPORT_TRUE / FALSE | S | PA1-pulssi kun reuna alkaa (kertakäyttöinen) / peru |
| 18 | VREF | S | ADC-ref: 0 = 0,55 V, 1 = 1,1 V, 2 = 2,5 V, 3 = 4,34 V, 4 = 1,5 V, 5 = VDD. **Aikaperusta alkaa alusta** |
| 19 | ID | L | UID:n 4 alinta tavua |
| 20 / 21 | TEMP_START / STOP | S / L | lämpötilamittaus (kuten ennen; ei `FEAT_TEMP=0`-käännöksessä) |
| **22** | STATUS | L | bitit 0–7: 0x01 tunnistus päällä, 0x02 seuranta lämmin, 0x04 rengasylivuoto, 0x08 näytteitä menetetty, 0x10 jono täynnä, 0x20 EXTCLK käytössä, 0x40 EXTCLK odottaa reunoja, 0x80 tasomittaus; 8–15: N; 16–19: ref; 20–22: ADC-esijakaja; 24–: jonon pituus |
| **25** | EDGE_NEXT | S | poista jonon vanhin tulos (lue ensin RESULT/PRINT) |
| **26** | EDGE_DATA | S+L | payload = aloitusindeksi; luku = **28 tavua**: 14 raakalukemaa little-endian (0xFFFF = ei dataa) viimeisimmästä reunasta |
| **27** | CLOCK | S | payload = ulkoisen kellon kHz (PA3), 0 = takaisin sisäiseen. **Aikaperusta alkaa alusta** |
| **28** | ADC_CFG | S | kertymäkoodi (0..3 → N = 1, 2, 4, 8; 7 = auto) \| SAMPLEN << 3. **Aikaperusta alkaa alusta** |
| **30** | TIME_ADJ | S | aikakorjaus sykleinä (16-bittinen kaksoiskomplementti, 3 × 7 bittiä), lisätään kaikkiin reunaaikoihin |
| **31** | EDGE_PARAM | S | `thr_min \| min_step << 10 \| thr_mult << 24` (oletus 6, 40, 5) |

Huom: reunan aikaleima- ja jonoarvot ovat `Rec{j0, tq8, H, flags}`; `flags`: 1 nouseva, 2 puskuri ylivuoto, 4 varalla
(regressio ei onnistunut), 8 heikko (hylätään, ei jonoon), 16 ei sovitusta (hylätään), 32 näyte menetetty reunan aikana.
Jonoon mahtuu 8 reunaa; lue ne ennen seuraavaa laukaisua.

## 4. Kalibrointi- ja mittausvirta (ESP32)

1. **Kaikki ledit päälle.** Aseta `CMD_VREF` → odota ≥ 5 ms (referenssin ja ADC:n asettuminen + aikaperusta alkaa alusta).
2. `CMD_ADC_START` → pollaa `CMD_ADC_RESULT` kunnes ≠ −1 (≈ 11–25 ms). Tulos = taso / muunnos × 65536 → `L = tulos / 65536`
   (0…1023 muunnosyksikköä).
3. Valitse ref: **pienin referenssi ≥ 1,1 V jolla `L ≤ ~0,85 · 1023`** (varaa ~15 % ledien kirkkauden ajautumiselle
   akun jännitteen mukana). 0,55 V:lla ADC on rajoitettu 260 kHz:iin (N = 1–2, P ≈ 90–170 µs) ja menettää ~8 × näytemäärän;
   se kannattaa vain jos 1,1 V:lla `L < ~35 %` ja mittaukset (kohina, `CMD_PRINT` 2) osoittavat sen paremmaksi.
   Vertailuun: tiedon määrä aikayksikköä kohti ∝ `(H/σ)² / Ts`, missä `H ≈ L·N/6` (lukemayksikköä) ja
   `σ ≈ 1,25 · (CMD_PRINT 2)/8`; valitse suurin.
4. `CMD_START` → odota ≥ 25 ms (lämmittely; tai pollaa STATUS bitti 0x02). Ledit pysyvät päällä.
5. Kappaleen ohituksen jälkeen: pollaa STATUS (jonon pituus 8) tai `CMD_REPORT_TRUE` (PA1). Lue jokaiselle reunalle
   `CMD_PRINT` 5 (ekaAika), `CMD_RESULT` (vikaAika), tarvittaessa `CMD_PRINT` 7 (H, liput) ja `CMD_EDGE_DATA`, sitten
   `CMD_EDGE_NEXT`.
6. Ajan yhdistäminen laitteiden välillä: kaikki ATtinyt samalle kellolle (PA3, `CMD_CLOCK`), yhteinen PA4-pulssi →
   jokainen tallentaa saman hetken (`CMD_PA4_TIME`). `t_globaali_i = t_i − pa4_i` (mod 2³²). Jos kello vaihdetaan
   tai ADC-asetus muuttuu, aikaperusta alkaa alusta → lähetä PA4-pulssi uudelleen.

**PA4-pulssi ja I2C:** TCB0:n keskeytyksen viive tulee olla alle yhden näytejakson (P ≈ 84 µs), muuten PA4-aika voi
heittää P:n verran. I2C-callbackit ajetaan TWI-keskeytyksessä – lähetä PA4-pulssi vain kun I2C-väylä on hiljaa.

Ulkoinen kello: PA3, ≤ 10 MHz (3,3 V:lla; datasheet 10.3.2: vaihto tapahtuu vasta kun reunoja tulee; kellon katoaminen
jäädyttää MCU:n, vain WDT nollaa → WDT ~1 s on päällä). `CMD_CLOCK` kHz-arvon on oltava tarkka (aikaperusta skaalautuu).

## 5. Reunantunnistus (edge_core)

* Ei kiinteitä tasoja: taso seurataan lohkoittain (8 lukemaa), kynnys = `thr_mult · kohinan keskipoikkeama`
  (min `thr_min`). Reuna havaitaan kun 3 peräkkäistä lukemaa ylittää kynnyksen samaan suuntaan.
* Reuna kerätään puskuriin (≤ 50 lukemaa, 7 ennakkolukemaa) kunnes signaali on tasaantunut (8 lukemaa).
* Sovitus: reunan 50 %-taso (vanha ja uusi taso keskiarvona), sen ympäriltä (±35 % askeleesta) lineaarinen regressio
  kokonaisluvuilla (ei float, ei int64), kaksi iteraatiota.
* Heikko reuna (< `min_step`) hylätään.
* Mitattu AVR-simulaattorilla (atmega328p proxy, AVRe+ -syklit): `ed_push` ka 231 / max 432 sykliä, `ed_process` ka
  7,5 k / max 8,7 k sykliä (≈ 0,87 ms @ 10 MHz). Lukemarenkaassa on 16 paikkaa (≈ 1,3 ms @ P = 84 µs), joten
  `ed_process` mahtuu rengasylivuotoa tekemättä, kun CPU ≥ 10 MHz ja I2C-liikenne on kohtuullista. **Alle 10 MHz:n kellolla
  rengas voi ylivuotaa** (STATUS bitti 0x04) → tunnistus käynnistyy uudelleen.

## 6. Käännös, flash ja RAM

Ominaisuuskytkimet (`-DFEAT_TEMP=0`, `-DFEAT_CLOCK=0`, `-DFEAT_REPORT=0`) poistavat osia, jos flash ei riitä (ATtiny816: 8 kt).
Mitattu (avr-gcc 7.3, sketsi + edge_core + libgcc, ei oikeaa megaTinyCorea/Wirea): kaikki ominaisuudet 6,4 kt (ei LTO) /
6,1 kt (LTO), ilman TEMP 6,1 / 5,7 kt, minimi 5,7 / 5,3 kt; SRAM ≈ 330 B. megaTinyCore/Wire vie arviolta 1,3 kt flashia ja
n. 100 B SRAM:ia → arvio **7,4–7,7 kt / 8,2 kt**. Todellinen koko pitää tarkistaa Arduino-käännöksestä (megaTinyCore
käyttää LTO:ta). Pinoa ei ole mitattu: tarkista (esim. täytä pino kuviolla), jos RAM on tiukalla.

## Tarkistuslista ensimmäiseen käynnistykseen

1. `CMD_I2C_TEST` → 17, `CMD_ID`.
2. `CMD_PRINT` 6 → P (sykliä) vastaa taulukkoa; `CMD_ADC_VALUE` → ≈ N × taso (N = STATUS-bitit 8–15).
3. Mittaa `CMD_RUNTIME`-aikaleimoilla, että aika kuluu `F_CPU`-tahdissa (kaksi lukua väli tunnetulla ajalla).
4. Anna tunnettu aikaleimattu valopulssi/portaikko (esim. ESP32 ohjaa ledin ja logiikka-analysaattori tai PA1-pulssi) →
   tarkista reunan `ekaAika + vikaAika` ja säädä `CMD_TIME_ADJ` (delta_eff-oletus on datasheetistä).
5. Tarkista TCB0 PA4-aikaleima: lähetä sama PA4-pulssi kahdelle laitteelle samalla kellolla; ero ≈ 0 sykliä.
6. Tarkista EXTCLK-vaihto (PA3) matalalla taajuudella ensin (esim. 1 MHz), ja että WDT ei nollaa.
7. Tarkista kokonaisflash ja pino oikealla käännöksellä (kohta 6); tarkista `STATUS` ei näytä rengasylivuotoa
   (0x04) tai menetettyjä näytteitä (0x08) ajon aikana.

## Tiedossa olevat rajoitukset

* Testaamatta laitteella (ks. yllä): ADC-kertymän ajoitus (`delta_eff`), TCB-restartin offset, EXTCLK-vaihto, Wire-puskurit, pino.
* `CMD_PA4_TIME` palauttaa vain viimeisimmän pulssin (kuten vanha koodi).
* Tasomittaus (`CMD_ADC_START`) ja reunantunnistus (`CMD_START`) käyttävät samaa ilmaisinta: älä aja niitä yhtä aikaa.
* Vanhat `CMD_ADC_LOWER/UPPER` eivät vaikuta (tunnistus on suhteellinen).
* ESP32-apukoodia ei ole mukana.

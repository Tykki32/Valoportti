# Valoportti816 – ATtiny816-valoportti, tarkka reunan keskiaika

Korvaa `ValoporttiAttiny13_3_message`-sketsin (alkuperäinen on koskematta). **I2C-protokolla on sama** (komennot 1–21,
7 bittiä / tavu, MSB ensin) + uudet komennot 22–32. Uusi koodi mittaa jokaiselle reunalle sen **50 %-ylityshetken**
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
* ADC:n RESRDY-keskeytys on korotettu korkeampaan prioriteettiin (`CPUINT.LVL1VEC`): tulos luetaan heti, vaikka TWI-keskeytys
  olisi käynnissä (datasheet ei lupaa, että RES säilyy seuraavan laukaisun yli). TCA0-ylivuoto, TCB0 ja TWI ovat LVL0:ssa:
  yksittäisen TWI-keskeytyksen on kestettävä alle P (~84 µs), muuten ylivuotolaskuri tai PA4-aikaleima voi heittää P:n.
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
| N = 8, Ts = 83 µs, σ = 2,1 / muunnos | 2,2 µs | 4,4 µs | alaraja 1,5 µs |
| N = 8, Ts = 83 µs, σ = 3,0 | 3,0 µs | 6,1 µs | alaraja 2,1 µs |
| N = 8, Ts = 83 µs, σ = 3,0, askel 120 | 4,2 µs | 8,3 µs | |
| N = 4, Ts = 100 µs, σ = 2,1 | 3,3 µs | 6,6 µs | alaraja 2,3 µs |

Testissä ledien askeleet ovat eri suuruisia (1 / 0,82 / 0,93 / 0,74 × nimellinen) ja amplitudi ajautuu ±10 %, joten heikoin
reuna määrää hännän. Ilman näitä (yksi nimellinen reuna, 300 kpl) sama estimaattori antaa RMS ≈ 1,8–2,0 µs (alaraja 1,5 µs).

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
| 15 | PRINT | S+L | valitsin = payload: 1 = reunapuskurin pituus, 2 = alaraja (vanha `CMD_ADC_LOWER`), 3 = yläraja, 4 = vikaAika, 5 = ekaAika, 6 = P (sykliä), 7 = askel H + 32768 \| liput << 16, 8 = kohinan keskipoikkeama × 8 (lukemayksikköä), ≥ 10 = raaka lukema nro (valitsin − 10); 0 = lista loppu; 60 = reunaa edeltävä taso, 61 = jälkitaso (48 lukeman keskiarvo; pätee kaappauksen jälkeen), molemmat Q3 (8 × lukeman keskiarvo) |
| 16 / 17 | REPORT_TRUE / FALSE | S | PA1 vedetään LOW 1–2 lukeman ajaksi kun ensimmäinen reuna alkaa (**ei** 50 %-kohdassa), kertakäyttöinen / peru. PA1 on avoin nielu (tulo + pull-up, vedetään alas vain pulssin ajaksi), joten usean laitteen yhteinen linja on turvallinen |
| 18 | VREF | S | ADC-ref: 0 = 0,55 V, 1 = 1,1 V, 2 = 2,5 V, 3 = 4,34 V, 4 = 1,5 V, 5 = VDD. **Aikaperusta alkaa alusta** |
| 19 | ID | L | UID:n 4 alinta tavua |
| 20 / 21 | TEMP_START / STOP | S / L | lämpötilamittaus (kuten ennen; ei `FEAT_TEMP=0`-käännöksessä) |
| **22** | STATUS | L | bitit 0–7: 0x01 tunnistus päällä, 0x02 seuranta lämmin, 0x04 rengasylivuoto, 0x08 näytteitä menetetty, 0x10 jono täynnä, 0x20 EXTCLK käytössä, 0x40 EXTCLK odottaa reunoja, 0x80 tasomittaus; 8–15: N; 16–19: ref; 20–22: ADC-esijakaja; 24–27: jonon pituus; 28–29: kaappaus (1 = odottaa, 2 = valmis) |
| **25** | EDGE_NEXT | S | poista jonon vanhin tulos (lue ensin RESULT/PRINT) |
| **26** | EDGE_DATA | S+L | payload = aloitusindeksi; luku = **16 tavua**: 8 raakalukemaa little-endian (0xFFFF = ei dataa) viimeksi käsitellystä / kaapatusta reunasta. Lyhyt lohko, jotta TWI-keskeytys pysyy alle P |
| **27** | CLOCK | S | payload = ulkoisen kellon kHz (PA3), 0 = takaisin sisäiseen. **Aikaperusta alkaa alusta** |
| **28** | ADC_CFG | S | kertymäkoodi (0..3 → N = 1, 2, 4, 8; 7 = auto) \| SAMPLEN << 3 \| (esijakajakoodi + 1) << 8 (0 = auto datasheetin rajoissa; 1…7 = DIV2…DIV256, **voi ylittää datasheetin ADC-kellorajan** – käytä vain jos kohinamittaus osoittaa sen paremmaksi). **Aikaperusta alkaa alusta** |
| **30** | TIME_ADJ | S | aikakorjaus sykleinä (16-bittinen kaksoiskomplementti, 3 × 7 bittiä), lisätään kaikkiin reunaaikoihin |
| **31** | EDGE_PARAM | S | `thr_min \| min_step << 10 \| thr_mult << 24` (oletus 6, 40, 5) |
| **32** | CAPTURE | S | payload = ohitettavien reunojen määrä (0…7): käynnistää tunnistuksen, ohittaa n reunaa, kaappaa seuraavan kokonaisena, **pysäyttää tunnistuksen ja säilyttää raakadatan** (jonoon jää vain kaapattu reuna). STATUS bitit 28–29 kertovat tilan |

Huom: reunan aikaleima- ja jonoarvot ovat `Rec{j0, tq8, H, flags}`; `flags`: 1 nouseva, 2 puskuri ylivuoto, 4 varalla
(regressio ei onnistunut), 8 heikko (hylätään, ei jonoon), 16 ei sovitusta (hylätään), 32 näyte menetetty reunan aikana.
Jonoon mahtuu 8 reunaa; lue ne ennen seuraavaa laukaisua.

## 4. Kalibrointi- ja mittausvirta (ESP32)

1. **Kaikki ledit päälle.** Aseta `CMD_VREF` → odota ≥ 5 ms (referenssin ja ADC:n asettuminen + aikaperusta alkaa alusta).
2. `CMD_ADC_START` → pollaa `CMD_ADC_RESULT` kunnes ≠ −1 (≈ 11–25 ms). Tulos = taso / muunnos × 65536 → `L = tulos / 65536`
   (0…1023 muunnosyksikköä).
3. Valitse ref: **pienin referenssi ≥ 1,1 V jolla `L ≤ ~0,85 · 1023`** (varaa ~15 % ledien kirkkauden ajautumiselle
   akun jännitteen mukana). 0,55 V:ta ei kannata käyttää datasheetin rajoissa: ADC-kello on silloin ≤ 260 kHz (N = 1–2,
   P ≈ 90–170 µs), eli ~8 × vähemmän muunnoksia, ja askel kasvaa vain 2 × → tietomäärä ∝ (H/σ)²·muunnosnopeus on silti
   ~2 × huonompi. Jos signaali täyttää alle puolet asteikosta jo 1,1 V:lla, paras keino on nostaa ledien tehoa (ei referenssin
   vaihtoa).
   Vertailuun (esim. eri `CMD_ADC_CFG`-asetukset): tiedon määrä aikayksikköä kohti ∝ `(H/σ)² / Ts`, missä `H ≈ L·N/6`
   (lukemayksikköä, jos askel on 1/6 asteikosta) ja `σ ≈ 1,25 · (CMD_PRINT 8)/8`; valitse suurin.
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
* Reuna kerätään puskuriin (≤ 50 lukemaa, 7 ennakkolukemaa) kunnes signaali on tasaantunut (8 lukemaa). Tasaantumisen
  jälkeen kerätään vielä `EDGE_POST` = 40 lukemaa pelkkään jälkitason summaan (ei puskuriin): jälkitaso = 48 lukeman keskiarvo
  (aiemmin 8), mikä pienensi virhettä n. 10 %. Reunat ovat ≥ 10 ms välein, joten tämä ehtii (reuna ≤ 2,6 ms + 48 lukemaa ≈ 4 ms).
* Sovitus: reunan 50 %-taso (vanha ja uusi taso keskiarvona), sen ympäriltä (±35 % askeleesta) lineaarinen regressio
  kokonaisluvuilla (ei float, ei int64), kaksi iteraatiota.
* Heikko reuna (< `min_step`) hylätään.
* Mitattu AVR-simulaattorilla (atmega328p proxy, AVRe+ -syklit): `ed_push` ka 224 / max 438 sykliä, `ed_process` ka
  8,3 k / max 9,5 k sykliä (≈ 0,95 ms @ 10 MHz). Lukemarenkaassa on 16 paikkaa (≈ 1,35 ms @ P = 84 µs), joten
  marginaali on ohut: **alle 10 MHz:n kellolla tai vilkkaalla I2C-liikenteellä rengas voi ylivuotaa** (STATUS bitti 0x04 ja 0x08).
  Ylivuoto tasaisella alueella (reunan jälkeen) on harmiton: lukemaindeksit korjataan eikä tunnistusta käynnistetä uudelleen;
  vain kesken reunaa tapahtuva ylivuoto käynnistää tunnistuksen alusta.

## 6. Käännös, flash ja RAM

Ominaisuuskytkimet (`-DFEAT_TEMP=0`, `-DFEAT_CLOCK=0`, `-DFEAT_REPORT=0`) poistavat osia, jos flash ei riitä (ATtiny816: 8 kt).
Mitattu (avr-gcc 7.3, sketsi + edge_core + libgcc, ei oikeaa megaTinyCorea/Wirea): kaikki ominaisuudet 7,03 kt (ei LTO) /
6,58 kt (LTO), ilman TEMP 6,71 / 6,26 kt; SRAM ≈ 355 B (staattinen). Alkuperäisen käännöksen symbolilistasta (TWI-keskeytys 332 B, TwoWire-funktiot ja ydin
yhteensä n. 0,8 kt) → arvio **7,4–7,8 kt / 8,19 kt** (+ SRAM ≈ 100 B Wire/ydin). Todellinen koko pitää tarkistaa Arduino-käännöksestä (megaTinyCore
käyttää LTO:ta). Pinoa ei ole mitattu: tarkista (esim. täytä pino kuviolla), jos RAM on tiukalla.

## 7. ESP32-yhteensopivuus (`ValoporttiESP32_multi_8_1.ino`, tarkistettu lukemalla koodi, ei ajettu)

**Toimii sellaisenaan** (sama komento, sama vastausmuoto): `CMD_I2C_TEST` (1 tavu), `CMD_ID`, `CMD_PA4_TIME` (8 × 7 bittiä, 1 = ei
pulssia), `CMD_START` / `CMD_STOP`, `CMD_RESULT` (−1 = 2⁵⁶−1 = ei valmis; ESP:n `virhe`-vertailu toimii), `CMD_REPORT_TRUE`
(pin 4: pulssi LOW), `CMD_ADC_START` / `CMD_ADC_RESULT` / `CMD_ADC_VALUE` (2 tavua), `CMD_ADC_LOWER/UPPER` (vastaanotetaan,
ei vaikutusta), `CMD_PRINT` 1…5 ja ≥ 10 (lista päättyy 0:aan, `printtaa kaikki` -silmukka päättyy), `CMD_TEMP_START/STOP`.
`CMD_SPEED` (13) ja `CMD_RESPONSE` (14) eivät olleet toteutettuna alkuperäisessäkään sketsissä eivätkä ole nytkään.

**Merkitys muuttunut – ESP-koodissa huomioitava**

| Kohta ESP-koodissa | Mitä muuttui | Mitä tehdä |
|---|---|---|
| `ATTINY_kpl = 2`, osoitteet 0x41, 0x42 | ATtinyn osoite valitaan UID:n mukaan `setup()`:n `switch`:ssä; tuntematon UID → 0x40 (kaikki uudet laitteet törmäävät) | Lue uusien kahden UID `id`-komennolla, lisää ne ATtiny-sketsin `switch`:iin (0x43, 0x44) ja kasvata `ATTINY_kpl` 4:ään. Lämpötilakertoimet 0 → `temp` antaa vain offsetin |
| `CMD_ADC_VALUE` (`one adc`, `run adc`, `aloita testi` `t<20`) | lukema on **N muunnoksen summa** (N = 8 oletuksena, 0…8184; ennen kahden summa 0…2046) | jaa N:llä (STATUS-bitit 8–15) ennen mV-laskentaa |
| `CMD_ADC_RESULT` (`test adc`, `set yla/ala`) | Q16 **yhtä muunnosta** kohti (ennen kahden summan keskiarvo → 2 × suurempi) | `mV = tulos/65536 · Vref/1023`; Vref luettavissa STATUS-bitit 16–19 (0 = 0,55 V, 1 = 1,1 V …). ESP:n `*550/1024` olettaa 0,55 V:n referenssin; **uusi oletusreferenssi on 1,1 V** (ESP ei kutsu `CMD_VREF`:iä; kalibrointivirta §4) |
| `one adc`: `uint8_t t` | 14-bittinen arvo typistyy 8 bittiin | käytä `uint16_t` |
| `CMD_RUNTIME` (`runtime`: `t /= 20000000`) | aika on **32-bittinen** (kiertää 2³² syklin välein) ja laskee F_CPU-sykleinä; aikaperusta alkaa alusta jokaisen ADC-asetuksen, `CMD_VREF`:n, `CMD_CLOCK`:n ja `CMD_TEMP_START`:n yhteydessä | jos F_CPU ≠ 20 MHz, jakaja on väärä; erotukset `(uint32_t)(b − a)`, ei uint64 |
| `CMD_RESULT` (`result`, `aloita testi`) | palauttaa jonon **vanhimman** reunan; jonoon mahtuu 8 reunaa (4 laskevaa + 4 nousevaa). Tulos on edelleen vikaAika (suhteessa ekaAika:an = `CMD_PRINT` 5), ei absoluuttinen | lue 8 kertaa: `CMD_PRINT` 5, `CMD_RESULT`, `CMD_EDGE_NEXT` (25); aika = ekaAika + vikaAika |
| `CMD_REPORT_TRUE` (`aloita testi`, `report`) | pulssi tulee kun reuna **alkaa** (3 lukemaa kynnyksen ylityksestä), ei 50 %-kohdassa; PA1 on avoin nielu (ei enää HIGH-lähtö) | ESP:n `while (digitalRead(4) == HIGH)` toimii; lisää aikakatkaisu. Tarkka aika vain `CMD_RESULT`:lla |
| `aloita testi`: I2C-lukuja heti pulssin jälkeen | kappale on vielä ohimenossa; I2C-liikenne keskeyttää ATtinyn ja voi viivästyttää lukemia | odota koko ohitus (STATUS: jonon pituus 8) ennen I2C-lukuja, älä lue I2C:llä kesken ohituksen |
| `CMD_TEMP_STOP` (`temp`) | lämpötila mitataan nyt datasheetin mukaan (1,1 V, SAMPLEN ≥ 32 µs); alkuperäinen ajoi ADC:n 2,5 MHz:llä ilman pitkää näytteistysaikaa → lukema voi poiketa vanhoista `Temp_kerroin`/`Temp_offset`-arvoista | tarkista vertaamalla referenssilämpömittariin, kalibroi tarvittaessa uudelleen |
| `CMD_PRINT` 2 / 3 | palauttavat taas alaraja/yläraja (kuten ennen); kohina on nyt valitsin 8 | – |

PA4-synkronointi (`sendSyncPulse`: 80 µs HIGH pulssi, nouseva reuna tallentuu) on yhteensopiva sellaisenaan. Neljällä anturilla
sama pulssi kaikille; lue `CMD_PA4_TIME` jokaiselta vasta kun I2C on hiljaa.

## 8. Reunan muodon tarkastelu (`CMD_CAPTURE`)

Tarkoitus: mitata yksi reuna kokonaisena ja lukea kaikki lukemat ESP32:lle (esim. CSV:ksi Serialiin), jotta reunan muoto
voidaan tutkia tarkemmin ja sovittaa mallit ESP:ssä.

1. Ledit päälle, referenssi ja ADC-asetus valittu (§4). Halutessasi tihennä näytteistys `CMD_ADC_CFG`:llä: N = 4 → P ≈ 45 µs
   (puskuri 50 lukemaa = 2,2 ms, riittää ≤ ~1,5 ms reunoille; CPU-kuorma ~65 %, rengasylivuoto reunan jälkeen on harmiton).
   N = 8 (oletus) kattaa koko 2,6 ms reunan.
2. `CMD_CAPTURE` payload `k` = ohitettavien reunojen määrä: 0 = ensimmäinen reuna, 1 = toinen … 7 = kahdeksas. Yhdessä ohituksessa
   saa talteen **yhden** reunan (RAM ei riitä usealle); muotoa tutkitaan siis reuna kerrallaan eri ohituksilla.
3. Odota ohitus; pollaa `CMD_STATUS`: bitit 28–29 = 2 → kaapattu. Tunnistus on silloin pysähtynyt ja raakadata säilyy
   seuraavaan `CMD_START` / `CMD_CAPTURE` / ADC-asetuksen muutokseen asti.
4. Lue: `CMD_PRINT` 1 (lukemien määrä), 5 (ekaAika), 6 (P), 60 / 61 (taso ennen / jälkeen, Q3), `CMD_RESULT` (firmwaren oma
   sovitus, vikaAika), `CMD_PRINT` 7 (H, liput), `CMD_STATUS` (N, referenssi), sitten `CMD_EDGE_DATA` aloitusindeksistä 0, 8, 16 …
5. Lukeman *i* aika = `ekaAika + i · P` (sykliä, jo korjattu `delta_eff`:llä); arvo = N muunnoksen summa (0…8184).
   Kaapatussa puskurissa on 7 ennakkolukemaa ennen reunaa (3 kynnyksen ylitystä + 4 ennen) ja 8 tasaantunutta lukemaa lopussa.
   Pidempää tasoa ei puskuriin mahdu: käytä tasoihin 60 / 61.

Esimerkki ESP32:lle (käyttää samoja `i2cWrite7` / `i2cRead7` / `i2cReset`-funktioita kuin nykyinen koodi):

```cpp
#define CMD_STATUS    22
#define CMD_EDGE_DATA 26
#define CMD_CAPTURE   32

void readCapture(uint8_t a) {
  i2cWrite7(a, CMD_STATUS);          uint32_t st  = i2cRead7(a, 8);
  if (((st >> 28) & 3) != 2) { Serial.println("ei kaapattu"); return; }
  uint8_t  N   = (st >> 8) & 0xFF;                                          // muunnoksia / lukema
  i2cWrite7(a, CMD_PRINT, 5, 1);     uint32_t eka = i2cRead7(a, 8);          // ekaAika (sykliä)
  i2cWrite7(a, CMD_PRINT, 6, 1);     uint32_t P   = i2cRead7(a, 8);          // lukemajakso (sykliä)
  i2cWrite7(a, CMD_PRINT, 1, 1);     uint8_t  len = i2cRead7(a, 8);          // lukemia
  i2cWrite7(a, CMD_RESULT);          uint32_t vika = i2cRead7(a, 8);         // firmwaren sovitus: eka + vika
  i2cWrite7(a, CMD_PRINT, 60, 1);    uint32_t lo  = i2cRead7(a, 8);          // taso ennen (Q3)
  i2cWrite7(a, CMD_PRINT, 61, 1);    uint32_t hi  = i2cRead7(a, 8);          // taso jälkeen (Q3)
  Serial.printf("N=%u P=%lu eka=%lu vika=%lu lo=%lu hi=%lu len=%u\n", N, (unsigned long)P, (unsigned long)eka, (unsigned long)vika, (unsigned long)lo, (unsigned long)hi, len);
  for (uint8_t i = 0; i < len; i += 8) {
    i2cWrite7(a, CMD_EDGE_DATA, i, 1);
    if (Wire.requestFrom(a, (uint8_t)16) != 16) { i2cReset(); return; }
    for (uint8_t j = 0; j < 8 && i + j < len; j++) {
      uint16_t v = Wire.read(); v |= (uint16_t)Wire.read() << 8;
      Serial.printf("%u,%lu,%u\n", i + j, (unsigned long)(eka + (uint32_t)(i + j) * P), v);   // indeksi, aika (sykliä), lukema
    }
  }
}
// käyttö: i2cWrite7(addr, CMD_CAPTURE, 0, 1);  ... odota ohitus ... readCapture(addr);
```

**Varoitus (TWI-keskeytyksen kesto):** I2C-callbackit (`requestEvent`, `receiveEvent`) ajetaan TWI-keskeytyksessä, joka ohittaa
TCA0-ylivuoto- ja TCB0-keskeytykset. Jos yksittäinen TWI-keskeytys kestää pidempään kuin P (84 µs = 844 sykliä @ 10 MHz),
TCA0:n ylivuoto voi jäädä laskematta → **kaikki myöhemmät aikaleimat siirtyvät P:n verran**. Siksi `EDGE_DATA`-lohko on vain
8 lukemaa (arvio: n. 600–700 sykliä sisältäen `Wire.write`:n; **ei mitattu**). Tarkista laitteella: lue `CMD_RUNTIME` ennen ja jälkeen
pitkän lukusarjan ja vertaa ESP:n `micros()`-aikaan. Jos aika hyppää, pienennä lohkoa (`EDGE_DATA`-silmukan 8) tai aja ATtiny 20 MHz:llä.

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

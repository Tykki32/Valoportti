/* =============================================================================
 * Valoportti816  -  ATtiny816, I2C-orja, tarkka valoportin reunan keskiaika
 * =============================================================================
 * Korvaa ValoporttiAttiny13_3_message-sketsin. I2C-protokolla on sama (komennot 1-21, 7 bittiä/tavu,
 * MSB ensin) + uudet komennot 22-32. Ks. README.md.
 *
 * MITEN AIKA SYNTYY
 *   - TCA0 käy vapaasti jaksolla P (CLK_PER-sykliä). Jokainen TCA0:n ylivuoto laukaisee ADC:n EVSYS:n kautta
 *     (ADC0 STARTEI). ADC kerryttää N muunnosta yhdeksi lukemaksi (SAMPNUM) ja keskeytys (RESRDY) tallettaa sen.
 *   - Lukeman j ajoitus on siis kokonaislukulaskentaa: laukaisu j+1 tapahtuu hetkellä (j+1)*P sykliä, ja
 *     vaikuttava näytehetki on laukaisu + vakio (datasheetin ADC-ajoituksesta, ks. delta_eff). Ei keskeytysviiveitä.
 *   - Aikaleima (RUNTIME, PA4-pulssi, reuna) on samalla aikaperustalla: t = ylivuotolaskuri * P + TCA0.CNT.
 *   - PA4-pulssi tallentuu TCB0:n capture-yksikköön. TCB0 käynnistyy uudelleen TCA0:n ylivuodossa (SYNCUPD),
 *     joten CCMP = TCA0.CNT tapahtumahetkellä (keskeytysviive ei vaikuta).
 *   - Reunat tunnistaa ja sovittaa edge_core.cpp (suhteellinen, ei kiinteitä kalibrointiarvoja).
 *
 * AJAT OVAT CLK_PER-SYKLEJÄ, 32-BITTISINÄ (kiertävät: 2^32 sykliä = n. 7 min @10 MHz; vastaukset 7 bittiä/tavu, 8 tavua,
 * ylimmät bitit nollia). Erotukset lasketaan modulo 2^32. Näytejakso (P) riippuu ADC-asetuksista: ADC-asetuksen vaihto (CMD_VREF, CMD_ADC_CFG,
 * CMD_CLOCK, lämpötilamittaus) KÄYNNISTÄÄ AIKAPERUSTAN ALUSTA (aika nollautuu). Lähetä PA4-synkronointi sen jälkeen.
 *
 * TESTAAMATON LAITTEELLA. Katso README.md "Tarkistuslista ensimmäiseen käynnistykseen".
 * ============================================================================= */
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "edge_core.h"

#ifndef _PROTECTED_WRITE
#define _PROTECTED_WRITE(reg, val) do { CCP = CCP_IOREG_gc; (reg) = (val); } while (0)
#endif

#define FW_VERSION          1

/* Ominaisuuskytkimet: jos käännös ylittää flashin (ATtiny816: 8 kt), aseta jokin arvoon 0 (esim. #define FEAT_TEMP 0). */
#ifndef FEAT_TEMP
#define FEAT_TEMP   1       /* lämpötilamittaus (CMD_TEMP_START/STOP) */
#endif
#ifndef FEAT_CLOCK
#define FEAT_CLOCK  1       /* ulkoinen yhteinen kello PA3:lla (CMD_CLOCK) */
#endif
#ifndef FEAT_REPORT
#define FEAT_REPORT 1       /* PA1-ilmoituspulssi reunan alkaessa (CMD_REPORT_*) */
#endif
#define I2C_ADDR_DEFAULT    0x40

/* ---------- I2C-komennot (1-21 = vanhat, 22-32 = uudet) ---------- */
#define CMD_PA4_TIME     1   /* L  viimeisimmän PA4-pulssin aika (sykliä); 1 = ei pulssia (uusi pulssi ylikirjoittaa edellisen) */
#define CMD_ADC_START    2   /* S  käynnistä keskiarvoistava mittaus (valotason tilastot) */
#define CMD_ADC_STOP     3   /* S  keskeytä */
#define CMD_ADC_RESULT   4   /* L  keskiarvo / muunnos x 65536 (Q16, 10-bittinen yksikkö) tai -1 jos ei valmis */
#define CMD_ADC_VALUE    5   /* L  viimeisin lukema (summa N muunnoksesta), 2 tavua */
#define CMD_ADC_LOWER    6   /* S  (vanha) tallennetaan, ei käytetä: tunnistus on suhteellinen */
#define CMD_ADC_UPPER    7   /* S  (vanha) tallennetaan, ei käytetä */
#define CMD_RESULT       8   /* L  jonon vanhimman reunan keskiaika suhteessa ekaAika:an (sykliä) tai -1 */
#define CMD_START        9   /* S  tyhjennä jono ja viritä reunantunnistus */
#define CMD_STOP        10   /* S  lopeta reunantunnistus */
#define CMD_I2C_TEST    11   /* L  17 */
#define CMD_RUNTIME     12   /* L  laitteen aikaleima nyt (sykliä) */
#define CMD_PRINT       15   /* L  debug, valitsin = payload (ks. requestEvent) */
#define CMD_REPORT_TRUE 16   /* S  PA1-pulssi kun reuna havaitaan (kertakäyttöinen) */
#define CMD_REPORT_FALSE 17  /* S  */
#define CMD_VREF        18   /* S  ADC-referenssi: 0=0,55 V 1=1,1 V 2=2,5 V 3=4,34 V 4=1,5 V 5=VDD */
#define CMD_ID          19   /* L  UID:n 4 alinta tavua */
#define CMD_TEMP_START  20   /* S  lämpötilamittaus (aikaperusta nollautuu) */
#define CMD_TEMP_STOP   21   /* L  lämpötilatulos tai -1 jos ei valmis */
#define CMD_STATUS      22   /* L  tila ja asetukset (ks. build_status) */
#define CMD_EDGE_NEXT   25   /* S  poista jonon vanhin tulos */
#define CMD_EDGE_DATA   26   /* S+L payload = aloitusindeksi; luku palauttaa 8 lukemaa RAAKANA (little-endian, 16 tavua) */
#define CMD_CLOCK       27   /* S  payload = ulkoisen kellon kHz (PA3), 0 = takaisin sisäiseen */
#define CMD_ADC_CFG     28   /* S  payload = kertymäkoodi (0..3 -> N = 1,2,4,8; 7 = auto) | samplen<<3 | esijakajakoodi+1 << 8 (0 = auto) */
#define CMD_ADC_NOISE   29   /* (varattu) kohina: CMD_PRINT valitsin 2 = kohinan keskipoikkeama x 8 */
#define CMD_TIME_ADJ    30   /* S  aikakorjaus (etumerkillinen, sykliä), lisätään kaikkiin reunaaikoihin */
#define CMD_EDGE_PARAM  31   /* S  thr_min | min_step<<10 | thr_mult<<24 */
#define CMD_CAPTURE     32   /* S  payload = ohitettavien reunojen määrä (0..7): kaappaa seuraava reuna kokonaisena, pysäytä tunnistus, säilytä raakadata */

/* ---------- vakiot ---------- */
#define RB_N            16                 /* ADC-rengas (2-potenssi) */
#define QN               8                 /* tulosjono */
#define T_READ_TARGET_HZ 12500UL           /* lukemajakso >= 1/12500 s = 80 us (reunapuskuri 50 lukemaa riittää 2,6 ms:lle) */
#define VREF_COUNT       6

/* ---------- tila ---------- */
struct __attribute__((packed)) Rec { uint32_t j0; int16_t tq8; int16_t H; uint8_t flags; };   /* 9 tavua */

static volatile uint32_t tca_ovf = 0;          /* TCA0-ylivuodot */
static volatile uint16_t rb_val[RB_N];
static volatile uint8_t  rb_w = 0;
static uint8_t           rb_r = 0;

static EdgeDetector det;
static EdgeCfg      edcfg = { 6, 40, 5 };
static Rec          q[QN];
static uint8_t      qh = 0, qn = 0;
static uint32_t     det_off = 0;               /* ilmaisimen ensimmäisen lukeman indeksi (j) */
static uint32_t     j_cons = 0;                /* seuraavan kulutettavan lukeman indeksi j (0-pohjainen ADC:n käynnistyksestä) */
static uint16_t     last_reading = 0;
static uint8_t      detect_on = 0;
static uint8_t      raw_valid = 0, raw_len = 0;
static uint8_t      cap_skip = 0xFF;           /* kaappaus: 0xFF = pois, muuten ohitettavien reunojen määrä */
static uint8_t      cap_done = 0;              /* 1 = reuna kaapattu, raakadata säilytetty */
static volatile uint8_t req_cap = 0, cap_arg = 0;

/* ADC-asetukset ja johdetut arvot */
static uint16_t clk_khz = F_CPU / 1000UL;
static uint8_t  cfg_vref = 1;                  /* oletus 1,1 V: nopea ADC (0,55 V:lla CLK_ADC <= 260 kHz, ks. README) */
static uint8_t  cfg_nacc_code = 7;             /* 7 = auto */
static uint8_t  cfg_samplen = 0;
static uint8_t  cfg_presc = 0;                 /* 0 = automaattinen (datasheetin rajoissa), 1..7 = esijakajakoodi + 1 (voi ylittää rajan) */
static uint8_t  nacc_code = 3, presc_code = 3, samplen = 0;
static uint16_t Pcyc = 800;                    /* TCA0-jakso */
static uint16_t delta_eff = 0;                 /* laukaisusta vaikuttavaan näytehetkeen (sykliä) */
static int16_t  time_adj = 0;
#if FEAT_TEMP
static uint8_t  temp_mode = 0;
static uint8_t  mux_sel = 0;                   /* 0 = AIN7 (valokenno), 1 = lämpötila-anturi */
#else
#define temp_mode 0
#define mux_sel   0
#endif
static int16_t  t_off = 0;                     /* delta_eff + time_adj (sykliä) */

/* pyynnöt keskeytyksistä pääsilmukalle */
static volatile uint8_t req_apply = 0, req_start = 0, req_stop = 0, req_stats = 0, req_stats_stop = 0,
                        req_temp = 0, req_temp_stop = 0, req_clock = 0;
static volatile uint16_t req_clock_khz = 0;

/* tason mittaus (CMD_ADC_START / lämpötila): ilmaisimen tasoseuranta toimii mittarina */
static uint8_t  lvl_on = 0;

/* vanhat/yhteensopivuus */
static uint8_t  last_cmd = 0;
static uint32_t last_msg = 0;
static uint8_t  report = 0, rep_wait = 0;

/* PA4-synkronointipulssit (TCB0-capture) */
static volatile uint32_t sync_ov = 0;
static volatile uint16_t sync_cap = 0;
static volatile uint8_t  sync_n = 0;           /* >0 kun pulssi on tullut */

/* tilaliput */
#define ST_RING_OVERRUN 0x04
#define ST_SLIP         0x08
#define ST_QUEUE_LOST   0x10
#define ST_CLK_EXT      0x20
#define ST_CLK_PENDING  0x40
static uint8_t st_flags = 0;
static uint8_t clk_ext_active = 0;
static uint8_t clk_saved_ctrlb = 0;

/* lämpötilakalibrointi (UID:n mukaan, kuten ennen) */
static uint8_t  I2C_ADDR = I2C_ADDR_DEFAULT;
static int16_t  Temp_offset = 0;
static uint8_t  Temp_kerroin = 0;

static uint32_t level_q16(void);

/* Pinon vapaan tilan mittaus: setup() maalaa vapaan RAM:in 0xAA:lla, stack_free() laskee koskemattomat tavut (CMD_PRINT 9) */
extern uint8_t __bss_end;
static void stack_paint(void) {
    uint8_t *p = &__bss_end, *sp = (uint8_t *)SP;
    while (p + 16 < sp) *p++ = 0xAA;
}
static uint16_t stack_free(void) {
    uint8_t *p = &__bss_end; uint16_t n = 0;
    while (p <= (uint8_t *)RAMEND && *p == 0xAA) { p++; n++; }
    return n;
}

/* ======================== apufunktiot ======================== */
static uint32_t read_uid() {
    uint32_t t = 0;
    t = (t << 8) | SIGROW.SERNUM6; t = (t << 8) | SIGROW.SERNUM7;
    t = (t << 8) | SIGROW.SERNUM8; t = (t << 8) | SIGROW.SERNUM9;
    return t;
}

/* ajan lukeminen atomisesti; korjaa "ylivuoto ehti tapahtua mutta ISR ei vielä ajettu" -tapauksen */
static void ts_now(uint32_t *ov, uint16_t *cnt) {
    uint8_t sreg = SREG; cli();
    uint32_t o = tca_ovf; uint16_t c = TCA0.SINGLE.CNT;
    if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) { o++; c = TCA0.SINGLE.CNT; }
    SREG = sreg;
    *ov = o; *cnt = c;
}

/* 7 bittiä / tavu, MSB ensin, n tavua (enintään 8); ylimmät tavut nollia */
static __attribute__((noinline)) void enc7(uint32_t v, uint8_t n, uint8_t *out) {
    while (n) { out[--n] = (uint8_t)(v & 0x7F); v >>= 7; }
}
/* lyhyt odotus (~us) ilman F_CPU-riippuvuutta */
static void wait_us(uint8_t us) {
    uint16_t loops = (uint16_t)((clk_khz / 1000) * us / 4);
    while (loops--) __asm__ __volatile__("nop");
}

/* ======================== ADC / aikaperusta ======================== */
static const uint8_t VREF_CODE[VREF_COUNT] = { VREF_ADC0REFSEL_0V55_gc, VREF_ADC0REFSEL_1V1_gc, VREF_ADC0REFSEL_2V5_gc,
                                               VREF_ADC0REFSEL_4V34_gc, VREF_ADC0REFSEL_1V5_gc, 0 };

/* Valitsee esijakajan, kertymän (N <= 8) ja laskee jakson P sekä vaikuttavan näytehetken (datasheet 30.3.2) */
static void adc_derive(void) {
    uint16_t lim = (cfg_vref == 0 && !mux_sel) ? 260 : 1500;       /* CLK_ADC:n yläraja kHz (taulukko 36-26) */
    if (mux_sel) lim = 500;                                         /* lämpötila-anturi: pitkä näytteistysaika */
    uint8_t code = 0;
    if (cfg_presc) code = (uint8_t)(cfg_presc - 1);                 /* käyttäjän valinta: ei rajatarkistusta */
    else while (code < 7 && (clk_khz >> (code + 1)) > lim) code++;
    presc_code = code;
    uint16_t div = (uint16_t)(2U << code);
    samplen = cfg_samplen;
    if (mux_sel) {                                                  /* SAMPLEN >= 32 us x f_ADC (datasheet 30.3.2.6) */
        uint16_t s = (uint16_t)((((uint32_t)clk_khz / div) * 32UL + 999UL) / 1000UL);
        samplen = (uint8_t)(s > 31 ? 31 : s);
    }
    uint16_t conv = (uint16_t)((13 + samplen) * div);              /* yhden muunnoksen kesto (CLK_PER-sykliä) */
    uint8_t c = cfg_nacc_code;
    if (c > 3) {                                                    /* auto (7) tai liian suuri: valitaan N <= 8 niin että jakso >= 80 us */
        uint16_t target = clk_khz / 12;                             /* 80 us = clk_khz * 0,08 */
        c = 0;
        while (c < 3 && (uint16_t)(conv << c) < target) c++;
    }
    nacc_code = c;
    uint8_t nacc = (uint8_t)(1U << c);
    uint16_t startd = (uint16_t)(div / 2 + 2);                     /* aloitusviive laukaisusta (datasheet) */
    Pcyc = (uint16_t)(startd + nacc * conv + 24);                  /* +24 sykliä: tulos valmis ja luettu ennen seuraavaa laukaisua */
    /* vaikuttava näytehetki: aloitusviive + (N-1)/2 muunnosta + (2 + SAMPLEN) CLK_ADC-sykliä */
    delta_eff = (uint16_t)((2U * (startd + (2 + samplen) * div) + (nacc - 1) * conv + 1) / 2);
    t_off = (int16_t)(delta_eff + time_adj);
}

static void tca_restart(void) {
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLD = 0;                                      /* SINGLE-tila */
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
    TCA0.SINGLE.PER = Pcyc - 1;
    TCA0.SINGLE.CNT = 0;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    tca_ovf = 0;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}

/* Ottaa ADC-asetukset käyttöön ja käynnistää aikaperustan alusta. Vain pääsilmukasta. */
static void adc_apply(void) {
    ADC0.CTRLA = 0;                                             /* pois päältä */
    ADC0.INTCTRL = 0;
    TCA0.SINGLE.CTRLA = 0;
    adc_derive();
    if (mux_sel) { VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc; }       /* lämpötila: 1,1 V */
    else if (cfg_vref < VREF_COUNT) VREF.CTRLA = VREF_CODE[cfg_vref];
    VREF.CTRLB = VREF_ADC0REFEN_bm;                             /* referenssi aina päällä */
    uint8_t ref_vdd = (!mux_sel && cfg_vref == 5);
    uint8_t sampcap = (!mux_sel && cfg_vref == 0) ? 0 : ADC_SAMPCAP_bm;     /* 0 = suositus < 1 V (datasheet CTRLC) */
    ADC0.CTRLB = nacc_code;                                     /* SAMPNUM */
    ADC0.CTRLC = (uint8_t)(presc_code | sampcap | (ref_vdd ? ADC_REFSEL_VDDREF_gc : ADC_REFSEL_INTREF_gc));
    ADC0.CTRLD = 0;                                             /* INITDLY = 0, ASDV = 0 (deterministinen ajoitus), SAMPDLY = 0 */
    ADC0.SAMPCTRL = samplen;
    ADC0.MUXPOS = mux_sel ? ADC_MUXPOS_TEMPSENSE_gc : ADC_MUXPOS_AIN7_gc;
    ADC0.EVCTRL = ADC_STARTEI_bm;                               /* laukaisu EVSYS:stä */
    PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
    wait_us(150);                                               /* referenssi ja anturi asettuvat (VREF ~22 us, lämpötila-anturi) */
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.INTCTRL = ADC_RESRDY_bm;
    EVSYS.SYNCCH0 = 0x02;                                       /* TCA0_OVF_LUNF */
    EVSYS.ASYNCUSER1 = 0x01;                                    /* ADC0 kuuntelee SYNCCH0:aa */
    rb_w = 0; rb_r = 0; j_cons = 0;
    ADC0.CTRLA = ADC_ENABLE_bm;                                 /* 10-bit, ei free-run */
    tca_restart();                                              /* ensimmäinen laukaisu P sykliä tästä: lukema j=0 */
    det_off = 0; sync_n = 0;
    ed_restart(&det, last_reading);
    raw_valid = 0; qn = 0; qh = 0;
}

/* ADC-lukeman keskeytys (LVL1, ohittaa TWI/TCA0/TCB0): vain lukema renkaaseen. RES:n luku kuittaa lipun. */
ISR(ADC0_RESRDY_vect) {
    uint8_t w = rb_w;
    rb_val[w & (RB_N - 1)] = ADC0.RES;
    rb_w = w + 1;
}
/* TCA0-ylivuoto: aikaperusta (LVL0; ts_now/TCB0 korjaavat "ylivuoto tapahtunut mutta ISR ei vielä ajettu" -tapauksen) */
ISR(TCA0_OVF_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    tca_ovf++;
}
/* PA4-pulssi: CCMP = TCA0.CNT tapahtumahetkellä (TCB0 restartaa TCA0:n ylivuodossa) */
ISR(TCB0_INT_vect) {
    uint16_t cap = TCB0.CCMP;                                   /* kuittaa lipun */
    uint16_t cnt = TCA0.SINGLE.CNT;
    uint32_t ov = tca_ovf;
    if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) { ov++; cnt = TCA0.SINGLE.CNT; }
    if (cap > cnt) ov--;                                        /* tapahtuma edellisessä jaksossa */
    sync_ov = ov; sync_cap = cap;
    uint8_t n = sync_n; sync_n = (n < 250) ? n + 1 : n;
}

/* ======================== tulosjono ======================== */
static void queue_push(uint32_t j0, int16_t tq8, int16_t H, uint8_t flags) {
    if (qn >= QN) { st_flags |= ST_QUEUE_LOST; return; }
    uint8_t t = (uint8_t)((qh + qn) % QN);
    q[t].j0 = j0; q[t].tq8 = tq8; q[t].H = H; q[t].flags = flags;
    qn++;
}
/* tuloksen aika (sykliä, mod 2^32): eka = 1 -> puskurin alun (buf[0]) aika, 0 -> ylityshetki */
static uint32_t rec_time(const Rec *r, uint8_t eka) {
    uint32_t t = (r->j0 + 1) * (uint32_t)Pcyc + (uint32_t)(int32_t)t_off;     /* laukaisu (j0+1) tapahtuu hetkellä (j0+1)*P */
    if (!eka) t += ((uint32_t)(uint16_t)r->tq8 * Pcyc) >> 8;
    return t;
}
static uint32_t head_vika(void) { return ((uint32_t)(uint16_t)q[qh].tq8 * Pcyc) >> 8; }   /* ylitys suhteessa ekaAika:an */

/* ======================== I2C ======================== */
static void receiveEvent(int) {
    if (Wire.available()) last_cmd = Wire.read();
    uint32_t m = 0;
    while (Wire.available()) m = (m << 7) | (Wire.read() & 0x7F);
    last_msg = m;
    switch (last_cmd) {
        case CMD_ADC_LOWER:  break;                                /* (vanha) vastaanotetaan, ei talleteta: tunnistus on suhteellinen */
        case CMD_ADC_UPPER:  break;
        case CMD_START:      req_start = 1; break;
        case CMD_CAPTURE:    cap_arg = (uint8_t)m; req_cap = 1; break;
        case CMD_STOP:       req_stop = 1; break;
        case CMD_ADC_START:  req_stats = 1; break;
        case CMD_ADC_STOP:   req_stats_stop = 1; break;
#if FEAT_REPORT
        case CMD_REPORT_TRUE:  report = 1; break;                  /* PA1 pysyy tulona + pull-up; vedetään alas vasta pulssin ajaksi */
        case CMD_REPORT_FALSE: report = 0; PORTA.DIRCLR = PIN1_bm; break;
#endif
        case CMD_VREF:       if (m < VREF_COUNT) { cfg_vref = (uint8_t)m; req_apply = 1; } break;
#if FEAT_TEMP
        case CMD_TEMP_START: req_temp = 1; break;
#endif
#if FEAT_CLOCK
        case CMD_CLOCK:      req_clock_khz = (uint16_t)m; req_clock = 1; break;
#endif
        case CMD_ADC_CFG:    cfg_nacc_code = (uint8_t)(m & 7); cfg_samplen = (uint8_t)((m >> 3) & 31); cfg_presc = (uint8_t)((m >> 8) & 7); req_apply = 1; break;
        case CMD_TIME_ADJ:   time_adj = (int16_t)(m & 0xFFFF); t_off = (int16_t)(delta_eff + time_adj); break;
        case CMD_EDGE_NEXT:  if (qn) { qh = (uint8_t)((qh + 1) % QN); qn--; } break;
        case CMD_EDGE_PARAM: edcfg.thr_min = (uint16_t)(m & 0x3FF); edcfg.min_step = (uint16_t)((m >> 10) & 0x3FFF);
                             edcfg.thr_mult = (uint8_t)((m >> 24) & 0x7F); break;
        default: break;
    }
}

static void requestEvent() {
    uint32_t v = 0;                                              /* vastaus (mod 2^32) */
    uint8_t n = 8, nr = 1, b[8];                                /* nr = 1: "ei valmis" (-1 = 8 x 0x7F), kunnes toisin asetetaan */
    uint8_t i = (uint8_t)last_msg;
    switch (last_cmd) {
        case CMD_I2C_TEST: v = 17; n = 1; nr = 0; break;
        case CMD_ID:       v = read_uid(); nr = 0; break;
        case CMD_PA4_TIME: {
            nr = 0; v = 1;                                       /* 1 = ei pulssia (kuten ennen) */
            if (sync_n) { uint8_t sreg = SREG; cli(); v = sync_ov * (uint32_t)Pcyc + sync_cap; SREG = sreg; }
            break; }
        case CMD_RUNTIME: {
            uint32_t ov; uint16_t cnt; ts_now(&ov, &cnt);
            v = ov * (uint32_t)Pcyc + cnt; nr = 0; break; }
        case CMD_ADC_RESULT: if (lvl_on && det.state >= ED_TRACK) { v = level_q16(); nr = 0; req_stats_stop = 1; } break;
        case CMD_ADC_VALUE:  v = last_reading; n = 2; nr = 0; break;
#if FEAT_TEMP
        case CMD_TEMP_STOP:
            if (temp_mode && det.state >= ED_TRACK) {
                v = Temp_kerroin ? level_q16() / Temp_kerroin : 0;
                v += (uint32_t)(int32_t)Temp_offset; nr = 0; req_temp_stop = 1;
            }
            break;
#endif
        case CMD_STATUS:
            v = (uint32_t)(st_flags | (detect_on ? 0x01 : 0) | (det.state >= ED_TRACK ? 0x02 : 0) | (lvl_on ? 0x80 : 0))
              | ((uint32_t)(1U << nacc_code) << 8) | ((uint32_t)cfg_vref << 16) | ((uint32_t)presc_code << 20)
              | ((uint32_t)qn << 24) | ((uint32_t)(cap_done ? 2 : (cap_skip != 0xFF ? 1 : 0)) << 28);
            nr = 0; break;
        case CMD_RESULT: if (qn) { v = head_vika(); nr = 0; } break;
        case CMD_PRINT: {                                        /* debug; valitsin = last_msg */
            uint16_t sel = (uint16_t)last_msg; nr = 0;
            if (sel == 1)      v = raw_len;                                           /* reunapuskurin pituus */
            else if (sel == 2 || sel == 3) v = 0;                                     /* (vanha) alaraja / yläraja: ei talleteta */
            else if (sel == 4) { if (qn) v = head_vika(); else nr = 1; }              /* vikaAika: ylitys suhteessa ekaAika:an */
            else if (sel == 5) { if (qn) v = rec_time(&q[qh], 1); else nr = 1; }      /* ekaAika */
            else if (sel == 6) v = Pcyc;                                              /* lukemajakso (sykliä) */
            else if (sel == 7) { if (qn) v = (uint32_t)(uint16_t)(q[qh].H + 32768) | ((uint32_t)q[qh].flags << 16); else nr = 1; }
            else if (sel == 9)  v = stack_free();                                     /* pinossa koskematonta tilaa (tavua): pienin havaittu pitää olla > ~20 */
            else if (sel == 60) v = det.old3;                                         /* reunaa edeltävä taso, Q3 (8 x keskiarvo) */
            else if (sel == 61) v = det.base3;                                        /* jälkitaso (48 lukeman keskiarvo), Q3; pätee kaappauksen jälkeen */
            else if (sel == 8) v = det.dev3;                                          /* kohinan keskipoikkeama x 8 (lukemayksikköä) */
            else if (sel >= 10) { uint16_t k = sel - 10; v = (raw_valid && k < raw_len) ? det.buf[k] : 0; }   /* 0 = lista loppu (kuten ennen) */
            else n = 0;
            break; }
        case CMD_EDGE_DATA: {                                    /* RAAKA lohko: 8 lukemaa little-endian alkaen indeksistä last_msg (lyhyt: TWI-keskeytys < P; ei välipuskuria: pino) */
            for (uint8_t j = 0; j < 8; j++) {
                uint8_t idx = (uint8_t)(i + j);
                uint16_t w = (raw_valid && idx < raw_len) ? det.buf[idx] : 0xFFFF;
                Wire.write((uint8_t)w); Wire.write((uint8_t)(w >> 8));
            }
            return; }
        default: n = 0; break;
    }
    if (!n) return;
    if (nr) for (uint8_t j = 0; j < 8; j++) b[j] = 0x7F; else enc7(v, n, b);
    Wire.write(b, n);
}

/* ======================== pääsilmukan palvelut ======================== */
/* yhden muunnoksen keskitaso x 65536 ilmaisimen tasoseurannasta (base3 = 8 x lukeman keskiarvo) */
static uint32_t level_q16(void) { return ((uint32_t)det.base3 << 13) >> nacc_code; }

static void det_start(void) {
    qn = 0; qh = 0; raw_valid = 0; cap_skip = 0xFF; cap_done = 0; st_flags &= (uint8_t)~(ST_QUEUE_LOST | ST_SLIP | ST_RING_OVERRUN);
    ed_init(&det, &edcfg, last_reading);
    det_off = j_cons;
    detect_on = 1;
}

static void handle_edge(void) {
    EdgeRes r;
    ed_process(&det, &r);
    raw_len = det.len; raw_valid = 1;
    uint8_t take = r.ok && !(r.flags & (EF_WEAK | EF_NOFIT));
    if (cap_skip != 0xFF && !(r.flags & EF_WEAK)) {            /* kaappaustila: kohinan aiheuttamat heikot tunnistukset ohitetaan */
        if (cap_skip) cap_skip--;                              /* ohitettava reuna: käsitellään normaalisti */
        else { qn = 0; qh = 0; take = 1; cap_done = 1; detect_on = 0; cap_skip = 0xFF; }   /* kaapattu: jonoon vain tämä (myös jos sovitus epäonnistui), tunnistus pysähtyy, raakadata säilyy */
    }
    if (take) queue_push(det_off + det.k0, r.tq8, r.H, (uint8_t)(r.flags | (st_flags & ST_SLIP ? EF_SLIP : 0)));
    ed_rearm(&det, &r);
}

static void service_readings(void) {
    uint8_t avail = (uint8_t)(rb_w - rb_r);
    if (avail > RB_N) {                                         /* rengas ylivuoti: lukemia menetettiin */
        uint8_t lost = (uint8_t)(avail - RB_N);
        rb_r = (uint8_t)(rb_r + lost); j_cons += lost; st_flags |= ST_RING_OVERRUN | ST_SLIP;
        if (detect_on) {
            if (det.state <= ED_TRACK) det.k_next += lost;     /* menetetyt lukemat ovat tasaista aluetta: indeksit pysyvät j:n mukana */
            else { ed_restart(&det, last_reading); det_off = j_cons; }   /* kesken reunaa: aloita alusta */
        }
    }
    while (rb_r != rb_w) {
        uint16_t x = rb_val[rb_r & (RB_N - 1)];
        rb_r++; j_cons++;
        last_reading = x;
        if (lvl_on || temp_mode) {                              /* tasoseuranta (ei reunatuloksia) */
            ed_push(&det, x);
            if (det.state >= ED_COLLECT) ed_restart(&det, x);   /* taso hyppäsi (esim. ledejä kytketty): seuranta uudelleen alusta */
        }
        else if (detect_on) {
            uint8_t before = det.state;
            if (det.state == ED_DONE) handle_edge();            /* edellinen vielä käsittelemättä (ei pitäisi tapahtua) */
            ed_push(&det, x);
            if (before != ED_COLLECT && det.state == ED_COLLECT) {
                raw_valid = 0;                                  /* puskuria aletaan täyttää: vanha raakadata ei enää voimassa */
#if FEAT_REPORT
                if (report) { PORTA.DIRSET = PIN1_bm; rep_wait = 2; }   /* PA1-pulssi (LOW) reunan alkaessa; OUT-säie on 0 */
#endif
            }
            if (det.state == ED_DONE) handle_edge();
        }
#if FEAT_REPORT
        if (rep_wait && --rep_wait == 0) {                      /* pulssi päättyy ~1-2 lukemaa myöhemmin */
            PORTA.DIRCLR = PIN1_bm; report = 0;
        }
#endif
    }
}

#if FEAT_CLOCK
static void clock_service(void) {
    if (req_clock) {
        req_clock = 0;
        uint16_t khz = req_clock_khz;
        if (khz) {
            if (!clk_ext_active) clk_saved_ctrlb = CLKCTRL.MCLKCTRLB;
            _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);              /* CLK_PER = CLK_MAIN */
            _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
            st_flags |= ST_CLK_PENDING;
            clk_ext_active = 2;                                  /* 2 = vaihto odottaa reunoja */
        } else {
            _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);
            if (clk_ext_active) { _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, clk_saved_ctrlb); }
            clk_ext_active = 0; st_flags &= (uint8_t)~(ST_CLK_PENDING | ST_CLK_EXT);
            clk_khz = F_CPU / 1000UL; req_apply = 1;
        }
    }
    if (clk_ext_active == 2) {                                   /* datasheet 10.3.2: vaihto tapahtuu vasta kun reunoja havaitaan */
        if (!(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) && (CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm)) {
            clk_ext_active = 1; st_flags = (uint8_t)((st_flags & ~ST_CLK_PENDING) | ST_CLK_EXT);
            clk_khz = req_clock_khz; req_apply = 1;
        }
    }
}
#else
static inline void clock_service(void) {}
#endif

static void service_requests(void) {
    clock_service();
    if (req_stop)       { req_stop = 0; detect_on = 0; cap_skip = 0xFF; }
    if (req_stats_stop) { req_stats_stop = 0; lvl_on = 0; }
#if FEAT_TEMP
    if (req_temp)       { req_temp = 0; temp_mode = 1; mux_sel = 1; adc_apply(); }
    if (req_temp_stop)  { req_temp_stop = 0; temp_mode = 0; mux_sel = 0; req_apply = 1; }
#endif
    if (req_apply)      { req_apply = 0; adc_apply(); }
    if (req_stats)      { req_stats = 0; ed_restart(&det, last_reading); lvl_on = 1; }
    if (req_start)      { req_start = 0; det_start(); }
    if (req_cap)        { req_cap = 0; det_start(); cap_skip = cap_arg; }
}

/* ======================== setup / loop ======================== */
void setup() {
    stack_paint();
    switch (read_uid()) {
        case 2235643953UL: I2C_ADDR = 0x41; Temp_kerroin = 114; Temp_offset = -5315; break;
        case 3314495277UL: I2C_ADDR = 0x42; Temp_kerroin = 115; Temp_offset = 4774; break;
        default: I2C_ADDR = I2C_ADDR_DEFAULT; break;
    }
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    PORTA.DIRCLR = PIN7_bm;
    PORTA.OUTCLR = PIN1_bm; PORTA.DIRCLR = PIN1_bm; PORTA.PIN1CTRL = PORT_PULLUPEN_bm;   /* PA1: raportointilinja, avoin nielu: lepotilassa tulo + pull-up */
    PORTA.DIRCLR = PIN4_bm; PORTA.PIN4CTRL = PORT_ISC_INTDISABLE_gc;   /* PA4: synkronointi (tulopuskuri päällä) */
    cli();
    CPUINT.LVL1VEC = ADC0_RESRDY_vect_num;                        /* ADC-tulos luetaan heti: TWI-keskeytys (LVL0) ei saa viivästyttää sitä */
    /* PA4 -> TCB0 capture (nouseva reuna, EDGE = 0) */
    EVSYS.ASYNCCH0 = 0x0E;                                        /* PORTA PIN4 */
    EVSYS.ASYNCUSER0 = 0x03;                                      /* TCB0 kuuntelee ASYNCCH0:aa */
    TCB0.CTRLA = 0;
    TCB0.CTRLB = TCB_CNTMODE_CAPT_gc;
    TCB0.EVCTRL = TCB_CAPTEI_bm;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    TCB0.CNT = 0;
    TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_SYNCUPD_bm | TCB_ENABLE_bm;
    sei();
    adc_apply();
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_1KCLK_gc);             /* ~1 s: jäädyttää/nollaa jos kello (EXTCLK) katoaa */
}

void loop() {
    wdt_reset();
    service_requests();
    service_readings();
}

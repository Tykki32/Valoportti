// edge_core.h - Valoportti816: kannettava (laiteriippumaton) reunantunnistus ja keskiajan estimointi.
// Ei Arduino- eikä rekisteririippuvuuksia: testattavissa isännällä (g++) ja simulaattorilla (simavr).
//
// ADC-lukemat (summa N <= 8 muunnoksesta, siis <= 8184) syötetään yksi kerrallaan ed_push():lle. Ilmaisin seuraa
// signaalin tasoa itse (ei kiinteitä kalibrointiarvoja), havaitsee reunan (nouseva tai laskeva askel), kerää
// sen lukemat ja ed_process() laskee reunan 50 %-tason ylityshetken lukemaindeksinä (Q8).
// Tasot talletetaan Q3-muodossa (8 x keskiarvo): 8 lukeman summa mahtuu uint16:een kun lukema <= 8184.
#ifndef EDGE_CORE_H
#define EDGE_CORE_H
#include <stdint.h>

#ifndef EDGE_BUF_N
#define EDGE_BUF_N   50     // reunapuskurin koko (lukemia): riittää ~2,6 ms reunalle kun lukemajakso >= ~80 us
#endif
#define EDGE_PRE      4     // ennakkonäytteitä = EDGE_PRE + EDGE_DET mukaan puskurin alkuun
#define EDGE_DET      3     // peräkkäisiä samansuuntaisia kynnysylityksiä havaintoon
#define EDGE_SETTLE   8     // peräkkäisiä "tasaantunut"-lukemia reunan loppuun
#ifndef EDGE_POST
#define EDGE_POST    40     // tasaantumisen jälkeen kerättäviä lukemia jälkitason tarkentamiseen (ei puskuriin)
#endif
#define EDGE_REG_MAX 20     // enintään näin monta lukemaa regressiossa
#define EDGE_WARMUP  16     // lohkoja (8 lukemaa) ennen kuin havainto sallitaan
#define EDGE_MAX_READING 8184

enum EdgeState { ED_WARMUP = 0, ED_TRACK = 1, ED_COLLECT = 2, ED_DONE = 3 };
enum EdgeFlags {
    EF_RISING   = 1,    // askel ylöspäin
    EF_OVERFLOW = 2,    // puskuri täyttyi ennen tasaantumista
    EF_FALLBACK = 4,    // regressio ei onnistunut: aika = lähin lukema
    EF_WEAK     = 8,    // askel alle min_step: tulosta ei käytetä
    EF_NOFIT    = 16,   // aikaa ei voitu laskea
    EF_SLIP     = 32    // näytteen menetys havaittu (asettaa kutsuja)
};

struct EdgeCfg {
    uint16_t thr_min;   // pienin havaintokynnys (lukemayksikköä)
    uint16_t min_step;  // pienin hyväksyttävä askel (lukemayksikköä)
    uint8_t  thr_mult;  // kynnys = thr_mult * kohinan keskipoikkeama
};

struct EdgeDetector {
    uint16_t buf[EDGE_BUF_N];  // reunan lukemat
    uint16_t ring[8];          // viimeiset 8 lukemaa
    uint32_t k_next;           // seuraavan lukeman indeksi
    uint32_t k0;               // buf[0]:n indeksi
    uint16_t base3;            // taso, Q3 (8 x keskiarvo)
    uint16_t prev3;            // edellisen lohkon summa (taso päivittyy lohkon viiveellä)
    uint16_t dev3;             // kohinan keskipoikkeama x 8
    uint16_t thr;              // havaintokynnys
    uint16_t old3;             // reunaa edeltävä taso (Q3)
    uint16_t sumad;            // lohkon |poikkeama|-summa
    uint32_t postsum;          // jälkitason lukemien summa (tasaantumisen jälkeen)
    uint8_t  rpos, len, state, cnt, settle, warm, post;
    uint8_t  overflow;
    struct EdgeCfg cfg;
};

struct EdgeRes {
    int16_t  tq8;     // ylityshetki, lukemaindeksejä buf[0]:sta, Q8 (256 = yksi lukemajakso)
    int16_t  H;       // askeleen koko (lukemayksikköä, etumerkillinen)
    uint8_t  flags;   // EdgeFlags
    uint8_t  npts;    // regressiossa käytetyt pisteet
    uint8_t  len;     // puskurin pituus
    uint8_t  ok;      // 1 = tulos käyttökelpoinen
};

#ifdef __cplusplus
extern "C" {
#endif
void    ed_init(struct EdgeDetector *e, const struct EdgeCfg *cfg, uint16_t level);
void    ed_restart(struct EdgeDetector *e, uint16_t level);       // lämmittely alusta
void    ed_push(struct EdgeDetector *e, uint16_t x);              // yksi uusi lukema; tila: e->state
void    ed_process(struct EdgeDetector *e, struct EdgeRes *r);    // kutsutaan kun state == ED_DONE
void    ed_rearm(struct EdgeDetector *e, const struct EdgeRes *r);// takaisin seurantaan
#ifdef __cplusplus
}
#endif
#endif

// edge_core.cpp - ks. edge_core.h
#include "edge_core.h"

static void set_thr(struct EdgeDetector *e) {
    uint16_t t = (uint16_t)(((uint32_t)e->dev3 * e->cfg.thr_mult) >> 3);
    e->thr = t < e->cfg.thr_min ? e->cfg.thr_min : t;
}

void ed_restart(struct EdgeDetector *e, uint16_t level) {
    uint8_t i;
    for (i = 0; i < 8; i++) e->ring[i] = level;
    e->base3 = (uint16_t)(level << 3); e->prev3 = e->base3;
    e->dev3 = 800;                          /* suuri alkuarvo (keskipoikkeama 100): ei vääriä havaintoja ennen konvergenssia */
    e->sumad = 0; e->rpos = 0; e->len = 0; e->cnt = 0; e->settle = 0;
    e->overflow = 0; e->warm = EDGE_WARMUP; e->state = ED_WARMUP;
    set_thr(e);
}

void ed_init(struct EdgeDetector *e, const struct EdgeCfg *cfg, uint16_t level) {
    e->cfg = *cfg; e->k_next = 0; e->k0 = 0; e->old3 = 0;
    ed_restart(e, level);
}

void ed_rearm(struct EdgeDetector *e, const struct EdgeRes *r) {
    (void)r;
    e->cnt = 0; e->settle = 0; e->len = 0; e->sumad = 0; e->overflow = 0; e->state = ED_TRACK;
}

void ed_push(struct EdgeDetector *e, uint16_t x) {
    uint8_t st = e->state;
    e->k_next++;
    if (st == ED_DONE) return;
    if (st == ED_COLLECT) {
        uint8_t n = e->len;
        e->buf[n] = x; e->len = ++n;
        int d = (int)x - (int)e->buf[n - 5];
        if (d < 0) d = -d;
        if ((unsigned)d <= e->thr + (e->thr >> 1)) { if (++e->settle >= EDGE_SETTLE) { e->state = ED_DONE; return; } }
        else e->settle = 0;
        if (n >= EDGE_BUF_N) { e->overflow = 1; e->state = ED_DONE; }
        return;
    }
    uint8_t rp = e->rpos;                          /* ED_WARMUP tai ED_TRACK */
    e->ring[rp & 7] = x;
    e->rpos = ++rp;
    int d = (int)x - (int)(e->base3 >> 3);
    unsigned ad = d < 0 ? -d : d;
    if (ad > e->thr) {
        if (st == ED_WARMUP) {                     /* taso muuttui pysyvästi: aloita lämmittely uudelta tasolta */
            if (++e->cnt >= 12) ed_restart(e, x);
            return;
        }
        if (++e->cnt >= EDGE_DET) {
            uint8_t i, k = EDGE_PRE + EDGE_DET;
            e->old3 = e->base3;
            for (i = 0; i < k; i++) e->buf[i] = e->ring[(uint8_t)(rp - k + i) & 7];
            e->len = k; e->k0 = e->k_next - k; e->settle = 0; e->overflow = 0; e->state = ED_COLLECT;
        }
        return;
    }
    e->cnt = 0;
    e->sumad += ad;
    if ((rp & 7) == 0) {                           /* 8 lukeman lohko täynnä */
        uint16_t sum = 0; uint8_t i;
        for (i = 0; i < 8; i++) sum += e->ring[i];
        e->base3 += (int16_t)((int32_t)e->prev3 - e->base3) >> 2;   /* viive: edellisen lohkon summalla */
        e->prev3 = sum;
        e->dev3 += (int16_t)(e->sumad - e->dev3) >> 2;
        e->sumad = 0;
        set_thr(e);
        if (e->warm && --e->warm == 0) e->state = ED_TRACK;
    }
}

/* lukema * 8 etäisyys tasosta M3 (Q3) */
static inline uint16_t dist3(uint16_t y, uint16_t M3) {
    uint16_t v = (uint16_t)(y << 3);
    return v > M3 ? (uint16_t)(v - M3) : (uint16_t)(M3 - v);
}

void ed_process(struct EdgeDetector *e, struct EdgeRes *r) {
    uint8_t n = e->len, i;
    uint16_t new3 = 0;
    for (i = 0; i < 8; i++) new3 += e->buf[n - 1 - i];      /* jälkitaso = 8 viimeisen summa (n >= 15), Q3 */
    uint16_t old3 = e->old3;
    uint8_t rising = new3 > old3;
    uint16_t aH3 = rising ? (uint16_t)(new3 - old3) : (uint16_t)(old3 - new3);
    r->ok = 0; r->tq8 = 0; r->npts = 0; r->len = n;
    r->flags = (uint8_t)((rising ? EF_RISING : 0) | (e->overflow ? EF_OVERFLOW : 0));
    { int16_t h = (int16_t)((aH3 + 4) >> 3); r->H = rising ? h : (int16_t)-h; }
    /* seuraava taso seurantaan riippumatta tuloksesta; vanhat lukemat pois renkaasta */
    e->base3 = new3; e->prev3 = new3;
    for (i = 0; i < 8; i++) e->ring[i] = (uint16_t)(new3 >> 3);
    if (aH3 < (uint16_t)(e->cfg.min_step << 3)) { r->flags |= EF_WEAK; return; }

    uint16_t M3 = (uint16_t)(((uint32_t)old3 + new3) >> 1);   /* 50 %-taso, Q3 */
    uint8_t ic = 0xFF;
    for (i = 0; i < n; i++) {
        uint16_t v = (uint16_t)(e->buf[i] << 3);
        if (rising ? v >= M3 : v <= M3) { ic = i; break; }
    }
    if (ic == 0xFF || ic == 0) { r->flags |= EF_NOFIT; return; }

    uint16_t q3 = (uint16_t)(((uint32_t)aH3 * 45) >> 7);        /* regressioikkuna: +-35 % askeleesta */
    uint8_t c = ic, ok = 0, pts = 0, it;
    int32_t x_q8 = (int32_t)ic * 256;                           /* varalla: lähin lukema */
    for (it = 0; it < 2; it++) {
        uint8_t lo = c, hi = c;
        while (lo > 0     && dist3(e->buf[lo - 1], M3) <= q3 && (uint8_t)(hi - lo) < EDGE_REG_MAX) lo--;
        while (hi < n - 1 && dist3(e->buf[hi + 1], M3) <= q3 && (uint8_t)(hi - lo) < EDGE_REG_MAX) hi++;
        if (hi - lo < 2) { lo = c > 0 ? c - 1 : 0; hi = (c + 1 < n) ? c + 1 : (uint8_t)(n - 1); }
        if (hi - lo < 2) break;
        uint8_t m = hi - lo + 1;
        int32_t Sy = 0, Sx2y = 0; int16_t D2 = 0;               /* keskitetty indeksi x2 = 2j - (m-1) */
        for (i = lo; i <= hi; i++) {
            int8_t x2 = (int8_t)(2 * (i - lo) - (m - 1));
            uint16_t y = e->buf[i];
            Sy += y; Sx2y += (int32_t)x2 * y; D2 += (int16_t)x2 * x2;
        }
        if (Sx2y == 0 || (Sx2y > 0) != rising) break;           /* kulmakertoimen etumerkin oltava askeleen suuntainen */
        int32_t dlt3 = (int32_t)M3 - (int32_t)((Sy * 8) / m);   /* tavoitetaso miinus ikkunan keskiarvo, Q3 */
        int32_t T = (dlt3 * 16 * D2) / Sx2y;                    /* Q8-indeksiä: 256*(dlt3/8)/b, b = 2*Sx2y/D2 per indeksi */
        int32_t xq = (int32_t)lo * 256 + (int32_t)(m - 1) * 128 + T;
        if (xq < 0 || xq > (int32_t)(n - 1) * 256) break;
        x_q8 = xq; ok = 1; pts = m;
        c = (uint8_t)((xq + 128) >> 8);
        if (c >= n) c = n - 1;
    }
    if (!ok) r->flags |= EF_FALLBACK;
    r->tq8 = (int16_t)x_q8; r->npts = pts; r->ok = 1;
}

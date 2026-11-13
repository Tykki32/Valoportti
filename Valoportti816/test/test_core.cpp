// Testi: portaikkosignaali (4 askelta alas, 4 ylös), kohina, ajautuva tasoero. Käyttää Valoportti816/edge_core:a sellaisenaan.
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>
#include "../edge_core.h"
static uint64_t S = 88172645463325252ULL;
static double rnd() { S ^= S << 13; S ^= S >> 7; S ^= S << 17; return (S >> 11) * (1.0 / 9007199254740992.0); }
static double gauss() { double u = rnd() + 1e-12, v = rnd(); return sqrt(-2 * log(u)) * cos(6.283185307179586 * v); }
static double smooth(double u) { if (u <= 0) return 0; if (u >= 1) return 1; return u * u * (3 - 2 * u); }
struct E { double t0, Te, delta; };
int main(int argc, char **argv) {
    double Ts   = (argc > 1 ? atof(argv[1]) : 83.0) * 1e-6;     // lukeman jakso
    int    N    = argc > 2 ? atoi(argv[2]) : 8;                 // kertymä (muunnosta / lukema)
    double sigc = argc > 3 ? atof(argv[3]) : 2.1;               // kohina per 10-bit muunnos
    double Hc   = argc > 4 ? atof(argv[4]) : 170.0;             // askel per muunnos (1/6 x 1023)
    int    runs = argc > 5 ? atoi(argv[5]) : 300;
    double rho  = argc > 6 ? atof(argv[6]) : 0.0;               // karheus (osuus askeleesta)
    double TeLo = argc > 7 ? atof(argv[7]) : 0.8, TeHi = argc > 8 ? atof(argv[8]) : 2.6;  // ms
    double gapMin = argc > 9 ? atof(argv[9]) : 10.0;            // reunojen väli (ms)
    double Ak[4] = {1.0, 0.82, 0.93, 0.74};                     // ledien suhteelliset askeleet
    int total = 0, matched = 0, falsepos = 0, weak = 0, nofit = 0, fb = 0, ovf = 0;
    std::vector<double> errs;
    EdgeCfg cfg; cfg.thr_min = 6; cfg.min_step = 40; cfg.thr_mult = 5;
    for (int run = 0; run < runs; run++) {
        E ed[8]; double drift = (rnd() - 0.5) * 0.2, slope = (rnd() - 0.5) * 0.06;
        double t = 30e-3, B0 = 100, L0 = B0;
        for (int k = 0; k < 4; k++) L0 += Ak[k] * Hc * (1 + drift);
        for (int k = 0; k < 8; k++) {
            double Te = (TeLo + (TeHi - TeLo) * rnd()) * 1e-3, g = 1 + drift + slope * (k / 7.0);
            ed[k].t0 = t; ed[k].Te = Te; ed[k].delta = (k < 4 ? -1 : 1) * Ak[k % 4] * Hc * g;
            t += (gapMin + 4 * rnd()) * 1e-3 + (k == 3 ? 40e-3 : 0);
        }
        double tend = t + 20e-3, phase = rnd() * Ts; int Nr = (int)(tend / Ts);
        EdgeDetector e; EdgeRes r; bool inited = false; double ar = 0, a = exp(-Ts / 0.25e-3);
        std::vector<double> est;
        for (int n = 0; n < Nr; n++) {
            double tt = n * Ts + phase, v = L0, w = 0;     // konversioarvo (10-bit yksikköä)
            for (int k = 0; k < 8; k++) { double u = (tt - ed[k].t0) / ed[k].Te, s = smooth(u); v += ed[k].delta * s; if (u > 0 && u < 1) w += fabs(ed[k].delta) * 4 * s * (1 - s); }
            ar = a * ar + sqrt(1 - a * a) * gauss();
            double sum = 0; for (int c = 0; c < N; c++) sum += std::min(1023.0, std::max(0.0, floor(v + rho * w * ar + sigc * gauss() + 0.5)));
            uint16_t x = (uint16_t)sum;
            if (!inited) { ed_init(&e, &cfg, x); inited = true; }
            ed_push(&e, x);
            if (e.state == ED_DONE) {
                ed_process(&e, &r);
                if (getenv("DBG") && run == 0) printf("  reuna: k0=%u len=%d H=%d flags=%d ok=%d tq8=%d thr=%d\n", (unsigned)e.k0, e.len, r.H, r.flags, r.ok, r.tq8, (int)e.thr);
                if (r.flags & EF_WEAK) weak++; else if (r.flags & EF_NOFIT) nofit++;
                if (r.flags & EF_FALLBACK) fb++;
                if (r.flags & EF_OVERFLOW) ovf++;
                if (r.ok) est.push_back(((double)e.k0 + r.tq8 / 256.0) * Ts + phase);
                ed_rearm(&e, &r);
            }
        }
        total += 8; std::vector<int> used(est.size(), 0);
        for (int k = 0; k < 8; k++) {
            double truth = ed[k].t0 + ed[k].Te / 2; int best = -1; double bd = 3e-3;
            for (size_t j = 0; j < est.size(); j++) if (!used[j] && fabs(est[j] - truth) < bd) { bd = fabs(est[j] - truth); best = j; }
            if (best >= 0) { used[best] = 1; matched++; errs.push_back((est[best] - truth) * 1e6); }
        }
        for (size_t j = 0; j < est.size(); j++) if (!used[j]) falsepos++;
    }
    if (errs.empty()) { printf("EI YHTÄÄN OSUMAA (tunnistettu %d)\n", total); return 1; }
    std::sort(errs.begin(), errs.end());
    double mean = 0, rms = 0; for (double x : errs) { mean += x; rms += x * x; }
    mean /= errs.size(); rms = sqrt(rms / errs.size());
    std::vector<double> ab; for (double x : errs) ab.push_back(fabs(x)); std::sort(ab.begin(), ab.end());
    double Hr = Hc * N, sr = sigc * sqrt((double)N), Te = 0.5 * (TeLo + TeHi) * 1e-3;
    double crb = (sr / Hr) * sqrt(Ts * Te / 1.2) * 1e6;
    printf("Ts=%5.1fus N=%2d sig_c=%.1f H_c=%.0f rho=%.2f Te=%.1f-%.1fms gap>=%.0fms | tunnistus %d/%d (%.1f%%) vääriä %d (heikko %d, nofit %d, varalla %d, ylivuoto %d) | virhe: ka %+5.2f RMS %5.2f 95%% %5.2f max %6.1f us  (alaraja ~%.2f)\n",
           Ts * 1e6, N, sigc, Hc, rho, TeLo, TeHi, gapMin, matched, total, 100.0 * matched / total, falsepos, weak, nofit, fb, ovf, mean, rms, ab[(int)(ab.size() * 0.95)], ab.back(), crb);
    return 0;
}

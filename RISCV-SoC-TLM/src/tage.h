#ifndef TAGE_H
#define TAGE_H

// =====================================================================
// TAGE: el predictor de saltos del FRONTEND (bloque "TAGE" del diagrama).
// Espejo EXACTO del tage.h de la pista HLS (RISCV-SoC/), con tipos
// planos de C++ en vez de ap_uint -- misma organizacion, mismos
// parametros, mismas decisiones, para conservar la paridad ciclo a
// ciclo entre las dos pistas.
//
//   BASE  : bimodal, 512 contadores de 2 bits, indexado solo por pc
//   T1/2/3: 128 entradas {tag 8b, ctr 3b con signo, u 2b},
//           historias de 4 / 8 / 16 bits (geometrica x2)
//   GHR   : 16 bits de historia global (1 bit por branch condicional)
//
// PREDICT en el dispatch; UPDATE en el commit (con la verdad, en orden);
// mispredict = redirigir + flush, el mismo camino de los traps.
// =====================================================================
#include <cstdint>

struct Tage {
    static const int BASE_BITS = 9;
    static const int BASE_SZ   = 1 << BASE_BITS;
    static const int TBL_BITS  = 7;
    static const int TBL_SZ    = 1 << TBL_BITS;
    static const int NTBL      = 3;

    struct Entry { uint8_t tag; int8_t ctr; uint8_t u; };

    uint8_t  base[BASE_SZ];       // bimodal 2b saturante
    Entry    t[NTBL][TBL_SZ];
    uint16_t ghr = 0;             // historia global (especulativa)

    static int hist_len(int i) { static const int H[NTBL] = {4, 8, 16}; return H[i]; }

    // pliega los `len` bits bajos de la historia sobre `out` bits
    static uint16_t fold(uint16_t h, int len, int out) {
        uint16_t masked = h & ((1u << len) - 1);
        uint16_t r = 0;
        for (int i = 0; i < len; i++)
            r ^= uint16_t(((masked >> i) & 1) << (i % out));
        return r;
    }
    int index(uint32_t pc, uint16_t g, int i) const {
        return ((pc >> 1) ^ fold(g, hist_len(i), TBL_BITS) ^ (pc >> (3 + i)))
               & (TBL_SZ - 1);
    }
    uint8_t tag_of(uint32_t pc, uint16_t g, int i) const {
        return uint8_t(((pc >> 1) ^ (uint32_t(fold(g, hist_len(i), 8)) << 1) ^ (pc >> 7)) & 0xFF);
    }

    void reset() {
        for (int i = 0; i < BASE_SZ; i++) base[i] = 1;   // debil no-tomado
        for (int k = 0; k < NTBL; k++)
            for (int i = 0; i < TBL_SZ; i++) t[k][i] = {0, 0, 0};
        ghr = 0;
    }

    // la tabla con match de historia MAS LARGA manda; si no, el bimodal
    bool predict(uint32_t pc) const {
        bool pred = (base[(pc >> 1) & (BASE_SZ - 1)] >= 2);
        for (int k = 0; k < NTBL; k++) {
            const Entry& e = t[k][index(pc, ghr, k)];
            if (e.tag == tag_of(pc, ghr, k)) pred = (e.ctr >= 0);
        }
        return pred;
    }

    void update(uint32_t pc, uint16_t ghr_snap, bool taken, bool mispred) {
        int bi = (pc >> 1) & (BASE_SZ - 1);
        if (taken)  { if (base[bi] < 3) base[bi]++; }
        else        { if (base[bi] > 0) base[bi]--; }

        int provider = -1;
        for (int k = 0; k < NTBL; k++)
            if (t[k][index(pc, ghr_snap, k)].tag == tag_of(pc, ghr_snap, k))
                provider = k;
        if (provider >= 0) {
            Entry& e = t[provider][index(pc, ghr_snap, provider)];
            if (taken)  { if (e.ctr <  3) e.ctr++; }
            else        { if (e.ctr > -4) e.ctr--; }
            if (!mispred) { if (e.u < 3) e.u++; }
            else          { if (e.u > 0) e.u--; }
        }
        if (mispred) {
            // asignar en la primera tabla mas larga con victima u==0
            for (int k = 0; k < NTBL; k++) {
                if (k > provider) {
                    Entry& e = t[k][index(pc, ghr_snap, k)];
                    if (e.u == 0 && e.tag != tag_of(pc, ghr_snap, k)) {
                        e.tag = tag_of(pc, ghr_snap, k);
                        e.ctr = taken ? 0 : -1;   // debil en la direccion vista
                        break;
                    }
                }
            }
        }
    }
};

#endif // TAGE_H

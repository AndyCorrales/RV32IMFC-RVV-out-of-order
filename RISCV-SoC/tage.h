#ifndef TAGE_H
#define TAGE_H

// =====================================================================
// TAGE: el predictor de saltos del FRONTEND (bloque "TAGE" del diagrama).
//
// TAGE = TAgged GEometric history length predictor (Seznec/Michaud,
// 2006). La idea: varios predictores en paralelo, cada uno indexado con
// una longitud DISTINTA de historia global (longitudes en progresion
// geometrica), mas un predictor base sin historia. Gana la tabla con
// match de tag que use la historia mas larga: los saltos que se
// correlacionan lejos usan historia larga, los triviales no gastan mas
// que el bimodal.
//
// Esta version es un TAGE reducido y sintetizable:
//   BASE  : bimodal, 512 contadores de 2 bits, indexado solo por pc
//   T1/2/3: 128 entradas {tag 8b, ctr 3b con signo, u 2b},
//           historias de 4 / 8 / 16 bits (geometrica x2)
//   GHR   : 16 bits de historia global (1 bit por branch condicional)
//
// Integracion con el pipeline (ver frontend_dispatch.h y soc_top.cpp):
//   - PREDICT en el dispatch del branch: decide el camino del fetch.
//   - La entrada del ROB guarda pred_taken y un SNAPSHOT del GHR.
//   - UPDATE en el COMMIT del branch, con el resultado real. Como el
//     commit es en orden, el predictor entrena con la verdad y en orden
//     de programa -- sin updates especulativos que luego haya que
//     deshacer.
//   - En un MISPREDICT el commit restaura GHR = snapshot|real y hace
//     pipeline_flush(): el mismo camino barato de los traps.
// =====================================================================
#include "soc_top.h"

// ---- parametros ----
static const int TAGE_BASE_BITS = 9;                 // 512 entradas bimodal
static const int TAGE_BASE_SZ   = 1 << TAGE_BASE_BITS;
static const int TAGE_TBL_BITS  = 7;                 // 128 entradas por tabla
static const int TAGE_TBL_SZ    = 1 << TAGE_TBL_BITS;
static const int TAGE_NTBL      = 3;
static const int TAGE_HIST[TAGE_NTBL] = {4, 8, 16};  // longitudes geometricas

struct TageEntry {
    ap_uint<8> tag;
    ap_int<3>  ctr;   // con signo: >=0 predice tomado
    ap_uint<2> u;     // "useful": protege entradas que han acertado
};

static ap_uint<2>  tage_base[TAGE_BASE_SZ]; // bimodal: 2b saturante
static TageEntry   tage_t[TAGE_NTBL][TAGE_TBL_SZ];
static ap_uint<16> tage_ghr;                // historia global (especulativa)

// Pliega los `len` bits bajos de la historia sobre `out` bits (XOR de
// trozos). Es el "folding" clasico de TAGE, en version directa.
static ap_uint<16> tage_fold(ap_uint<16> h, int len, int out) {
    ap_uint<16> masked = h & ap_uint<16>((1u << len) - 1);
    ap_uint<16> r = 0;
    for (int i = 0; i < 16; i++) {
#pragma HLS UNROLL
        if (i < len) r ^= ap_uint<16>(((masked >> i) & 1) << (i % out));
    }
    return r;
}

static int tage_index(ap_uint<32> pc, ap_uint<16> ghr, int t) {
    ap_uint<16> f = tage_fold(ghr, TAGE_HIST[t], TAGE_TBL_BITS);
    return ((pc >> 1) ^ ap_uint<32>(f) ^ ap_uint<32>(pc >> (3 + t))).to_uint()
           & (TAGE_TBL_SZ - 1);
}
static ap_uint<8> tage_tag(ap_uint<32> pc, ap_uint<16> ghr, int t) {
    ap_uint<16> f = tage_fold(ghr, TAGE_HIST[t], 8);
    return ap_uint<8>(((pc >> 1) ^ ap_uint<32>(f << 1) ^ (pc >> 7)).to_uint() & 0xFF);
}

static void tage_reset() {
    for (int i = 0; i < TAGE_BASE_SZ; i++) tage_base[i] = 1; // debil no-tomado
    for (int t = 0; t < TAGE_NTBL; t++)
        for (int i = 0; i < TAGE_TBL_SZ; i++) {
            tage_t[t][i].tag = 0; tage_t[t][i].ctr = 0; tage_t[t][i].u = 0;
        }
    tage_ghr = 0;
}

// PREDICT: la tabla con match de tag de historia MAS LARGA manda; si
// ninguna matchea, el bimodal. (Sin "alternate prediction" ni tracking
// de confianza: la version minima que conserva la esencia del algoritmo.)
static bool tage_predict(ap_uint<32> pc) {
    bool pred = (tage_base[(pc >> 1).to_uint() & (TAGE_BASE_SZ - 1)] >= 2);
    for (int t = 0; t < TAGE_NTBL; t++) {
#pragma HLS UNROLL
        const TageEntry& e = tage_t[t][tage_index(pc, tage_ghr, t)];
        if (e.tag == tage_tag(pc, tage_ghr, t)) pred = (e.ctr >= 0);
    }
    return pred;
}

// UPDATE (en el commit, con el resultado real y el GHR de ese momento):
//  - el proveedor (match de historia mas larga) mueve su contador;
//  - el bimodal entrena siempre (es el fallback);
//  - en mispredict se ASIGNA una entrada en una tabla de historia mas
//    larga que el proveedor (u==0), apostando a que ese salto necesita
//    mas contexto. Eso es lo que hace "aprender patrones" a TAGE.
static void tage_update(ap_uint<32> pc, ap_uint<16> ghr_snap,
                        bool taken, bool mispred) {
    int base_i = (pc >> 1).to_uint() & (TAGE_BASE_SZ - 1);
    if (taken)  { if (tage_base[base_i] < 3) tage_base[base_i] = tage_base[base_i] + 1; }
    else        { if (tage_base[base_i] > 0) tage_base[base_i] = tage_base[base_i] - 1; }

    int provider = -1;
    for (int t = 0; t < TAGE_NTBL; t++) {
#pragma HLS UNROLL
        if (tage_t[t][tage_index(pc, ghr_snap, t)].tag == tage_tag(pc, ghr_snap, t))
            provider = t;
    }
    if (provider >= 0) {
        TageEntry& e = tage_t[provider][tage_index(pc, ghr_snap, provider)];
        if (taken)  { if (e.ctr <  3) e.ctr = e.ctr + 1; }
        else        { if (e.ctr > -4) e.ctr = e.ctr - 1; }
        if (!mispred) { if (e.u < 3) e.u = e.u + 1; }
        else          { if (e.u > 0) e.u = e.u - 1; }
    }
    if (mispred) {
        // asignar en la primera tabla mas larga con una victima u==0
        for (int t = 0; t < TAGE_NTBL; t++) {
#pragma HLS UNROLL
            if (t > provider) {
                TageEntry& e = tage_t[t][tage_index(pc, ghr_snap, t)];
                if (e.u == 0 && e.tag != tage_tag(pc, ghr_snap, t)) {
                    e.tag = tage_tag(pc, ghr_snap, t);
                    e.ctr = taken ? ap_int<3>(0) : ap_int<3>(-1); // debil
                    break;
                }
            }
        }
    }
}

#endif // TAGE_H

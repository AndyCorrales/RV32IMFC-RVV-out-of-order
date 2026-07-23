#ifndef FP_OPS_H
#define FP_OPS_H

#include <cmath>
#include <cstdint>
#include <cstring>

// Semantica de la extension F que no es puro bitwise de decodificacion
// (eso vive en rv32i_defs.h): reinterpretacion bits<->float, conversion
// saturada entera<->flotante, y clasificacion IEEE-754. Igual que
// immediates.h separa "como se extraen los campos" de "que significan",
// este archivo separa "como se codifica la instruccion" de "que hace
// numericamente".
namespace rv32i {

// Reinterpreta los mismos 32 bits como float <-> uint32_t, sin conversion
// numerica (esto es lo que hacen FMV.X.W / FMV.W.X, y lo que hace falta
// para mover el contenido de un registro flotante por el Bus como bytes
// crudos en FLW/FSW). memcpy en vez de un cast directo o una union: es la
// forma portable y bien definida en C++ de hacer "type punning" sin caer
// en undefined behavior por aliasing estricto.
inline uint32_t float_to_bits(float f) {
    uint32_t u;
    std::memcpy(&u, &f, sizeof(u));
    return u;
}
inline float bits_to_float(uint32_t u) {
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

// FCVT.W.S: float -> int32_t con signo, saturando en overflow y NaN,
// tal como exige el manual RISC-V (no es un simple cast: un cast de C++
// en overflow es undefined behavior, y con NaN da un resultado no
// especificado).
inline int32_t fcvt_w_s(float f) {
    if (std::isnan(f)) return 0x7FFFFFFF;
    if (f >= 2147483648.0f) return 0x7FFFFFFF;              // overflow positivo
    if (f <= -2147483648.0f) return static_cast<int32_t>(0x80000000); // overflow negativo
    return static_cast<int32_t>(std::nearbyint(f));
}

// FCVT.WU.S: float -> uint32_t sin signo, misma logica de saturacion.
inline uint32_t fcvt_wu_s(float f) {
    if (std::isnan(f)) return 0xFFFFFFFF;
    if (f < 0.0f) return 0;
    if (f >= 4294967296.0f) return 0xFFFFFFFF;
    return static_cast<uint32_t>(std::nearbyint(f));
}

// FCLASS.S: clasifica f en una de 10 categorias IEEE-754, devuelve una
// mascara de un solo bit (bit0=-inf ... bit9=NaN silencioso), tal como
// especifica el ISA.
inline uint32_t fclass_s(float f) {
    uint32_t bits = float_to_bits(f);
    bool sign = (bits >> 31) & 0x1;

    switch (std::fpclassify(f)) {
        case FP_INFINITE:  return sign ? (1u << 0) : (1u << 7);
        case FP_NAN: {
            bool quiet = (bits >> 22) & 0x1; // bit mas alto de la mantisa
            return quiet ? (1u << 9) : (1u << 8);
        }
        case FP_ZERO:      return sign ? (1u << 3) : (1u << 4);
        case FP_SUBNORMAL: return sign ? (1u << 2) : (1u << 5);
        case FP_NORMAL:
        default:           return sign ? (1u << 1) : (1u << 6);
    }
}

} // namespace rv32i

#endif // FP_OPS_H

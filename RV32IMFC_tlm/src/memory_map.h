#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

#include <cstdint>

// Mapa de direcciones GLOBAL del sistema.
// El Bus decodifica contra estos rangos para rutear cada transaccion TLM.
namespace memory_map {

// ── RAM principal ────────────────────────────────────────────────────────────
// Subida de 64 KB → 512 KB para poder cargar CoreMark compilado con newlib.
// CoreMark ocupa ~50-80 KB de codigo mas stack y heap; 64 KB no alcanzaba.
constexpr uint32_t RAM_BASE = 0x00000000;
constexpr uint32_t RAM_SIZE = 512 * 1024; // 512 KB

// Tope del stack: 16 bytes por debajo del limite de RAM para alineacion.
// El CRT0 (crt0.S) carga esta direccion en sp (x2) antes de llamar a main.
// El ELF loader en main.cpp tambien inicializa cpu.regs[2] con este valor.
constexpr uint32_t STACK_TOP = RAM_BASE + RAM_SIZE - 16;

// ── Rango reservado RVV ──────────────────────────────────────────────────────
// La VectorUnit actua como INITIATOR hacia la RAM (loads/stores vle/vse).
// Este rango esta reservado para cuando la VectorUnit exponga registros de
// control (vcsr, vl, vtype) como TARGET en el Bus.
constexpr uint32_t RVV_BASE = 0x10000000;
constexpr uint32_t RVV_SIZE = 64 * 1024;

} // namespace memory_map

#endif // MEMORY_MAP_H

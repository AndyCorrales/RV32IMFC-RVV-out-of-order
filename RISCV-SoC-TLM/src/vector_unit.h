#ifndef VECTOR_UNIT_H
#define VECTOR_UNIT_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>

// Unidad vectorial RVV (esqueleto). Por ahora es un INITIATOR TLM-2.0
// independiente que comparte el Bus con el Processor: cada uno tiene su
// propio target_socket de entrada en el Bus (ver bus.h), y ambos rutean
// hacia la misma Memory.
//
// ETAPA ACTUAL: solo se valida el camino VectorUnit -> Bus -> Memory con
// un metodo de prueba (vector_load_test) que simula, a mano, lo que en el
// futuro hara una instruccion vle32.v: un load de VLEN bits desde una
// direccion GLOBAL dada, hacia un registro vectorial del banco.
//
// PENDIENTE (siguiente etapa RVV): reemplazar vector_load_test() por un
// decoder real de instrucciones vectoriales (vadd.vv, vle32.v, vse32.v,
// etc.), y agregar aqui la logica de vtype/vl para variar el ancho de
// elemento (SEW) y el numero de elementos activos por instruccion.
SC_MODULE(VectorUnit) {
    // VLEN: ancho en bits de cada registro vectorial. 128 bits es un punto
    // de partida razonable para RV32 embebido/didactico (multiplo de 32 y
    // 64 bits, suficiente para probar el ruteo sin manejar vtype/LMUL
    // todavia). Se puede subir a 256/512 mas adelante sin tocar la
    // interfaz del Bus, ya que el ancho de la transaccion TLM es
    // independiente de VLEN.
    static constexpr unsigned int VLEN_BITS  = 128;
    static constexpr unsigned int VLEN_BYTES = VLEN_BITS / 8;
    static constexpr unsigned int NUM_VREGS  = 32;

    // Banco de registros vectoriales: 32 registros de VLEN_BYTES bytes
    // cada uno. Se elige un array de bytes (en vez de, por ejemplo,
    // std::array<uint32_t,4>) para no asumir todavia un SEW (element
    // width) fijo: el decoder vectorial futuro reinterpretara estos bytes
    // como elementos de 8/16/32 bits segun vtype.
    using VReg = std::array<uint8_t, VLEN_BYTES>;
    std::array<VReg, NUM_VREGS> vregs{};

    tlm_utils::simple_initiator_socket<VectorUnit, 32> init_socket;

    SC_CTOR(VectorUnit) : init_socket("init_socket") {}

    // Lectura/escritura generica de 'len' bytes en direccion GLOBAL via Bus.
    void bus_access(tlm::tlm_command cmd, uint32_t addr, uint8_t* data, unsigned int len) {
        tlm::tlm_generic_payload trans;
        sc_time delay = SC_ZERO_TIME;

        trans.set_command(cmd);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(len);
        trans.set_streaming_width(len);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

        init_socket->b_transport(trans, delay);
        wait(delay);

        if (trans.get_response_status() != tlm::TLM_OK_RESPONSE) {
            std::cerr << "[VectorUnit] Error de bus en direccion 0x" << std::hex << addr << std::dec << std::endl;
        }
    }

    // Prueba de extremo a extremo: carga VLEN_BYTES bytes desde 'addr'
    // (direccion GLOBAL) hacia el registro vectorial vreg_idx. Equivale a
    // lo que hara vle32.v cuando exista el decoder real, pero sin decodificar
    // ninguna instruccion: se invoca directamente desde main.cpp / SC_THREAD
    // de prueba.
    void vector_load_test(unsigned int vreg_idx, uint32_t addr) {
        bus_access(tlm::TLM_READ_COMMAND, addr, vregs[vreg_idx].data(), VLEN_BYTES);
    }

    // Imprime el contenido de un registro vectorial en hexadecimal, byte a
    // byte (orden de memoria, es decir Little Endian tal como se almacena).
    void dump_vreg(unsigned int vreg_idx) const {
        std::cout << std::hex;
        for (unsigned int i = 0; i < VLEN_BYTES; ++i) {
            std::cout << static_cast<int>(vregs[vreg_idx][i]);
            if (i + 1 < VLEN_BYTES) std::cout << " ";
        }
        std::cout << std::dec << std::endl;
    }
};

#endif // VECTOR_UNIT_H

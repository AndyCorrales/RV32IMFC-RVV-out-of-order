#ifndef BUS_H
#define BUS_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>

#include "memory_map.h"

// Bus TLM-2.0. Actua como TARGET hacia cada initiator del sistema (CPU,
// VectorUnit, ...) y como INITIATOR hacia cada periferico (Memory, ...).
// decode() rutea por rango de direccion GLOBAL, sin importar quien inicio
// la transaccion: por eso agregar un segundo initiator (cpu_target ->
// vector_target) no requiere tocar la logica de decode().
SC_MODULE(Bus) {
    // Targets: uno por cada initiator que puede generar trafico en el bus.
    tlm_utils::simple_target_socket<Bus, 32> cpu_target;
    tlm_utils::simple_target_socket<Bus, 32> vector_target;

    // Initiators: uno por cada periferico direccionable del mapa de memoria.
    tlm_utils::simple_initiator_socket<Bus, 32> mem_initiator;
    tlm_utils::simple_initiator_socket<Bus, 32> uart_initiator;

    // PENDIENTE (siguiente etapa RVV): cuando la VectorUnit exponga
    // registros de estado/control mapeados en memoria (vcsr, vl, vtype),
    // agregar aqui un segundo initiator hacia ese target:
    //   tlm_utils::simple_initiator_socket<Bus, 32> rvv_init;
    // y extender decode() para rutear el rango RVV_BASE..RVV_BASE+RVV_SIZE
    // hacia rvv_init en lugar de mem_initiator.

    SC_CTOR(Bus) : cpu_target("cpu_target"),
                   vector_target("vector_target"),
                   mem_initiator("mem_initiator"),
                   uart_initiator("uart_initiator") {
        cpu_target.register_b_transport(this, &Bus::b_transport);
        vector_target.register_b_transport(this, &Bus::b_transport);
    }

    // Punto de entrada comun para cualquier initiator conectado al Bus.
    void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
        decode(trans, delay);
    }

    // Decodifica la direccion GLOBAL de la transaccion y la reenvia al
    // periferico correspondiente con la direccion LOCAL a ese periferico.
    void decode(tlm::tlm_generic_payload& trans, sc_time& delay) {
        sc_dt::uint64 global_addr = trans.get_address();

        if (global_addr >= memory_map::RAM_BASE &&
            global_addr < memory_map::RAM_BASE + memory_map::RAM_SIZE) {
            trans.set_address(global_addr - memory_map::RAM_BASE);
            mem_initiator->b_transport(trans, delay);
            return;
        }

        if (global_addr >= memory_map::UART_BASE &&
            global_addr < memory_map::UART_BASE + memory_map::UART_SIZE) {
            trans.set_address(global_addr - memory_map::UART_BASE);
            uart_initiator->b_transport(trans, delay);
            return;
        }

        // PENDIENTE: rango memory_map::RVV_BASE .. +RVV_SIZE -> rvv_init.

        trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    }
};

#endif // BUS_H

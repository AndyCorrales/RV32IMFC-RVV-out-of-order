#ifndef UART_H
#define UART_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_target_socket.h>
#include <iostream>

#include "memory_map.h"

// UART mapeado en memoria: TARGET TLM-2.0 colgado del Bus, igual que la
// Memory. Una escritura a su rango imprime el byte bajo por la consola;
// una lectura devuelve un registro de estado con "listo para transmitir".
//
// A diferencia de la pista HLS —donde el UART es un caso especial dentro
// del camino de store del core— acá es un periférico de verdad: el Bus
// decodifica la dirección y rutea la transacción, sin que el procesador
// sepa que del otro lado hay algo distinto de una memoria. Esa es
// justamente la ventaja de haber modelado un Bus con mapa de direcciones.
SC_MODULE(Uart) {
    tlm_utils::simple_target_socket<Uart, 32> socket;

    SC_CTOR(Uart) : socket("socket") {
        socket.register_b_transport(this, &Uart::b_transport);
    }

    void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
        tlm::tlm_command cmd = trans.get_command();
        unsigned char*   ptr = trans.get_data_ptr();
        unsigned int     len = trans.get_data_length();

        if (cmd == tlm::TLM_WRITE_COMMAND) {
            // El byte bajo del dato es el caracter a transmitir.
            std::cout << static_cast<char>(ptr[0]) << std::flush;
        } else if (cmd == tlm::TLM_READ_COMMAND) {
            // Registro de estado: siempre listo (no hay FIFO que se llene).
            for (unsigned int i = 0; i < len; i++) ptr[i] = 0;
            if (len > 0) ptr[0] = 0x01; // bit 0 = TX ready
        } else {
            trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
            return;
        }

        trans.set_response_status(tlm::TLM_OK_RESPONSE);
        delay += sc_time(5, SC_NS); // latencia nominal del periférico
    }
};

#endif // UART_H

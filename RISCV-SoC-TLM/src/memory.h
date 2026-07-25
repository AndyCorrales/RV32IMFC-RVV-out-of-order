#ifndef MEMORY_H
#define MEMORY_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_target_socket.h>
#include <cstring>
#include <vector>

#include "memory_map.h"

// Memoria TLM-2.0 de 64KB. Es TARGET puro: recibe direcciones LOCALES
// (el Bus ya restó la base del rango antes de reenviar la transaccion).
// Soporta accesos de 1/2/4 bytes en Little Endian, incluyendo desalineados.
SC_MODULE(Memory) {
    tlm_utils::simple_target_socket<Memory, 32> socket;

    SC_CTOR(Memory) : socket("socket"), data(memory_map::RAM_SIZE, 0) {
        socket.register_b_transport(this, &Memory::b_transport);
    }

    void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
        tlm::tlm_command cmd = trans.get_command();
        sc_dt::uint64     addr = trans.get_address();
        unsigned char*    ptr  = trans.get_data_ptr();
        unsigned int       len  = trans.get_data_length();

        if (addr + len > data.size()) {
            trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }

        if (cmd == tlm::TLM_READ_COMMAND) {
            std::memcpy(ptr, &data[addr], len);
        } else if (cmd == tlm::TLM_WRITE_COMMAND) {
            std::memcpy(&data[addr], ptr, len);
        } else {
            trans.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
            return;
        }

        trans.set_response_status(tlm::TLM_OK_RESPONSE);
        delay += sc_time(10, SC_NS);
    }

    std::vector<uint8_t> data;
};

#endif // MEMORY_H

#include <cstdio>
#include <cstdint>
#include <cstring>
#include "rv32_ooo.h"
#include "bm_elf.h" // binario ELF32 real (RV32IMFC) compilado con riscv gcc

// Testbench de BARE-METAL para el core OOO. A diferencia de rv32_ooo_tb.cpp
// (programa ensamblado a mano), aca:
//   1. Se parsea un ELF32 REAL producido por riscv64-unknown-elf-gcc
//      (-march=rv32imfc) con un loader de secciones PT_LOAD -> imem/dmem.
//   2. Se corre el core hasta que el programa se detiene por su propio
//      ECALL (el `_start`/crt0 llama a main y ejecuta ecall al volver).
//   3. Se verifican los resultados que el binario dejo en memoria y en
//      los CSRs, SIN backdoor: todo salio de ejecutar las instrucciones
//      reales del binario (loop, sw, csrw/csrr, ecall).
//
// El binario (ver bm.c / crt0.s, documentado en el README) hace:
//   out[0]=1+2+3+4+5 = 15   (0x80)
//   out[1]=1*2*3*4*5 = 120  (0x84)
//   csrw mscratch, 0xABCD; csrr -> out[2] = 0xABCD  (0x88)
// Esto ejercita items 1-5 del plan bare-metal: loader de ELF (1), opcode
// SYSTEM (2), ECALL (3), instrucciones CSR (4), banco de CSRs (5).

// ---- Loader de ELF32 (little-endian) --------------------------------
// Parser minimo: valida el magic, recorre los program headers y copia
// cada segmento PT_LOAD a imem (si es ejecutable) y/o dmem, respetando
// p_vaddr. Suficiente para binarios bare-metal estaticos como este.
struct Elf32_Ehdr {
    uint8_t  e_ident[16];
    uint16_t e_type, e_machine;
    uint32_t e_version, e_entry, e_phoff, e_shoff, e_flags;
    uint16_t e_ehsize, e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx;
};
struct Elf32_Phdr {
    uint32_t p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align;
};
static const uint32_t PT_LOAD = 1;
static const uint32_t PF_X    = 1;

static bool load_elf(const unsigned char* img, unsigned len,
                     ap_uint<32> imem[OOO_IMEM_WORDS],
                     ap_uint<32> dmem[OOO_DMEM_WORDS],
                     uint32_t& entry) {
    if (len < sizeof(Elf32_Ehdr)) return false;
    Elf32_Ehdr eh;
    std::memcpy(&eh, img, sizeof(eh));
    if (!(eh.e_ident[0] == 0x7f && eh.e_ident[1] == 'E' &&
          eh.e_ident[2] == 'L'  && eh.e_ident[3] == 'F')) {
        printf("FAIL  no es un ELF valido (magic incorrecto)\n");
        return false;
    }
    if (eh.e_ident[4] != 1) { printf("FAIL  no es ELF32\n"); return false; }
    entry = eh.e_entry;

    for (int i = 0; i < eh.e_phnum; i++) {
        Elf32_Phdr ph;
        std::memcpy(&ph, img + eh.e_phoff + i * eh.e_phentsize, sizeof(ph));
        if (ph.p_type != PT_LOAD) continue;
        bool is_exec = (ph.p_flags & PF_X) != 0;
        printf("      PT_LOAD vaddr=0x%x filesz=%u flags=%s%s -> %s\n",
               ph.p_vaddr, ph.p_filesz, (ph.p_flags & 4) ? "R" : "",
               is_exec ? "X" : "", is_exec ? "imem" : "dmem");
        // copia byte a byte a la memoria correspondiente (word-addressed)
        for (uint32_t b = 0; b < ph.p_filesz; b++) {
            uint32_t vaddr = ph.p_vaddr + b;
            uint8_t byte = img[ph.p_offset + b];
            uint32_t widx = (vaddr >> 2);
            uint32_t shift = (vaddr & 3) * 8;
            if (is_exec) {
                if (widx < OOO_IMEM_WORDS) {
                    uint32_t w = imem[widx].to_uint();
                    w = (w & ~(0xFFu << shift)) | (uint32_t(byte) << shift);
                    imem[widx] = w;
                }
            } else {
                if (widx < OOO_DMEM_WORDS) {
                    uint32_t w = dmem[widx].to_uint();
                    w = (w & ~(0xFFu << shift)) | (uint32_t(byte) << shift);
                    dmem[widx] = w;
                }
            }
        }
    }
    return true;
}

int main() {
    ap_uint<32> imem[OOO_IMEM_WORDS] = {0};
    ap_uint<32> dmem[OOO_DMEM_WORDS] = {0};

    printf("Cargando binario bare-metal ELF32 real (%u bytes)...\n", bm_elf_len);
    uint32_t entry = 0;
    if (!load_elf(bm_elf, bm_elf_len, imem, dmem, entry)) {
        printf("Al menos un check fallo.\n");
        return 1;
    }
    printf("      entry point = 0x%x\n\n", entry);

    // puertos del core
    ap_uint<1> disp_valid; ap_uint<3> disp_tag; ap_uint<32> disp_pc;
    ap_uint<1> alu0_done, alu1_done, md_done, fpu_done, lsu_done, br_done, vec_done;
    ap_uint<3> alu0_tag, alu1_tag, md_tag, fpu_tag, lsu_tag, br_tag, vec_tag;
    ap_uint<1> commit_valid, commit_is_fp; ap_uint<5> commit_rd; ap_uint<32> commit_value;
    ap_uint<32> vregs_out[OOO_VEC_REGFILE_LEN];
    ap_uint<1> halted;

    rv32_ooo_tick(1, imem, dmem, disp_valid, disp_tag, disp_pc,
                  alu0_done, alu0_tag, alu1_done, alu1_tag, md_done, md_tag,
                  fpu_done, fpu_tag, lsu_done, lsu_tag, br_done, br_tag,
                  vec_done, vec_tag, commit_valid, commit_is_fp, commit_rd,
                  commit_value, vregs_out, halted);

    int cycle = 0, n_commit = 0;
    const int MAX_CYCLES = 500;
    while (!halted && cycle < MAX_CYCLES) {
        cycle++;
        rv32_ooo_tick(0, imem, dmem, disp_valid, disp_tag, disp_pc,
                      alu0_done, alu0_tag, alu1_done, alu1_tag, md_done, md_tag,
                      fpu_done, fpu_tag, lsu_done, lsu_tag, br_done, br_tag,
                      vec_done, vec_tag, commit_valid, commit_is_fp, commit_rd,
                      commit_value, vregs_out, halted);
        if (commit_valid) n_commit++;
    }

    printf("El binario se detuvo por su propio ECALL en el ciclo %d (%d commits).\n\n",
           cycle, n_commit);

    bool ok = true;
    if (!halted) { printf("FAIL  no se detuvo (no llego al ECALL en %d ciclos)\n", MAX_CYCLES); ok = false; }

    // resultados dejados en memoria por el binario (dmem word-addressed)
    struct { uint32_t byte_addr; uint32_t expect; const char* what; } checks[] = {
        {0x80, 15,     "out[0] = 1+2+3+4+5 (loop)"},
        {0x84, 120,    "out[1] = 1*2*3*4*5 (loop)"},
        {0x88, 0xABCD, "out[2] = csrr mscratch tras csrw 0xABCD (banco de CSRs)"},
    };
    for (auto& c : checks) {
        uint32_t got = dmem[(c.byte_addr >> 2) & (OOO_DMEM_WORDS - 1)].to_uint();
        if (got != c.expect) {
            printf("FAIL  mem[0x%x] = 0x%x, esperado 0x%x (%s)\n", c.byte_addr, got, c.expect, c.what);
            ok = false;
        } else {
            printf("OK    mem[0x%x] = 0x%x (%s)\n", c.byte_addr, got, c.what);
        }
    }

    if (!ok) { printf("\nAl menos un check fallo.\n"); return 1; }
    printf("\nTodos los checks pasaron. El core ejecuto un binario ELF real "
           "compilado con gcc, incluyendo acceso a CSRs y ECALL.\n");
    return 0;
}

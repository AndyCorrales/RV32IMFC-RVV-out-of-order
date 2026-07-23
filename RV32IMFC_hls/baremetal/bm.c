// Programa bare-metal real: loop (suma+producto), acceso a CSR (mscratch),
// y stores a direcciones fijas. Compilado con riscv gcc -march=rv32imfc.
int main() {
    volatile int* out = (int*)0x80;   // dmem word 32
    int s = 0, p = 1;
    for (int i = 1; i <= 5; i++) { s += i; p *= i; }
    out[0] = s;   // 0x80 -> 15
    out[1] = p;   // 0x84 -> 120

    // ejercita el banco de CSRs desde codigo compilado real
    unsigned int scratch;
    __asm__ volatile ("csrw mscratch, %0" :: "r"(0xABCD));
    __asm__ volatile ("csrr %0, mscratch" : "=r"(scratch));
    out[2] = scratch; // 0x88 -> 0xABCD
    return 0;
}

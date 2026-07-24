/* isa_test.c — Prueba anchos de load/store y variantes de la extension M.
 * Imprime ANTES de cada instruccion: donde se detenga, esa es la que falta.
 * Compilar con -O0. Los volatile fuerzan el acceso real a memoria.
 */
extern int printf(const char *fmt, ...);

static volatile unsigned char  barr[4];
static volatile unsigned short harr[4];
static volatile unsigned int   warr[4];

int main(void) {
    /* ---------- Anchos de memoria ---------- */
    printf("== LOAD/STORE ==\n");

    printf("1: sb  (store byte)\n");        barr[0] = 0xAB;
    printf("2: lbu (load byte unsigned)\n"); { unsigned char b = barr[0];
    printf("   -> lbu OK = 0x%x\n", (unsigned)b); }

    printf("3: sh  (store halfword)\n");    harr[0] = 0x1234;
    printf("4: lhu (load halfword unsigned)\n"); { unsigned short h = harr[0];
    printf("   -> lhu OK = 0x%x\n", (unsigned)h); }

    printf("5: sw  (store word)\n");        warr[0] = 0xDEADBEEF;
    printf("6: lw  (load word)\n"); { unsigned int w = warr[0];
    printf("   -> lw OK = 0x%x\n", w); }

    printf("7: lh  (load halfword SIGNED)\n"); { short s = (short)harr[0];
    printf("   -> lh OK = %d\n", (int)s); }

    printf("8: lb  (load byte SIGNED)\n"); { signed char c = (signed char)barr[0];
    printf("   -> lb OK = %d\n", (int)c); }

    /* ---------- Extension M ---------- */
    printf("== EXTENSION M ==\n");
    volatile int a = -7, b = 3;
    int x = 0;

    printf("9:  mul\n");    __asm__ volatile("mul    %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> mul OK = %d\n", x);
    printf("10: mulh\n");   __asm__ volatile("mulh   %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> mulh OK\n");
    printf("11: mulhu\n");  __asm__ volatile("mulhu  %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> mulhu OK\n");
    printf("12: mulhsu\n"); __asm__ volatile("mulhsu %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> mulhsu OK\n");
    printf("13: div\n");    __asm__ volatile("div    %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> div OK = %d\n", x);
    printf("14: divu\n");   __asm__ volatile("divu   %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> divu OK\n");
    printf("15: rem\n");    __asm__ volatile("rem    %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> rem OK = %d\n", x);
    printf("16: remu\n");   __asm__ volatile("remu   %0,%1,%2":"=r"(x):"r"(a),"r"(b)); printf("   -> remu OK\n");

    printf("TODO OK: todos los anchos y todas las variantes de M funcionan\n");
    return 0;
}

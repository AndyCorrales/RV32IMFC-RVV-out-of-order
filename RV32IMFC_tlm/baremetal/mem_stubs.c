/* mem_stubs.c
 * Funciones freestanding para enlazar con -nostdlib.
 *
 * IMPORTANTE: memset/memcpy/memmove llevan el atributo
 *   optimize("no-tree-loop-distribute-patterns")
 * para que GCC NO reemplace su bucle interno por una llamada a si mismas
 * (recursion infinita). Esto las protege aunque se compile a -O2 sin
 * -fno-builtin. Ver: GCC loop-distribute-patterns sobre memset/memcpy.
 */
#include <stddef.h>

extern int printf(const char *fmt, ...);

/* Macro para el atributo anti-recursion (idioma de GCC). */
#define NO_LOOPDIST __attribute__((optimize("no-tree-loop-distribute-patterns")))

/* ---- puts / putchar (destino de la optimizacion de printf en GCC) ---- */
int puts(const char *s) {
    return printf("%s\n", s ? s : "(null)");
}

int putchar(int c) {
    printf("%c", (char)c);
    return c;
}

/* ---- mem* ---- */
NO_LOOPDIST
void *memset(void *dst, int c, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    while (n--) *d++ = (unsigned char)c;
    return dst;
}

NO_LOOPDIST
void *memcpy(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    while (n--) *d++ = *s++;
    return dst;
}

NO_LOOPDIST
void *memmove(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    if (d < s) {
        while (n--) *d++ = *s++;
    } else {
        d += n; s += n;
        while (n--) *--d = *--s;
    }
    return dst;
}

int memcmp(const void *a, const void *b, size_t n) {
    const unsigned char *pa = (const unsigned char *)a;
    const unsigned char *pb = (const unsigned char *)b;
    while (n--) {
        if (*pa != *pb) return (int)*pa - (int)*pb;
        pa++; pb++;
    }
    return 0;
}

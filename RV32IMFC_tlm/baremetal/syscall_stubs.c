/* syscall_stubs.c
 * Auto-contenido: sin stdio.h, sin sys/stat.h, sin errno.h.
 * Solo stdarg.h y stdint.h (headers del COMPILADOR, siempre disponibles).
 * printf ahora soporta ancho de campo y relleno con cero (%04x, %03llu, %8d...).
 */
#include <stdarg.h>
#include <stdint.h>

/* Heap estatico */
extern char _heap_start[];
extern char _heap_end[];
static char* heap_ptr = 0;
void* _sbrk(int incr) {
    if (!heap_ptr) heap_ptr = _heap_start;
    char* prev = heap_ptr;
    if (heap_ptr + incr > _heap_end) return (void*)-1;
    heap_ptr += incr;
    return (void*)prev;
}

/* Escritura directa via ECALL SYS_WRITE */
static int ecall_write(int fd, const char* buf, int count) {
    register long a0 __asm__("a0") = (long)fd;
    register long a1 __asm__("a1") = (long)buf;
    register long a2 __asm__("a2") = (long)count;
    register long a7 __asm__("a7") = 64L;
    __asm__ volatile ("ecall" : "+r"(a0) : "r"(a1), "r"(a2), "r"(a7) : "memory");
    return (int)a0;
}
int _write(int fd, const void* buf, int count) {
    return ecall_write(fd, (const char*)buf, count);
}

/* Salida via ECALL SYS_EXIT */
__attribute__((noreturn))
void _exit(int code) {
    register long a0 __asm__("a0") = (long)code;
    register long a7 __asm__("a7") = 93L;
    __asm__ volatile ("ecall" : : "r"(a0), "r"(a7) : "memory");
    while (1) {}
}

/* Stubs vacios */
int _close(int fd)          { (void)fd; return -1; }
int _fstat(int fd, void* s) { (void)fd; (void)s; return -1; }
int _isatty(int fd)         { return (fd==1||fd==2)?1:0; }
long _lseek(int f,long o,int w){ (void)f;(void)o;(void)w; return -1; }
int _read(int f,void* b,int n) { (void)f;(void)b;(void)n; return -1; }
int _getpid(void)           { return 1; }
int _kill(int p,int s)      { (void)p;(void)s; return -1; }

/* ---- printf con ancho de campo y relleno ---- */
static void _putc(char c)        { ecall_write(1, &c, 1); }
static void _puts(const char* s) {
    if (!s) s = "(null)";
    const char* p = s; while (*p) p++;
    ecall_write(1, s, (int)(p - s));
}

/* Entero sin signo con ancho 'width' y relleno con '0' (zero=1) o ' '. */
static void _putu_w(unsigned long long v, int base, int up, int width, int zero) {
    char buf[24]; int i = 24;
    const char* d = up ? "0123456789ABCDEF" : "0123456789abcdef";
    if (v == 0) buf[--i] = '0';
    else while (v) { buf[--i] = d[v % base]; v /= base; }
    int len = 24 - i;
    for (int pad = width - len; pad > 0; pad--) _putc(zero ? '0' : ' ');
    while (i < 24) _putc(buf[i++]);
}

static void _puti_w(long long v, int width, int zero) {
    if (v < 0) {
        _putc('-');
        _putu_w((unsigned long long)(-v), 10, 0, width > 0 ? width - 1 : 0, zero);
    } else {
        _putu_w((unsigned long long)v, 10, 0, width, zero);
    }
}

int printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    for (const char* p = fmt; *p; p++) {
        if (*p != '%') { _putc(*p); continue; }
        p++;
        int zero = 0, width = 0, l = 0, ll = 0;
        if (*p == '0') { zero = 1; p++; }                 /* flag de relleno con cero */
        while (*p >= '0' && *p <= '9') { width = width*10 + (*p - '0'); p++; } /* ancho */
        while (*p == 'l') { if (l) ll = 1; l = 1; p++; }  /* modificador l / ll */
        switch (*p) {
            case 'c': _putc((char)va_arg(ap, int)); break;
            case 's': _puts(va_arg(ap, const char*)); break;
            case 'd': case 'i':
                _puti_w(ll ? va_arg(ap, long long)
                           : (l ? va_arg(ap, long) : va_arg(ap, int)),
                        width, zero);
                break;
            case 'u':
                _putu_w(ll ? va_arg(ap, unsigned long long)
                           : (l ? va_arg(ap, unsigned long) : va_arg(ap, unsigned int)),
                        10, 0, width, zero);
                break;
            case 'x': case 'X': {
                int up = (*p == 'X');
                _putu_w(ll ? va_arg(ap, unsigned long long)
                           : (l ? va_arg(ap, unsigned long) : va_arg(ap, unsigned int)),
                        16, up, width, zero);
                break;
            }
            case '%': _putc('%'); break;
            default:  _putc('%'); if (*p) _putc(*p); break;
        }
    }
    va_end(ap);
    return 0;
}

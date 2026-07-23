// Stubs de syscall mínimos para newlib sobre este core bare-metal.
// _write manda los bytes al UART mapeado en memoria; _sbrk da memoria
// para el heap que usa printf internamente.
#include <sys/stat.h>
#include <errno.h>
#undef errno
extern int errno;

#define UART_TX ((volatile unsigned int*)0x20000000)

int _write(int fd, const char* buf, int len) {
    (void)fd;
    for (int i = 0; i < len; i++) *UART_TX = (unsigned char)buf[i];
    return len;
}

extern char _end;                 // fin de .bss, provisto por el linker
static char* brk_ptr = 0;
void* _sbrk(int incr) {
    if (brk_ptr == 0) brk_ptr = &_end;
    char* prev = brk_ptr;
    brk_ptr += incr;
    return (void*)prev;
}

int _close(int f) { (void)f; return -1; }
int _fstat(int f, struct stat* st) { (void)f; st->st_mode = S_IFCHR; return 0; }
int _isatty(int f) { (void)f; return 1; }
int _lseek(int f, int o, int w) { (void)f; (void)o; (void)w; return 0; }
int _read(int f, char* b, int l) { (void)f; (void)b; (void)l; return 0; }
void _exit(int c) { (void)c; __asm__ volatile("ecall"); while (1) {} }
int _kill(int p, int s) { (void)p; (void)s; errno = EINVAL; return -1; }
int _getpid(void) { return 1; }

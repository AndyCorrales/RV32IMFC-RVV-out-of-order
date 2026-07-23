// Demo con printf REAL de la biblioteca C (newlib-nano).
// La salida sale por el UART gracias a los stubs de syscall de abajo.
#include <stdio.h>

int main(void) {
    int suma = 0;
    for (int i = 1; i <= 10; i++) suma += i;
    printf("Hola desde RV32IMFC+RVV+OOO\n");
    printf("suma(1..10) = %d\n", suma);
    printf("hex=%x  char=%c  str=%s\n", 255, 'A', "ok");
    return 0;
}

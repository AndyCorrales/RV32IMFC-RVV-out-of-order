#!/bin/sh
# Compila los TRES binarios bare-metal y regenera sus headers embebidos:
#   trap.elf   -> ../trap_elf.h    (excepciones precisas y reanudables)
#   full.elf   -> ../full_elf.h    (UART + timer + modos M/U)
#   printf.elf -> ../printf_elf.h  (printf de la biblioteca C real)
#
# Uso: sh baremetal/build.sh   (desde RV32IMFC+RVV+OOO-HLS/)
#
# Toolchains (se pueden sobreescribir por variable de entorno):
#   RISCV_GCC       gcc RISC-V bare-metal SIN libc  (default: el del PATH)
#   RISCV_GCC_LIBC  gcc RISC-V CON biblioteca C     (default: autodetectado)
#
# Por que dos: trap.elf y full.elf no usan libc y les alcanza cualquier
# gcc bare-metal. printf.elf SI necesita newlib, y el paquete de Ubuntu
# gcc-riscv64-unknown-elf viene sin biblioteca C -- por eso se busca un
# toolchain que la tenga (el que instala Vivado/Vitis la trae).
set -e
cd "$(dirname "$0")"

# ---- toolchain sin libc -------------------------------------------------
RISCV_GCC="${RISCV_GCC:-riscv64-unknown-elf-gcc}"
if ! command -v "$RISCV_GCC" >/dev/null 2>&1; then
    echo "ERROR: no se encontro '$RISCV_GCC'."
    echo "  Instalalo (Ubuntu: sudo apt install gcc-riscv64-unknown-elf)"
    echo "  o indicalo con:  RISCV_GCC=/ruta/al/gcc sh baremetal/build.sh"
    exit 1
fi
STRIP="$(dirname "$(command -v "$RISCV_GCC")")/$(basename "$RISCV_GCC" -gcc)-strip"

# ---- toolchain CON libc (para printf) -----------------------------------
# Si no se indica por entorno, se buscan las ubicaciones tipicas de un
# toolchain con newlib (incluidas las que instalan Vitis y Vivado).
if [ -z "$RISCV_GCC_LIBC" ]; then
    for c in \
        "$XILINX_VITIS"/gnu/riscv/lin/riscv64-unknown-elf/bin/riscv64-unknown-elf-gcc \
        "$XILINX_VIVADO"/gnu/riscv/lin/riscv64-unknown-elf/bin/riscv64-unknown-elf-gcc \
        "$HOME"/Programas/Vivado/Vivado/*/gnu/riscv/lin/riscv64-unknown-elf/bin/riscv64-unknown-elf-gcc \
        /opt/Xilinx/Vivado/*/gnu/riscv/lin/riscv64-unknown-elf/bin/riscv64-unknown-elf-gcc \
        /opt/riscv/bin/riscv64-unknown-elf-gcc
    do
        [ -x "$c" ] && RISCV_GCC_LIBC="$c" && break
    done
fi

# ---- binarios sin libc --------------------------------------------------
FLAGS="-march=rv32imfc -mabi=ilp32f -nostdlib -nostartfiles -O2"
LDF="-Wl,-z,max-page-size=4 -Wl,--build-id=none"

build_bare() {   # $1 = nombre base, $2 = nombre del simbolo
    "$RISCV_GCC" $FLAGS -c "${1}_crt0.s" -o "${1}_crt0.o"
    "$RISCV_GCC" $FLAGS $LDF -T "${1}_link.ld" "${1}_crt0.o" "${1}.c" -o "${1}.elf"
    "$STRIP" "${1}.elf"
    {
      echo "// Binario bare-metal generado por baremetal/build.sh."
      echo "// Fuente: baremetal/${1}.c + ${1}_crt0.s + ${1}_link.ld."
      xxd -i -n "${2}" "${1}.elf"
    } > "../${2}.h"
    echo "  ${1}.elf -> ../${2}.h"
}

build_bare trap trap_elf
build_bare full full_elf

# ---- binario con libc (printf) ------------------------------------------
if [ -z "$RISCV_GCC_LIBC" ] || [ ! -x "$RISCV_GCC_LIBC" ]; then
    echo
    echo "AVISO: no se encontro un toolchain RISC-V con biblioteca C."
    echo "  printf_elf.h NO se regenero (se conserva el que ya estaba)."
    echo "  Para regenerarlo, indica un gcc con newlib para rv32imfc:"
    echo "    RISCV_GCC_LIBC=/ruta/al/gcc sh baremetal/build.sh"
    echo "  El toolchain que instala Vitis/Vivado sirve; tambien uno"
    echo "  construido con riscv-gnu-toolchain (--with-arch=rv32imfc)."
    exit 0
fi

echo "  (libc: $RISCV_GCC_LIBC)"
XFLAGS="-march=rv32imfc_zicsr -mabi=ilp32f -O2"
"$RISCV_GCC_LIBC" $XFLAGS -c printf_crt0.s -o printf_crt0.o
"$RISCV_GCC_LIBC" $XFLAGS -nostartfiles -T printf_link.ld \
    printf_crt0.o printf_demo.c syscalls.c -o printf.elf
{
  echo "// Demo con printf REAL de la biblioteca C (newlib), salida por UART."
  echo "// Fuente: baremetal/printf_demo.c + syscalls.c + printf_crt0.s."
  xxd -i -n printf_elf printf.elf
} > ../printf_elf.h
echo "  printf.elf -> ../printf_elf.h"
echo "=== headers regenerados ==="

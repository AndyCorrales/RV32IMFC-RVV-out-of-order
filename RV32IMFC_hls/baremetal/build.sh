#!/bin/sh
# Reproduce el binario bare-metal ELF32 y regenera bm_elf.h (embebido en
# el testbench). Requiere el toolchain riscv64-unknown-elf-gcc.
#
# Uso: sh baremetal/build.sh   (desde RV32IMFC_hls/)
set -e

CC=riscv64-unknown-elf-gcc
FLAGS="-march=rv32imfc -mabi=ilp32f -nostdlib -nostartfiles -O2"

cd "$(dirname "$0")"

$CC $FLAGS -c crt0.s -o crt0.o
$CC $FLAGS -Wl,-z,max-page-size=4 -Wl,--build-id=none -T link.ld crt0.o bm.c -o bm.elf
riscv64-unknown-elf-strip bm.elf

echo "=== tamano ==="
riscv64-unknown-elf-size bm.elf
echo "=== disassembly ==="
riscv64-unknown-elf-objdump -d bm.elf

# regenera el header embebido que usa rv32_ooo_baremetal_tb.cpp
{
  echo "// Binario bare-metal ELF32 real (RV32IMFC) compilado con riscv64-unknown-elf-gcc."
  echo "// Generado por baremetal/build.sh (xxd -i). Fuente: baremetal/bm.c + crt0.s + link.ld."
  xxd -i -n bm_elf bm.elf
} > ../bm_elf.h

echo "=== bm_elf.h regenerado en RV32IMFC_hls/ ==="

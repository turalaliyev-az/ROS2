#!/bin/bash
# usage: batch.sh <harness> <ff> <k0 values...>; static = 1.6*k0, rolling at speed = 0.8*k0
h=$1; ff=$2; shift 2
for k0 in "$@"; do
  st=$(python3 -c "print($k0*1.6)"); fast=$(python3 -c "print($k0*0.8)")
  ./$h 0.15 1 $k0 $st $ff $fast | awk -v k0=$k0 '{t=$1+0; v=$5+0; if (t>=1.0) {n++; s+=v; if (v<0.01) z++; if (v>m) m=v}} END {printf "  K0 %3d: orta %.3f m/s, maks %.3f, dayanıqlı %2.0f%%\n", k0, s/n, m, 100*z/n}'
done

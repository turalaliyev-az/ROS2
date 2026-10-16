#!/bin/bash
# usage: batch2.sh <harness> <target m/s> <ff> <k0...>: peak, mean after 1.5 s, stalls
h=$1; tgt=$2; ff=$3; shift 3
for k0 in "$@"; do
  st=$(python3 -c "print($k0*1.6)"); fast=$(python3 -c "print($k0*0.8)")
  ./$h $tgt 1 $k0 $st $ff $fast | awk -v k0=$k0 '{t=$1+0; v=$5+0; if (v>m) m=v; if (t>=1.5) {n++; s+=v; if (v<0.01) z++}} END {printf "  K0 %3d: pik %.2f m/s, orta(1.5s+) %.3f, dayanıqlı %2.0f%%\n", k0, m, s/n, 100*z/n}'
done

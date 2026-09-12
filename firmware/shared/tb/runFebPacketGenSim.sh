#!/usr/bin/env bash
#-----------------------------------------------------------------------------
# This file is part of the 'Simple-10GbE-RUDP-KCU105-Example'. It is subject to
# the license terms in the LICENSE.txt file found in the top-level directory
# of this distribution and at:
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
# No part of the 'Simple-10GbE-RUDP-KCU105-Example', including this file, may be
# copied, modified, propagated, or distributed except according to the terms
# contained in the LICENSE.txt file.
#-----------------------------------------------------------------------------
# Simulate FebPacketGen with Vivado xsim, then check every packet against the
# FEB ICD model. Needs Vivado's settings64.sh sourced.
set -e

TB=$(cd "$(dirname "$0")" && pwd)
FW=$(cd "$TB/../.." && pwd)
SURF=$FW/submodules/surf
OUT=$FW/build/FebPacketGenSim

mkdir -p "$OUT"
cd "$OUT"

xvhdl -work surf \
   "$SURF/base/general/rtl/StdRtlPkg.vhd" \
   "$SURF/base/general/rtl/TextUtilPkg.vhd" \
   "$SURF/axi/axi4/rtl/AxiPkg.vhd" \
   "$SURF/axi/axi-stream/rtl/AxiStreamPkg.vhd" \
   "$SURF/axi/axi-lite/rtl/AxiLitePkg.vhd" \
   "$SURF/protocols/ssi/rtl/SsiPkg.vhd"
xvhdl "$FW/shared/rtl/FebPacketGen.vhd" "$TB/FebPacketGenTb.vhd"
xelab -debug off FebPacketGenTb -s FebPacketGenTb
xsim FebPacketGenTb -runall

python3 "$TB/febPacketCheck.py" feb_packets.txt

#!/usr/bin/env python3
#-----------------------------------------------------------------------------
# This file is part of the 'Simple-10GbE-RUDP-KCU105-Example'. It is subject to
# the license terms in the LICENSE.txt file found in the top-level directory
# of this distribution and at:
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
# No part of the 'Simple-10GbE-RUDP-KCU105-Example', including this file, may be
# copied, modified, propagated, or distributed except according to the terms
# contained in the LICENSE.txt file.
#-----------------------------------------------------------------------------
"""Rate frontier of the AdvCam FEB emulator, packet size by packet size.

For each num_samples: the packet rate when the generator is triggered every clock cycle, and
the highest periodic trigger rate it sustains without dropping one. Frames are counted by
rogue's C++ stream slave. Where even that cannot keep up, the host NIC runs out of receive
buffers (its out_of_buffer counter) and answers "receiver not ready", and the engine sends the
packet again: its transmissions (DmaReadCount) then exceed its successes, and the row is
marked as limited by the host rather than by the board or the link.
"""
import setupLibPaths  # noqa: F401 -- MUST be first: adds firmware/python + surf/python

import argparse
import math
import time

import pyrogue.protocols
import rogue.interfaces.stream as ris

from febEmulatorTest import FebRoot, packet_bytes

CLK_HZ = 156.25e6
LINE_BPS = 10e9
# Per RoCEv2 SEND on the wire: Ethernet 14 + IPv4 20 + UDP 8 + BTH 12 + ICRC 4 + FCS 4,
# plus preamble 8 and inter-frame gap 12.
WIRE_OVERHEAD = 82


def nic_counter(dev, name):
    with open(f'/sys/class/infiniband/{dev}/ports/1/hw_counters/{name}') as f:
        return int(f.read())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='192.168.2.10', help='FPGA IP address')
    ap.add_argument('--roceDevice', default='mlx5_0', help='host RDMA device (mlx5_*)')
    ap.add_argument('--roceGidIndex', type=int, default=-1)
    ap.add_argument('--samples', default='1,2,4,8,16,24,32,40,48,53',
                    help='num_samples values to measure')
    ap.add_argument('--seconds', type=float, default=1.0, help='length of each measurement')
    args = ap.parse_args()

    cfg = pyrogue.protocols.RoCEv2ServerCfg(ip=args.ip, deviceName=args.roceDevice,
                                            gidIndex=args.roceGidIndex, rxQueueDepth=4096)
    transport = pyrogue.protocols.RoCEv2TransportCfg(minRnrTimer=1)
    count = ris.Slave()
    with FebRoot(cfg, transport, args.ip, count) as root:
        feb = root.App.FebPacketGen
        dma = root.Core.RoCEv2AxiStreamRdma.Core
        dma.LKey.set(root.rdmaRx.FpgaLkey.get())
        dma.SQpn.set(root.rdmaRx.HostQpn.get())
        root.App.SsiPrbsTx.TxEn.set(False)
        feb.TrigEnable.set(False)
        feb.PatternOffset.set(False)
        feb.CorruptCrc.set(False)
        feb.setTaiNow()
        dma.DispatchEnable.set(True)

        def measure(period):
            """Trigger every `period` cycles for args.seconds: (sent/s, dropped, received,
            engine unsuccessful, host out_of_buffer, sent, engine retransmissions)."""
            feb.CountReset()
            dma.ResetCounters()
            f0, oob0 = count.getFrameCount(), nic_counter(args.roceDevice, 'out_of_buffer')
            feb.TrigPeriod.set(period)
            feb.TrigEnable.set(True)
            time.sleep(args.seconds)
            feb.TrigEnable.set(False)
            time.sleep(0.2)
            sent, drop = feb.PacketCount.get(), feb.DropCount.get()
            return (sent / args.seconds, drop, count.getFrameCount() - f0,
                    dma.UnsuccessCounter.get(),
                    nic_counter(args.roceDevice, 'out_of_buffer') - oob0, sent,
                    dma.DmaReadCount.get() - dma.SuccessCounter.get())

        print(f'{"S":>3} {"bytes":>5} | {"saturated":>10} {"Gb/s":>5} {"wire":>5} {"retries":>8} | '
              f'{"clean max":>10} {"Gb/s":>5} {"wire":>5} | {"link max":>9} | limited by')
        for s in (int(x) for x in args.samples.split(',')):
            feb.NumSamples.set(s)
            b = packet_bytes(s)
            link_pps = LINE_BPS / ((b + WIRE_OVERHEAD) * 8)

            # Saturated: a trigger every cycle keeps the generator always busy.
            sat_pps, _, rx, uns, oob, sent, retries = measure(1)
            lost = sent - rx

            # Clean: the shortest trigger period with no trigger dropped. Search between the
            # saturated packet period (cannot be clean) and four times it.
            lo = max(1, math.floor(CLK_HZ / sat_pps)) if sat_pps else 1
            hi = 4 * lo
            while measure(hi)[1] > 0:
                hi *= 2
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if measure(mid)[1] == 0:
                    hi = mid
                else:
                    lo = mid
            clean_pps, drop, rx2, uns2, oob2, sent2, _ = measure(hi)
            clean_hz = CLK_HZ / hi
            limit = ('host receiver' if oob or retries else
                     'link' if sat_pps >= 0.97 * link_pps else 'board')
            print(f'{s:3d} {b:5d} | {sat_pps / 1e3:8.1f} k {sat_pps * b * 8 / 1e9:5.2f} '
                  f'{sat_pps * (b + WIRE_OVERHEAD) * 8 / 1e9:5.2f} {retries:8d} | '
                  f'{clean_hz / 1e3:8.1f} k {clean_hz * b * 8 / 1e9:5.2f} '
                  f'{clean_hz * (b + WIRE_OVERHEAD) * 8 / 1e9:5.2f} | {link_pps / 1e3:7.1f} k | '
                  f'{limit}'
                  f'{"  LOST " + str(lost + sent2 - rx2) if lost or sent2 - rx2 else ""}'
                  f'{"  ENGINE ERRORS" if uns or uns2 else ""}'
                  f'{"  (host short of buffers at the clean rate)" if oob2 else ""}')
        feb.NumSamples.set(40)
        feb.TrigPeriod.set(3906)


if __name__ == '__main__':
    main()

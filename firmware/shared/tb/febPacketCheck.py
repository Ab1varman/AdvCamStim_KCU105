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
"""Check FebPacketGenTb output against the FEB ICD packet model, byte for byte."""
import struct
import sys
import zlib

QNS_PER_SEC = 4_000_000_000

# What the testbench programs
TEL, BOARD, TRIGGER, EVENT, PIXEL = 0x11, 0x22, 0x33, 0x44, 0x00012345_89ABCDEF
TIME_SET_SEC = 0x6A000000


def icd_packet(samples, sec, qns, offset):
    """FEB ICD format 1, no-neighbours: header, packed 12-bit samples, CRC-32."""
    n = 49 * samples
    payload_bytes = 4 * ((12 * n + 31) // 32)
    body, acc, nbits = bytearray(), 0, 0
    for i in range(n):
        acc = (acc << 12) | ((i + offset) & 0xFFF)
        nbits += 12
        while nbits >= 8:
            nbits -= 8
            body.append((acc >> nbits) & 0xFF)
        acc &= (1 << nbits) - 1
    if nbits:
        body.append((acc << (8 - nbits)) & 0xFF)
    body += bytes(payload_bytes - len(body))
    pkt = (bytes([TEL, BOARD, 0x01, TRIGGER, EVENT, samples, 0x01, 0x00])
           + struct.pack('>IIQ', sec, qns, PIXEL) + body)
    return pkt + struct.pack('>I', zlib.crc32(pkt))


def main(path):
    cfg, regs, fails, count, last_t = {}, {}, 0, 0, None
    for line in open(path):
        f = line.split()
        if not f:
            continue
        if f[0] == 'CFG':
            cfg = dict(kv.split('=') for kv in f[1:] if '=' in kv)
            continue
        if f[0] == 'REG':
            regs[f[1]] = int(f[2], 16)
            continue
        if f[0] != 'PKT':
            continue

        pkt = bytes.fromhex(f[1])
        count += 1
        samples, corrupt = int(cfg['S']), cfg['corrupt'] == '1'
        sec, qns = struct.unpack_from('>II', pkt, 8)
        offset = ((pkt[24] << 4) | (pkt[25] >> 4)) if cfg['offset'] == '1' else 0
        want = icd_packet(samples, sec, qns, offset)
        if corrupt:
            want = want[:-4] + struct.pack('>I', zlib.crc32(want[:-4]) ^ 0xFFFFFFFF)

        problems = []
        if pkt != want:
            if len(pkt) != len(want):
                problems.append(f'{len(pkt)} B, expected {len(want)} B')
            else:
                first = next(i for i in range(len(pkt)) if pkt[i] != want[i])
                problems.append(f'differs from byte {first}')
        if qns >= QNS_PER_SEC:
            problems.append(f'qns {qns} out of range')
        if not TIME_SET_SEC <= sec <= TIME_SET_SEC + 1:
            problems.append(f'seconds {sec:#x} not near the value loaded')
        t = sec * QNS_PER_SEC + qns
        if last_t is not None and t <= last_t:
            problems.append('event time did not increase')
        last_t = t

        fails += bool(problems)
        print(f'packet {count:2d}  S={samples:2d}  {len(pkt):4d} B  t={sec:#x}.{qns:010d}  '
              f'offset={offset:4d}  {"bad CRC injected  " if corrupt else ""}'
              f'{"OK" if not problems else "FAIL: " + "; ".join(problems)}')

    checks = [
        ('PacketCount matches the packets seen', regs.get('PacketCount') == count),
        ('TrigCount = PacketCount + DropCount',
         regs.get('TrigCount') == regs.get('PacketCount', -1) + regs.get('DropCount', -1)),
        ('periodic triggers faster than the drain were dropped', regs.get('DropCount', 0) > 0),
        ('PacketBytes = 2968 at 40 samples', regs.get('PacketBytes') == 2968),
        ('FormatVersion = 1', regs.get('FormatVersion') == 1),
        ('seconds rolled over', last_t is not None and last_t // QNS_PER_SEC == TIME_SET_SEC + 1),
    ]
    for name, ok in checks:
        print(f'{"ok  " if ok else "FAIL"} {name}')
        fails += not ok
    print(f'{count} packets, {"all checks passed" if fails == 0 else f"{fails} failure(s)"}')
    return 1 if fails or count == 0 else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else 'feb_packets.txt'))

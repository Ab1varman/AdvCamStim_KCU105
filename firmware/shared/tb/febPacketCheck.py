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
"""Check FebPacketGenTb output against the FEB ICD v0.3 packet model, byte for byte."""
import struct
import sys
import zlib

QNS_PER_SEC = 4_000_000_000

# What the testbench programs
TEL, BOARD, TRIGGER, EVENT = 0x11, 0x22, 0x33, 0x44
PEDESTAL, PIXEL_STATUS = 0x1234, 0x56
TIME_SET_SEC = 0x6A000000


def icd_packet(samples, sec, qns, event_id, offset):
    """FEB ICD v0.3 format 1, no-neighbours: header, pedestal_feb, pixel_status_feb,
    packed 12-bit samples, CRC-32. pedestal_feb and pixel_status_feb are one value
    repeated for every channel, matching what the generator writes."""
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
    pkt = (bytes([0x01, TEL, BOARD, TRIGGER, EVENT, samples, 0x00, 0x00])
           + struct.pack('>IIQ', sec, qns, event_id)
           + struct.pack('>H', PEDESTAL) * 49
           + bytes([PIXEL_STATUS]) * 49
           + bytes(5) + body)
    return pkt + struct.pack('>I', zlib.crc32(pkt))


def main(path):
    cfg, regs, fails, count, last_t, last_id = {}, {}, 0, 0, None, -1
    gaps, reset_ids = 0, []
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
        samples, corrupt = int(cfg['S']), cfg['corrupt'] == '1'
        after_reset = cfg.get('reset') == '1'
        if not after_reset:
            count += 1
        sec, qns = struct.unpack_from('>II', pkt, 8)
        event_id = struct.unpack_from('>Q', pkt, 16)[0]
        offset = ((pkt[176] << 4) | (pkt[177] >> 4)) if cfg['offset'] == '1' else 0
        want = icd_packet(samples, sec, qns, event_id, offset)
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
        # A dropped trigger consumes its number, so ids increase by 1 plus the triggers
        # dropped since the previous packet. Packets sent after CountReset restart at 0.
        if after_reset:
            reset_ids.append(event_id)
        elif event_id <= last_id:
            problems.append(f'tel_event_id {event_id} after {last_id}')
        else:
            gaps += event_id - last_id - 1
            last_id = event_id
        if cfg['offset'] == '1' and offset != event_id & 0xFFF:
            problems.append(f'pattern offset {offset} is not tel_event_id mod 4096')

        fails += bool(problems)
        print(f'packet {count + len(reset_ids):2d}  S={samples:2d}  {len(pkt):4d} B  id={event_id}  '
              f't={sec:#x}.{qns:010d}  offset={offset:4d}  '
              f'{"bad CRC injected  " if corrupt else ""}'
              f'{"OK" if not problems else "FAIL: " + "; ".join(problems)}')

    checks = [
        ('PacketCount matches the packets seen', regs.get('PacketCount') == count),
        ('TrigCount = PacketCount + DropCount',
         regs.get('TrigCount') == regs.get('PacketCount', -1) + regs.get('DropCount', -1)),
        ('periodic triggers faster than the drain were dropped', regs.get('DropCount', 0) > 0),
        ('every dropped trigger left a tel_event_id gap',
         gaps + (regs.get('TrigCount', 0) - 1 - last_id) == regs.get('DropCount')),
        ('the first packet after CountReset is tel_event_id 0', reset_ids[:1] == [0]),
        ('CountReset zeroed PacketCount', regs.get('PacketCountAfterReset') == len(reset_ids)),
        ('PacketBytes = 3120 at 40 samples', regs.get('PacketBytes') == 3120),
        ('FormatVersion = 1', regs.get('FormatVersion') == 1),
        ('MapVersion = 1', regs.get('MapVersion') == 1),
        ('seconds rolled over', last_t is not None and last_t // QNS_PER_SEC == TIME_SET_SEC + 1),
    ]
    for name, ok in checks:
        print(f'{"ok  " if ok else "FAIL"} {name}')
        fails += not ok
    print(f'{count} packets, {"all checks passed" if fails == 0 else f"{fails} failure(s)"}')
    return 1 if fails or count == 0 else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else 'feb_packets.txt'))

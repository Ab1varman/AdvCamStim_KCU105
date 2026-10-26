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
"""Board check of the AdvCam FEB emulator (docs/advcam_feb_emulator.md).

Brings the board up as rocev2PrbsTest.py does, then has the FEB packet generator send at
several rates and checks every packet received against the FEB ICD format-1 model, and the
generator's and the engine's counters against what was received. Needs an mlx5 NIC.
"""
import setupLibPaths  # noqa: F401 -- MUST be first: adds firmware/python + surf/python

import argparse
import struct
import sys
import threading
import time
import zlib

import numpy as np
import pyrogue as pr
import pyrogue.protocols
import rogue
import rogue.interfaces.stream as ris
import rogue.protocols.srp

import simple_10gbe_rudp_kcu105_example as baseBoard
import rocev2_10gbe_rudp_kcu105_example as roceBoard

QNS = 4_000_000_000


def packet_bytes(s):
    return 180 + 4 * ((147 * s + 7) // 8)


class FebRx(ris.Slave):
    """Checks every received frame against what the generator was told to send."""

    def __init__(self, sample_every):
        super().__init__()
        self._lock = threading.Lock()
        self._sample_every = sample_every
        self.arm({})

    def arm(self, expect):
        with self._lock:
            self.expect = dict(expect)
            self.count = 0
            self.bad = {}
            self.first_id = None
            self.last_id = None
            self.gaps = 0
            self.last_t = None
            self.crc_bad = 0
            self.sizes = set()
            self.samples_checked = 0
            self.t0 = time.time()

    def _fail(self, what):
        self.bad[what] = self.bad.get(what, 0) + 1

    def _acceptFrame(self, frame):
        with frame.lock():
            d = frame.getNumpy(0, frame.getPayload()).tobytes()
        with self._lock:
            self._check(d)

    def _check(self, d):
        e = self.expect
        self.count += 1
        self.sizes.add(len(d))
        s = e.get('samples', 40)
        if len(d) != packet_bytes(s):
            self._fail(f'size {len(d)} != {packet_bytes(s)}')
            return
        if tuple(d[0:8]) != (1, e['tel'], e['board'], e['trig'], e['evt'], s, 0, 0):
            self._fail(f'header bytes 0-7 {tuple(d[0:8])}')
        sec, qns, eid = struct.unpack_from('>IIQ', d, 8)
        if qns >= QNS:
            self._fail('qns out of range')
        t = sec * QNS + qns
        if self.last_t is not None and t <= self.last_t:
            self._fail('event time not increasing')
        self.last_t = t
        if abs(sec - (time.time() + 37)) > 5:
            self._fail('event time not near host TAI')
        if self.first_id is None:
            self.first_id = eid
        elif eid <= self.last_id:
            self._fail('tel_event_id not increasing')
        else:
            self.gaps += eid - self.last_id - 1
        self.last_id = eid
        if d[24:122] != struct.pack('>H', e['ped']) * 49:
            self._fail('pedestal_feb')
        if d[122:171] != bytes([e['pix']]) * 49 or any(d[171:176]):
            self._fail('pixel_status_feb / reserved')
        crc_ok = zlib.crc32(d[:-4]) == struct.unpack_from('>I', d, len(d) - 4)[0]
        if not crc_ok:
            self.crc_bad += 1
        if crc_ok == e.get('corrupt', False):
            self._fail('CRC ' + ('valid although corrupted' if crc_ok else 'invalid'))
        if self._sample_every and self.count % self._sample_every == 0:
            self.samples_checked += 1
            n = 49 * s
            b = np.frombuffer(d, np.uint8, count=(3 * n + 1) // 2, offset=176).astype(np.uint16)
            got = np.empty(n + (n & 1), np.uint16)
            got[0::2] = (b[0::3] << 4) | (b[1::3] >> 4)
            got[1::2][:len(b[2::3])] = ((b[1::3][:len(b[2::3])] & 0xF) << 8) | b[2::3]
            off = (eid & 0xFFF) if e.get('offset') else 0
            want = (np.arange(n, dtype=np.uint32) + off) & 0xFFF
            if not np.array_equal(got[:n], want.astype(np.uint16)):
                self._fail('samples')


class FebRoot(pr.Root):
    """SLAC's RoCEv2 Root (rocev2_10gbe_rudp_kcu105_example._Root), feeding what it receives to
    rxSlave instead of a PRBS checker."""

    def __init__(self, rocev2Cfg, transportCfg, ip, rxSlave, **kwargs):
        super().__init__(timeout=5.0, pollEn=False, initRead=True, **kwargs)
        self._cfg = transportCfg
        self.rudp = pr.protocols.UdpRssiPack(name='SwRudpClient', host=ip, port=8192, packVer=2,
                                             jumbo=False, expand=False)
        self.add(self.rudp)
        self.srp = rogue.protocols.srp.SrpV3()
        self.srp == self.rudp.application(0)
        self.add(baseBoard.Core(offset=0x0, memBase=self.srp, rocev2=True, dcqcn=True,
                                expand=False))
        self.add(roceBoard.App(offset=0x8000_0000, memBase=self.srp, expand=True))
        self.add(pr.protocols.RoCEv2Server(name='rdmaRx', rocev2Cfg=rocev2Cfg, expand=False))
        self.febRx = rxSlave
        self.rdmaRx.stream >> self.febRx

    def start(self, **kwargs):
        super().start(**kwargs)
        try:
            cfg = self._cfg
            params = self.rdmaRx.getHostParams()
            fpga = self.Core.RoCEv2AxiStreamRdma.Engine.setupConnection(
                **params._asdict(), pmtu=cfg.pmtu, minRnrTimer=cfg.minRnrTimer,
                rnrRetry=cfg.rnrRetry, retryCount=cfg.retryCount)
            self.rdmaRx.completeConnection(
                fpga.fpgaQpn, fpgaLkey=fpga.lkey, pmtu=cfg.pmtu, minRnrTimer=cfg.minRnrTimer,
                rnrRetry=cfg.rnrRetry, retryCount=cfg.retryCount)
            udp = self.Core.UdpEngine
            udp.ClientRemotePort[0].set(4791)
            udp.ClientRemoteIp[0].set(self.rdmaRx.HostIp.get())
            udp.EcnFlag.set(0)
            udp.Dscp.set(0)
            self.Core.RoCEv2AxiStreamRdma.Dcqcn.DcqcnBypass.set(True)
        except Exception:
            self.stop()
            raise

    def stop(self):
        try:
            self.Core.RoCEv2AxiStreamRdma.Core.DispatchEnable.set(False)
            time.sleep(0.1)
            self.Core.RoCEv2AxiStreamRdma.Engine.teardownConnection()
            self.Core.RoCEv2AxiStreamRdma.Engine.SoftReset()
        except Exception as e:
            print(f'stop: teardown skipped ({e})')
        finally:
            super().stop()


RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append(ok)
    print(f'  {"ok  " if ok else "FAIL"} {name}{"  (" + detail + ")" if detail else ""}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='192.168.2.10', help='FPGA IP address')
    ap.add_argument('--roceDevice', default='mlx5_0', help='host RDMA device (mlx5_*)')
    ap.add_argument('--roceGidIndex', type=int, default=-1,
                    help='host GID index; -1 detects the one matching the board\'s subnet')
    ap.add_argument('--seconds', type=float, default=10.0,
                    help='length of the 10 kHz and 40 kHz runs')
    ap.add_argument('--sampleEvery', type=int, default=20,
                    help='check the samples of every Nth packet (0: never)')
    args = ap.parse_args()

    cfg = pyrogue.protocols.RoCEv2ServerCfg(ip=args.ip, deviceName=args.roceDevice,
                                            gidIndex=args.roceGidIndex)
    transport = pyrogue.protocols.RoCEv2TransportCfg(minRnrTimer=1)
    with FebRoot(cfg, transport, args.ip, FebRx(args.sampleEvery)) as root:
        rx, feb = root.febRx, root.App.FebPacketGen
        dma = root.Core.RoCEv2AxiStreamRdma.Core
        ver = root.Core.AxiVersion

        print('== identity')
        git = ver.GitHash.get()
        print(f'  AxiVersion: {ver.BuildStamp.get()}  git {git:040x}'
              if isinstance(git, int) else f'  AxiVersion: {ver.BuildStamp.get()}  git {git}')
        check('MapVersion = 1', feb.MapVersion.get() == 1, str(feb.MapVersion.get()))
        check('FormatVersion = 1', feb.FormatVersion.get() == 1, str(feb.FormatVersion.get()))

        print('== run parameters read back')
        params = dict(TelId=7, BoardId=42, NumSamples=40, TriggerType=1, EventType=2,
                      Pedestal=0x1234, PixelStatus=0x04)
        for k, v in params.items():
            getattr(feb, k).set(v)
        back = {k: getattr(feb, k).get() for k in params}
        check('TelId, BoardId, NumSamples, TriggerType, EventType, Pedestal, PixelStatus',
              back == params, str(back) if back != params else '')
        expect = dict(tel=7, board=42, trig=1, evt=2, ped=0x1234, pix=0x04, samples=40)

        print('== dispatch')
        maxsize, maxpay = dma.MaxSize.get(), root.rdmaRx.MaxPayload.get()
        check('MaxSize fits the host receive buffers', maxsize <= maxpay,
              f'MaxSize {maxsize}, MaxPayload {maxpay}')
        dma.LKey.set(root.rdmaRx.FpgaLkey.get())
        dma.SQpn.set(root.rdmaRx.HostQpn.get())
        root.App.SsiPrbsTx.TxEn.set(False)
        feb.TrigEnable.set(False)
        feb.setTaiNow()
        dma.ResetCounters()
        dma.DispatchEnable.set(True)

        def run(label, rate, seconds, **extra):
            e = dict(expect, **extra)
            feb.NumSamples.set(e['samples'])
            feb.PatternOffset.set(bool(e.get('offset')))
            feb.CorruptCrc.set(bool(e.get('corrupt')))
            feb.CountReset()
            dma.ResetCounters()
            rx.arm(e)
            feb.TrigRate.set(rate)
            feb.TrigEnable.set(True)
            time.sleep(seconds)
            feb.TrigEnable.set(False)
            time.sleep(1.0)
            trig, pkt, drop = feb.TrigCount.get(), feb.PacketCount.get(), feb.DropCount.get()
            succ, unsucc, over = (dma.SuccessCounter.get(), dma.UnsuccessCounter.get(),
                                  dma.OversizeCount.get())
            with rx._lock:
                n, bad, gaps, first, last = rx.count, dict(rx.bad), rx.gaps, rx.first_id, rx.last_id
                crc_bad, smp = rx.crc_bad, rx.samples_checked
            print(f'== {label}: {trig} triggers, {pkt} sent, {drop} dropped, received {n} '
                  f'({n / seconds:,.0f}/s), samples checked on {smp}; engine success {succ}, '
                  f'unsuccess {unsucc}, oversize {over}')
            check('every packet matches the ICD model', not bad, str(bad) if bad else '')
            check('received = PacketCount = engine successes', n == pkt == succ,
                  f'{n} / {pkt} / {succ}')
            check('TrigCount = PacketCount + DropCount', trig == pkt + drop)
            check('first tel_event_id is 0 after CountReset', first == 0, str(first))
            trailing = (trig - 1 - last) if last is not None else trig
            check('every dropped trigger left a tel_event_id gap', gaps + trailing == drop,
                  f'gaps {gaps} + trailing {trailing} vs DropCount {drop}')
            check('no unsuccessful or oversize completions', unsucc == 0 and over == 0)
            return n, crc_bad

        run('10 kHz, S=40', 10_000, args.seconds)
        run('40 kHz, S=40', 40_000, args.seconds)
        run('10 kHz, S=40, PatternOffset', 10_000, 3, offset=True)
        n, crc_bad = run('1 kHz, S=40, CorruptCrc', 1_000, 3, corrupt=True)
        check('every packet carried a bad CRC', n > 0 and crc_bad == n, f'{crc_bad} of {n}')
        feb.CorruptCrc.set(False)
        feb.PatternOffset.set(False)

        # Faster than this Python receiver drains: RNR stalls the FEB, which must drop triggers,
        # count them, and leave a gap in tel_event_id for each.
        run('200 kHz, S=40, receiver falling behind', 200_000, 2)
        with rx._lock:
            check('triggers were dropped', feb.DropCount.get() > 0, str(feb.DropCount.get()))

        print('== size limit: SEND of one MTU at most')
        for s, should_pass in ((53, True), (54, False)):
            feb.NumSamples.set(s)
            feb.CountReset()
            dma.ResetCounters()
            rx.arm(dict(expect, samples=s))
            for _ in range(10):
                feb.SoftTrig()
                time.sleep(0.01)
            time.sleep(0.5)
            over = dma.OversizeCount.get()
            if should_pass:
                check(f'S={s} ({packet_bytes(s)} B): 10 received intact, none oversize',
                      rx.count == 10 and over == 0 and not rx.bad,
                      f'{rx.count} received, {over} oversize, problems {rx.bad}')
            else:
                check(f'S={s} ({packet_bytes(s)} B): none received, 10 oversize',
                      rx.count == 0 and over == 10, f'{rx.count} received, {over} oversize')
        feb.NumSamples.set(40)

    failed = RESULTS.count(False)
    print(f'\n{len(RESULTS) - failed} of {len(RESULTS)} checks passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

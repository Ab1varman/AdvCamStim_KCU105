#-----------------------------------------------------------------------------
# This file is part of the 'Simple-10GbE-RUDP-KCU105-Example'. It is subject to
# the license terms in the LICENSE.txt file found in the top-level directory
# of this distribution and at:
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
# No part of the 'Simple-10GbE-RUDP-KCU105-Example', including this file, may be
# copied, modified, propagated, or distributed except according to the terms
# contained in the LICENSE.txt file.
#-----------------------------------------------------------------------------

import time

import pyrogue as pr

# TAI - UTC, unchanged since 2017-01-01
TAI_UTC_S = 37


class FebPacketGen(pr.Device):
    """AdvCam FEB packet generator: one FEB ICD packet per trigger (see FebPacketGen.vhd).

    0x000-0x0FF is what the FEB ICD asks of a FEB; 0x100-0x1FF exists only in this
    emulator, standing in for the CTP's triggers, White Rabbit time and the front-end.
    Sending is started and stopped by the RoCEv2 engine's DispatchEnable, not here.
    """

    def __init__(self, clkFreq=156.25e6, **kwargs):
        super().__init__(**kwargs)
        self._clkFreq = clkFreq

        # ---- What the ICD asks of a FEB ----
        self.add(pr.RemoteVariable(
            name='MapVersion', offset=0x000, bitSize=32, mode='RO', disp='{:d}',
            description='Layout of this register block'))
        self.add(pr.RemoteVariable(
            name='FormatVersion', offset=0x004, bitSize=8, mode='RO', disp='{:d}',
            description='format_version of the packets sent'))
        self.add(pr.RemoteVariable(
            name='TelId', offset=0x008, bitSize=8, mode='RW', disp='{:d}',
            description='tel_id, header byte 1'))
        self.add(pr.RemoteVariable(
            name='BoardId', offset=0x00C, bitSize=8, mode='RW', disp='{:d}',
            description='board_id, header byte 2'))
        self.add(pr.RemoteVariable(
            name='NumSamples', offset=0x010, bitSize=8, mode='RW', disp='{:d}',
            description='num_samples: samples per channel, 1-255'))
        self.add(pr.RemoteCommand(
            name='CountReset', offset=0x018, bitSize=1, function=pr.RemoteCommand.touchOne,
            description='Zero the counters, so tel_event_id restarts at 0'))
        for name, off, desc in (
                ('TrigCount', 0x040, 'Triggers received, sent or dropped; also the next '
                                     'tel_event_id'),
                ('PacketCount', 0x044, 'Packets sent'),
                ('DropCount', 0x048, 'Triggers dropped: a packet was still going out')):
            self.add(pr.RemoteVariable(
                name=name, offset=off, bitSize=32, mode='RO', disp='{:d}', pollInterval=1,
                description=desc))

        # ---- Emulator only ----
        self.add(pr.RemoteVariable(
            name='TrigEnable', offset=0x100, bitSize=1, bitOffset=0, base=pr.Bool, mode='RW',
            description='Emulator: trigger every TrigPeriod cycles'))
        self.add(pr.RemoteVariable(
            name='PatternOffset', offset=0x100, bitSize=1, bitOffset=1, base=pr.Bool, mode='RW',
            description='Emulator: offset the sample ramp by tel_event_id'))
        self.add(pr.RemoteVariable(
            name='CorruptCrc', offset=0x100, bitSize=1, bitOffset=2, base=pr.Bool, mode='RW',
            description='Emulator: send inverted CRCs (error injection)'))
        self.add(pr.RemoteVariable(
            name='TrigPeriod', offset=0x104, bitSize=32, mode='RW', disp='{:d}',
            description='Emulator: clock cycles between periodic triggers'))
        self.add(pr.LinkVariable(
            name='TrigRate', units='Hz', mode='RW', disp='{:0.1f}', dependencies=[self.TrigPeriod],
            linkedGet=lambda dev, var, read: (
                dev._clkFreq / p if (p := var.dependencies[0].get(read=read)) else 0.0),
            linkedSet=lambda dev, var, value, write: var.dependencies[0].set(
                int(round(dev._clkFreq / value)) if value > 0 else 0, write=write)))
        self.add(pr.RemoteCommand(
            name='SoftTrig', offset=0x108, bitSize=1, function=pr.RemoteCommand.touchOne,
            description='Emulator: one trigger'))
        self.add(pr.RemoteVariable(
            name='TriggerType', offset=0x10C, bitSize=8, bitOffset=0, mode='RW',
            description='Emulator: trigger_type for the header; a real FEB takes it from the '
                        'reading command'))
        self.add(pr.RemoteVariable(
            name='EventType', offset=0x10C, bitSize=8, bitOffset=8, mode='RW',
            description='Emulator: event_type for the header'))
        self.add(pr.RemoteVariable(
            name='Pedestal', offset=0x110, bitSize=16, mode='RW', disp='{:d}',
            description='Emulator: pedestal_feb, one value repeated for every channel '
                        '(averaging window and u12.4 format still TBC in the ICD)'))
        self.add(pr.RemoteVariable(
            name='PixelStatus', offset=0x114, bitSize=8, mode='RW',
            description='Emulator: pixel_status_feb, one value repeated for every channel '
                        '(bit layout still TBC in the ICD)'))
        self.add(pr.RemoteVariable(
            name='TimeSetSec', offset=0x118, bitSize=32, mode='RW',
            description='Emulator: TAI seconds TimeLoad copies into the event-time counter'))
        self.add(pr.RemoteVariable(
            name='TimeSetQns', offset=0x11C, bitSize=32, mode='RW',
            description='Emulator: and quarter-nanoseconds'))
        self.add(pr.RemoteCommand(
            name='TimeLoad', offset=0x120, bitSize=1, function=pr.RemoteCommand.touchOne,
            description='Emulator: load TimeSetSec/TimeSetQns into the event-time counter'))
        for name, off, desc in (
                ('TimeSec', 0x140, 'Emulator: event-time counter, TAI seconds'),
                ('TimeQns', 0x144, 'Emulator: event-time counter, quarter-nanoseconds'),
                ('PacketBytes', 0x148, 'Emulator: bytes per packet at NumSamples, CRC included')):
            self.add(pr.RemoteVariable(
                name=name, offset=off, bitSize=32, mode='RO', disp='{:d}', pollInterval=1,
                description=desc))

    def setTaiNow(self):
        """Load the host clock, as TAI, into the event-time counter."""
        t = time.time() + TAI_UTC_S
        self.TimeSetSec.set(int(t))
        self.TimeSetQns.set(int((t % 1.0) * 4e9))
        self.TimeLoad()

    def countReset(self):
        self.CountReset()

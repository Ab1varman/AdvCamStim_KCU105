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
    """AdvCam FEB packet generator: one FEB ICD packet per trigger (see FebPacketGen.vhd)."""

    def __init__(self, clkFreq=156.25e6, **kwargs):
        super().__init__(**kwargs)
        self._clkFreq = clkFreq

        self.add(pr.RemoteVariable(
            name='Enable', offset=0x000, bitSize=1, bitOffset=0, base=pr.Bool, mode='RW',
            description='Trigger every TrigPeriod cycles'))
        self.add(pr.RemoteVariable(
            name='PatternOffset', offset=0x000, bitSize=1, bitOffset=1, base=pr.Bool, mode='RW',
            description='Offset the sample ramp by the trigger count'))
        self.add(pr.RemoteVariable(
            name='CorruptCrc', offset=0x000, bitSize=1, bitOffset=2, base=pr.Bool, mode='RW',
            description='Send inverted CRCs (error injection)'))
        self.add(pr.RemoteVariable(
            name='TrigPeriod', offset=0x004, bitSize=32, mode='RW', disp='{:d}',
            description='Clock cycles between periodic triggers'))
        self.add(pr.LinkVariable(
            name='TrigRate', units='Hz', mode='RW', disp='{:0.1f}', dependencies=[self.TrigPeriod],
            linkedGet=lambda dev, var, read: (
                dev._clkFreq / p if (p := var.dependencies[0].get(read=read)) else 0.0),
            linkedSet=lambda dev, var, value, write: var.dependencies[0].set(
                int(round(dev._clkFreq / value)) if value > 0 else 0, write=write)))
        self.add(pr.RemoteVariable(
            name='NumSamples', offset=0x008, bitSize=8, mode='RW', disp='{:d}',
            description='Samples per channel'))
        for name, bit in (('TelId', 0), ('BoardId', 8), ('TriggerType', 16), ('EventType', 24)):
            self.add(pr.RemoteVariable(name=name, offset=0x00C, bitSize=8, bitOffset=bit, mode='RW'))
        self.add(pr.RemoteVariable(
            name='Pedestal', offset=0x010, bitSize=16, mode='RW', disp='{:d}',
            description='pedestal_feb: one value, repeated for every channel '
                        '(averaging window and u12.4 format still TBC in the ICD)'))
        self.add(pr.RemoteVariable(
            name='PixelStatus', offset=0x014, bitSize=8, mode='RW',
            description='pixel_status_feb: one value, repeated for every channel '
                        '(bit layout still TBC in the ICD)'))
        self.add(pr.RemoteVariable(name='TimeSetSec', offset=0x018, bitSize=32, mode='RW'))
        self.add(pr.RemoteVariable(name='TimeSetQns', offset=0x01C, bitSize=32, mode='RW'))
        self.add(pr.RemoteCommand(
            name='TimeLoad', offset=0x020, bitSize=1, function=pr.RemoteCommand.touchOne,
            description='Load TimeSetSec/TimeSetQns into the event-time counter'))
        self.add(pr.RemoteCommand(
            name='SoftTrig', offset=0x024, bitSize=1, function=pr.RemoteCommand.touchOne))
        self.add(pr.RemoteCommand(
            name='CountReset', offset=0x028, bitSize=1, function=pr.RemoteCommand.touchOne))

        for name, off, desc in (
                ('TrigCount', 0x040, 'Triggers, sent or dropped'),
                ('PacketCount', 0x044, 'Packets sent; also the next packet\'s tel_event_id'),
                ('DropCount', 0x048, 'Triggers dropped: a packet was still going out'),
                ('TimeSec', 0x04C, 'Event-time counter, TAI seconds'),
                ('TimeQns', 0x050, 'Event-time counter, quarter-nanoseconds'),
                ('PacketBytes', 0x054, 'Bytes per packet at NumSamples, CRC included')):
            self.add(pr.RemoteVariable(
                name=name, offset=off, bitSize=32, mode='RO', disp='{:d}', pollInterval=1,
                description=desc))
        self.add(pr.RemoteVariable(name='FormatVersion', offset=0x058, bitSize=8, mode='RO'))

    def setTaiNow(self):
        """Load the host clock, as TAI, into the event-time counter."""
        t = time.time() + TAI_UTC_S
        self.TimeSetSec.set(int(t))
        self.TimeSetQns.set(int((t % 1.0) * 4e9))
        self.TimeLoad()

    def countReset(self):
        self.CountReset()

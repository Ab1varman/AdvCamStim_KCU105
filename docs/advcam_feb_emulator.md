# AdvCam FEB emulator on the KCU105

This branch turns SLAC's RoCEv2 example design for the KCU105 into an emulator of one AdvCam
front-end board (FEB). The board sends one FEB data packet per trigger, in format version 1 of
the AdvCam FEB ICD v0.3 (no-neighbours scenario), as an RDMA SEND from SURF's RoCEv2 engine.
A host sets it up through its registers, over the same SRPv3/RSSI/UDP link the example already
uses. The AdvCam DAQ receives from it exactly as it would from a real FEB.

It is also meant to start the discussion of the FEB register map, which the ICD leaves to be
defined with the firmware team, starting from the SURF example.

## What is ours, what is SLAC's

Everything is SLAC's example (target `RoCEv2_10GbeRudpKcu105Example`) except:

- `firmware/shared/rtl/FebPacketGen.vhd`: the packet generator and its registers;
- in `firmware/shared/rtl/App.vhd`, a two-input mux in front of the RDMA engine, so either
  the example's PRBS source or the FEB generator feeds it (enable one at a time);
- `firmware/python/rocev2_10gbe_rudp_kcu105_example/_FebPacketGen.py`: its PyRogue device;
- `firmware/shared/tb/`: a testbench and a checker that compares every packet, byte for byte,
  with a Python model of the ICD (`runFebPacketGenSim.sh`, needs Vivado's xsim);
- `software/scripts/programJtag.tcl`: a volatile JTAG load, which leaves the flash alone.

The RoCEv2 engine, the UDP engine and the register link are SURF's, unmodified.

## Address map

| address | block | from |
| --- | --- | --- |
| `0x0000_0000` | `AxiVersion`: firmware version, git hash, build time, FPGA DNA | SURF |
| `0x0011_0000` | `UdpEngine`: where packets go (`ClientRemoteIp`, `ClientRemotePort` = 4791), ECN and DSCP | SURF |
| `0x0012_0000` | `RssiCore`: the register link (UDP port 8192) | SURF |
| `0x0015_0000` | RoCEv2 `Engine`: queue-pair set-up over the metadata bus, `SoftReset` at `+0xF50` | SURF |
| `0x0015_1000` | RoCEv2 `Dcqcn`: congestion control, `DcqcnBypass` | SURF |
| `0x0015_2000` | RoCEv2 `Core`, the dispatch block: `DispatchEnable` `+0x00`, `MaxSize` `+0x04`, `LKey` `+0x0C`, `SQpn` `+0x10`, counters from `+0x100` | SURF |
| `0x8002_0000` | `SsiPrbsTx`: the example's PRBS source | SLAC example |
| `0x8003_0000` | `RdmaAxisMon`: rate monitor on the RDMA stream | SLAC example |
| `0x8004_0000` | `FebPacketGen`: the FEB packet generator | ours |

## The FEB packet generator's registers

Byte offsets from `0x8004_0000`. The block has two halves:

- `0x000-0x0FF` is **what the ICD asks of a FEB**: the run parameters the host writes, the
  versions it checks, and the counters it reads. This is the part proposed as a starting
  point for the FEB register map.
- `0x100-0x1FF` exists **only in this emulator**. It stands in for what a real FEB gets from
  elsewhere: triggers from the CTP, time from White Rabbit, pedestals and pixel status from
  its front-end.

### What the ICD asks of a FEB

| offset | name | access | reset | |
| --- | --- | --- | --- | --- |
| `0x000` | `MapVersion` | RO | 1 | layout of this block; changes whenever it does |
| `0x004` | `FormatVersion` | RO | 1 | `format_version` of the packets sent |
| `0x008` | `TelId` | RW, 8 bits | 0 | `tel_id`, header byte 1 |
| `0x00C` | `BoardId` | RW, 8 bits | 0 | `board_id`, header byte 2 |
| `0x010` | `NumSamples` | RW, 8 bits | 40 | `num_samples`, 1-255 (0 sends 1); see the size limit below |
| `0x014` | | | | reserved, for instance for `packet_type` once the neighbours scenario is settled |
| `0x018` | `CountReset` | W1 | | zero the three counters: the next packet is `tel_event_id` 0 |
| `0x040` | `TrigCount` | RO | 0 | triggers received, sent or dropped |
| `0x044` | `PacketCount` | RO | 0 | packets sent |
| `0x048` | `DropCount` | RO | 0 | triggers not sent, because a packet was still going out |

`tel_event_id` is the trigger's own number, `TrigCount` before it counts that trigger. A
dropped trigger therefore consumes its number, and the receiver sees the gap a real FEB's lost
trigger would leave, since a real FEB's `tel_event_id` comes from the CTPB. The counters are
32 bits here; `tel_event_id` is that count zero-extended to the header's 64 bits.

### Emulator only

| offset | name | access | reset | |
| --- | --- | --- | --- | --- |
| `0x100` | `TrigEnable` (bit 0), `PatternOffset` (bit 1), `CorruptCrc` (bit 2) | RW | 0 | periodic triggers; sample ramp offset by `tel_event_id`; inverted CRCs |
| `0x104` | `TrigPeriod` | RW | 3906 | clock cycles (156.25 MHz) between periodic triggers: 40 kHz |
| `0x108` | `SoftTrig` | W1 | | one trigger |
| `0x10C` | `TriggerType` (bits 7-0), `EventType` (bits 15-8) | RW | 0 | header bytes 3 and 4; a real FEB takes them from the reading command |
| `0x110` | `Pedestal` | RW, 16 bits | 0 | `pedestal_feb`, the same value for every channel |
| `0x114` | `PixelStatus` | RW, 8 bits | 0 | `pixel_status_feb`, the same value for every channel |
| `0x118` | `TimeSetSec` | RW | 0 | TAI seconds and |
| `0x11C` | `TimeSetQns` | RW | 0 | quarter-nanoseconds that |
| `0x120` | `TimeLoad` | W1 | | copies into the event-time counter |
| `0x140` | `TimeSec` | RO | | event-time counter, TAI seconds |
| `0x144` | `TimeQns` | RO | | and quarter-nanoseconds |
| `0x148` | `PacketBytes` | RO | | bytes per packet at `NumSamples`, CRC included |

Sample `i` of a packet (`i = channel * num_samples + sample`) is `(i + offset) mod 4096`, the
offset being 0 or, with `PatternOffset`, `tel_event_id mod 4096`, so a receiver can check every
sample.

## Bringing a FEB up

What the host does, in order, following SLAC's own bring-up (`_Root.py`,
`software/scripts/rocev2PrbsTest.py`):

1. **Queue pair.** `Engine.setupConnection()` pulses `SoftReset`, then allocates a protection
   domain and a memory region, creates the queue pair and takes it to RTS, all over the
   metadata bus, rolling back whatever it allocated if a step fails. It returns the FEB's
   queue pair number and `LKey`; the host completes its own queue pair with them.
2. **Destination.** `UdpEngine.ClientRemoteIp` and `ClientRemotePort` (4791) point the FEB at
   the host's NIC; `EcnFlag`, `Dscp` and `DcqcnBypass` set the congestion-control posture
   (off on a point-to-point link).
3. **Dispatch.** The dispatch block takes `LKey` and the host's queue pair number (`SQpn`).
   The host checks that `MaxSize` fits its receive buffers.
4. **Run parameters.** The host writes `TelId`, `BoardId` and `NumSamples`, checks
   `FormatVersion` and `MapVersion`, and pulses `CountReset`.
5. **Start.** `DispatchEnable` on. From then on, every trigger produces a packet. In the
   emulator, triggers come from `TrigEnable` and `TrigPeriod`, or `SoftTrig`.

To stop, `DispatchEnable` off, then `Engine.teardownConnection()` and `SoftReset`.

## Open points for the register map

- **Start and stop.** Sending is gated by the RoCEv2 engine's `DispatchEnable`, so the FEB
  block has no run-enable register of its own. A real FEB may want one, separate from the
  engine: accept triggers, or not.
- **Packet size.** SURF's engine sends each packet as a single RDMA SEND of at most one MTU
  (`MaxSize`, 4096 bytes) and drops a larger one, counted in the dispatch block's
  `OversizeCount`. The ICD lets a packet span several MTUs. Here, `num_samples` above 53
  (4,076 bytes) is therefore dropped.
- **Hardware identity.** `AxiVersion` already exposes the FPGA's DNA, a hardware identifier,
  which bears on the ICD's open question of `board_id` as a position or a hardware id.
- **Per-trigger fields.** `trigger_type` and `event_type` come from the CTP with each trigger
  on a real FEB, and `num_samples` may depend on them (ICD open points on `event_type` and the
  `num_samples` maximum).
- **Per-channel fields.** `pedestal_feb` and `pixel_status_feb` are one value for all 49
  channels here; a real FEB computes them per channel.
- **Counter width.** 32-bit counters, against a 64-bit `tel_event_id`.

## Building and programming

Vivado 2024.2, with the submodules checked out in full (ruckus reads their version tags) and
Git LFS (it holds SURF's netlists):

```bash
source /tools/Xilinx/Vivado/2024.2/settings64.sh
cd firmware/targets/RoCEv2_10GbeRudpKcu105Example
make
```

The images land in `images/`: a `.bit` and the `_primary`/`_secondary` `.mcs` pair for the
flash. `software/scripts/programJtag.tcl` loads a `.bit` over JTAG without touching the flash;
`software/scripts/updateBootProm.py` writes the `.mcs` pair to the flash, so the board boots
into it.

The board answers on 192.168.2.10, the example's fixed address.

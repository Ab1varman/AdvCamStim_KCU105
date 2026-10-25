-------------------------------------------------------------------------------
-- Company    : HEPIA
-------------------------------------------------------------------------------
-- Description: AdvCam FEB packet generator
--
-- Emits one FEB data packet per trigger, in the layout of the AdvCam FEB ICD v0.3
-- (no-neighbours scenario, format version 1). Multi-byte fields are big-endian:
--
--   byte    0     format_version (0x01)
--   byte    1     tel_id
--   byte    2     board_id
--   byte    3     trigger_type
--   byte    4     event_type
--   byte    5     num_samples
--   bytes 6-7     reserved
--   bytes 8-15    event time: TAI seconds, then quarter-nanoseconds
--   bytes 16-23   tel_event_id: the number of this packet's trigger (TrigCount before
--                 it counts that trigger), zero-extended to 64 bits. The merge key. A
--                 dropped trigger (below) consumes its number, so the receiver sees the
--                 gap a real FEB's lost trigger would leave.
--   bytes 24-121  pedestal_feb: the Pedestal register, repeated for all 49 channels
--   bytes 122-170 pixel_status_feb: the PixelStatus register, repeated for all 49
--                 channels
--   bytes 171-175 reserved (padding to a 16-byte boundary)
--   bytes 176-    49 x num_samples 12-bit samples, channel-major, packed MSB
--                 first, zero-padded to a multiple of 32 bits
--   last 4        CRC-32 (IEEE 802.3, as zlib) of every byte before it
--
-- pedestal_feb and pixel_status_feb are each one host-set value, not a real
-- per-channel computation: the ICD itself still leaves their per-channel meaning
-- open (averaging window and u12.4 format for the former, bit layout for the
-- latter). A single replicated register is enough to exercise the field on the
-- wire; widen it to one register per channel if per-channel testing is needed.
--
-- Sample i of a packet (i = channel * num_samples + sample) is
-- (i + offset) mod 4096, offset being 0 or the trigger count, so a receiver
-- can check every sample. Triggers come from a period register or a software
-- strobe; a trigger arriving while a packet is still going out is counted as
-- dropped, as a FEB with a full buffer would.
--
-- Registers, by byte offset. 0x000-0x0FF is what the FEB ICD asks of a FEB, and
-- is meant as a starting point for its register map. 0x100-0x1FF stands in for
-- what a real FEB gets from elsewhere -- triggers from the CTP, time from White
-- Rabbit, pedestals and pixel status from its front-end -- and exists only in
-- this emulator. Sending is started and stopped by the RoCEv2 engine's
-- DispatchEnable, not here. MAP_VERSION_C changes whenever this layout does.
--
--   0x000  RO  MapVersion
--   0x004  RO  FormatVersion   format_version of the packets sent
--   0x008  RW  TelId           tel_id, header byte 1
--   0x00C  RW  BoardId         board_id, header byte 2
--   0x010  RW  NumSamples      num_samples, 1-255 (0 sends 1)
--   0x018  W1  CountReset      zero the counters, so tel_event_id restarts at 0
--   0x040  RO  TrigCount       triggers received, sent or dropped
--   0x044  RO  PacketCount     packets sent
--   0x048  RO  DropCount       triggers dropped: a packet was still going out
--
--   0x100  RW  TrigEnable (bit 0), PatternOffset (bit 1), CorruptCrc (bit 2)
--   0x104  RW  TrigPeriod      clock cycles between periodic triggers
--   0x108  W1  SoftTrig        one trigger
--   0x10C  RW  TriggerType (bits 7-0), EventType (bits 15-8), for the header
--   0x110  RW  Pedestal        pedestal_feb, for every channel
--   0x114  RW  PixelStatus     pixel_status_feb, for every channel
--   0x118  RW  TimeSetSec      \
--   0x11C  RW  TimeSetQns       > TimeLoad copies them into the event-time counter
--   0x120  W1  TimeLoad        /
--   0x140  RO  TimeSec         event-time counter, TAI seconds
--   0x144  RO  TimeQns         and quarter-nanoseconds
--   0x148  RO  PacketBytes     bytes per packet at NumSamples, CRC included
-------------------------------------------------------------------------------
-- This file is part of 'Simple-10GbE-RUDP-KCU105-Example'.
-- It is subject to the license terms in the LICENSE.txt file found in the
-- top-level directory of this distribution and at:
--    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
-- No part of 'Simple-10GbE-RUDP-KCU105-Example', including this file,
-- may be copied, modified, propagated, or distributed except according to
-- the terms contained in the LICENSE.txt file.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

library surf;
use surf.StdRtlPkg.all;
use surf.AxiStreamPkg.all;
use surf.AxiLitePkg.all;
use surf.SsiPkg.all;

entity FebPacketGen is
   generic (
      TPD_G         : time     := 1 ns;
      AXIS_CONFIG_G : AxiStreamConfigType;  -- 8-byte stream
      -- Event-time tick: TICK_CYCLES_G clock cycles are TICK_QNS_G quarter-nanoseconds
      -- (5 cycles of 156.25 MHz = 32 ns = 128 qns, exactly).
      TICK_CYCLES_G : positive := 5;
      TICK_QNS_G    : positive := 128);
   port (
      -- Clock and Reset
      axilClk         : in  sl;
      axilRst         : in  sl;
      -- AXI-Stream Interface
      txMaster        : out AxiStreamMasterType;
      txSlave         : in  AxiStreamSlaveType;
      -- AXI-Lite Interface
      axilReadMaster  : in  AxiLiteReadMasterType;
      axilReadSlave   : out AxiLiteReadSlaveType;
      axilWriteMaster : in  AxiLiteWriteMasterType;
      axilWriteSlave  : out AxiLiteWriteSlaveType);
end FebPacketGen;

architecture rtl of FebPacketGen is

   constant MAP_VERSION_C    : slv(31 downto 0) := x"00000001";
   constant FORMAT_VERSION_C : slv(7 downto 0)  := x"01";
   constant QNS_PER_SEC_C    : slv(31 downto 0) := x"EE6B2800";  -- 4e9
   constant CRC_POLY_C       : slv(31 downto 0) := x"EDB88320";  -- 0x04C11DB7 reflected

   -- CRC-32 as zlib computes it (reflected, init and final XOR all ones), folding the
   -- first nBytes lanes of data in wire order, lane 0 first.
   function crc32Update (crc : slv(31 downto 0); data : slv(63 downto 0); nBytes : natural) return slv is
      variable c : slv(31 downto 0) := crc;
   begin
      for l in 0 to 7 loop
         if l < nBytes then
            c := c xor (x"000000" & data(8*l+7 downto 8*l));
            for b in 0 to 7 loop
               if c(0) = '1' then
                  c := ('0' & c(31 downto 1)) xor CRC_POLY_C;
               else
                  c := '0' & c(31 downto 1);
               end if;
            end loop;
         end if;
      end loop;
      return c;
   end function;

   -- Big-endian onto the stream: lane 0 carries the most significant byte.
   function toLanes (value : slv) return slv is
      constant W   : natural                := value'length;
      variable x   : slv(W-1 downto 0)      := value;
      variable ret : slv(W-1 downto 0);
   begin
      for l in 0 to W/8-1 loop
         ret(8*l+7 downto 8*l) := x(W-1-8*l downto W-8-8*l);
      end loop;
      return ret;
   end function;

   -- Beat k (0..2) of a 16-sample group: 16 x 12 bits = 24 bytes, first sample's MSB first.
   function groupBeat (samples : slv(191 downto 0); k : natural) return slv is
      variable ret : slv(63 downto 0);
   begin
      for l in 0 to 7 loop
         ret(8*l+7 downto 8*l) := samples(191-8*(8*k+l) downto 184-8*(8*k+l));
      end loop;
      return ret;
   end function;

   -- One 8-byte beat of the 176-byte header (offsets 0-175, lane 0 first on the wire).
   -- pedestal_feb and pixel_status_feb are each one value tiled across their range, so
   -- beats 3-20 are just that value repeated; only beat 15 (where the two fields meet)
   -- and beat 21 (where the reserved pad begins) mix two values in one beat.
   function headerBeatWord (
      beat           : natural range 0 to 21;
      pktHeader      : slv(31 downto 0);  -- tel_id, board_id, trigger_type, event_type
      pktSamples     : slv(7 downto 0);
      pktSec         : slv(31 downto 0);
      pktQns         : slv(31 downto 0);
      pktEventId     : slv(63 downto 0);
      pktPedestal    : slv(15 downto 0);
      pktPixelStatus : slv(7 downto 0)) return slv is
      variable w : slv(63 downto 0);
   begin
      case beat is
         when 0 =>
            w(7 downto 0)   := FORMAT_VERSION_C;
            w(15 downto 8)  := pktHeader(7 downto 0);    -- tel_id
            w(23 downto 16) := pktHeader(15 downto 8);   -- board_id
            w(31 downto 24) := pktHeader(23 downto 16);  -- trigger_type
            w(39 downto 32) := pktHeader(31 downto 24);  -- event_type
            w(47 downto 40) := pktSamples;
            w(63 downto 48) := (others => '0');          -- reserved
         when 1 =>
            w := toLanes(pktSec & pktQns);
         when 2 =>
            w := toLanes(pktEventId);
         when 3 to 14 =>
            w := toLanes(pktPedestal & pktPedestal & pktPedestal & pktPedestal);
         when 15 =>
            w := toLanes(pktPedestal & pktPixelStatus & pktPixelStatus & pktPixelStatus
                          & pktPixelStatus & pktPixelStatus & pktPixelStatus);
         when 16 to 20 =>
            w := toLanes(pktPixelStatus & pktPixelStatus & pktPixelStatus & pktPixelStatus
                          & pktPixelStatus & pktPixelStatus & pktPixelStatus & pktPixelStatus);
         when others =>  -- beat 21: 3 bytes of pixel_status_feb, then the reserved pad
            w := toLanes(pktPixelStatus & pktPixelStatus & pktPixelStatus
                          & x"00" & x"00" & x"00" & x"00" & x"00");
      end case;
      return w;
   end function;

   type StateType is (
      IDLE_S,
      HDR_S,
      PAYLOAD_S,
      CRC_S);

   type RegType is record
      -- Run parameters (ICD)
      telId          : slv(7 downto 0);
      boardId        : slv(7 downto 0);
      numSamples     : slv(7 downto 0);
      -- Emulator controls
      trigEnable     : sl;
      patternOffset  : sl;
      corruptCrc     : sl;
      trigPeriod     : slv(31 downto 0);
      triggerType    : slv(7 downto 0);
      eventType      : slv(7 downto 0);
      pedestal       : slv(15 downto 0);
      pixelStatus    : slv(7 downto 0);
      timeSetSec     : slv(31 downto 0);
      timeSetQns     : slv(31 downto 0);
      timeLoad       : sl;
      softTrig       : sl;
      cntRst         : sl;
      -- Derived from numSamples
      numTotal       : slv(15 downto 0);  -- 49 x num_samples
      payloadBytes   : slv(15 downto 0);  -- samples + padding
      -- Event time
      tickCnt        : natural range 0 to TICK_CYCLES_G-1;
      timeSec        : slv(31 downto 0);
      timeQns        : slv(31 downto 0);
      -- Triggering
      trigTimer      : slv(31 downto 0);
      trigCnt        : slv(31 downto 0);
      pktCnt         : slv(31 downto 0);
      dropCnt        : slv(31 downto 0);
      -- Packet being sent, latched at its trigger
      pktHeader      : slv(31 downto 0);
      pktSamples     : slv(7 downto 0);
      pktEventId     : slv(63 downto 0);
      pktPedestal    : slv(15 downto 0);
      pktPixelStatus : slv(7 downto 0);
      pktSec         : slv(31 downto 0);
      pktQns         : slv(31 downto 0);
      pktTotal       : slv(15 downto 0);
      pktPayload     : slv(15 downto 0);
      pktCorrupt     : sl;
      sentBytes      : slv(15 downto 0);  -- payload bytes sent so far
      sampleBase     : slv(15 downto 0);  -- index of the current group's first sample
      valueBase      : slv(11 downto 0);  -- and its value
      beat           : natural range 0 to 2;   -- beat within the 16-sample group
      hdrBeat        : natural range 0 to 21;  -- beat within the 176-byte header
      crc            : slv(31 downto 0);
      state          : StateType;
      txMaster       : AxiStreamMasterType;
      axilReadSlave  : AxiLiteReadSlaveType;
      axilWriteSlave : AxiLiteWriteSlaveType;
   end record RegType;

   constant REG_INIT_C : RegType := (
      telId          => (others => '0'),
      boardId        => (others => '0'),
      numSamples     => toSlv(40, 8),
      trigEnable     => '0',
      patternOffset  => '0',
      corruptCrc     => '0',
      trigPeriod     => toSlv(3906, 32),  -- 40 kHz at 156.25 MHz
      triggerType    => (others => '0'),
      eventType      => (others => '0'),
      pedestal       => (others => '0'),
      pixelStatus    => (others => '0'),
      timeSetSec     => (others => '0'),
      timeSetQns     => (others => '0'),
      timeLoad       => '0',
      softTrig       => '0',
      cntRst         => '0',
      numTotal       => toSlv(49*40, 16),
      payloadBytes   => toSlv(2940, 16),
      tickCnt        => 0,
      timeSec        => (others => '0'),
      timeQns        => (others => '0'),
      trigTimer      => (others => '0'),
      trigCnt        => (others => '0'),
      pktCnt         => (others => '0'),
      dropCnt        => (others => '0'),
      pktHeader      => (others => '0'),
      pktSamples     => (others => '0'),
      pktEventId     => (others => '0'),
      pktPedestal    => (others => '0'),
      pktPixelStatus => (others => '0'),
      pktSec         => (others => '0'),
      pktQns         => (others => '0'),
      pktTotal       => (others => '0'),
      pktPayload     => (others => '0'),
      pktCorrupt     => '0',
      sentBytes      => (others => '0'),
      sampleBase     => (others => '0'),
      valueBase      => (others => '0'),
      beat           => 0,
      hdrBeat        => 0,
      crc            => (others => '1'),
      state          => IDLE_S,
      txMaster       => axiStreamMasterInit(AXIS_CONFIG_G),
      axilReadSlave  => AXI_LITE_READ_SLAVE_INIT_C,
      axilWriteSlave => AXI_LITE_WRITE_SLAVE_INIT_C);

   signal r   : RegType := REG_INIT_C;
   signal rin : RegType;

begin

   assert (AXIS_CONFIG_G.TDATA_BYTES_C = 8)
      report "FebPacketGen: AXIS_CONFIG_G must be an 8-byte stream" severity failure;

   comb : process (axilReadMaster, axilRst, axilWriteMaster, r, txSlave) is
      variable v         : RegType;
      variable axilEp    : AxiLiteEndPointType;
      variable fire      : sl;
      variable ns        : natural range 1 to 255;
      variable gv        : slv(191 downto 0);
      variable d         : slv(63 downto 0);
      variable remaining : slv(15 downto 0);
      variable c         : slv(31 downto 0);
   begin
      -- Latch the current value
      v := r;

      -- Write strobes last one cycle
      v.timeLoad := '0';
      v.softTrig := '0';
      v.cntRst   := '0';

      ----------------------------------------------------------------------
      --                AXI-Lite Register Logic
      ----------------------------------------------------------------------

      axiSlaveWaitTxn(axilEp, axilWriteMaster, axilReadMaster, v.axilWriteSlave, v.axilReadSlave);

      -- What the ICD asks of a FEB
      axiSlaveRegisterR(axilEp, x"000", 0, MAP_VERSION_C);
      axiSlaveRegisterR(axilEp, x"004", 0, FORMAT_VERSION_C);
      axiSlaveRegister (axilEp, x"008", 0, v.telId);
      axiSlaveRegister (axilEp, x"00C", 0, v.boardId);
      axiSlaveRegister (axilEp, x"010", 0, v.numSamples);
      axiSlaveRegister (axilEp, x"018", 0, v.cntRst);
      axiSlaveRegisterR(axilEp, x"040", 0, r.trigCnt);
      axiSlaveRegisterR(axilEp, x"044", 0, r.pktCnt);
      axiSlaveRegisterR(axilEp, x"048", 0, r.dropCnt);

      -- Emulator only
      axiSlaveRegister (axilEp, x"100", 0, v.trigEnable);
      axiSlaveRegister (axilEp, x"100", 1, v.patternOffset);
      axiSlaveRegister (axilEp, x"100", 2, v.corruptCrc);
      axiSlaveRegister (axilEp, x"104", 0, v.trigPeriod);
      axiSlaveRegister (axilEp, x"108", 0, v.softTrig);
      axiSlaveRegister (axilEp, x"10C", 0, v.triggerType);
      axiSlaveRegister (axilEp, x"10C", 8, v.eventType);
      axiSlaveRegister (axilEp, x"110", 0, v.pedestal);
      axiSlaveRegister (axilEp, x"114", 0, v.pixelStatus);
      axiSlaveRegister (axilEp, x"118", 0, v.timeSetSec);
      axiSlaveRegister (axilEp, x"11C", 0, v.timeSetQns);
      axiSlaveRegister (axilEp, x"120", 0, v.timeLoad);
      axiSlaveRegisterR(axilEp, x"140", 0, r.timeSec);
      axiSlaveRegisterR(axilEp, x"144", 0, r.timeQns);
      axiSlaveRegisterR(axilEp, x"148", 0, r.payloadBytes + 180);  -- whole packet

      axiSlaveDefault(axilEp, v.axilWriteSlave, v.axilReadSlave, AXI_RESP_DECERR_C);

      ----------------------------------------------------------------------
      --                Packet size, from the configured sample count
      ----------------------------------------------------------------------

      ns := 1;
      if r.numSamples /= 0 then
         ns := conv_integer(r.numSamples);
      end if;
      v.numTotal     := toSlv(49*ns, 16);
      v.payloadBytes := toSlv(4*((147*ns + 7)/8), 16);  -- 12 x 49 x ns bits, in 32-bit words

      ----------------------------------------------------------------------
      --                Event time
      ----------------------------------------------------------------------

      if r.tickCnt = TICK_CYCLES_G-1 then
         v.tickCnt := 0;
         if r.timeQns >= QNS_PER_SEC_C - TICK_QNS_G then
            v.timeQns := r.timeQns + TICK_QNS_G - QNS_PER_SEC_C;
            v.timeSec := r.timeSec + 1;
         else
            v.timeQns := r.timeQns + TICK_QNS_G;
         end if;
      else
         v.tickCnt := r.tickCnt + 1;
      end if;

      if r.timeLoad = '1' then
         v.tickCnt := 0;
         v.timeSec := r.timeSetSec;
         v.timeQns := r.timeSetQns;
      end if;

      ----------------------------------------------------------------------
      --                Triggers
      ----------------------------------------------------------------------

      fire := r.softTrig;
      if (r.trigEnable = '1') and (r.trigPeriod /= 0) then
         if r.trigTimer >= r.trigPeriod - 1 then
            v.trigTimer := (others => '0');
            fire        := '1';
         else
            v.trigTimer := r.trigTimer + 1;
         end if;
      else
         v.trigTimer := (others => '0');
      end if;

      if fire = '1' then
         v.trigCnt := r.trigCnt + 1;
         if r.state = IDLE_S then
            v.pktHeader      := r.eventType & r.triggerType & r.boardId & r.telId;
            v.pktSamples     := toSlv(ns, 8);
            v.pktEventId     := x"00000000" & r.trigCnt;  -- this trigger's number
            v.pktPedestal    := r.pedestal;
            v.pktPixelStatus := r.pixelStatus;
            v.pktSec         := r.timeSec;
            v.pktQns         := r.timeQns;
            v.pktTotal       := r.numTotal;
            v.pktPayload     := r.payloadBytes;
            v.pktCorrupt     := r.corruptCrc;
            v.sentBytes      := (others => '0');
            v.sampleBase     := (others => '0');
            v.valueBase      := (others => '0');
            if r.patternOffset = '1' then
               v.valueBase := r.trigCnt(11 downto 0);
            end if;
            v.beat    := 0;
            v.hdrBeat := 0;
            v.crc     := (others => '1');
            v.state   := HDR_S;
         else
            v.dropCnt := r.dropCnt + 1;
         end if;
      end if;

      if r.cntRst = '1' then
         v.trigCnt := (others => '0');
         v.pktCnt  := (others => '0');
         v.dropCnt := (others => '0');
      end if;

      ----------------------------------------------------------------------
      --                AXI Stream TX Logic
      ----------------------------------------------------------------------

      -- The current 16-sample group; samples past the last one are zero, which is the
      -- padding, and bytes past the payload are never sent.
      for j in 0 to 15 loop
         if (r.sampleBase + j) < r.pktTotal then
            gv(191-12*j downto 180-12*j) := r.valueBase + j;
         else
            gv(191-12*j downto 180-12*j) := (others => '0');
         end if;
      end loop;

      d := (others => '0');

      if (txSlave.tReady = '1') then
         v.txMaster := axiStreamMasterInit(AXIS_CONFIG_G);
      end if;

      case r.state is
         ----------------------------------------------------------------------
         when IDLE_S =>
            null;  -- left by the trigger logic above
         ----------------------------------------------------------------------
         when HDR_S =>
            if (v.txMaster.tValid = '0') then
               d := headerBeatWord(r.hdrBeat, r.pktHeader, r.pktSamples, r.pktSec, r.pktQns,
                                    r.pktEventId, r.pktPedestal, r.pktPixelStatus);
               v.txMaster.tValid             := '1';
               v.txMaster.tData(63 downto 0) := d;
               v.txMaster.tKeep              := (others => '0');
               v.txMaster.tKeep(7 downto 0)  := x"FF";
               if r.hdrBeat = 0 then
                  ssiSetUserSof(AXIS_CONFIG_G, v.txMaster, '1');
               end if;
               v.crc := crc32Update(r.crc, d, 8);
               if r.hdrBeat = 21 then
                  v.state := PAYLOAD_S;
               else
                  v.hdrBeat := r.hdrBeat + 1;
               end if;
            end if;
         ----------------------------------------------------------------------
         when PAYLOAD_S =>
            if (v.txMaster.tValid = '0') then
               case r.beat is
                  when 0      => d := groupBeat(gv, 0);
                  when 1      => d := groupBeat(gv, 1);
                  when others => d := groupBeat(gv, 2);
               end case;
               remaining := r.pktPayload - r.sentBytes;
               v.txMaster.tValid            := '1';
               v.txMaster.tKeep             := (others => '0');
               v.txMaster.tKeep(7 downto 0) := x"FF";
               if remaining = 4 then
                  -- The last four payload bytes and the CRC share this beat
                  c := not crc32Update(r.crc, d, 4);
                  if r.pktCorrupt = '1' then
                     c := not c;
                  end if;
                  d(63 downto 32)   := toLanes(c);
                  v.txMaster.tLast  := '1';
                  v.pktCnt          := r.pktCnt + 1;
                  v.state           := IDLE_S;
               else
                  v.crc       := crc32Update(r.crc, d, 8);
                  v.sentBytes := r.sentBytes + 8;
                  if remaining = 8 then
                     v.state := CRC_S;
                  end if;
                  if r.beat = 2 then
                     v.beat       := 0;
                     v.sampleBase := r.sampleBase + 16;
                     v.valueBase  := r.valueBase + 16;
                  else
                     v.beat := r.beat + 1;
                  end if;
               end if;
               v.txMaster.tData(63 downto 0) := d;
            end if;
         ----------------------------------------------------------------------
         when CRC_S =>
            if (v.txMaster.tValid = '0') then
               c := not r.crc;
               if r.pktCorrupt = '1' then
                  c := not c;
               end if;
               d(31 downto 0)                := toLanes(c);
               v.txMaster.tValid             := '1';
               v.txMaster.tData(63 downto 0) := d;
               v.txMaster.tKeep              := (others => '0');
               v.txMaster.tKeep(3 downto 0)  := x"F";
               v.txMaster.tLast              := '1';
               v.pktCnt := r.pktCnt + 1;
               v.state  := IDLE_S;
            end if;
      ----------------------------------------------------------------------
      end case;

      ----------------------------------------------------------------------

      -- Outputs
      axilWriteSlave <= r.axilWriteSlave;
      axilReadSlave  <= r.axilReadSlave;
      txMaster       <= r.txMaster;

      -- Reset
      if (axilRst = '1') then
         v := REG_INIT_C;
      end if;

      -- Register the variable for next clock cycle
      rin <= v;

   end process comb;

   seq : process (axilClk) is
   begin
      if rising_edge(axilClk) then
         r <= rin after TPD_G;
      end if;
   end process seq;

end rtl;

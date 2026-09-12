-------------------------------------------------------------------------------
-- Company    : HEPIA
-------------------------------------------------------------------------------
-- Description: AdvCam FEB packet generator
--
-- Emits one FEB data packet per trigger, in the layout of the proposed AdvCam
-- FEB ICD (no-neighbours scenario, format version 1). Multi-byte fields are
-- big-endian:
--
--   bytes  0-7   tel_id, board_id, packet_type (0x01), trigger_type,
--                event_type, num_samples, format_version (0x01), reserved
--   bytes  8-15  event time: TAI seconds, then quarter-nanoseconds
--   bytes 16-23  pixel_status_feb
--   bytes 24-    49 x num_samples 12-bit samples, channel-major, packed MSB
--                first, zero-padded to a multiple of 32 bits
--   last 4       CRC-32 (IEEE 802.3, as zlib) of every byte before it
--
-- Sample i of a packet (i = channel * num_samples + sample) is
-- (i + offset) mod 4096, offset being 0 or the trigger count, so a receiver
-- can check every sample. Triggers come from a period register or a software
-- strobe; a trigger arriving while a packet is still going out is counted as
-- dropped, as a FEB with a full buffer would.
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

   constant PACKET_TYPE_C    : slv(7 downto 0)  := x"01";
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

   type StateType is (
      IDLE_S,
      HDR0_S,
      HDR1_S,
      HDR2_S,
      PAYLOAD_S,
      CRC_S);

   type RegType is record
      -- Configuration
      enable         : sl;
      patternOffset  : sl;
      corruptCrc     : sl;
      trigPeriod     : slv(31 downto 0);
      numSamples     : slv(7 downto 0);
      header         : slv(31 downto 0);  -- tel_id, board_id, trigger_type, event_type
      pixelLo        : slv(31 downto 0);
      pixelHi        : slv(31 downto 0);
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
      pktPixel       : slv(63 downto 0);
      pktSec         : slv(31 downto 0);
      pktQns         : slv(31 downto 0);
      pktTotal       : slv(15 downto 0);
      pktPayload     : slv(15 downto 0);
      pktCorrupt     : sl;
      sentBytes      : slv(15 downto 0);  -- payload bytes sent so far
      sampleBase     : slv(15 downto 0);  -- index of the current group's first sample
      valueBase      : slv(11 downto 0);  -- and its value
      beat           : natural range 0 to 2;  -- beat within the 16-sample group
      crc            : slv(31 downto 0);
      state          : StateType;
      txMaster       : AxiStreamMasterType;
      axilReadSlave  : AxiLiteReadSlaveType;
      axilWriteSlave : AxiLiteWriteSlaveType;
   end record RegType;

   constant REG_INIT_C : RegType := (
      enable         => '0',
      patternOffset  => '0',
      corruptCrc     => '0',
      trigPeriod     => toSlv(3906, 32),  -- 40 kHz at 156.25 MHz
      numSamples     => toSlv(40, 8),
      header         => (others => '0'),
      pixelLo        => (others => '0'),
      pixelHi        => (others => '0'),
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
      pktPixel       => (others => '0'),
      pktSec         => (others => '0'),
      pktQns         => (others => '0'),
      pktTotal       => (others => '0'),
      pktPayload     => (others => '0'),
      pktCorrupt     => '0',
      sentBytes      => (others => '0'),
      sampleBase     => (others => '0'),
      valueBase      => (others => '0'),
      beat           => 0,
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

      axiSlaveRegister (axilEp, x"000", 0, v.enable);
      axiSlaveRegister (axilEp, x"000", 1, v.patternOffset);
      axiSlaveRegister (axilEp, x"000", 2, v.corruptCrc);
      axiSlaveRegister (axilEp, x"004", 0, v.trigPeriod);
      axiSlaveRegister (axilEp, x"008", 0, v.numSamples);
      axiSlaveRegister (axilEp, x"00C", 0, v.header);
      axiSlaveRegister (axilEp, x"010", 0, v.pixelLo);
      axiSlaveRegister (axilEp, x"014", 0, v.pixelHi);
      axiSlaveRegister (axilEp, x"018", 0, v.timeSetSec);
      axiSlaveRegister (axilEp, x"01C", 0, v.timeSetQns);
      axiSlaveRegister (axilEp, x"020", 0, v.timeLoad);
      axiSlaveRegister (axilEp, x"024", 0, v.softTrig);
      axiSlaveRegister (axilEp, x"028", 0, v.cntRst);

      axiSlaveRegisterR(axilEp, x"040", 0, r.trigCnt);
      axiSlaveRegisterR(axilEp, x"044", 0, r.pktCnt);
      axiSlaveRegisterR(axilEp, x"048", 0, r.dropCnt);
      axiSlaveRegisterR(axilEp, x"04C", 0, r.timeSec);
      axiSlaveRegisterR(axilEp, x"050", 0, r.timeQns);
      axiSlaveRegisterR(axilEp, x"054", 0, r.payloadBytes + 28);  -- whole packet
      axiSlaveRegisterR(axilEp, x"058", 0, FORMAT_VERSION_C);

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
      if (r.enable = '1') and (r.trigPeriod /= 0) then
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
            v.pktHeader  := r.header;
            v.pktSamples := toSlv(ns, 8);
            v.pktPixel   := r.pixelHi & r.pixelLo;
            v.pktSec     := r.timeSec;
            v.pktQns     := r.timeQns;
            v.pktTotal   := r.numTotal;
            v.pktPayload := r.payloadBytes;
            v.pktCorrupt := r.corruptCrc;
            v.sentBytes  := (others => '0');
            v.sampleBase := (others => '0');
            v.valueBase  := (others => '0');
            if r.patternOffset = '1' then
               v.valueBase := r.trigCnt(11 downto 0);
            end if;
            v.beat  := 0;
            v.crc   := (others => '1');
            v.state := HDR0_S;
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
         when HDR0_S =>
            if (v.txMaster.tValid = '0') then
               d(7 downto 0)   := r.pktHeader(7 downto 0);    -- tel_id
               d(15 downto 8)  := r.pktHeader(15 downto 8);   -- board_id
               d(23 downto 16) := PACKET_TYPE_C;
               d(31 downto 24) := r.pktHeader(23 downto 16);  -- trigger_type
               d(39 downto 32) := r.pktHeader(31 downto 24);  -- event_type
               d(47 downto 40) := r.pktSamples;
               d(55 downto 48) := FORMAT_VERSION_C;
               v.txMaster.tValid              := '1';
               v.txMaster.tData(63 downto 0)  := d;
               v.txMaster.tKeep               := (others => '0');
               v.txMaster.tKeep(7 downto 0)   := x"FF";
               ssiSetUserSof(AXIS_CONFIG_G, v.txMaster, '1');
               v.crc   := crc32Update(r.crc, d, 8);
               v.state := HDR1_S;
            end if;
         ----------------------------------------------------------------------
         when HDR1_S =>
            if (v.txMaster.tValid = '0') then
               d                              := toLanes(r.pktSec & r.pktQns);
               v.txMaster.tValid              := '1';
               v.txMaster.tData(63 downto 0)  := d;
               v.txMaster.tKeep               := (others => '0');
               v.txMaster.tKeep(7 downto 0)   := x"FF";
               v.crc   := crc32Update(r.crc, d, 8);
               v.state := HDR2_S;
            end if;
         ----------------------------------------------------------------------
         when HDR2_S =>
            if (v.txMaster.tValid = '0') then
               d                              := toLanes(r.pktPixel);
               v.txMaster.tValid              := '1';
               v.txMaster.tData(63 downto 0)  := d;
               v.txMaster.tKeep               := (others => '0');
               v.txMaster.tKeep(7 downto 0)   := x"FF";
               v.crc   := crc32Update(r.crc, d, 8);
               v.state := PAYLOAD_S;
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

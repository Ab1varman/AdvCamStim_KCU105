-------------------------------------------------------------------------------
-- Company    : HEPIA
-------------------------------------------------------------------------------
-- Description: FebPacketGen testbench. Drives the registers over AXI-Lite,
-- sinks the stream under random backpressure and writes every packet to
-- feb_packets.txt, which febPacketCheck.py compares against the ICD model.
--
-- Covers: packets whose CRC shares the last beat (S = 40, 1, 55) and whose CRC
-- gets its own beat (S = 41, 16), a seconds rollover, the offset pattern, CRC
-- corruption, periodic triggers faster than packets drain (drops, which leave
-- tel_event_id gaps), and a counter reset (tel_event_id restarts at 0).
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
use std.textio.all;

library surf;
use surf.StdRtlPkg.all;
use surf.AxiStreamPkg.all;
use surf.AxiLitePkg.all;
use surf.SsiPkg.all;

entity FebPacketGenTb is
end FebPacketGenTb;

architecture tb of FebPacketGenTb is

   constant CLK_PERIOD_C  : time                := 6.4 ns;  -- 156.25 MHz
   constant AXIS_CONFIG_C : AxiStreamConfigType := ssiAxiStreamConfig(dataBytes => 8);

   signal clk     : sl      := '0';
   signal rst     : sl      := '1';
   signal simDone : boolean := false;
   signal lfsr    : slv(15 downto 0) := x"ACE1";
   signal pktSeen : natural := 0;

   signal txMaster : AxiStreamMasterType;
   signal txSlave  : AxiStreamSlaveType;
   signal wMaster  : AxiLiteWriteMasterType := AXI_LITE_WRITE_MASTER_INIT_C;
   signal wSlave   : AxiLiteWriteSlaveType;
   signal rMaster  : AxiLiteReadMasterType  := AXI_LITE_READ_MASTER_INIT_C;
   signal rSlave   : AxiLiteReadSlaveType;

   file outFile : text open write_mode is "feb_packets.txt";

   function hex2 (b : slv(7 downto 0)) return string is
      constant H : string(1 to 16) := "0123456789abcdef";
   begin
      return H(conv_integer(b(7 downto 4))+1) & H(conv_integer(b(3 downto 0))+1);
   end function;

   function hex8 (w : slv(31 downto 0)) return string is
   begin
      return hex2(w(31 downto 24)) & hex2(w(23 downto 16)) & hex2(w(15 downto 8)) & hex2(w(7 downto 0));
   end function;

begin

   clkGen : process is
   begin
      while not simDone loop
         clk <= '0';
         wait for CLK_PERIOD_C/2;
         clk <= '1';
         wait for CLK_PERIOD_C/2;
      end loop;
      wait;
   end process clkGen;

   -- Ready 3 cycles in 4 on average, in a pseudo-random pattern
   backpressure : process (clk) is
   begin
      if rising_edge(clk) then
         lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
      end if;
   end process backpressure;
   txSlave.tReady <= '1' when lfsr(1 downto 0) /= "00" else '0';

   U_Dut : entity work.FebPacketGen
      generic map (
         TPD_G         => 1 ns,
         AXIS_CONFIG_G => AXIS_CONFIG_C)
      port map (
         axilClk         => clk,
         axilRst         => rst,
         txMaster        => txMaster,
         txSlave         => txSlave,
         axilReadMaster  => rMaster,
         axilReadSlave   => rSlave,
         axilWriteMaster => wMaster,
         axilWriteSlave  => wSlave);

   capture : process (clk) is
      variable L       : line;
      variable started : boolean := false;
   begin
      if rising_edge(clk) then
         if (txMaster.tValid = '1') and (txSlave.tReady = '1') then
            if not started then
               write(L, string'("PKT "));
               started := true;
            end if;
            for lane in 0 to 7 loop
               if txMaster.tKeep(lane) = '1' then
                  write(L, hex2(txMaster.tData(8*lane+7 downto 8*lane)));
               end if;
            end loop;
            if txMaster.tLast = '1' then
               writeline(outFile, L);
               started := false;
               pktSeen <= pktSeen + 1;
            end if;
         end if;
      end if;
   end process capture;

   stim : process is
      variable L    : line;
      variable n    : natural := 0;
      variable data : slv(31 downto 0);

      procedure axiWr (addr : in slv(31 downto 0); val : in slv(31 downto 0)) is
      begin
         wait until rising_edge(clk);
         wMaster.awaddr  <= addr;
         wMaster.awvalid <= '1';
         wMaster.wdata   <= val;
         wMaster.wstrb   <= x"F";
         wMaster.wvalid  <= '1';
         wMaster.bready  <= '1';
         loop
            wait until rising_edge(clk);
            exit when wSlave.awready = '1';
         end loop;
         wMaster.awvalid <= '0';
         wMaster.wvalid  <= '0';
         while wSlave.bvalid /= '1' loop
            wait until rising_edge(clk);
         end loop;
         wait until rising_edge(clk);
         wMaster.bready <= '0';
      end procedure;

      procedure axiRd (addr : in slv(31 downto 0); val : out slv(31 downto 0)) is
      begin
         wait until rising_edge(clk);
         rMaster.araddr  <= addr;
         rMaster.arvalid <= '1';
         rMaster.rready  <= '1';
         loop
            wait until rising_edge(clk);
            exit when rSlave.arready = '1';
         end loop;
         rMaster.arvalid <= '0';
         while rSlave.rvalid /= '1' loop
            wait until rising_edge(clk);
         end loop;
         val := rSlave.rdata;
         wait until rising_edge(clk);
         rMaster.rready <= '0';
      end procedure;

      procedure log (s : in string) is
      begin
         write(L, s);
         writeline(outFile, L);
      end procedure;

      procedure logReg (name : in string; addr : in slv(31 downto 0)) is
         variable val : slv(31 downto 0);
      begin
         axiRd(addr, val);
         log("REG " & name & " " & hex8(val));
      end procedure;

      procedure waitPackets (count : in natural) is
      begin
         for i in 0 to 200000 loop
            exit when pktSeen >= count;
            wait until rising_edge(clk);
         end loop;
         assert pktSeen >= count report "timed out waiting for a packet" severity failure;
      end procedure;

      procedure softPacket (samples : in natural; cfg : in string) is
      begin
         axiWr(x"00000008", conv_std_logic_vector(samples, 32));  -- NumSamples
         for i in 0 to 3 loop
            wait until rising_edge(clk);
         end loop;
         log("CFG S=" & integer'image(samples) & " " & cfg);
         axiWr(x"00000024", x"00000001");  -- SoftTrig
         n := n + 1;
         waitPackets(n);
      end procedure;

   begin
      wait for 100 ns;
      rst <= '0';
      wait for 100 ns;

      -- Start 10 us before a seconds rollover
      axiWr(x"00000018", x"6A000000");  -- TimeSetSec
      axiWr(x"0000001C", x"EE6B00F0");  -- TimeSetQns = 4e9 - 10000
      axiWr(x"00000020", x"00000001");  -- TimeLoad
      axiWr(x"0000000C", x"44332211");  -- tel 0x11, board 0x22, trigger 0x33, event 0x44
      axiWr(x"00000010", x"00001234");  -- Pedestal
      axiWr(x"00000014", x"00000056");  -- PixelStatus

      softPacket(40, "offset=0 corrupt=0");
      softPacket(41, "offset=0 corrupt=0");
      softPacket(1, "offset=0 corrupt=0");
      softPacket(55, "offset=0 corrupt=0");
      softPacket(16, "offset=0 corrupt=0");

      axiWr(x"00000000", x"00000002");  -- PatternOffset
      softPacket(40, "offset=1 corrupt=0");
      softPacket(40, "offset=1 corrupt=0");

      axiWr(x"00000000", x"00000004");  -- CorruptCrc
      softPacket(40, "offset=0 corrupt=1");
      axiWr(x"00000000", x"00000000");

      -- Periodic triggers every 300 cycles, faster than a 40-sample packet drains
      axiWr(x"00000004", conv_std_logic_vector(300, 32));  -- TrigPeriod
      log("CFG S=40 offset=0 corrupt=0 periodic");
      axiWr(x"00000000", x"00000001");  -- Enable
      wait for 20 us;
      axiWr(x"00000000", x"00000000");
      wait for 10 us;

      logReg("TrigCount", x"00000040");
      logReg("PacketCount", x"00000044");
      logReg("DropCount", x"00000048");
      logReg("PacketBytes", x"00000054");
      logReg("FormatVersion", x"00000058");

      -- The start of a new run: counters and tel_event_id from 0
      n := pktSeen;  -- the periodic triggers above sent packets of their own
      axiWr(x"00000028", x"00000001");  -- CountReset
      softPacket(40, "offset=0 corrupt=0 reset=1");
      logReg("PacketCountAfterReset", x"00000044");
      log("DONE");

      simDone <= true;
      wait;
   end process stim;

end tb;

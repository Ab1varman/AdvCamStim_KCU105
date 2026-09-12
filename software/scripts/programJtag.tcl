#-----------------------------------------------------------------------------
# This file is part of the 'Simple-10GbE-RUDP-KCU105-Example'. It is subject to
# the license terms in the LICENSE.txt file found in the top-level directory
# of this distribution and at:
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
# No part of the 'Simple-10GbE-RUDP-KCU105-Example', including this file, may be
# copied, modified, propagated, or distributed except according to the terms
# contained in the LICENSE.txt file.
#-----------------------------------------------------------------------------
# Load a bitstream into the KCU105 over JTAG. Volatile only: the flash is not
# touched, so a power cycle brings back whatever image the board boots from.
#
#   vivado -mode batch -nojournal -nolog -source programJtag.tcl -tclargs <file.bit> [cable-serial]

if {[llength $argv] < 1} {
   puts "usage: -tclargs <file.bit> \[cable-serial\]"
   exit 1
}
set bitFile [lindex $argv 0]
set pattern [expr {[llength $argv] > 1 ? "*[lindex $argv 1]*" : "*"}]

open_hw_manager
connect_hw_server
set target [lindex [get_hw_targets $pattern] 0]
if {$target eq ""} {
   puts "no JTAG target matches $pattern"
   exit 1
}
open_hw_target $target

set device [lindex [get_hw_devices xcku040*] 0]
if {$device eq ""} {
   puts "no xcku040 on $target"
   exit 1
}
current_hw_device $device
set_property PROBES.FILE {} $device
set_property PROGRAM.FILE $bitFile $device
program_hw_devices $device
refresh_hw_device $device
puts "programmed $device on $target with $bitFile"

close_hw_target

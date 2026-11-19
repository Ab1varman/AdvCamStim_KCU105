# Simple-10GbE-RUDP-KCU105-Example

Documentation is posted here:

https://slaclab.github.io/Simple-10GbE-RUDP-KCU105-Example/

## AdvCam FEB emulator

This repository adds an emulator of one AdvCam front-end board (FEB) to the RoCEv2 example:
the board sends FEB data packets in the AdvCam FEB ICD format, as RDMA SENDs. What was added,
its register map, how to build it and how to check a board are described in
[docs/advcam_feb_emulator.md](docs/advcam_feb_emulator.md).

Hi!

Here we offer a MAC-Sublayer implementation for networked control system applications. 
It is based on IEEE802.15.4 implementation of Bastian Bloessl.

The IEEE802.15.4 O-QPSK transceiver for GNU Radio, based on Thomas
Schmid's implementation.

### Development

This module uses *maint-3.7* branch for development. This branch is supposed to be used with GNU Radio 3.7 branch.


## Features

- Flexible slotted superframe is created by MAC-Controller block.
- Synchronization is provided by regular beacon packets, which are originated from MAC-Controller block.
- Different queuing strategies are implemented to MAC-Plant block.
- An example control application with inverted pendulum, which consist of 
controller and plant applications can be found in inverted_pendulum_gui file.
- The O-QPSK PHY encapsulated in a hierarchical block.
- A block that implements the Rime communication stack. Rime is a lightweight
  communication stack designed for Wireless Sensor Networks and is part of the
  Contiki Operating System.
- A transceiver flow graph with USRP <-> PHY <-> MAC <-> Network layer (Rime)
  <-> UDP Socket / APP which resembles pretty well the ISO/OSI structure.
- An IEEE 802.15.4 and Rime dissector for Wireshark.

Some interesting properties:
- Packets can be piped to Wireshark.
- The complete physical modulation is done with plain GNU Radio blocks.
- It uses a block to tag packet bursts with `tx_sob` and `tx_eob` tags. This
  tags are understood by the UHD blocks and allow fast switching between
  transmission and reception.

## Dependencies

- GNU Radio

- gr-foo (Wireshark Connector, Packet Pad and Burst Tagger blocks) <br>
  https://github.com/bastibl/gr-foo.git

- python-matplotlib (if you want to run the GUI sample application) <br>
  `sudo apt-get install python-matplotlib`


## Installation

Please see [www.wime-project.net](https://www.wime-project.net/installation/)
for installation instructions.

## Usage

- The structure of the superframe can be controlled from MAC-Controller block.
- The duration of a timeslot, the number of the timeslots inside a superframe are variable.
- Controller application should be connected with the flowgraph with MAC-Controller block through UDP.
- Plant application should be connected with the flowgraph with MAC-Plant block through UDP.
- The number of plants should be defined correctly to allocate timeslots for each plant.
- The maximum number of timeslot in a superframe is 32 and first timeslot of every superframe is reserved for beacon packet.
- The number of retransmission attempts can be changed from mac_plant_impl.cc. If maximum retransmission number is more than 1 ACK packet will be activated.

Open the `examples/transceiver_*.grc` flow graph with gnuradio-companion and
check if all blocks are connected. Enable either the UHD blocks to interface
with real hardware or the Packet Pad block to loop back the samples. Open some
Rime connections and connect messages sources or Socket PDUs. You can easily
connect to the Socket PDU blocks with netcat. Netcat can be started for example
with

```
nc -u localhost 52001
```


There are also startup scripts in the apps folder.

Have fun!


/* -*- c++ -*- */
/*
 * Copyright 2020 TUM LKN.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_IEEE802_15_4_PROTOCOL_H
#define INCLUDED_IEEE802_15_4_PROTOCOL_H

#include <ieee802_15_4/api.h>

#define STATE_VECTOR_LEN 4
#define CONTROL_INPUT_LEN 1
#define MASK_FRAME_TYPE 0x0007
#define MASK_ACK_ENABLE 0x0020

namespace gr {
namespace ieee802_15_4 {

#pragma pack(push, 1)

struct PlantToControllerPacket {
    uint16_t frameControlField;
    uint8_t MACSeqNum;
    uint16_t dstPANaddr;
    uint16_t dstAddr;
    uint16_t srcAddr;
    char RIMEHeader[4];
    uint32_t loopID;
    uint32_t seqNum;
    float state[STATE_VECTOR_LEN];
    uint16_t crc;
};

struct ControllerToPlantPacket {
    uint16_t frameControlField;
    uint8_t MACSeqNum;
    uint16_t dstPANaddr;
    uint16_t dstAddr;
    uint16_t srcAddr;
    char RIMEHeader[4];
    uint32_t loopID;
    uint32_t seqNum;
    float control_input[CONTROL_INPUT_LEN];
    uint16_t crc;
};

struct BeaconPacket {
    uint16_t frameControlField;
    uint8_t MACSeqNum;
    uint16_t dstPANaddr;
    uint16_t dstAddr;
    uint16_t srcAddr;
    uint8_t numTimeslotPerSuperframe;
    uint16_t timeslotDur;
    uint8_t schedule[31];
    uint32_t timeslotNum;
    uint16_t crc;
};

struct AckPacket {
    uint16_t frameControlField;
    uint8_t MACSeqNum;
    uint16_t dstPANaddr;
    uint16_t dstAddr;
    uint16_t srcAddr;
    uint8_t ackSeq;
    uint16_t crc;
};

#pragma pack(pop)

enum MACFrameType {
    Beacon = 0,
    Data = 1,
    Ack = 2,
    MACCommand = 3
};

enum PlantMACAddr {
    Loop1 = 0x3340,
    Loop2 = 0x3341,
    Loop3 = 0x3342,
    Loop4 = 0x3343,
    Broadcast = 0xFFFF
};


/*!
 * \brief Collection of Packet Formats is defined in this file
 *
 */
class IEEE802_15_4_API protocol
{
public:

    protocol();
    ~protocol();

    static MACFrameType getFrameType(const uint16_t& fcf) {
        return (MACFrameType) (fcf & MASK_FRAME_TYPE);
    }

    static bool getAckEnable(const uint16_t& fcf) {
    	if(fcf & MASK_ACK_ENABLE){
        	return true;
    	} else {
    		return false;
    	}
    }

    static bool isBeacon(const uint16_t& fcf) {
        return getFrameType(fcf) == MACFrameType::Beacon;
    }

    static bool isACK(const uint16_t& fcf) {
        return getFrameType(fcf) == MACFrameType::Ack;
    }

    static uint16_t plantMACFromLoopID(const uint16_t loop_id) {
    	switch(loop_id)
    	{
    		case 1 : return PlantMACAddr::Loop1;
    		case 2 : return PlantMACAddr::Loop2;
    		case 3 : return PlantMACAddr::Loop3;
    		case 4 : return PlantMACAddr::Loop4;
    		default : return PlantMACAddr::Broadcast;
    	}
    }

    static void printUint16LSBtoMSB(const uint16_t& u) {
        for (int i = 0; i < 16; i++)
            printf("%d ", (u & (0x0001 << i)) >> i);

        printf("\n");
    }


private:
};

} // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_PROTOCOL_H */


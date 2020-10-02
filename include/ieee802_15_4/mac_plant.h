/*
 * Copyright (C) 2013 Bastian Bloessl <bloessl@ccs-labs.org>
 *               2020 Hasan Yagiz Özkan <yagiz.oezkan@tum.de>
 *               2020 Onur Ayan <onur.ayan@tum.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef INCLUDED_IEEE802_15_4_MAC_PLANT_H
#define INCLUDED_IEEE802_15_4_MAC_PLANT_H

#include <ieee802_15_4/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ieee802_15_4 {

/*!
 * \brief This block implements the MAC Layer processing of the plant side
 * \ingroup ieee802_15_4
 *
 */
class IEEE802_15_4_API mac_plant : virtual public block
{
public:
    virtual int get_num_packet_errors() = 0;
    virtual int get_num_packets_received() = 0;
    virtual float get_packet_error_ratio() = 0;
    
    typedef boost::shared_ptr<mac_plant> sptr;
    static sptr make(bool debug=false,
                     int method = 0,
                     /* default values for receive sensitivity testing in Zigbee test spec 14-0332-01 */
                     int fcf=0x8841,        // Frame control field
                     int seq_nr=0,          // Initial sequence number
                     int dst_pan=0x1aaa,    // Personal Area Network Address
                     int dst=0xffff,        // Destination Address (0xffff broadcast)
                     int src=0x3344,        // Source Address
                     int schedule = 0x01);  // Plant ID TODO To be removed
    
protected:
    enum QueuingStrategies {
        FCFS_TailDrop = 0,
        LCFS_PacketDiscard = 1,
        TOD_TailDrop = 2,
        FCFS_FrontDrop = 3,
        TOD_FrontDrop = 4,
        No_Queue = 5
    };
};

} // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_MAC_PLANT_H */

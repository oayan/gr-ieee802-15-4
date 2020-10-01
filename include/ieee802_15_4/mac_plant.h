/* -*- c++ -*- */
/*
 * Copyright 2020 gr-ieee802_15_4 author.
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


#ifndef INCLUDED_IEEE802_15_4_MAC_PLANT_H
#define INCLUDED_IEEE802_15_4_MAC_PLANT_H

#include <ieee802_15_4/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ieee802_15_4 {

/*!
 * \brief <+description of block+>
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
                     int fcf=0x8841,
                     int seq_nr=0,
                     int dst_pan=0x1aaa,
                     int dst=0xffff,
                     int src=0x3344,
                     int schedule = 0x01);
};

} // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_MAC_PLANT_H */

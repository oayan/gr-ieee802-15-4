/*
 * Copyright (C) 2013 Bastian Bloessl <bloessl@ccs-labs.org>
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
#ifndef INCLUDED_GR_IEEE802_15_4_MAC_CONTROLLER_H
#define INCLUDED_GR_IEEE802_15_4_MAC_CONTROLLER_H

#include <ieee802_15_4/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ieee802_15_4 {

/*!
 * \brief This is the MAC Block.
 *
 * \details
 * The MAC block...
 */
class IEEE802_15_4_API mac_controller: virtual public block
{
public:
  virtual int get_num_packet_errors() = 0;
  virtual int get_num_packets_received() = 0;
  virtual float get_packet_error_ratio() = 0;
  
  typedef boost::shared_ptr<mac_controller> sptr;
  static sptr make(bool debug=false,
          /* default values for receive sensitivity testing in Zigbee test spec 14-0332-01 */ 
          int fcf=0x8841,
          int seq_nr=0,
          int dst_pan=0x1aaa,
          int ts_dur_ms=10,
          int nr_loop=1,
          int slot_len=20,
          bool beacon_eanble=true,
          int schedule_method = 0 ); // TODO Save Stats
protected:
    enum SchedulingMethods {
        Round_Robin = 0
    };
};

}  // namespace ieee802_11
}  // namespace gr

#endif /* INCLUDED_GR_IEEE802_15_4_MAC_H */

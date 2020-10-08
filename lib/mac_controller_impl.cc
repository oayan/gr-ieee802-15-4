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
#include <ieee802_15_4/mac_controller.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/high_res_timer.h>
#include <ieee802_15_4/protocol.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <time.h>

#define NUM_LOOP 2

// a container type to collect the arrival time (timeslot) of a packet
// struct measurementElement {
//     uint8_t  controller_num;  // identification number of controller-plant pair
//     uint32_t sequence;      // sequence number of the arrived packet
//     uint32_t arrival;     // arrival time in timeslot (timeslot duration depends on d_timeslot_dur, which is received from app)
// };

// struct plantToControllerFormat {
//
// };

using namespace gr::ieee802_15_4;

class mac_controller_impl : public mac_controller {
public:

#define dout d_debug && std::cout

    mac_controller_impl(bool debug, int fcf, int seq_nr, int dst_pan, int ts_dur_ms, int slot_len, bool beacon_enable, int schedule_method):
        block ("mac_controller",
               gr::io_signature::make(0, 0, 0),
               gr::io_signature::make(0, 0, 0)),
        d_msg_offset(0),
        d_debug(debug),
        d_fcf(fcf),
        d_seq_nr(seq_nr),
        d_dst_pan(dst_pan),
        d_save_stats(0),
        d_beacon_enable(beacon_enable),
        d_timeslot_dur_ms(ts_dur_ms),
        d_timeslot_dur((gr::high_res_timer_tps() / 1000) * ts_dur_ms),
        d_slot_len(slot_len),
        d_beacon_schedule_method(static_cast<SchedulingMethods>(schedule_method)),
        d_num_packet_errors(0),
        d_num_packets_received(0) {

        controller_init();

        message_port_register_in(pmt::mp("app in"));
        set_msg_handler(pmt::mp("app in"), boost::bind(&mac_controller_impl::app_controller_in, this, _1));
        message_port_register_in(pmt::mp("pdu in"));
        set_msg_handler(pmt::mp("pdu in"), boost::bind(&mac_controller_impl::mac_controller_in, this, _1));

        message_port_register_out(pmt::mp("app out"));
        message_port_register_out(pmt::mp("pdu out"));
        // this thread has a while loop for repetitive tasks
        // the structure of the thread is taken from "periodic_msg_source_impl.cc" of gr_foo
        // https://github.com/bastibl/gr-foo/blob/maint-3.7/lib/periodic_msg_source_impl.cc
        d_thread = new boost::thread(boost::bind(&mac_controller_impl::run, this, this));

        // measurement.reserve(1024);
    }

    // includes requirements of the thread with repetitive tasks
    ~mac_controller_impl(void) {
        gr::thread::scoped_lock(d_mutex);

        d_finished = true;
        d_thread->interrupt();
        d_thread->join();
        delete d_thread;
    }

    // transmits periodic beacon packet and save the measurements at the end of experiment
    void run(mac_controller_impl *instance) {
        try {
            // flow graph startup delay
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));

            while(1) {
                // sleep a little to reduce the workload
                boost::this_thread::sleep(boost::posix_time::microseconds(250));

                gr::thread::scoped_lock(d_mutex);
                // end of the measurements when do not receive any packet for 10 seconds
                // TODO Save stats
                // if(gr::high_res_timer_now() - d_last_recieved > (gr::high_res_timer_tps() * 10) && d_last_recieved != 0) {
                //     // at the end of current measurement set d_last_receive to 0 so another received packet restarts the process
                //     d_last_recieved = 0;
                //     // save the measurements with time and date info
                //     time_t now = time(0);
                //     tm *ltm = localtime(&now);
                //     char file_name [255];
                //     sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACcontroller.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
                //     std::ofstream tmpfile;
                //     tmpfile.open (file_name);
                //     while(!measurement.empty()) {
                //         struct measurementElement toWrite = measurement[0];
                //         measurement.erase(measurement.begin());
                //         tmpfile << unsigned(toWrite.controller_num) << "," << toWrite.sequence << "," << toWrite.arrival << std::endl;
                //     }
                //     printf("end of controller\n");
                // }
                // check for the beginning of the timeslot
                if(gr::high_res_timer_now() - d_tnow > d_timeslot_dur && d_slotframe_dur != 0) {
                    d_tnow = gr::high_res_timer_now();
                    // check for the beacon timeslot
                    if(d_cnt_timeslot % d_slot_len == 0) {
                        // add sequence number of the current timeslot to the beacon
                        // so plants can synchronize their timeslots for measurements
                        d_beacon_str.timeslotNum = d_cnt_timeslot;
                        generate_update_beacon_mac();  //add mac header and footer
                        // send packet
                        message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                                         pmt::make_blob((char*)(&d_beacon_str), sizeof(d_beacon_str))));
                        // TODO This should be redundant in radio !!!!!!!!!!!!!
                        // message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                        //                  pmt::make_blob("1", 1)));
                        // TODO: remove if not necessary !!!!!!!!!!!!!!!!!!!
                        // TODO save stats
                        // if(d_cnt_timeslot - d_last_received_ts_num > 750 & d_last_received_ts_num != 0) {
                        //     d_last_received_ts_num = 0;
                        //     time_t now = time(0);
                        //     tm *ltm = localtime(&now);
                        //     char file_name [255];
                        //     sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACcontroller.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
                        //     std::ofstream tmpfile;
                        //     tmpfile.open (file_name);
                        //     while(!measurement.empty()) {
                        //         struct measurementElement toWrite = measurement[0];
                        //         measurement.erase(measurement.begin());
                        //         tmpfile << unsigned(toWrite.controller_num) << "," << toWrite.sequence << "," << toWrite.arrival << std::endl;
                        //     }
                        //     printf("end of controller\n");
                        // }
                    }
                    // at the end of every timeslot update the timeslot number
                    d_cnt_timeslot++;
                }
            }

        } catch(boost::thread_interrupted) {
            gr::thread::scoped_lock(d_mutex);
            d_finished = true;
        }
    }


    /**
    * @brief The mac_in method is called when a new PDU is received from the PHY layer (lower layer).
    * TODO define what this block is doing in 1-2 sentences
    * @param msg The received message of pmt::pmt_t type.
    */
    void mac_controller_in(pmt::pmt_t msg) {
        // TODO Check whether d_mutex is needed
        gr::thread::scoped_lock(d_mutex);
        d_last_recieved = gr::high_res_timer_now();   // update timer for measurement
        // TODO This could have been a char array
        pmt::pmt_t blob;

        if(pmt::is_pair(msg)) {
            blob = pmt::cdr(msg);
        } else {
            assert(false);
        }

        size_t data_len = pmt::blob_length(blob);
        if(data_len < 11) {
            // dout << "MAC: frame too short. Dropping!" << std::endl;
            return;
        }
        PlantToControllerPacket* rx_pkt = (PlantToControllerPacket*)pmt::blob_data(blob);

        uint16_t crc = crc16((char*)rx_pkt,sizeof(*rx_pkt));
        d_num_packets_received++;
        if(crc) {
            d_num_packet_errors++;
            dout << "MAC: wrong crc. Dropping packet!" << std::endl;
            return;
        }
        else {
            d_last_received_ts_num = d_cnt_timeslot;  // timeslot of received packet
            // space to save received data
            unsigned char buf[256];
            std::memcpy(buf, pmt::blob_data(blob), data_len);

            // struct measurementElement newElement;
            // // TODO packetToControllerFormat as struct
            // newElement.controller_num = rx_pkt->loopID;
            // newElement.sequence = rx_pkt->seqNum;
            // newElement.arrival = d_cnt_timeslot;
            // measurement.push_back(newElement);

            // send ACK if ACK-enabled
            if(protocol::getAckEnable(rx_pkt->frameControlField)) {
              generate_send_ack(rx_pkt);
            }
        }
        // Drops MAC Header & Footer and forwards the data up to Rime Stack
        pmt::pmt_t mac_payload = pmt::make_blob((char*)pmt::blob_data(blob) + 9, data_len - 9 - 2);

        message_port_pub(pmt::mp("app out"), pmt::cons(pmt::PMT_NIL, mac_payload));
    }


    /**
    * @brief The app_in method is called when a new PDU is received from the Rime Stack (upper layer).
    * TODO Define what this method is doing in 1-2 sentences
    * @param msg The received message of pmt::pmt_t type.
    */
    void app_controller_in(pmt::pmt_t msg) {
        // TODO Check whether d_mutex is needed
        gr::thread::scoped_lock(d_mutex);
        // TODO These 2 could have been char array probably
        pmt::pmt_t blob;

        if(pmt::is_eof_object(msg)) {
            dout << "MAC: exiting" << std::endl;
            detail().get()->set_done(true);
            return;
        } else if(pmt::is_blob(msg)) {
            blob = msg;
        } else if(pmt::is_pair(msg)) {
            blob = pmt::cdr(msg);
        } else {
            dout << "MAC: unknown input" << std::endl;
            return;
        }
        const char* data = (const char*) pmt::blob_data(blob);

        ControllerToPlantPacket tx_pkt;
        tx_pkt = generate_mac(data);

        message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                  pmt::make_blob((char*)(&tx_pkt), sizeof(tx_pkt))));
        //TODO REMOVE THIS FOR USRP usage !!!!!!!!!!
        // message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
        //             pmt::make_blob("1", 1)));

    }

    uint16_t crc16(char *buf, int len) {
        uint16_t crc = 0;

        for(int i = 0; i < len; i++) {
            for(int k = 0; k < 8; k++) {
                int input_bit = (!!(buf[i] & (1 << k)) ^ (crc & 1));
                crc = crc >> 1;
                if(input_bit) {
                    crc ^= (1 << 15);
                    crc ^= (1 << 10);
                    crc ^= (1 <<  3);
                }
            }
        }

        return crc;
    }

    ControllerToPlantPacket generate_mac(const char *data) {
        // Copy the Payload into correct place
      ControllerToPlantPacket tx_pkt;
      
      std::memcpy(&(tx_pkt.RIMEHeader),
                  data,
                  sizeof(tx_pkt.RIMEHeader) + sizeof(tx_pkt.loopID) + sizeof(tx_pkt.seqNum) + CONTROL_INPUT_LEN * sizeof(*(tx_pkt.control_input)));
      
      tx_pkt.frameControlField = d_fcf;
      tx_pkt.MACSeqNum = d_seq_nr++;
      tx_pkt.dstPANaddr = d_dst_pan;
      tx_pkt.dstAddr = protocol::plantMACFromLoopID(tx_pkt.loopID);
      tx_pkt.srcAddr = d_src;

      // Calculate & Fill 16-bit CRC
      uint16_t crc = crc16((char*)(&tx_pkt), sizeof(tx_pkt) - sizeof(tx_pkt.crc));
      tx_pkt.crc = crc;

      return tx_pkt;
    }

    void generate_send_ack(PlantToControllerPacket* rx_pkt) {
      AckPacket ack_pkt;

      ack_pkt.frameControlField = d_fcf_ack;
      ack_pkt.MACSeqNum = d_seq_nr++;
      ack_pkt.dstPANaddr = d_dst_pan;
      ack_pkt.dstAddr = rx_pkt->srcAddr;
      ack_pkt.srcAddr = d_src;
      ack_pkt.ackSeq = rx_pkt->MACSeqNum;

      uint16_t crc = crc16((char*)(&ack_pkt), (sizeof(ack_pkt) - sizeof(ack_pkt.crc)));
      ack_pkt.crc = crc;

      message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                        pmt::make_blob((char*)(&ack_pkt), sizeof(ack_pkt))));
      // TODO this should be redundant with USRP !!!!!!!!!!!!!!!!!
      // message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
      //                  pmt::make_blob("1", 1)));
    }

    void generate_update_beacon_mac() {

        // FCF
        // data frame, no security
        d_beacon_str.frameControlField = d_fcf_beacon;
        d_beacon_str.MACSeqNum = d_seq_nr++;
        d_beacon_str.dstPANaddr = d_dst_pan;
        d_beacon_str.dstAddr = protocol::getBroadcastMACAddr();
        d_beacon_str.srcAddr = d_src;

        uint16_t crc = crc16((char*)(&d_beacon_str), (sizeof(d_beacon_str) - sizeof(d_beacon_str.crc)));
        d_beacon_str.crc = crc;
    }

    void controller_init(){
      d_src = protocol::getContollerMACAddr();
      beacon_init(d_beacon_enable);
    }

    void beacon_init(bool beacon) {
      if(beacon){
        int i;
        d_slotframe_dur = d_timeslot_dur * d_slot_len;
        d_beacon_str.numTimeslotPerSuperframe = d_slot_len;
        d_beacon_str.timeslotDur = d_timeslot_dur_ms;
        if(d_beacon_schedule_method == SchedulingMethods::Round_Robin){
            for(i = 0; i < d_slot_len; i++){
                // +1 since the smallest plant id is not 0 but 1
                d_beacon_str.schedule[i] = (i % NUM_LOOP) + 1;
            }
        }
        d_beacon_str.schedule[0] = 0x00;
        d_beacon_str.timeslotNum = d_cnt_timeslot;
        d_tnow = gr::high_res_timer_now();  
        printf("beacon init\n");
      }
    }

    void print_message() {
        for(int i = 0; i < d_msg_len; i++) {
            dout << std::setfill('0') << std::setw(2) << std::hex << ((unsigned int)d_msg[i] & 0xFF) << std::dec << " ";
            if(i % 16 == 15) {
                dout << std::endl;
            }
        }
        dout << std::endl;
    }

    int get_num_packet_errors() {
        return d_num_packet_errors;
    }

    int get_num_packets_received() {
        return d_num_packets_received;
    }

    float get_packet_error_ratio() {
        return float(d_num_packet_errors)/d_num_packets_received;
    }

private:
    bool        d_debug;
    int         d_msg_offset;
    int         d_msg_len;
    uint16_t    d_fcf;
    uint16_t    d_fcf_beacon = 0x8840;
    uint16_t    d_fcf_ack = 0x8842;
    uint8_t     d_seq_nr;
    uint16_t    d_dst_pan;
    uint16_t    d_src;
    uint16_t    d_save_stats;
    char        d_msg[256];

    bool        d_beacon_enable;
    gr::high_res_timer_type d_tnow;     // to save the start time of current timeslot
    long long int    d_timeslot_dur_ms;
    long long int    d_timeslot_dur = 0;  // this value is taken from application duration of timeslot (ms)
    uint8_t     d_slot_len;         // this value is taken from application number of timeslots in a superframe
    // this value is taken from application duration of superframe (d_timeslot_dur x d_slot_len)
    long long int    d_slotframe_dur = 0;
    long long int    d_last_recieved = 0; // stores the reception time of latest received packet
    BeaconPacket     d_beacon_str;
    SchedulingMethods d_beacon_schedule_method;
    
    uint32_t  d_cnt_timeslot = 0;     // this value used to understand own timeslot (beacon)
    // std::vector<measurementElement> measurement;
    uint32_t  d_last_received_ts_num = 0;

    bool d_finished = false;
    // used for while loop
    boost::thread *d_thread;
    gr::thread::mutex d_mutex;

    int d_num_packet_errors;
    int d_num_packets_received;
};

mac_controller::sptr
mac_controller::make(bool debug, int fcf, int seq_nr, int dst_pan, int ts_dur_ms, int slot_len, bool beacon_enable, int schedule_method) {
    return gnuradio::get_initial_sptr(new mac_controller_impl(debug,fcf,seq_nr,dst_pan,ts_dur_ms,slot_len,beacon_enable,schedule_method));
}

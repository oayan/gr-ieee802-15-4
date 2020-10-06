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

#include <ieee802_15_4/mac_plant.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/high_res_timer.h>
#include <ieee802_15_4/protocol.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

#define MIN_PKT_LEN 11
#define BROADCAST_ADDR 0xFFFF
#define SEND_SINGLE_BYTE false
#define BITS_IN_BYTE 8

using namespace gr::ieee802_15_4;

struct queueElement {
    uint32_t seq;			//
    uint8_t remainingRetransmissionAttempts;	// number of remaining retransmission attempts
    int len;
    PlantToControllerPacket p;
};

struct queueMeasureElement {
    uint16_t queuesize;
    uint32_t timeslot_seq;
};

struct aoiMeasurementElement {
    uint32_t sequence;
    uint32_t arrival;
    uint32_t sent;
    uint32_t answer;
};



class mac_plant_impl : public mac_plant {
public:

#define dout d_debug && std::cout

    mac_plant_impl(bool debug, int method, int fcf, int seq_nr, int dst_pan, int dst, int src, int schedule) :
        block ("mac_plant",
               gr::io_signature::make(0, 0, 0),
               gr::io_signature::make(0, 0, 0)),

        d_debug(debug),
        d_fcf(fcf),
        d_seq_nr(seq_nr),
        d_dst_pan(dst_pan),
        d_dst(dst),
        mSrcAddr(src),
        mQueuingStrategy(static_cast<QueuingStrategies>(method)),
        d_schedule(schedule),
        d_num_packet_errors(0),
        d_num_packets_received(0) {

        message_port_register_in(pmt::mp("app in"));
        set_msg_handler(pmt::mp("app in"), boost::bind(&mac_plant_impl::app_in, this, _1));
        message_port_register_in(pmt::mp("pdu in"));
        set_msg_handler(pmt::mp("pdu in"), boost::bind(&mac_plant_impl::mac_plant_in, this, _1));

        message_port_register_out(pmt::mp("app out"));
        message_port_register_out(pmt::mp("pdu out"));
        d_thread = new boost::thread(boost::bind(&mac_plant_impl::run, this, this));
    }

    ~mac_plant_impl(void) {
        gr::thread::scoped_lock(d_mutex);

        d_finished = true;
        d_thread->interrupt();
        d_thread->join();
        delete d_thread;
    }

    void run(mac_plant_impl *instance) {
        try {
            // flow graph startup delay
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            queue.reserve(1000);
            queueValues.reserve(1000);

            while(1) {

                boost::this_thread::sleep(boost::posix_time::microseconds(250));
                gr::thread::scoped_lock(d_mutex);
                // TODO Measurement variables, notations & implementation

                if ((mQueuingStrategy == QueuingStrategies::No_Queue) && (gr::high_res_timer_now() - d_last_pack_recieved) > (gr::high_res_timer_tps() * 5) && d_last_pack_recieved != 0) {
                    saveStats();
                    printf("end of plant\n");
                }


                // TODO Slot Dur <--> Method type
                if(gr::high_res_timer_now() - d_tnow > d_slot_dur && d_slot_dur != 0) {
                    d_tnow = gr::high_res_timer_now();
                    d_current_position = (d_current_position + 1) % d_num_timeslot_per_superframe;
                    d_seq_timeslot++;
                    if(queue.size() != d_old_queue_size) {
                        d_old_queue_size = queue.size();
                        struct queueMeasureElement newMeasure;
                        newMeasure.queuesize = d_old_queue_size;
                        newMeasure.timeslot_seq = d_seq_timeslot;
                        queueValues.push_back(newMeasure);
                    }
                    
                    // Check if the current slot is allocated to me (the corresponding bit should already be set to '1' upon a beacon reception)
                    if(d_current_slotframe & (0x1 << d_current_position) && !queue.empty()) {
                        struct queueElement newElement;
                        rtt_start = gr::high_res_timer_now();
                        newElement = queue[0];
                        if((newElement.remainingRetransmissionAttempts == 1) || (mQueuingStrategy == QueuingStrategies:: TOD_TailDrop) || (mQueuingStrategy == QueuingStrategies::TOD_FrontDrop)) {
                            queue.erase(queue.begin());
                        } else {
                            queue[0].remainingRetransmissionAttempts = queue[0].remainingRetransmissionAttempts - 1;
                        }

                        aoiValues[newElement.seq].sent = d_seq_timeslot;
                        mac_send[newElement.seq] = (rtt_start / tpus);

                        // Forward TX data packet to PHY layer
                        message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                                         pmt::make_blob(&(newElement.p), sizeof(PlantToControllerPacket))));

                        if (SEND_SINGLE_BYTE) {
                            message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                                             pmt::make_blob("1", 1)));
                        }

                    }

                    // TODO REMOVE THIS WITH SAVE STATS
                    // TODO redundant with above
                    if ((gr::high_res_timer_now() - d_last_pack_recieved) > (gr::high_res_timer_tps() * 5) && d_last_pack_recieved != 0) {
                        saveStats();
                        printf("end of plant\n");
                    }
//                     d_current_slotframe = d_current_slotframe << 1 | ( 0x1 & (d_slotframe >> (d_num_timeslot_per_superframe - d_last_position)));
                    
                }
            }

        } catch(boost::thread_interrupted) {
            gr::thread::scoped_lock(d_mutex);
            d_finished = true;
        }
    }

    /*!
    * \brief The mac_plant_in method is called when a new PDU is received from the PHY layer (lower layer).
    * TODO define what this block is doing in 1-2 sentences
    * \param msg The received message of pmt::pmt_t type.
    */
    void mac_plant_in(pmt::pmt_t msg) {
        gr::thread::scoped_lock(d_mutex);
        pmt::pmt_t blob;

        if(pmt::is_pair(msg)) {
            blob = pmt::cdr(msg);
        }
        else {
            assert(false);
        }

        size_t data_len = pmt::blob_length(blob);
        unsigned char buf[256];
        std::memcpy(buf, pmt::blob_data(blob), data_len);

        ControllerToPlantPacket* rx_pkt = (ControllerToPlantPacket*)pmt::blob_data(blob);

//         for(int i = 0; i < data_len; i++) {
//             dout << std::setfill('0') << std::setw(2) << std::hex << ((unsigned int)buf[i] & 0xFF) << std::dec << " ";
//             if(i % 16 == 15) {
//                 dout << std::endl;
//             }
//         }
//         dout << std::endl;

        uint16_t dest = rx_pkt->dstAddr;

        // Return if packet contains only header & footer
        if(data_len < MIN_PKT_LEN) {
            // printf("MAC: frame too short. Dropping!\n");
            return;
        }

        uint16_t crc = crc16((char*)pmt::blob_data(blob), data_len);
        if(crc) {
            d_num_packet_errors++;
            dout << "MAC: wrong crc. Dropping packet!" << std::endl;
            return;
        }

        if(protocol::isBeacon(rx_pkt->frameControlField)) {
            d_tnow = gr::high_res_timer_now();
            extract_beacon_info((BeaconPacket*) rx_pkt);
            return;
        } else if(protocol::isACK(rx_pkt->frameControlField) && (dest == mSrcAddr)) {
            uint8_t ack_seq = buf[9];                             // MAC Layer sequence number
            uint32_t pck_seq = findAndRemoveFromQueue(ack_seq);   // APP Layer seq_num
            if(pck_seq < 10000) {
                ack_receive[pck_seq] = (gr::high_res_timer_now() / tpus);  // MAC2MAC
            }
            return;
        } else if(dest != d_schedule && dest != BROADCAST_ADDR ) {
            // Not broadcast or not my packet
            // TODO Mapping to Dest Address & Source Address
            std::cout << "Wrong!" << std::endl;
            return;
        }

        uint32_t seqNum = rx_pkt->seqNum;
        d_num_packets_received++;


        aoiValues[seqNum].answer = d_seq_timeslot;
        if(mac_to_app[seqNum] == 0)
            mac_to_app[seqNum] = (gr::high_res_timer_now() / tpus);

        // Drops MAC Header & Footer and forwards the data up to Rime Stack
        pmt::pmt_t mac_plant_payload = pmt::make_blob((char*)pmt::blob_data(blob) + 9, data_len - 9 - 2);

        message_port_pub(pmt::mp("app out"), pmt::cons(pmt::PMT_NIL, mac_plant_payload));
    }

    /*!
    * \brief The mac_plant_in method is called when a new PDU is received from the Rime Stack (upper layer).
    * TODO define what this block is doing in 1-2 sentences
    * \param msg The received message of pmt::pmt_t type.
    */
    void app_in(pmt::pmt_t msg) {
        gr::thread::scoped_lock(d_mutex);
        pmt::pmt_t blob;

        mac_wait_start = gr::high_res_timer_now();
        dout << "MAC: message received from Rime:" << std::endl;
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

        //dout << "MAC: received new message from APP of length " << pmt::blob_length(blob) << std::endl;

        PlantToControllerPacket tx_pkt = generateMacPlant(data);
        struct aoiMeasurementElement newElement;
        newElement.sequence = tx_pkt.seqNum;
        newElement.arrival = d_seq_timeslot;
        newElement.sent = 0;
        newElement.answer = 0;
        aoiValues[tx_pkt.seqNum] = newElement;
        app_to_mac[tx_pkt.seqNum] = mac_wait_start / tpus;

        // TODO If method .. add to queue or send directly
        addToQueue(tx_pkt);
        if(tx_pkt.seqNum == 9999) {
            d_last_pack_recieved = gr::high_res_timer_now();
        }
        // printf("queue size = %ld, msg_len = %d\n", queue.size(), d_msg_len);
        //message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
        //		pmt::make_blob(newElement.data, newElement.len)));
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

// Private Attributes
private:

    bool        d_debug;
    int         d_msg_len;
    uint16_t    d_fcf;
    uint8_t     d_seq_nr;
    uint16_t    d_dst_pan;
    uint16_t    d_dst;
    uint16_t    mSrcAddr;
    char        d_msg[256];
    char        response[20];
    // picking 10 as the array size
    // std::vector<std::array<char, 10>> va;
    uint8_t    	d_queue_size = 5;
    QueuingStrategies    mQueuingStrategy;
    std::vector<queueElement> queue;
    gr::high_res_timer_type d_tnow;
    uint16_t	d_schedule;
    long long int    d_slot_dur = 0;
    uint32_t    d_seq_timeslot = 0;
    uint32_t    d_current_slotframe = 0;
    uint32_t    d_slotframe = 0;
    uint8_t     d_current_position;
    uint8_t     d_num_timeslot_per_superframe;
    uint8_t		d_retransmission_attempt = 2;
    uint16_t    d_old_queue_size = 0;
    uint64_t 	mac_to_app[10000];
    uint64_t 	ack_receive[10000];
    long long int 	d_last_pack_recieved = 0;
    long long int 	rtt_start;
    uint64_t 	mac_send[10000];
    uint64_t 	app_to_mac[10000];
    long long int 	mac_wait_start;
    long long int tpus = (gr::high_res_timer_tps() / 1000000);
    std::vector<queueMeasureElement>	queueValues;
    aoiMeasurementElement	aoiValues[10000];


    bool d_finished = false;
    boost::thread *d_thread;
    gr::thread::mutex d_mutex;

    int d_num_packet_errors;
    int d_num_packets_received;

    // Private Methods
private:
    void saveStats() {
        d_last_pack_recieved = 0;
        std::ofstream tmpfile;
        time_t now = time(0);
        tm *ltm = localtime(&now);
        char file_name [255];
        sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACrttduration(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule, mQueuingStrategy);
        tmpfile.open (file_name);
        if(!tmpfile) {
            printf("could not opened\n");
        }
        for(int i = 0; i < 10000; i++) {
            tmpfile <<  app_to_mac[i] << "," << mac_send[i] << "," << ack_receive[i] << "," << mac_to_app[i] << std::endl;
            // printf("%d, ", (rtt_measure[i]));
            if(i%10 == 100)
                printf("1\n");
        }
        tmpfile.close();
        sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACqueue(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule, mQueuingStrategy);
        tmpfile.open (file_name);
        while(!queueValues.empty()) {
            struct queueMeasureElement toWrite = queueValues[0];
            queueValues.erase(queueValues.begin());
            tmpfile << toWrite.timeslot_seq << "," << toWrite.queuesize << std::endl;
        }
        tmpfile.close();
        // TODO Change this, RTT measurement could be enough
        sprintf(file_name,"../../results/results/%d-%d-%d_%d-%d-%dMACaoi(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule, mQueuingStrategy);
        tmpfile.open (file_name);
        int i = 0;
        while(i < 10000) {
            struct aoiMeasurementElement toWrite = aoiValues[i];
            tmpfile << toWrite.sequence << "," << toWrite.arrival << "," << toWrite.sent << "," << toWrite.answer << std::endl;
            i++;
        }
        tmpfile.close();
    }

    void addToQueue(const PlantToControllerPacket& pkt) {
        struct queueElement newElement;
        newElement.len = sizeof(pkt);
        newElement.seq = pkt.MACSeqNum;
        newElement.remainingRetransmissionAttempts = d_retransmission_attempt;
        std::memcpy(&(newElement.p), &pkt, sizeof(pkt));

        if((mQueuingStrategy == QueuingStrategies::LCFS_PacketDiscard) & !queue.empty()) {
            while(!queue.empty()) {
                queue.erase(queue.begin());
            }
            queue.push_back(newElement);
        } else if (mQueuingStrategy == QueuingStrategies::No_Queue) {
            // RANDOM ACCESS - No Wait
            message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                             pmt::make_blob(&(newElement.p), newElement.len)));
            if (SEND_SINGLE_BYTE) {
                message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
                                 pmt::make_blob("1", 1)));
            }
        } else if (queue.size() < d_queue_size) {
            // There is space in the TX queue
            queue.push_back(newElement);
        } else if (mQueuingStrategy == QueuingStrategies::FCFS_FrontDrop || mQueuingStrategy == QueuingStrategies::TOD_FrontDrop) {
            // Front drop
            while(queue.size() >= d_queue_size) {
                queue.erase(queue.begin());
            }
            // TODO QUEUE implementation (no vector)
            queue.push_back(newElement);
        }
    }

    uint32_t findAndRemoveFromQueue(uint8_t seq) {

        for(int i = 0; i < queue.size(); i++) {
            //printf("remove from queue, queuesize = %d, seq = %d\n", queue.size(), seq);
            // Read the MAC Sequence Number from the queue element
            uint8_t taken = queue[i].p.MACSeqNum;
            // Compare the given seq with the found seq
            if(seq == taken) {
                // Found in queue
                uint32_t appSeq = queue[i].p.seqNum;
                queue.erase(queue.begin() + i);
                return appSeq;
            }
        }
        return 0;
    }

    void extract_beacon_info(BeaconPacket* beacon_packet) {
        
        assert(d_num_timeslot_per_superframe < (sizeof(d_current_slotframe) * BITS_IN_BYTE));
        
        uint16_t dur = beacon_packet->timeslotDur;
        d_num_timeslot_per_superframe = beacon_packet->numTimeslotPerSuperframe;
        assert(d_num_timeslot_per_superframe != 0);                // This method should not be called if this is the case
        int i = 0;
        // d_current_slotframe contains flags about the ownership of next 32 timeslots (every one bit of uint32_t indicates ownership of one timeslot)
        // transmission of a packet is allowed, if the flag is 1. 
        // It may contain more timeslot than superframe in order to continiue transmitting even if the beacon is lost.
        // d_slotframe contains flags belong to one superframe. d_last_position contains the number of the last timeslot represented by d_current_slotframe.
        // d_slotframe and d_last_position are used to recalculate ownership of the next 32 timeslot at the end of every timeslot. 
        
        d_current_slotframe = 0;               // Placeholder of the next 32 slots' schedule to extract into 
        for (i = 0; i < d_num_timeslot_per_superframe; i++) {
//             d_current_slotframe = d_current_slotframe << 1;
            if(beacon_packet->schedule[i] == d_schedule) {
                d_current_slotframe = (0x00000001 << i) | d_current_slotframe;
            }
        }
//         d_last_position = (sizeof(d_current_slotframe) * BITS_IN_BYTE) % d_num_timeslot_per_superframe;
        d_current_position = 0;
//         d_slotframe = d_current_slotframe >> (31 - d_num_timeslot_per_superframe);
        d_seq_timeslot = beacon_packet->timeslotNum;
        d_slot_dur = (gr::high_res_timer_tps() / 1000) * dur;
    }

    /*!
    * \brief generateMacPlant constructs a PHY layer packet given the data coming in from Rime Stack.
    * Output of this method is a PlantToControllerPacket type object, which is ready to be written into TX Queue.
    * \param data is a pointer to the data segment of the message received from the Rime Stack.
    */
    PlantToControllerPacket generateMacPlant(const char *data) {

        PlantToControllerPacket tx_pkt;

        // FCF
        // data frame, no security
        if((mQueuingStrategy == QueuingStrategies::FCFS_TailDrop) || (mQueuingStrategy == QueuingStrategies::LCFS_PacketDiscard) || (mQueuingStrategy == QueuingStrategies::FCFS_FrontDrop)) {
            tx_pkt.frameControlField = d_fcf | 0x0020;
        }
        else {
            tx_pkt.frameControlField = d_fcf;
        }

        // seq nr
        tx_pkt.MACSeqNum = d_seq_nr++;

        // addr info
        tx_pkt.dstPANaddr = d_dst_pan;
        tx_pkt.dstAddr = d_dst;
        tx_pkt.srcAddr = mSrcAddr;

        // Copy the Payload into correct place
        std::memcpy(&(tx_pkt.RIMEHeader),
                    data,
                    sizeof(tx_pkt.RIMEHeader) + sizeof(tx_pkt.loopID) + sizeof(tx_pkt.seqNum) + STATE_VECTOR_LEN * sizeof(*(tx_pkt.state)));

        // Calculate & Fill 16-bit CRC
        uint16_t crc = crc16((char*)(&tx_pkt), sizeof(tx_pkt) - sizeof(tx_pkt.crc));
        tx_pkt.crc = crc;

        return tx_pkt;
    }


};


mac_plant::sptr
mac_plant::make(bool debug, int method, int fcf, int seq_nr, int dst_pan, int dst, int src, int schedule) {
    return gnuradio::get_initial_sptr(new mac_plant_impl(debug,method,fcf,seq_nr,dst_pan,dst,src,schedule));
}

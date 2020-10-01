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

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

struct queueElement {
	uint32_t seq;			//
	uint8_t retransmission;	// number of remaining retransmission attempts
 	int len;			
  	char data[256];
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

using namespace gr::ieee802_15_4;

class mac_plant_impl : public mac_plant {
public:

#define dout d_debug && std::cout

mac_plant_impl(bool debug, int method, int fcf, int seq_nr, int dst_pan, int dst, int src, int schedule) :
  block ("mac_plant",
    gr::io_signature::make(0, 0, 0),
    gr::io_signature::make(0, 0, 0)),
  	d_msg_offset(0),
  	d_debug(debug),
	d_fcf(fcf),
	d_seq_nr(seq_nr),
	d_dst_pan(dst_pan),
	d_dst(dst),
	d_src(src),
	d_schedule(schedule),
	d_method(method),
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

void run(mac_plant_impl *instance){
  	try {

		// flow graph startup delay
		// boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		int cnt = 0;
		int i = 0;
		while(1) {
			//{
			boost::this_thread::sleep(boost::posix_time::microseconds(250));
			gr::thread::scoped_lock(d_mutex);
			if (d_method == 5 && (gr::high_res_timer_now() - d_last_pack_recieved) > (gr::high_res_timer_tps()*5) && d_last_pack_recieved != 0){
				d_last_pack_recieved = 0;
				std::ofstream tmpfile;
				time_t now = time(0);
				tm *ltm = localtime(&now);
				char file_name [255];
				sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACrttduration(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule, d_method);
				tmpfile.open (file_name);
				if(!tmpfile){
					printf("could not opened\n");
				}
				for(i = 0; i < 10000; i++){
					tmpfile <<  app_to_mac[i] << "," << mac_send[i] << "," << ack_receive[i] << "," << mac_to_app[i] << std::endl;
					// printf("%d, ", (rtt_measure[i]));
					if(i%10 == 100)
						 printf("1\n");
				}
				tmpfile.close();
				sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACqueue(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule,d_method);
				tmpfile.open (file_name);
				while(!queueValues.empty()){
					struct queueMeasureElement toWrite = queueValues[0];
					queueValues.erase(queueValues.begin());
					tmpfile << toWrite.timeslot_seq << "," << toWrite.queuesize << std::endl;
				}
				tmpfile.close();
				sprintf(file_name,"../../results/results/%d-%d-%d_%d-%d-%dMACaoi(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule,d_method);
				tmpfile.open (file_name);
				i = 0;
				while(i < 10000){
					struct aoiMeasurementElement toWrite = aoiValues[i];
					tmpfile << toWrite.sequence << "," << toWrite.arrival << "," << toWrite.sent << "," << toWrite.answer << std::endl;
					i++;
				}
				tmpfile.close();
	  			printf("end of plant\n");	
	    	}
			if(gr::high_res_timer_now() - d_tnow > d_slot_dur && d_slot_dur != 0){
				d_seq_timeslot++;
				if(queue.size() != d_old_queue_size){
					d_old_queue_size = queue.size();
					struct queueMeasureElement newMeasure;
					newMeasure.queuesize = d_old_queue_size;
					newMeasure.timeslot_seq = d_seq_timeslot;
					queueValues.push_back(newMeasure);
				}
				d_tnow = gr::high_res_timer_now();
				//printf("------queue size = %d\n", queue.size());
				//??????
				if(d_current_slotframe & (0x1<<31) && !queue.empty()){
					struct queueElement newElement;
					rtt_start = gr::high_res_timer_now();
					// if (cnt < 10000 ){
					// 	mac_send[cnt] = (gr::high_res_timer_now() / tpus);
					// }
					newElement = queue[0];
					if(newElement.retransmission == 1 || d_method == 2 || d_method == 4){
						queue.erase(queue.begin());
					} else {
						queue[0].retransmission = queue[0].retransmission - 1;
					}
					//????????????
					aoiValues[newElement.seq].sent = d_seq_timeslot;
					mac_send[newElement.seq] = (gr::high_res_timer_now() / tpus);
					
					message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
						pmt::make_blob(newElement.data, newElement.len)));
					message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
						pmt::make_blob("1", 1)));
					// cnt++;
					// printf("------queue size = %d\n", queue.size());

				}
				if ((gr::high_res_timer_now() - d_last_pack_recieved) > (gr::high_res_timer_tps()*5) && d_last_pack_recieved != 0){
					d_last_pack_recieved = 0;
					std::ofstream tmpfile;
					time_t now = time(0);
					tm *ltm = localtime(&now);
					char file_name [255];
					sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACrttduration(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule, d_method);
					tmpfile.open (file_name);
					if(!tmpfile){
						printf("could not opened\n");
					}
					for(i = 0; i < 10000; i++){
						tmpfile <<  app_to_mac[i] << "," << mac_send[i] << "," << ack_receive[i] << "," << mac_to_app[i] << std::endl;
						// printf("%d, ", (rtt_measure[i]));
						if(i%1000 == 9)
							 printf("%d\n",i);
					}
					tmpfile.close();
					sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACqueue(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule,d_method);
					tmpfile.open (file_name);
					while(!queueValues.empty()){
						struct queueMeasureElement toWrite = queueValues[0];
						queueValues.erase(queueValues.begin());
						tmpfile << toWrite.timeslot_seq << "," << toWrite.queuesize << std::endl;
					}
					tmpfile.close();
					sprintf(file_name,"../../results/%d-%d-%d_%d-%d-%dMACaoi(%d)Method%d.csv",(1900 + ltm->tm_year),(1+ltm->tm_mon), ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec, d_schedule,d_method);
					tmpfile.open (file_name);
					i = 0;
					while(i < 10000){
						struct aoiMeasurementElement toWrite = aoiValues[i];
						tmpfile << toWrite.sequence << "," << toWrite.arrival << "," << toWrite.sent << "," << toWrite.answer << std::endl;
						i++;
					}
					tmpfile.close();
	      			printf("end of plant\n");	
	        	}
	      		d_current_slotframe = d_current_slotframe << 1 | ( 0x1 & (d_slotframe>>(d_slot_len-d_last_position)));
	      		d_last_position = (d_last_position + 1) % d_slot_len;
	    	}
		}

	} catch(boost::thread_interrupted) {
		gr::thread::scoped_lock(d_mutex);
		d_finished = true;
	}
}

void mac_plant_in(pmt::pmt_t msg) {
	pmt::pmt_t blob;
	//@yagiz
	//dout << "MAC: new message received:\n" << std::endl;

	if(pmt::is_pair(msg)) {
		blob = pmt::cdr(msg);
	} else {
		assert(false);
	}

    // printf("MAC: packet received, \n");

	size_t data_len = pmt::blob_length(blob);
	unsigned char buf[256];
	std::memcpy(buf, pmt::blob_data(blob), data_len);
	// printf("%ld\n", data_len);
	// for(int i = 0; i < data_len; i++) {
	// 	printf("%02x ", buf[i]);
	//   	if(i % 16 == 15) {
	//     	printf("\n");
	//   	}
	// }
	// printf("\n");
	uint16_t fcf = (buf[1] << 8) | buf[0];
	uint8_t mac_seq = buf[2];
	uint16_t dest = buf[5] | buf[6]<<8;
	if(data_len < 11) {
		// printf("MAC: frame too short. Dropping!\n");
		return;
	}

	uint16_t crc = crc16((char*)pmt::blob_data(blob), data_len);
	if(crc) {
		d_num_packet_errors++;
		dout << "MAC: wrong crc. Dropping packet!" << std::endl;
		return;
	}

	if((fcf & 0x0007) == 0) {
		uint16_t dur = buf[11] | buf[12] << 8;
		d_slot_len = buf[10] + 1;
    	int i = 0;
    	for (i = 0; i < 32; i++){
      		d_current_slotframe = d_current_slotframe << 1;
      		if(buf[(12 + i%d_slot_len)] == d_schedule && (i%d_slot_len != 0)){
        		d_current_slotframe = 0x01 | d_current_slotframe;
      		}
  		}
	  	d_last_position = i%d_slot_len;
	  	d_slotframe = d_current_slotframe >> (31 - d_slot_len);
	  	d_seq_timeslot = buf[12 + d_slot_len] << 24 | buf[13 + d_slot_len] << 16 | buf[14 + d_slot_len] << 8 | buf[15 + d_slot_len];
	  	// printf("MAC: beacon received,time =%lld, tps = %lld, slotframe = %03x -- %08x, dur = %d, last = %d\n", (gr::high_res_timer_now() - d_tnow), gr::high_res_timer_tps(), d_slotframe, d_current_slotframe, dur, d_last_position);
	  	d_slot_dur = (gr::high_res_timer_tps()/1000) * dur;
	  	d_tnow = gr::high_res_timer_now();
	    //queue.erase()
		return;
	} else if((fcf & 0x0007) == 0x0002 & dest == d_src) {
		uint8_t ack_seq = buf[9];
		uint32_t pck_seq = findAndRemoveFromQueue(ack_seq);
		if(pck_seq < 10000){
			ack_receive[pck_seq] = (gr::high_res_timer_now() / tpus);
		}
		return;
	} else if(dest != d_schedule && dest != 0xFF ){
		return;
	}
	uint32_t seq = (buf[20] << 24) | (buf[19] << 16) | (buf[18] << 8) | buf[17];
	d_num_packets_received++;
	
	if(seq < 9999){
		// rtt_measure[rtt_count] = (gr::high_res_timer_now() - rtt_start) / tpus;
		aoiValues[seq].answer = d_seq_timeslot;
		if(mac_to_app[seq] == 0)
			mac_to_app[seq] = (gr::high_res_timer_now() / tpus);

	} else if(seq == 9999){
		mac_to_app[seq] = (gr::high_res_timer_now() / tpus);
		// rtt_measure[rtt_count] = (gr::high_res_timer_now() - rtt_start) / tpus;
		aoiValues[seq].answer = d_seq_timeslot;

	}

	pmt::pmt_t mac_plant_payload = pmt::make_blob((char*)pmt::blob_data(blob) + 9 , data_len - 9 - 2);

	message_port_pub(pmt::mp("app out"), pmt::cons(pmt::PMT_NIL, mac_plant_payload));
}

void app_in(pmt::pmt_t msg) {
	pmt::pmt_t blob;
	//@yagiz
	mac_wait_start = gr::high_res_timer_now();
	dout << "MAC: message sent:" << std::endl;
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

	size_t data_len = pmt::blob_length(blob);
	unsigned char buf[256];
	std::memcpy(buf, pmt::blob_data(blob), data_len);
	uint32_t seq = (buf[7] << 24) | (buf[6] << 16) | (buf[5] << 8) | buf[4];
	// printf("%d\n", seq);
	struct aoiMeasurementElement newElement;
	newElement.sequence = seq;
	newElement.arrival = d_seq_timeslot;
	newElement.sent = 0;
	newElement.answer = 0;
	aoiValues[seq] = newElement;
	app_to_mac[seq] = mac_wait_start / tpus;

	//dout << "MAC: received new message from APP of length " << pmt::blob_length(blob) << std::endl;

	generate_mac_plant((const char*)pmt::blob_data(blob), pmt::blob_length(blob));
	//add ACK-enable


	// print_message();
	// @yagiz
  	addToQueue(d_msg, d_msg_len, seq);
  	if(seq == 9999){
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

void addToQueue(char *pck, int len, uint32_t seq_no) {
  	struct queueElement newElement;
  	newElement.len = d_msg_len;
  	newElement.seq = seq_no;
  	newElement.retransmission = d_retransmission_attempt;
	int i;
	for (i = 0; i < len; i++) {
		newElement.data[i] = pck[i];
	}
	if(d_method == 1 & !queue.empty()){
		while(!queue.empty()){
			queue.erase(queue.begin());
		}
		queue.push_back(newElement);
	} else if (d_method == 5) {
		message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
			pmt::make_blob(newElement.data, newElement.len)));
		message_port_pub(pmt::mp("pdu out"), pmt::cons(pmt::PMT_NIL,
			pmt::make_blob("1", 1)));
	} else if (queue.size() < d_queue_size){
		queue.push_back(newElement);
	} else if (d_method == 3 || d_method == 4){
		while(queue.size() >= d_queue_size){
			queue.erase(queue.begin());
		}
		queue.push_back(newElement);
	}
}

uint32_t findAndRemoveFromQueue(uint8_t seq) {
	
	for(int i = 0; i < queue.size(); i++){
		//printf("remove from queue, queuesize = %d, seq = %d\n", queue.size(), seq);
		uint8_t taken = queue[i].data[2];
		if(seq == taken){
			//printf("found in queue\n");
			uint32_t real_seq = ((queue[i].data[16]&0xff) << 24) | ((queue[i].data[15]&0xff) << 16) | ((queue[i].data[14]&0xff) << 8) | (queue[i].data[13]&0xff);
			queue.erase(queue.begin() + i);
			return real_seq;
		}
	}
	return 0;
}

void generate_mac_plant(const char *buf, int len) {

	// FCF
	// data frame, no security
	d_msg[0] = d_fcf & 0xFF;
	if(d_method == 0 || d_method == 1 || d_method == 3){
		d_msg[0] = d_msg[0] | 0x20;
	}
	//printf("%c", d_msg[0]);
	d_msg[1] = (d_fcf>>8) & 0xFF;

	// seq nr
	d_msg[2] = d_seq_nr++;

	// addr info
	d_msg[3] = d_dst_pan & 0xFF;
	d_msg[4] = (d_dst_pan>>8) & 0xFF;
	d_msg[5] = d_dst & 0xFF;
	d_msg[6] = (d_dst>>8) & 0xFF;
	d_msg[7] = d_src & 0xFF;
	d_msg[8] = (d_src>>8) & 0xFF;

	std::memcpy(d_msg + 9, buf, len);

	uint16_t crc = crc16(d_msg, len + 9);

	d_msg[ 9 + len] = crc & 0xFF;
	d_msg[10 + len] = crc >> 8;

	d_msg_len = 9 + len + 2;
}

int generate_mac_plant_ack(char *buf, int len) {

	// FCF
	// data frame, no security
	buf[0] = d_fcf & 0xFF;
	//printf("%c", d_msg[0]);
	buf[1] = (d_fcf>>8) & 0xFF;

	// seq nr
	buf[2] = d_seq_nr++;

	// addr info
	buf[3] = d_dst_pan & 0xFF;
	buf[4] = (d_dst_pan>>8) & 0xFF;
	buf[5] = d_dst & 0xFF;
	buf[6] = (d_dst>>8) & 0xFF;
	buf[7] = d_src & 0xFF;
	buf[8] = (d_src>>8) & 0xFF;

	//std::memcpy(d_msg + 9, buf, len);

	uint16_t crc = crc16(buf, len + 9);

	buf[ 9 + len] = crc & 0xFF;
	buf[10 + len] = crc >> 8;

	return (9 + len + 2);
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

    int get_num_packet_errors(){ return d_num_packet_errors; }

    int get_num_packets_received(){ return d_num_packets_received; }

    float get_packet_error_ratio(){ return float(d_num_packet_errors)/d_num_packets_received; }

    private:
    	bool        d_debug;
    	int         d_msg_offset;
    	int         d_msg_len;
    	uint16_t    d_fcf;
    	uint8_t     d_seq_nr;
    	uint16_t    d_dst_pan;
    	uint16_t    d_dst;
    	uint16_t    d_src;
    	char        d_msg[256];
    	char        response[20];
    	// picking 10 as the array size
    	// std::vector<std::array<char, 10>> va;
    	uint8_t    	d_queue_size = 5;
    	uint32_t    d_method;
    	std::vector<queueElement> queue;
      	gr::high_res_timer_type d_tnow;
      	int loopStarted = 0;
      	uint16_t	d_schedule;
      	long long int    d_slot_dur = 0;
      	uint32_t    d_seq_timeslot = 0;
      	uint32_t    d_current_slotframe = 0;
      	uint32_t    d_slotframe = 0;
      	uint8_t     d_last_position;
      	uint8_t     d_slot_len;
      	uint8_t		d_retransmission_attempt = 2;
      	uint16_t    d_old_queue_size = 0;
      	uint64_t 	mac_to_app[10000];
      	uint64_t 	ack_receive[10000];
      	long long int 	d_last_pack_recieved = 0;
      	long long int 	rtt_start;
      	uint64_t 	mac_send[10000];
      	uint64_t 	app_to_mac[10000];
      	long long int 	mac_wait_start;
      	uint16_t		rtt_count = 0;
      	long long int tpus = (gr::high_res_timer_tps() / 1000000);
      	std::vector<queueMeasureElement>	queueValues;
      	aoiMeasurementElement	aoiValues[10000];


      	bool d_finished = false;
      	boost::thread *d_thread;
      	gr::thread::mutex d_mutex;

    	int d_num_packet_errors;
    	int d_num_packets_received;
    };


    mac_plant::sptr
    mac_plant::make(bool debug, int method, int fcf, int seq_nr, int dst_pan, int dst, int src, int schedule) {
    	return gnuradio::get_initial_sptr(new mac_plant_impl(debug,method,fcf,seq_nr,dst_pan,dst,src,schedule));
    }

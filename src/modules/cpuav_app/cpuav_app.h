#ifndef __CPUAV_APP__
#define __CPUAV_APP__

///constants///
#define NUM_INTERVALS 10
#define USEC_PSEC 1000000.0
#define MSEC_PUSEC 1000.0

#define SYNC 1129336832
#define DEST_ID 0
#define VEHICLE_ID 101
#define SEQUENCE 0
#define TTL 0

#define NUM_POS 10

#define MSG_TYPE_STATUS 2000
#define MSG_TYPE_POSITION 2002
#define MSG_LEN_STATUS 12
#define MSG_LEN_POSITION 32
#define PACKET_LEN_STATUS 25
#define PACKET_LEN_POSITION 45

///error constants///
#define E_TIMEOUT    -1

///typedefs///
/*
 * There are two types of packets: "position", and "system status".
 * "Position" packets are exchanged with the other plane.
 * "System" packets are sent to the ground station.
 */
typedef struct {
   int32_t sync;
   uint8_t source_id;
   uint8_t dest_id;
   uint8_t seq;
   uint8_t ttl;
   uint16_t message_type;
   uint16_t message_length;
   int8_t checksum;
} header_t; // __attribute__((packed));

//Struct for vehicle status
typedef struct {
	int64_t time_stamp;
	uint16_t vehicle_id;
 	uint8_t vehicle_mode;
	uint8_t vehicle_state;
} status_body_t; // __attribute__((packed));

/*
//Struct for global position
typedef struct {
	uint64_t timestamp;
	uint16_t vehicle_id;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t heading;
	int16_t x_speed;
	int16_t y_speed;
	int16_t z_speed;
} pos_body_t __attribute__((packed));
*/

//Packet for vehicle status
typedef struct {
	header_t header;
   status_body_t body;
} status_packet_t; //__attribute__((packed));

//Packet for global position
typedef struct {
	header_t header;
   pos_body_t body;
} pos_packet_t; //__attribute__((packed));


///prototypes///
static int uart_init(const char *device);
static int xbee_send(int fd, void *buf, int numBytes, double timeoutMs);
static int xbee_recv(int fd, void *buf, int requested, double timeoutMs);
static int get_our_gps(pos_body_t *us, int sensorSubFd);
static int get_other_gps(int fd, pos_body_t *them);
int send_our_gps(void *us, int fd, bool isPos);

#endif

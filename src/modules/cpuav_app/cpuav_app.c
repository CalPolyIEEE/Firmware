/**
 * @file cpuav_app.c
 * Application for first iteration of system.
 * Includes functions for sending and receiving via XBEE,
 * initializing our UART, sense and avoid algorithm and
 * other necessary helper functions.
 */

#include <errno.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <nuttx/config.h>
#include <poll.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "sanda.h"
#include "cpuav_app.h"

__EXPORT int cpuav_app_main(int argv, char *argc[]);

/* static void pos_body_to_gps_pos(pos_body_t* source, struct gps_pos* dest); */

int cpuav_app_main(int argv, char *argc[]) {
	bool isPos = true, test = false;
   pos_body_t *us, *them, next, *goal;
   int fd, res, iteration1 = 0, iteration2 = 0, gpsFd;
   float printLat, printLon;

   /* Variables for use in testing mode */

   us = malloc(NUM_POS * sizeof(pos_body_t));
   them = malloc(NUM_POS * sizeof(pos_body_t));
   goal = malloc(sizeof(pos_body_t) * 2);

   memset(us, 0, NUM_POS * sizeof(pos_body_t));
   memset(them, 0, NUM_POS * sizeof(pos_body_t));
   memset(goal, 0, 2 * sizeof(pos_body_t));
   memset(&next, 0, sizeof(pos_body_t));

   ///initialization code///
   gpsFd = orb_subscribe(ORB_ID(vehicle_gps_position));
   orb_set_interval(gpsFd, 1000);

   fd = uart_init("/dev/ttyS2");
   if (fd < 0) {
      perror("uart_init");
      return EXIT_FAILURE;
   }

   printf("Initializing\n");

   if(argv > 1) {
      printf("Test Mode\n");
      test = true;

      initialize_us(us, &goal[0]);
      initialize_them(them, &goal[1]);
   }

   ///main loop///
   while (true) {
      if(!test) {
	      if ((res = get_our_gps(&us[0], gpsFd)) < 0)
		      fprintf(stderr, "get_our_gps failed with error %d\n", res);
         else {
            memmove(&us[1], us, (NUM_POS - 1) * sizeof(pos_body_t));

            if ((res = send_our_gps(&us[0], fd, isPos)) < 0)
               fprintf(stderr, "xbee_send failed with error %d\n", res);
         }

         if ((res = get_other_gps(fd, &them[0])) < 0)
            fprintf(stderr, "get_other_gps failed\nEither no nearby planes or an error has occured (Error %d)\n", res);
         else
            memmove(&them[1], them, (NUM_POS - 1) * sizeof(pos_body_t));
      }
      else {
         if(memcmp(us, &us[1], sizeof(pos_body_t))) {
            iteration1++;
            printLat = to_deg(us[0].lat);
            printLon = to_deg(us[0].lon);

            printf("Our GPS:\t%d, %d.%.7d, %d.%.7d\t",
             iteration1, (int)printLat, (int)((printLat-(int)printLat)*10000000),
             (int)printLon, (int)((printLon-(int)printLon)*10000000));

            memmove(&us[1], us, (NUM_POS - 1) * sizeof(pos_body_t));

            get_next_waypoint(us, them, &next, goal);
         }
         
         if(memcmp(them, &them[1], sizeof(pos_body_t))) {
            iteration2++;
            printLat = to_deg(them[0].lat);
            printLon = to_deg(them[0].lon);

            printf("Their GPS:\t%d, %d.%.7d, %d.%.7d\n",
             iteration2, (int)printLat, (int)((printLat-(int)printLat)*10000000),
             (int)printLon, (int)((printLon-(int)printLon)*10000000));

            memmove(&them[1], them, (NUM_POS - 1) * sizeof(pos_body_t));
            get_next_waypoint(us, them, &next, goal);
         }

         gen_dummy_plane(them, &goal[1]);
         gen_dummy_plane(us, &next);
      }
   }

   return 0;
}

/**
 * @brief Opens and configures a UART port for data transfer.
 * @param device String name of the port to be opened.
 * @return File descriptor of the opened device.
 */
static int uart_init(const char* device) {
   struct termios options;
   int fd;

   fd = open(device, O_RDWR);
   if (fd == -1) {
      perror("open");
      return errno;
   }

   if (tcgetattr(fd, &options) == -1) {
      perror("tcgetattr");
      return errno;
   }

   cfsetospeed(&options, B9600);
   cfsetispeed(&options, B9600);
   options.c_cflag &= ~CSTOPB;  // 1 stop bit
   options.c_cflag |= CLOCAL;   // Ignore modem control lines
   options.c_cflag |= CS8;      // 8-bit width
   options.c_cflag &= ~PARENB;  // Disable parity generation
   options.c_cflag &= ~CRTSCTS; // Disable hardware flow control

   if (tcsetattr(fd, TCSANOW, &options) == -1) {
      perror("tcsetattr");
      return errno;
   }

   return fd;
}

/**
 * @brief Sends a buffer of data via UART port.
 * @param fd File descriptor of the UART port.
 * @param buf Buffer of data to send.
 * @param numBytes Number of bytes to be sent.
 * @param timeoutMs Function timeout in milliseconds.
 * @return 0 is success, -1 is E_TIMEOUT, any other positive
 *         number is a poll error.
 */
static int xbee_send(int fd, void *buf, int numBytes, double timeoutMs) {
   double interval = timeoutMs / NUM_INTERVALS, timeDiff;
   struct pollfd fds[1];
   int bytesSent = 0, res;
   struct timeval start, current;

   fds[0].fd = fd;
   fds[0].events = POLLOUT;

   if (gettimeofday(&start, NULL) == -1) {
      perror("gettimeofday");
      return errno;
   }

   while (bytesSent < numBytes) {
      if ((res = poll(fds, 1, interval)) == -1) {
         perror("poll");
         return errno;
      } else if (res == 0) {
         // If no event occured on the file descriptor in the time given
      } else {
         // We can now send data on the xbee.
         if ((res = write(fd, (char *)buf + bytesSent, numBytes - bytesSent)) == -1) {
            perror("read");
            return errno;
         } else {
            bytesSent += res;
         }
      }

      if (gettimeofday(&current, NULL) == -1) {
         perror("gettimeofday");
         return errno;
      }

      timeDiff = (current.tv_sec * USEC_PSEC + current.tv_usec) -
         (start.tv_sec * USEC_PSEC + start.tv_usec);

      if (timeDiff / MSEC_PUSEC > timeoutMs) {
         break;
      }
   }

   if (bytesSent < numBytes) {
      return E_TIMEOUT;
   } else {
      return 0;
   }
}

/**
 * @brief Receives data from xbee connected via UART.
 * @param fd File descriptor of port to read data from.
 * @param buf Buffer to read data into.
 * @param requested Number of bytes requested to read.
 * @param timeoutMs Millisecond value defining timeout period.
 * @return 0 is success, -1 is E_TIMEOUT, any other negative number
 *         is a poll error.
 */
static int xbee_recv(int fd, void* buf, int requested, double timeoutMs) {
   double interval = timeoutMs / NUM_INTERVALS, timeDiff;
   struct pollfd fds[1];
   int bytesReceived = 0, res;
   struct timeval start, current;

   fds[0].fd = fd;
   fds[0].events = POLLIN;

   if (gettimeofday(&start, NULL) == -1) {
      perror("gettimeofday");
      return errno;
   }

   while (bytesReceived < requested) {
      if ((res = poll(fds, 1, interval)) == -1) {
         perror("poll");
         return errno;
      } else if (res == 0) {
         // If no event occured on the file descriptor in the time given
      } else {
         // There is a poll event on the file descriptor
         // And we know it is POLLIN bc that is the only event
         // we are listening for.
         if ((res = read(fd, (char *)buf + bytesReceived, requested -
                         bytesReceived)) == -1) {
            perror("read");
            return errno;
         } else {
            bytesReceived += res;
         }
      }

      if (gettimeofday(&current, NULL) == -1) {
         perror("gettimeofday");
         return errno;
      }

      timeDiff = (current.tv_sec * USEC_PSEC + current.tv_usec) -
         (start.tv_sec * USEC_PSEC + start.tv_usec);

      if (timeDiff / MSEC_PUSEC > timeoutMs) {
         break;
      }
   }

   if (bytesReceived < requested) {
      printf("Bytes Received: %d, Bytes Requested: %d\n", bytesReceived,
             requested);
      return E_TIMEOUT;
   } else {
      return 0;
   }
}

int send_our_gps(void *us, int fd, bool isPos) {
	int ret = 0;
	pos_packet_t posPacket;
	status_packet_t statPacket;
	header_t header;

	header.sync = SYNC;
	header.source_id = VEHICLE_ID;
	header.dest_id = DEST_ID;
	header.seq = SEQUENCE;
	header.ttl = TTL;
	header.message_type = htons(isPos ? MSG_TYPE_POSITION : MSG_TYPE_STATUS);
	header.message_length = htons(isPos ? MSG_LEN_STATUS : MSG_LEN_POSITION);
	header.checksum = 0;
	
	if(isPos) {
		memcpy(&posPacket.header, &header, sizeof(header_t));
		memcpy(&posPacket.body, (pos_body_t *)us, sizeof(pos_body_t));
		ret = xbee_send(fd, &posPacket, sizeof(pos_packet_t), 2000);
	} else {
		memcpy(&statPacket.header, &header, sizeof(header_t));
		memcpy(&statPacket.body, (status_body_t *)us, sizeof(status_body_t));
		ret = xbee_send(fd, &statPacket, sizeof(status_packet_t), 2000);
	}
	return ret;
}

/**
 * @brief Polls and receives "our" current gps location from the gps module.
 *	@param myGps Struct to be filled with "our" gps point.
 * @return Positive: activity on the polled module. Zero: No activity.
 *         Negative: poll error.
 */
static int get_our_gps(pos_body_t *us, int sensorSubFd) {
   struct vehicle_gps_position_s loc;
   int res = 0;
   int pollCount = 0;
   struct pollfd fds[1];

   memset(&loc, 0, sizeof(struct vehicle_gps_position_s));

   fds[0].fd = sensorSubFd;
   fds[0].events = POLLIN;

   do {
      res = poll(fds, 1, 1000);
      if (res < 0) {
         return res;
      }
   } while (res == 0 && (pollCount++ < 5));

   if (fds[0].revents & POLLIN) {
      orb_copy(ORB_ID(vehicle_gps_position), sensorSubFd, &loc);

      // Maybe we should be using "timestamp_time? Not sure."
      us->timestamp = loc.timestamp_time;
      us->vehicle_id = htons(VEHICLE_ID);
      us->lat = loc.lat;
      us->lon = loc.lon;
      us->alt = loc.alt;
      us->x_speed = loc.vel_e_m_s;
      us->y_speed = loc.vel_n_m_s;
      us->z_speed = loc.vel_d_m_s;
      us->heading = 0; /*FIXMEFIXME*/
   }

   return res;
}

/* static void pos_body_to_gps_pos(pos_body_t* source, struct gps_pos* dest) { */
/*    dest->lat = source->lat; */
/*    dest->lon = source->lon; */
/*    dest->timestemp = source->timestamp; */
/* } */

/**
 * @brief Wrapper function that uses xbee_recv function and processes its
 *        results.
 *	@param fd File descriptor for port to read from.
 *	@param othGps Struct to be filled with read data from port.
 * @return 0 for success, any other number is an error with the read function.
 */
static int get_other_gps(int fd, pos_body_t *othGps) {
   pos_packet_t theirLoc;
   int error;

   memset(&theirLoc, 0, sizeof(pos_packet_t));

   error = xbee_recv(fd, (char *)(&theirLoc), sizeof(pos_packet_t), 2000);

	memcpy(othGps, &theirLoc.body, sizeof(pos_body_t));

   if(!error) {
		printf("lat: %x, lon: %x\n", othGps->lat, othGps->lon);
      /*othGps->lat = ntohl(theirLoc.lat);
      othGps->lon = ntohl(theirLoc.lon);
		printf("lat: %x, lon: %x\n", othGps->lat, othGps->lon);
      othGps->timestamp = ntohl(theirLoc.timestamp);*/
   }

   return error;
}

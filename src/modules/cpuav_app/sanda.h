#ifndef _SANDA_H_
#define _SANDA_H_

#define RADIUS_EARTH 6371160
#define TIME_CONST 3000000

#define MAX_OBJECTS 5
#define MAX_HISTORY 10

//#define DEBUG 1

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
} pos_body_t; // __attribute__((packed));

void get_next_waypoint(pos_body_t *myGps, pos_body_t *othGps,
                       pos_body_t *nextGps, pos_body_t *goal);
void gen_dummy_plane(pos_body_t *othGps, pos_body_t *goal);
void initialize_us(pos_body_t *gps, pos_body_t *goal);
void initialize_them(pos_body_t *gps, pos_body_t *goal);
float to_deg(int32_t value);

#endif

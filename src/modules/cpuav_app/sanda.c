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
#include <time.h>
#include <unistd.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "sanda.h"

typedef struct dist_hist {
   int32_t vehicle_id;
   int32_t dist[MAX_HISTORY];
   void *next;
} dist_hist;

static int32_t get_dist(pos_body_t *myGps, pos_body_t *othGps);
//static void get_midpoint(pos_body_t *gps1, pos_body_t *gps2, pos_body_t *midpoint);
static int32_t get_vel(pos_body_t *gps);
static float get_dir(pos_body_t *gps);
static float get_angle(pos_body_t *gps1, pos_body_t *gps2);
static void next_waypoint(pos_body_t *next, float dir, double mag);

static float deg_to_rad(float deg);
static float rad_to_deg(float rad);
static int32_t from_deg(float deg);
static float to_rad(int32_t value);
static int32_t from_rad(float rad);
static float mod_dir(float dir);

/**
 * @brief Core of sense and avoid functionality. Takes in obstacle gps and
 *        returns a waypoint for the autopilot.
 * @param myGps Array of two location structs for this plane.
 * @param othGps Array of obstacle location structs.
 * @param nextGps Will be filled with the next waypoint.
 * @param numObstacles Length of the othGps array
 * @return None, its void!
 */
void get_next_waypoint(pos_body_t *myGps, pos_body_t *othGps,
                              pos_body_t *nextGps, pos_body_t *goalGps) {
   static int32_t kickTimer = 0;
   static float newDir;

   double mag;
   float myDir, othDir,
         betDir, relDir,
         closingAngle = 0;
   //uint8_t counter;
   int32_t dist,
           myVel, othVel,
           closingVel, closingTime;
   //pos_body_t mid;

   memcpy(nextGps, myGps, sizeof(pos_body_t));

   /* Calculate pull to the goal */
   next_waypoint(nextGps, get_angle(myGps, goalGps), 30.0);
   //printf("Pull in the direction of %.2f with a magnitude of 30\n", get_angle(goalGps, myGps));

   if(othGps->vehicle_id) {

      /* Calculate necessary values */
      /* Distance from GPS coordinates */
      dist = get_dist(myGps, othGps);

#ifdef DEBUG
      printf("\tDistance %li\n", dist);
#endif

      /* Absolute velocity from directional velocities */
      myVel = get_vel(myGps);
      othVel = get_vel(othGps);

#ifdef DEBUG
      printf("\tVelocities: %ld, %ld\n", myVel, othVel);
      printf("\t\t%ld, %ld & %ld, %ld\n", myGps->x_speed, myGps->y_speed, othGps->x_speed, othGps->y_speed);
#endif

      /* Calculate relative velocity
      for(counter = 1; counter < MAX_HISTORY; counter++) {
         if(!othGps[counter].vehicle_id)
            break;
         printf("%li / %llu\n", get_dist(&othGps[counter], myGps), (myGps->timestamp - myGps[counter].timestamp) / 1000000.0);
         relVel += get_dist(&othGps[counter], myGps) / ((myGps->timestamp - myGps[counter].timestamp) / 1000000.0);
      }
      relVel /= counter;

      printf("\tRelative velocity: %d\n", relVel);

      */

      /* Calculate headings from directional velocities */
      myDir = get_dir(myGps);
      othDir = get_dir(othGps);

#ifdef DEBUG
      printf("\tHeadings: Ours: %d%.2d, Theirs: %d%.2d\n", 
       (int)myDir, (int)((myDir - (int)myDir) * 100), 
       (int)othDir, (int)((othDir - (int)othDir) * 100));
#endif

      /* Calculate angle to the object and relative heading */
      betDir = get_angle(myGps, othGps);
      relDir = mod_dir(othDir - myDir);

#ifdef DEBUG
      printf("\tBearing: %d%.2d and relative heading: %d%.2d\n", 
       (int)betDir, (int)((betDir - (int)betDir) * 100),
       (int)relDir, (int)((relDir - (int)relDir) * 100));

      printf("Pull in the direction of %.2f with a magnitude of 30\n", get_angle(myGps, goalGps));
#endif

      /* Calculate opposite push */
      if((betDir >= 270 || betDir <= 90)
       || (betDir <= 180 && relDir >= 270)
       || (betDir >= 180 && relDir <= 90))
         mag = 100.0 * exp(-0.0425 * dist);
      else
         mag = 0;

      next_waypoint(nextGps, mod_dir(betDir + 180), mag);
#ifdef DEBUG
      printf("Push in the direction of %.2f with magnitude %.2f\n", mod_dir(betDir + 180.0), mag);
#endif

      /* Calculate drift */
      // Determine if the paths intersect
      if((betDir >= 180 && relDir <= betDir - 180)
       || (betDir <= 180 && relDir >= betDir)) {

         // Generate the magnitude
         if(betDir >= 180)
            closingAngle = mod_dir(betDir - relDir - 180);
         else if(betDir < 180)
            closingAngle = mod_dir(relDir - betDir - 180);

         closingVel = (fabs((cos(betDir) * myVel)) + fabs((cos(closingAngle) * othVel)));
         closingTime = dist / closingVel;
#ifdef DEBUG
         printf("Kicktimer: %d, Closing angle: %.2f, velocity: %ld, time: %ld\n", kickTimer, closingAngle, closingVel, closingTime);
#endif

         if(closingTime < 20 && closingTime > 5)
            mag = 60.0 * exp(-0.06 * closingTime);
         else
            mag = 0;

         // Determine whether to cross or speed up
         if(kickTimer <= 0 && mag > 0) {
            if((relDir >= betDir - 180 && relDir <= betDir) || (relDir <= betDir + 180 && relDir >= betDir))
               newDir = mod_dir(relDir < 180 ? betDir - 90 : betDir + 90);
            else
               newDir = get_angle(myGps, goalGps);

            kickTimer = 20;
         }
         else
            kickTimer--;

         next_waypoint(nextGps, newDir, mag);
#ifdef DEBUG
         printf("Drift in the direction of %.2f with magnitude %.2f\n", newDir, mag);
#endif
      }
      else
         kickTimer = 0;
#ifdef DEBUG
      printf("New waypoint in the direction of %.2f with magnitude %d\n\n", get_angle(myGps, nextGps), get_dist(myGps, nextGps));
#endif
   }
}

/**
 * @brief Calculates the distance between two gps points. The formula used
 * calculates the great circle distance
 * @param myGps Poiter to the struct holding the first gps location.
 * @param othGps Pointer to the struct holding the second gps location.
 * @return The distance between passed in gps points in meters.
 */
static int32_t get_dist(pos_body_t *gps1, pos_body_t *gps2) {
   float dLat, dLon, a, b;
   dLat = to_rad(gps1->lat) - to_rad(gps2->lat);
   dLon = to_rad(gps1->lon) - to_rad(gps2->lon);

   a = sin(dLat / 2) * sin(dLat / 2)
      + cos(to_rad(gps2->lat))
      * cos(to_rad(gps1->lat))
      * sin(dLon / 2) * sin(dLon / 2);

   b = 2 * atan2(sqrt(a), sqrt(1 - a));

   return RADIUS_EARTH * b;
}

/*
static void get midpoint(pos_body_t *gps1, pos_body_t *gps2, pos_body_t *midpoint) {
   float dLon, x, y;

   dLon = to_rad(gps1->lon) - to_rad(gps2->lon);

   x = cos(to_rad(gps2->lat)) * cos(dLon);
   y = cos(to_rad(gps2->lat)) * sin(dLon);

   midpoint->lat = from_rad(atan2(sin(to_rad(gps1->lat)) + sin(to_rad(gps2->lat)),
     sqrt((cos(to_rad(gps1->lat)) +  x) * (cos(to_rad(gps1->lat)) + x) + y * y)));
   midpoint->lon = from_rad(to_rad(gps1->lon) + atan2(y, cos(to_rad(gps1->lat)) + x));
}
*/

/**
 * @brief Calculates the total velocity based on the velocity components given
 * in the pos_body_t struct
 * @param gps
 * @return Velocity in meters per second.
 */
static int32_t get_vel(pos_body_t *gps) {
   return sqrt(gps->x_speed * gps->x_speed + gps->y_speed * gps->y_speed);
}
/*static float get_vel(pos_body_t *gps1, pos_body_t *gps2) {
   return get_dist(gps2, gps1) / ((gps2->timestamp - gps1->timestamp) / 1000000);
}*/


/** 
 * @brief Calculates the heading based on the velocity components given in the
 * pos_body_t struct.
 * @param gps
 * @return Heading, in degrees, between 0 and 359.
 */
static float get_dir(pos_body_t *gps) {
   return mod_dir(rad_to_deg(atan2(gps->x_speed, gps->y_speed)));
}

/**
 * @brief Calculates the angle from one position to the next as degrees
 * clockwise from North.
 * @param gps1 The object positioned at origin
 * @param gps2 The object to which an angle is calculated
 * @return The bearing of object 2 from the perspective of object 1, in 
 * degrees, between 0 and 359
 */
static float get_angle(pos_body_t *gps1, pos_body_t *gps2) {
   float dLon, y, x;
   
   dLon = to_rad(gps2->lon) - to_rad(gps1->lon);

   y = sin(dLon) * cos(to_rad(gps2->lat));
   x = cos(to_rad(gps1->lat)) * sin(to_rad(gps2->lat)) - sin(to_rad(gps1->lat)) * cos(to_rad(gps2->lat)) * cos(dLon);

   return mod_dir(rad_to_deg(atan2(y, x)));
}

/**
 * @brief Calculates a waypoint given a starting position, heading, and
 * distance.
 * @param next The current position. The struct is modified to hold the new
 *    position.
 * @param dir The desired direction. 
 * @param mag The desired distance.
 */

static void next_waypoint(pos_body_t *next, float dir, double mag) {
   pos_body_t *temp;

   temp = malloc(sizeof(pos_body_t));
   memcpy(temp, next, sizeof(pos_body_t));

   mag /= RADIUS_EARTH;

   next->lat = from_rad(asin(sin((double)to_rad(temp->lat)) * cos(mag) 
    + cos(to_rad(temp->lat)) * sin(mag) * cos(deg_to_rad(dir))));

   next->lon = from_rad((double)to_rad(temp->lon) + atan2(sin(deg_to_rad(dir)) * sin(mag) * cos(to_rad(temp->lat)), 
    cos(mag) - sin(to_rad(temp->lat)) * sin(to_rad(next->lat))));

   free(temp);
}

/**
 * @brief Helper function to convert radians into degrees.
 * @param deg Degree value to be converted.
 * @return Converted value in radians.
 */
static float deg_to_rad(float deg) {
   return (deg * (float)M_PI) / 180;
}

/**
 * @brief Helper function to convert radians into degrees.
 * @param rad Input value in radians
 * @return Converted value in degrees.
 */
static float rad_to_deg(float rad) {
   return (rad * 180) / (float)M_PI;
}

/**
 * @brief Helper function that converts lat/lon to degree values.
 * @param value 32-bit signed int value to be converted.
 * @return Converted value in degrees.
 */
float to_deg(int32_t value) {
   return value / 10000000.0;
}

static int32_t from_deg(float deg) {
   return deg * 10000000;
}

/**
 * @brief Helper function that converts stored lat/lon to radian values.
 * @param value 32-bit signed int value to be converted.
 * @return Converted value in radians.
 */
static float to_rad(int32_t value) {
   return ((((float)value) / 10000000) * (float)M_PI) / 180;
}

static int32_t from_rad(float rad) {
   return ((rad * 180) / (float)M_PI) * 10000000;
}

static float mod_dir(float dir) {
   if(dir < 0)
      dir += 360;
   else if(dir >= 360)
      dir -= 360;
   return dir;
}

// FUNCTIONS FOR TESTING //

void initialize_us(pos_body_t *gps, pos_body_t *goal) {
#ifdef DEBUG
   printf("Initializing 'us'\n");
#endif

   goal->lat = 352987510;
   goal->lon = -1206582150;

   gps->vehicle_id = 1;
   gps->lat = 353023230;
   gps->lon = -1206582150;
   gps->heading = from_deg(get_angle(gps, goal));
   gps->x_speed = sin(gps->heading) * 10;
   gps->y_speed = sin(gps->heading) * 10;
}

void initialize_them(pos_body_t *gps, pos_body_t *goal) {
#ifdef DEBUG
   printf("Intizializing 'them'\n");
#endif
   goal->lat = 353023230;
   goal->lon = -1206582150;

   gps->vehicle_id = 2;
   gps->lat = 352987510;
   gps->lon = -1206582150;
   gps->heading = from_deg(get_angle(gps, goal));
   gps->x_speed = sin(gps->heading) * 10;
   gps->y_speed = cos(gps->heading) * 10;
}

void gen_dummy_plane(pos_body_t *gps, pos_body_t *goal) {
#ifdef DEBUG
   printf("Iterating movement\n");
#endif

   static time_t timerUs, timerThem;
   time_t timer, currTime = time(NULL);

   switch(gps->vehicle_id) {
      case 1:
         timer = timerUs;
         break;
      case 2:
         timer = timerThem;
         break;
      default:
         timer = currTime;
         break;
   }

   if(currTime - timer > 1) {
      if(gps[1].vehicle_id) {
         if(mod_dir(get_angle(gps, goal) - to_deg(gps[1].heading)) <= 180)
            gps->heading = (0.8 * gps[1].heading) + (0.2 * from_deg(get_angle(gps, goal)));
         else
            gps->heading = (0.8 * gps[1].heading) - (0.2 * from_deg(get_angle(gps, goal)));
      }
      else
         gps->heading = from_deg(get_angle(gps, goal));

      gps->x_speed = (sin(to_rad(gps->heading)) * 10.0) / (currTime - timer);
      gps->y_speed = (cos(to_rad(gps->heading)) * 10.0) / (currTime - timer);

      next_waypoint(gps, to_deg(gps->heading), 10.0);

      switch(gps->vehicle_id) {
         case 1:
            timerUs = time(NULL);
            break;
         case 2:
            timerThem = time(NULL);
            break;
         default:
            break;
      }
   }
}

#include <nuttx/config.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

__EXPORT int ex_hwtest_main(int argc, char *argv[]);

int ex_hwtest_main(int argc, char *argv[]) {
   bool test_pwm = false;
   bool test_att = false;

   for (int i = 1; i < argc; i++) {
      if(strcmp(argv[i], "--pwm") == 0 || strcmp(argv[i], "--actuator") == 0) {
         test_pwm = true;
      }
      if(strcmp(argv[i], "--att") == 0 || strcmp(argv[i], "--attitude") == 0) {
         test_att = true;
      }
   }

   if(test_pwm) {

      struct actuator_controls_s actuators;
      memset(&actuators, 0, sizeof(actuators));
      orb_advert_t actuator_pub_fd = orb_advertise(ORB_ID(actuator_controls_0), &actuators);
   
      struct actuator_armed_s arm;
      memset(&arm, 0, sizeof(arm));

      arm.timestamp = hrt_absolute_time();
      arm.ready_to_arm = true;
      arm.armed = true;
      orb_advert_t arm_pub_fd = orb_advertise(ORB_ID(actuator_armed), &arm);
      orb_publish(ORB_ID(actuator_armed), arm_pub_fd, &arm);

      int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
      orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

      if(arm.ready_to_arm && arm.armed) {
         warnx("Actuator armed");
      }
      else {
         errx(1, "Arming actuators failed");
      }

      hrt_abstime stime;

      int count = 0;

      while(true) {
         stime = hrt_absolute_time();

         while(hrt_absolute_time() - stime < 1000000) {
            for(int i = 4; i != 5; i++) {
               if(count <= 5) {
                  actuators.control[i] = -1.0f;
               }
               else if(count <= 10) {
                  actuators.control[i] = -0.7f;
               }
               else if(count <= 15) {
                  actuators.control[i] = -0.5f;
               }
               else if(count <= 20) {
                  actuators.control[i] = -0.3f;
               }
               else if(count <= 25) {
                  actuators.control[i] = 0.0f;
               }
               else if(count >= 31) {
                  count = 0;
               }
            }
         
            actuators.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
            usleep(10000);
         }

         warnx("count %i", count);
         count++;
      }
   }

   else if(test_att) {
      struct vehicle_attitude_s att;
      memset(&att, 0, sizeof(att));

      int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

      struct pollfd fds[1] = {
         { .fd = att_sub, .events = POLLIN}
      };

      while(true) {
         int ret = poll(fds, 1, 500);

         if(ret < 0) {
            warnx("poll error");
         }
         else if(ret == 0) {
            /* no op */
         }
         else {
            if (fds[0].revents & POLLIN) {
               orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

               printf("roll: %d.%.3d\tpitch: %d.%.3d\tyaw: %d.%.3d\n",
                  (int)att.roll, (int)((att.roll-(int)att.roll)*1000),
                  (int)att.pitch, (int)((att.pitch-(int)att.pitch)*1000),
                  (int)att.yaw, (int)((att.yaw-(int)att.yaw)*1000));
            }
         }
      }
   }

   return 0;
}



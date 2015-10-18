#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_armed.h>

__EXPORT int ex_hwtest_main(int argc, char *argv[]);

int ex_hwtest_main(int argc, char *argv[]) {
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
      warnx("Actautor armed");
   }
   else {
      errx(1, "Arming actuators failed");
   }

   hrt_abstime stime;

   int count = 0;

   while(true) {
      stime = hrt_absolute_time();

      while(hrt_absolute_time() - stime < 1000000) {
         for(int i = 0; i != 2; i++) {
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

   return OK;
}


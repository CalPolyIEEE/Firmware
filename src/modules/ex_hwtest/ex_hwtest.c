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
   char test_att_flags = 0x00;

   // Check arguments for syntax
   if(argc == 2 && strcmp(argv[1], "--att") == 0) {
      test_att = true;
   }
   else if(argc > 2 && strcmp(argv[1], "--pwm") == 0) {
      test_pwm = true;

      for(int i = 2; i < argc; i++) {
         for(unsigned int j = 0; j < strlen(argv[i]); j++) {
            switch(argv[i][j]) {
               case 'r':
                  test_att_flags |= 0x01;
                  break;
               case 'p':
                  test_att_flags |= 0x02;
                  break;
               case 'y':
                  test_att_flags |= 0x04;
                  break;
               case 't':
                  test_att_flags |= 0x08;
                  break;
               case '-':
               default:
                  break;
            }
         }
      }
   }
   else {
      warnx("Usage: ex_hwtest --<argument>");
      warnx("\t--pwm -<r,p,y,t> (test motor control)");
      warnx("\t--att (test attitude sensors)");
      return 0;
   }


   // Test the PWM output to the motors
   if(test_pwm) {
      warnx("Remember to stop commander!");
      warnx("\t<commander stop>");
      warnx("Remember to link the mixer!");
      warnx("\t<mixer load /dev/pwm_output0 /etc/mixers/quad_x.main.mix>");

      // Initialize the struct for passing control information to the motors
      struct actuator_controls_s actuators;
      memset(&actuators, 0, sizeof(actuators));

      // Indicate that we'll be publishing control information for the motors
      orb_advert_t actuator_pub_fd = 
       orb_advertise(ORB_ID(actuator_controls_0), &actuators);
   
      // Initialize the struct for passing arming information
      struct actuator_armed_s arm;
      memset(&arm, 0, sizeof(arm));

      // Indicate that we'll be publishing arming information for the motors
      orb_advert_t arm_pub_fd = orb_advertise(ORB_ID(actuator_armed), &arm);

      // Arm the motors
      arm.timestamp = hrt_absolute_time();
      arm.ready_to_arm = true;
      arm.armed = true;

      // Publish the arming information, indicating new data is available
      orb_publish(ORB_ID(actuator_armed), arm_pub_fd, &arm);

      // Subscribe to the information we just subscribed to verify it
      int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));

      // Read the arming information in from uORB into our struct
      orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

      // Confirm arming of the motors
      if(arm.ready_to_arm && arm.armed) {
         warnx("Actuators armed");
      }
      else {
         errx(1, "Arming actuators failed");
      }

      hrt_abstime stime;
      unsigned int count = 0;
      unsigned int test = 0;

      switch(test_att_flags) {
         case 1:
            test = 0;
            break;
         case 2:
            test = 1;
            break;
         case 4:
            test = 2;
            break;
         case 8:
            test = 3;
            break;
         default:
            break;
      }

      // Incrementally increase output to the motors every 5 seconds
      while(count <= 45) {
         stime = hrt_absolute_time();

         while(hrt_absolute_time() - stime < 1000000) {
            if(count <= 5) {
               actuators.control[test] = -1.0f;
            }
            else if(count <= 10) {
               actuators.control[test] = -0.7f;
            }
            else if(count <= 15) {
               actuators.control[test] = -0.5f;
            }
            else if(count <= 20) {
               actuators.control[test] = -0.3f;
            }
            else if(count <= 25) {
               actuators.control[test] = 0.0f;
            }
            else if(count <= 30) {
               actuators.control[test] = 0.3f;
            }
            else if(count <= 35) {
               actuators.control[test] = 0.5f;
            }
            else if(count <= 40) {
               actuators.control[test] = 0.7f;
            }
            
            // Publish the control information
            actuators.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
            
            // Pass control for 10ms
            usleep(10000);
         }

         warnx("count %i", count);
         count++;
      }
   }

   // Test reading from the attitude sensors
   else if(test_att) {

      // Intialize the struct for reading attitude information
      struct vehicle_attitude_s att;
      memset(&att, 0, sizeof(att));

      // Subscribe to the topic that publishes attitude information
      int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

      // Setup the loop that checks for new information
      struct pollfd fds[1] = {
         { .fd = att_sub, .events = POLLIN}
      };

      hrt_abstime stime;

      stime = hrt_absolute_time();

      // Run for 30 seconds
      while(hrt_absolute_time() - stime < 30000000) {

         // Wait for sensor or parameter update.
         int ret = poll(fds, 1, 500);

         // Error
         if(ret < 0) {
            warnx("poll error");
         }
         // No new data, ignore
         else if(ret == 0) {
         }
         // New data received
         else {
            // Copy the sensor data into our struct
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



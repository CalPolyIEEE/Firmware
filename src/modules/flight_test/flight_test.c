#include <drivers/drv_hrt.h>
#include <errno.h>
#include <fcntl.h>
#include <geo/geo.h>
#include <math.h>
#include <nuttx/config.h>
#include <poll.h>
#include <pthread.h>
//#include <px4_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/systemlib.h>
#include <time.h>
#include <unistd.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

/* Daemon management function */
__EXPORT int flight_test_main(int argc, char *argv[]);

/* Daemon main loop */
int flight_test_thread_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
/*
static struct params p;
static struct param_handles ph;
*/
int flight_test_main(int argc, char *argv[]) {

   if(argc < 2) {
      warnx("missing command");
      warnx("usage: flight_test {start|stop|status}");
   }

   else if(!strcmp(argv[1], "start")) {

      if(thread_running) {
         warnx("already running");
         exit(0);
      }

      thread_should_exit = false;
      daemon_task = task_spawn_cmd("daemon", SCHED_RR, SCHED_PRIORITY_DEFAULT,
       1024, flight_test_thread_main, 
       (argv) ? &argv[2] : NULL);

      /* Wait function to allow the process to start */
      unsigned const max_wait_us = 1000000;
      unsigned const max_wait_steps = 2000;
      unsigned int i;

      for(i = 0; i < max_wait_steps; i++) {
         usleep(max_wait_us / max_wait_steps);
         if(thread_running) {
            break;
         }
      }

      exit(!(i < max_wait_steps));
   }

   else if(!strcmp(argv[1], "stop")) {
      if(!thread_running)
         errx(0, "already stopped");
      
      thread_should_exit = true;

      while(thread_running) {
         usleep(200000);
         warnx(".");
      }

      warnx("stopped");
      exit(0);
   }

   else if(!strcmp(argv[1], "status")) {
      if(thread_running) {
         warnx("running");
      }
      else {
         warnx("not started");
      }

      exit(0);
   }

   else {
      warnx("unrecognized command");
   }

   return 1;
}

int flight_test_thread_main(int argc, char *argv[]) {
   
   thread_running = true;
   
   warnx("flight_test starting");

   while(!thread_should_exit) {
      usleep(10000);
   }

   thread_running = false;
   
   return 0;
}

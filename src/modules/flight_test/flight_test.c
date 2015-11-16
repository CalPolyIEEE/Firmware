#include <drivers/drv_hrt.h>
#include <errno.h>
#include <fcntl.h>
#include <geo/geo.h>
#include <math.h>
#include <nuttx/config.h>
#include <poll.h>
//#include <px4_config.h>
#include <stdio.h>
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

static void usage(const char *reason) {
   if(reason) {
      warnx("%s\n", reason);
   }

   warnx("usage: flight_test {start|stop|status}\n\n");
}

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
/*
static struct params p;
static struct param_handles ph;
*/
int flight_test_main(int argc, char *argv[]) {

   if(argc < 2) {
      usage("missing command");
      return 1;
   }
   else if(!strcmp(argv[1], "start")) {
      if(thread_running) {
         warnx("app is already running\n");
         return 0;
      }
      thread_should_exit = false;
      /*
      daemon_task = task_spawn_cmd("daemon", SCHED_RR, SCHED_PRIORITY_DEFAULT,
       4096, flight_test_thread_main, 
       (argv) ? (const char **)&argv[2] : (const char **)NULL);
      */
      daemon_task = task_spawn_cmd("daemon", SCHED_RR, SCHED_PRIORITY_DEFAULT,
       4096, flight_test_thread_main, NULL);

      thread_running = true;
      return 0;
   }
   else if(!strcmp(argv[1], "stop")) {
      thread_should_exit = true;
      return 0;
   }
   else if(!strcmp(argv[1], "status")) {
      if(thread_running) {
         warnx("running\n");
      }
      else {
         warnx("not started\n");
      }

      return 0;
   }

   usage("unrecognized command");
   return 1;
}

int flight_test_thread_main(int argc, char *argv[]) {
   warnx("flight_test starting\n");

   while(!thread_should_exit) {
      sleep(10000);
   }
   
   warnx("flight_test stopping\n");

   thread_running = false;
   
   return 0;
}

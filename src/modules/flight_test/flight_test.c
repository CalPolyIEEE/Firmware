#include <drivers/drv_hrt.h>
#include <errno.h>
#include <fcntl.h>
#include <geo/geo.h>
#include <math.h>
#include <nuttx/config.h>
#include <poll.h>
#include <px4_config.h>
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

static void usage(const char *reason);

void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp,
 const struct vehicle_attitude_s *att,
 struct vehicle_rates_setpoint_s *rates_sp,
 struct actuator_constrols_s *actuators);

void control_heading(const struct vehicle_global_position_s *pos,
 const struct position_setpoint_s *sp,
 const struct vehicle_attitude_s *att,
 struct vehicle_attitude_setpoint_s *att_sp);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
static struct params p;
static struct param_handles ph;

void control attitude(const struct vehicle_attitude_setpoint_s *att_sp,
 const struct vehicle_attitude_s *att,
 struct vehicle_rates_setpoint_s *rates_sp,
 struct actuato_controls_s *actuators) {


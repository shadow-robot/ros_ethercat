/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Modified 2014, by Shadow Robot Company Ltd.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cstdio>
#include <cstdarg>
#include <getopt.h>
#include <execinfo.h>
#include <csignal>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <numeric>

#include "ros_ethercat_model/ros_ethercat.hpp"
#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

using namespace boost::accumulators;
using boost::ptr_vector;
using std::string;
using std::vector;
using std::accumulate;
using realtime_tools::RealtimePublisher;

static struct
{
  char *program_;
  char *interface_;
  char *rosparam_;
  bool allow_unprogrammed_;
  bool stats_;
  double period;
} g_options;

string g_robot_desc;

void Usage(const string &msg = "")
{
  fprintf(stderr, "Usage: %s [options]\n", g_options.program_);
  fprintf(stderr, "  Available options\n");
  fprintf(stderr, "    -i, --interface <interface> Connect to EtherCAT devices on this interface\n");
  fprintf(stderr, "    -p, --period                RT loop period in msec\n");
  fprintf(stderr, "    -s, --stats                 Publish statistics on the RT loop jitter on \"ros_ros_ethercat_eml/realtime\" in seconds\n");
  fprintf(stderr, "    -r, --rosparam <param>      Load the robot description from this parameter name\n");
  fprintf(stderr, "    -u, --allow_unprogrammed    Allow control loop to run with unprogrammed devices\n");
  fprintf(stderr, "    -h, --help                  Print this message and exit\n");
  if (msg != "")
  {
    fprintf(stderr, "Error: %s\n", msg.c_str());
    exit(-1);
  }
  else
    exit(0);
}

static int g_quit = 0;
static const int SEC_2_NSEC = 1e+9;
static const int SEC_2_USEC = 1e6;

static struct
{
  accumulator_set<double, stats<tag::max, tag::mean> > ec_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > cm_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > loop_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > jitter_acc;
  int overruns;
  int recent_overruns;
  int last_overrun;
  int last_severe_overrun;
  double overrun_loop_sec;
  double overrun_ec;
  double overrun_cm;

  // These values are set when realtime loop does not meet performance expectations
  bool rt_loop_not_making_timing;
  double halt_rt_loop_frequency;
  double rt_loop_frequency;
} g_stats;

static void publishDiagnostics(RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher)
{
  if (publisher.trylock())
  {
    accumulator_set<double, stats<tag::max, tag::mean> > zero;
    vector<diagnostic_msgs::DiagnosticStatus> statuses;
    diagnostic_updater::DiagnosticStatusWrapper status;

    static double max_ec = 0, max_cm = 0, max_loop = 0, max_jitter = 0;
    double avg_ec, avg_cm, avg_loop, avg_jitter;

    avg_ec = extract_result<tag::mean>(g_stats.ec_acc);
    avg_cm = extract_result<tag::mean>(g_stats.cm_acc);
    avg_loop = extract_result<tag::mean>(g_stats.loop_acc);
    max_ec = std::max(max_ec, extract_result<tag::max>(g_stats.ec_acc));
    max_cm = std::max(max_cm, extract_result<tag::max>(g_stats.cm_acc));
    max_loop = std::max(max_loop, extract_result<tag::max>(g_stats.loop_acc));
    g_stats.ec_acc = zero;
    g_stats.cm_acc = zero;
    g_stats.loop_acc = zero;

    // Publish average loop jitter
    avg_jitter = extract_result<tag::mean>(g_stats.jitter_acc);
    max_jitter = std::max(max_jitter, extract_result<tag::max>(g_stats.jitter_acc));
    g_stats.jitter_acc = zero;

    static bool first = true;
    if (first)
    {
      first = false;
      status.add("Robot Description", g_robot_desc);
    }

    status.addf("Max EtherCAT roundtrip (us)", "%.2f", max_ec * SEC_2_USEC);
    status.addf("Avg EtherCAT roundtrip (us)", "%.2f", avg_ec * SEC_2_USEC);
    status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm * SEC_2_USEC);
    status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm * SEC_2_USEC);
    status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop * SEC_2_USEC);
    status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop * SEC_2_USEC);
    status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * SEC_2_USEC);
    status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * SEC_2_USEC);
    status.addf("Control Loop Overruns", "%d", g_stats.overruns);
    status.addf("Recent Control Loop Overruns", "%d", g_stats.recent_overruns);
    status.addf("Last Control Loop Overrun Cause", "ec: %.2fus, cm: %.2fus",
                g_stats.overrun_ec*SEC_2_USEC, g_stats.overrun_cm * SEC_2_USEC);
    status.addf("Last Overrun Loop Time (us)", "%.2f", g_stats.overrun_loop_sec * SEC_2_USEC);
    status.addf("Realtime Loop Frequency", "%.4f", g_stats.rt_loop_frequency);

    status.name = "Realtime Control Loop";
    if (g_stats.overruns > 0 && g_stats.last_overrun < 30)
    {
      if (g_stats.last_severe_overrun < 30)
        status.level = 1;
      else
        status.level = 0;
      status.message = "Realtime loop used too much time in the last 30 seconds.";
    }
    else
    {
      status.level = 0;
      status.message = "OK";
    }
    g_stats.recent_overruns = 0;
    ++g_stats.last_overrun;
    ++g_stats.last_severe_overrun;

    if (g_stats.rt_loop_not_making_timing)
      status.mergeSummaryf(status.ERROR, "realtime loop only ran at %.4f Hz", g_stats.halt_rt_loop_frequency);

    statuses.push_back(status);
    publisher.msg_.status = statuses;
    publisher.msg_.header.stamp = ros::Time::now();
    publisher.unlockAndPublish();
  }
}

static inline double now()
{
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return double(n.tv_nsec) / SEC_2_NSEC + n.tv_sec;
}

void *diagnosticLoop(void *args)
{
  ptr_vector<EthercatHardware>* ec = (ptr_vector<EthercatHardware>*) args;
  struct timespec tick;
  clock_gettime(CLOCK_MONOTONIC, &tick);
  while (!g_quit)
  {
    for (ptr_vector<EthercatHardware>::iterator eh = ec->begin(); eh != ec->end(); ++eh)
    {
      eh->collectDiagnostics();
    }
    ++tick.tv_sec;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
  }
  return NULL;
}

static void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

class RTLoopHistory
{
public:

  RTLoopHistory(unsigned length, double default_value) :
    index_(0),
    length_(length),
    history_(length, default_value)
  {
  }

  void sample(double value)
  {
    index_ = (index_ + 1) % length_;
    history_[index_] = value;
  }

  double average() const
  {
    return accumulate(history_.begin(), history_.end(), 0.0) / (double) length_;
  }

protected:
  unsigned index_;
  unsigned length_;
  vector<double> history_;
};

static void* terminate_control(RealtimePublisher<diagnostic_msgs::DiagnosticArray> *publisher,
                               RealtimePublisher<std_msgs::Float64> *rtpublisher,
                               const char* message,
                               const char* data = NULL)
{
  ROS_FATAL(message, data);
  publisher->stop();
  delete rtpublisher;
  ros::shutdown();
  return (void*) - 1;
}

void *controlLoop(void *)
{
  double last_published, last_loop_start;
  int policy;
  TiXmlElement *root;
  TiXmlElement *root_element;

  ros::NodeHandle node(name);

  RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher(node, "/diagnostics", 2);
  RealtimePublisher<std_msgs::Float64> *rtpublisher = NULL;

  // Realtime loop should be running at least 3/4 of given frequency
  // or at specified min acceptable frequency
  double period_in_secs = 1e+9 * g_options.period;
  double given_frequency = 1 / period_in_secs;
  double min_acceptable_rt_loop_frequency = 0.75 * given_frequency;
  if (node.getParam("min_acceptable_rt_loop_frequency", min_acceptable_rt_loop_frequency))
    ROS_WARN("min_acceptable_rt_loop_frequency changed to %f", min_acceptable_rt_loop_frequency);

  unsigned rt_cycle_count = 0;
  double last_rt_monitor_time;

  // Calculate realtime loop frequency every 200msec
  double rt_loop_monitor_period = 0.2;
  // Keep history of last 3 calculation intervals.
  RTLoopHistory rt_loop_history(3, 1000.0);

  if (g_options.stats_)
    rtpublisher = new RealtimePublisher<std_msgs::Float64>(node, "realtime", 2);

  // Load robot description
  TiXmlDocument xml;
  struct stat st;

  if (ros::param::get(g_options.rosparam_, g_robot_desc))
    xml.Parse(g_robot_desc.c_str());
  else
    return terminate_control(&publisher, rtpublisher,
                             "Could not load the xml from parameter server: %s", g_options.rosparam_);

  root_element = xml.RootElement();
  root = xml.FirstChildElement("robot");
  if (!root || !root_element)
    return terminate_control(&publisher, rtpublisher, "Failed to parse the xml from %s", g_options.rosparam_);

  // Initialize the hardware interface
  ros::NodeHandle nh;
  RosEthercat seth(nh, g_options.interface_, g_options.allow_unprogrammed_, root);

  // Create controller manager
  controller_manager::ControllerManager cm(&seth);

  // Publish one-time before entering real-time to pre-allocate message vectors
  publishDiagnostics(publisher);

  //Start Non-realtime diagnostic thread
  static pthread_t diagnosticThread;
  int rv = pthread_create(&diagnosticThread, NULL, diagnosticLoop, &seth.ethercat_hardware_);
  if (rv != 0)
    return terminate_control(&publisher, rtpublisher,
                             "Unable to create control thread: rv = %s", boost::lexical_cast<string>(rv).c_str());

  // Set to realtime scheduler for this thread
  struct sched_param thread_param;
  policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);

  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  ros::Duration durp(g_options.period / 1e+9);

  // Snap to the nearest second
  tick.tv_nsec = (tick.tv_nsec / g_options.period + 1) * g_options.period;
  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

  last_published = now();
  last_rt_monitor_time = now();
  last_loop_start = now();
  while (!g_quit)
  {
    // Track how long the actual loop takes
    double this_loop_start = now();
    g_stats.loop_acc(this_loop_start - last_loop_start);
    last_loop_start = this_loop_start;

    double start = now();

    ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
    seth.read(this_moment, durp);
    double after_ec = now();
    cm.update(this_moment, durp);
    seth.write(this_moment, durp);
    double end = now();

    g_stats.ec_acc(after_ec - start);
    g_stats.cm_acc(end - after_ec);

    if ((end - last_published) > 1.0)
    {
      publishDiagnostics(publisher);
      last_published = end;
    }

    // Realtime loop should run about with the set frequency by default 1000Hz.
    // Missing timing on control cycles usually causes a controller glitch and actuators to jerk.
    // When realtime loop misses a lot of cycles controllers will perform poorly and may cause robot to shake.
    ++rt_cycle_count;
    if ((start - last_rt_monitor_time) > rt_loop_monitor_period)
    {
      // Calculate new average rt loop frequency
      double rt_loop_frequency = double(rt_cycle_count) / rt_loop_monitor_period;

      // Use last X samples of frequency when deciding whether or not to halt
      rt_loop_history.sample(rt_loop_frequency);
      double avg_rt_loop_frequency = rt_loop_history.average();
      if (avg_rt_loop_frequency < min_acceptable_rt_loop_frequency)
      {
        if (!g_stats.rt_loop_not_making_timing)
        {
          // Only update this value if motors when this first occurs (used for diagnostics error message)
          g_stats.halt_rt_loop_frequency = avg_rt_loop_frequency;
        }
        g_stats.rt_loop_not_making_timing = true;
      }
      g_stats.rt_loop_frequency = avg_rt_loop_frequency;
      rt_cycle_count = 0;
      last_rt_monitor_time = start;
    }

    // Compute end of next g_options.period
    timespecInc(tick, g_options.period);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + double(before.tv_nsec) / SEC_2_NSEC) > (tick.tv_sec + double(tick.tv_nsec) / SEC_2_NSEC))
    {
      // Total amount of time the loop took to run
      g_stats.overrun_loop_sec = (before.tv_sec + double(before.tv_nsec) / SEC_2_NSEC) -
        (tick.tv_sec + double(tick.tv_nsec) / SEC_2_NSEC);

      // We overran, snap to next "g_options.period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / g_options.period) * g_options.period;
      timespecInc(tick, g_options.period);

      // initialize overruns
      if (g_stats.overruns == 0)
      {
        g_stats.last_overrun = 1000;
        g_stats.last_severe_overrun = 1000;
      }
      // check for overruns
      if (g_stats.recent_overruns > 10)
        g_stats.last_severe_overrun = 0;
      g_stats.last_overrun = 0;

      ++g_stats.overruns;
      ++g_stats.recent_overruns;
      g_stats.overrun_ec = after_ec - start;
      g_stats.overrun_cm = end - after_ec;
    }

    // Sleep until end of g_options.period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

    // Calculate RT loop jitter
    struct timespec after;
    clock_gettime(CLOCK_REALTIME, &after);
    double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec - tick.tv_nsec) / SEC_2_NSEC);

    g_stats.jitter_acc(jitter);

    // Publish realtime loops statistics, if requested
    if (rtpublisher && rtpublisher->trylock())
    {
      rtpublisher->msg_.data = jitter;
      rtpublisher->unlockAndPublish();
    }
  }

  // Shutdown all of the motors on exit
  seth.shutdown();

  publisher.stop();
  delete rtpublisher;
  ros::shutdown();
  return NULL;
}

void quitRequested(int sig)
{
  g_quit = 1;
}

static int lock_fd(int fd)
{
  struct flock lock;

  lock.l_type = F_WRLCK;
  lock.l_whence = SEEK_SET;
  lock.l_start = 0;
  lock.l_len = 0;

  return fcntl(fd, F_SETLK, &lock);
}


static const char* PIDDIR = "/var/tmp/run/";

string generatePIDFilename(const char* interface)
{
  string filename;
  filename = string(PIDDIR) + "EtherCAT_" + string(interface) + ".pid";
  return filename;
}

static int setupPidFile(const char* interface)
{
  pid_t pid;
  int fd;
  FILE *fp = NULL;

  string filename = generatePIDFilename(interface);

  umask(0);
  mkdir(PIDDIR, 0777);
  int PID_FLAGS = O_RDWR | O_CREAT | O_EXCL;
  int PID_MODE = S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH;
  fd = open(filename.c_str(), PID_FLAGS, PID_MODE);
  if (fd == -1)
  {
    if (errno != EEXIST)
    {
      ROS_FATAL("Unable to create pid file '%s': %s", filename.c_str(), strerror(errno));
      return -1;
    }

    if ((fd = open(filename.c_str(), O_RDWR)) < 0)
    {
      ROS_FATAL("Unable to open pid file '%s': %s", filename.c_str(), strerror(errno));
      return -1;
    }

    if ((fp = fdopen(fd, "rw")) == NULL)
    {
      ROS_FATAL("Can't read from '%s': %s", filename.c_str(), strerror(errno));
      return -1;
    }
    pid = -1;
    if ((fscanf(fp, "%d", &pid) != 1) || (pid == getpid()) || (lock_fd(fileno(fp)) == 0))
    {
      int rc;

      if ((rc = unlink(filename.c_str())) == -1)
      {
        ROS_FATAL("Can't remove stale pid file '%s': %s", filename.c_str(), strerror(errno));
        return -1;
      }
    }
    else
    {
      ROS_FATAL("Another instance of ros_ethercat is already running with pid: %d", pid);
      return -1;
    }
  }

  unlink(filename.c_str());
  fd = open(filename.c_str(), PID_FLAGS, PID_MODE);

  if (fd == -1)
  {
    ROS_FATAL("Unable to open pid file '%s': %s", filename.c_str(), strerror(errno));
    return -1;
  }

  if (lock_fd(fd) == -1)
  {
    ROS_FATAL("Unable to lock pid file '%s': %s", filename.c_str(), strerror(errno));
    return -1;
  }

  if ((fp = fdopen(fd, "w")) == NULL)
  {
    ROS_FATAL("fdopen failed: %s", strerror(errno));
    return -1;
  }

  fprintf(fp, "%d\n", getpid());

  /* We do NOT close fd, since we want to keep the lock. */
  fflush(fp);
  fcntl(fd, F_SETFD, (long) 1);

  return 0;
}

static void cleanupPidFile(const char* interface)
{
  string filename = generatePIDFilename(interface);
  unlink(filename.c_str());
}

#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;

int main(int argc, char *argv[])
{
  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
  {
    perror("Failed to lock memory. It is recommended to do rosrun ros_ethercat_loop ethercat_grant");
    exit(EXIT_FAILURE);
  }

  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv, "realtime_loop");

  // Parse options
  g_options.program_ = argv[0];
  g_options.rosparam_ = NULL;
  g_options.period = 1e+6; // 1 ms in nanoseconds

  while (true)
  {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"stats", no_argument, 0, 's'},
      {"allow_unprogrammed", no_argument, 0, 'u'},
      {"interface", required_argument, 0, 'i'},
      {"rosparam", required_argument, 0, 'r'},
      {"period", no_argument, 0, 'p'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hi:usx:r:", long_options, &option_index);
    if (c == -1)
      break;

    switch (c)
    {
      case 'h':
        Usage();
        break;
      case 'u':
        g_options.allow_unprogrammed_ = true;
        break;
      case 'i':
        g_options.interface_ = optarg;
        break;
      case 'r':
        g_options.rosparam_ = optarg;
        break;
      case 's':
        g_options.stats_ = true;
        break;
      case 'p':
        // convert period given in msec to nsec
        g_options.period = fabs(atof(optarg))*1e+6;
        break;
    }
  }
  if (optind < argc)
    Usage("Extra arguments");

  if (!g_options.interface_)
    Usage("You must specify a network interface");
  if (!g_options.rosparam_)
    Usage("You must specify a rosparam for robot description");

  // EtherCAT lock for this interface (e.g. Ethernet port)
  if (setupPidFile(g_options.interface_) < 0)
    exit(EXIT_FAILURE);

  ros::NodeHandle node(name);

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  //Start thread
  int rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0);
  if (rv != 0)
  {
    ROS_FATAL("Unable to create control thread: rv = %d", rv);
    exit(EXIT_FAILURE);
  }

  ros::spin();
  pthread_join(controlThread, (void **) &rv);

  // Cleanup pid files
  cleanupPidFile(NULL);
  cleanupPidFile(g_options.interface_);

  return rv;
}


/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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

#include <stdio.h>
#include <getopt.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include "ros_ethercat/ros_ethercat.hpp"

using namespace boost::accumulators;
using std::string;
using realtime_tools::RealtimePublisher;

static const string name = "ros_ethercat";

static struct
{
  char *program_;
  char *interface_;
  char *xml_;
  char *rosparam_;
  bool allow_unprogrammed_;
  bool stats_;
} g_options;

string g_robot_desc;

void Usage(string msg = "")
{
  fprintf(stderr, "Usage: %s [options]\n", g_options.program_);
  fprintf(stderr, "  Available options\n");
  fprintf(stderr, "    -i, --interface <interface> Connect to EtherCAT devices on this interface\n");
  fprintf(stderr, "    -s, --stats                 Publish statistics on the RT loop jitter on \"pr2_ethercat/realtime\" in seconds\n");
  fprintf(stderr, "    -x, --xml <file>            Load the robot description from this file\n");
  fprintf(stderr, "    -r, --rosparam <param>      Load the robot description from this parameter name\n");
  fprintf(stderr, "    -u, --allow_unprogrammed    Allow control loop to run with unprogrammed devices\n");
  fprintf(stderr, "    -h, --help                  Print this message and exit\n");
  if (msg != "")
  {
    fprintf(stderr, "Error: %s\n", msg.c_str());
    exit(-1);
  }
  else
  {
    exit(0);
  }
}

static int g_quit = 0;
static bool g_reset_motors = true;
static bool g_halt_motors = false;
static bool g_halt_requested = false;
static volatile bool g_publish_trace_requested = false;
static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e6;

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

  // These values are set when realtime loop does not meet performace expections
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

    avg_ec           = extract_result<tag::mean>(g_stats.ec_acc);
    avg_cm           = extract_result<tag::mean>(g_stats.cm_acc);
    avg_loop         = extract_result<tag::mean>(g_stats.loop_acc);
    max_ec           = std::max(max_ec, extract_result<tag::max>(g_stats.ec_acc));
    max_cm           = std::max(max_cm, extract_result<tag::max>(g_stats.cm_acc));
    max_loop         = std::max(max_loop, extract_result<tag::max>(g_stats.loop_acc));
    g_stats.ec_acc   = zero;
    g_stats.cm_acc   = zero;
    g_stats.loop_acc = zero;

    // Publish average loop jitter
    avg_jitter         = extract_result<tag::mean>(g_stats.jitter_acc);
    max_jitter         = std::max(max_jitter, extract_result<tag::max>(g_stats.jitter_acc));
    g_stats.jitter_acc = zero;

    static bool first = true;
    if (first)
    {
      first = false;
      status.add("Robot Description", g_robot_desc);
    }

    status.addf("Max EtherCAT roundtrip (us)", "%.2f", max_ec*USEC_PER_SECOND);
    status.addf("Avg EtherCAT roundtrip (us)", "%.2f", avg_ec*USEC_PER_SECOND);
    status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm*USEC_PER_SECOND);
    status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm*USEC_PER_SECOND);
    status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop*USEC_PER_SECOND);
    status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop*USEC_PER_SECOND);
    status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * USEC_PER_SECOND);
    status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * USEC_PER_SECOND);
    status.addf("Control Loop Overruns", "%d", g_stats.overruns);
    status.addf("Recent Control Loop Overruns", "%d", g_stats.recent_overruns);
    status.addf("Last Control Loop Overrun Cause", "ec: %.2fus, cm: %.2fus",
                g_stats.overrun_ec*USEC_PER_SECOND, g_stats.overrun_cm*USEC_PER_SECOND);
    status.addf("Last Overrun Loop Time (us)", "%.2f", g_stats.overrun_loop_sec * USEC_PER_SECOND);
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
    g_stats.last_overrun++;
    g_stats.last_severe_overrun++;

    if (g_stats.rt_loop_not_making_timing)
    {
      status.mergeSummaryf(status.ERROR, "Halting, realtime loop only ran at %.4f Hz", g_stats.halt_rt_loop_frequency);
    }

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
  return double(n.tv_nsec) / NSEC_PER_SECOND + n.tv_sec;
}


void *diagnosticLoop(void *args)
{
  EthercatHardware *ec((EthercatHardware *) args);
  struct timespec tick;
  clock_gettime(CLOCK_MONOTONIC, &tick);
  while (!g_quit) {
    ec->collectDiagnostics();
    tick.tv_sec += 1;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
  }
  return NULL;
}

static void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= NSEC_PER_SECOND)
  {
    tick.tv_nsec -= NSEC_PER_SECOND;
    tick.tv_sec++;
  }
}


class RTLoopHistory
{
public:
  RTLoopHistory(unsigned length, double default_value) :
    index_(0),
    length_(length),
    history_(new double[length])
  {
    for (unsigned i=0; i<length_; ++i)
      history_[i] = default_value;
  }

  ~RTLoopHistory()
  {
    delete[] history_;
    history_ = NULL;
  }

  void sample(double value)
  {
    index_ = (index_+1) % length_;
    history_[index_] = value;
  }

  double average() const
  {
    double sum(0.0);
    for (unsigned i=0; i<length_; ++i)
      sum+=history_[i];
    return sum / double(length_);
  }

protected:
  unsigned index_;
  unsigned length_;
  double *history_;
};

void *controlLoop(void *)
{
  size_t rv = 0;
  double last_published, last_loop_start;
  int period;
  int policy;
  TiXmlElement *root;
  TiXmlElement *root_element;

  ros::NodeHandle node(name);

  RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher(node, "/diagnostics", 2);
  RealtimePublisher<std_msgs::Float64> *rtpublisher = 0;

  // Realtime loop should be running at least 750Hz
  // Calculate realtime loop frequency every 200mseec
  // Halt motors if average frequency over last 600msec is less than 750Hz
  double min_acceptable_rt_loop_frequency;
  if (!node.getParam("min_acceptable_rt_loop_frequency", min_acceptable_rt_loop_frequency))
  {
    min_acceptable_rt_loop_frequency = 750.0;
  }
  else
  {
    ROS_WARN("min_acceptable_rt_loop_frequency changed to %f", min_acceptable_rt_loop_frequency);
  }
  unsigned rt_cycle_count = 0;
  double last_rt_monitor_time;
  double rt_loop_monitor_period = 0.6 / 3;
  // Keep history of last 3 calculation intervals.
  RTLoopHistory rt_loop_history(3, 1000.0);

  if (g_options.stats_){
    rtpublisher = new RealtimePublisher<std_msgs::Float64>(node, "realtime", 2);
  }

  // Initialize the hardware interface
  EthercatHardware ec(name);
  ec.init(g_options.interface_, g_options.allow_unprogrammed_);
  ros_ethercat seth(ec.hw_, node);

  // Load robot description
  TiXmlDocument xml;
  struct stat st;
  if (g_options.rosparam_ != NULL)
  {
    if (ros::param::get(g_options.rosparam_, g_robot_desc))
    {
      xml.Parse(g_robot_desc.c_str());
    }
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s", g_options.rosparam_);
      rv = -1;
      publisher.stop(); delete rtpublisher; ros::shutdown(); return (void *)rv;
    }
  }
  else if (0 == stat(g_options.xml_, &st))
  {
    xml.LoadFile(g_options.xml_);
  }
  else
  {
    // In ROS Galapagos remove this fall-back to rosparam functionality
    ROS_INFO("Xml file not found, reading from parameter server");
    ros::NodeHandle top_level_node;
    if (top_level_node.getParam(g_options.xml_, g_robot_desc))
    {
      xml.Parse(g_robot_desc.c_str());
      ROS_WARN("Using -x to load robot description from parameter server is depricated.  Use -r instead.");
    }
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s", g_options.xml_);
      rv = -1;
      publisher.stop(); delete rtpublisher; ros::shutdown(); return (void *)rv;
    }
  }

  root_element = xml.RootElement();
  root = xml.FirstChildElement("robot");
  if (!root || !root_element)
  {
      ROS_FATAL("Could not parse the xml from %s", g_options.xml_);
      rv = -1;
      publisher.stop(); delete rtpublisher; ros::shutdown(); return (void *)rv;
  }

  // Initialize the controller manager from robot description
  if (!seth.initXml(root))
  {
      ROS_FATAL("Could not initialize the controller manager");
      rv = -1;
      publisher.stop(); delete rtpublisher; ros::shutdown(); return (void *)rv;
  }

  // Create controller manager
  controller_manager::ControllerManager cm(&seth, node);

  // Publish one-time before entering real-time to pre-allocate message vectors
  publishDiagnostics(publisher);

  //Start Non-realtime diagonostic thread
  static pthread_t diagnosticThread;
  if ((rv = pthread_create(&diagnosticThread, NULL, diagnosticLoop, &ec)) != 0)
  {
    ROS_FATAL("Unable to create control thread: rv = %zu", rv);
    publisher.stop(); delete rtpublisher; ros::shutdown(); return (void *)rv;
  }

  // Set to realtime scheduler for this thread
  struct sched_param thread_param;
  policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);

  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  period = 1e+6; // 1 ms in nanoseconds
  ros::Duration durp(period);

  // Snap to the nearest second
  //tick.tv_sec = tick.tv_sec;
  tick.tv_nsec = (tick.tv_nsec / period + 1) * period;
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
    if (g_reset_motors)
    {
      ec.update(true, g_halt_motors);
      g_reset_motors = false;
      // Also, clear error flags when motor reset is requested
      g_stats.rt_loop_not_making_timing = false;
    }
    else
    {
      ec.update(false, g_halt_motors);
    }
    if (g_publish_trace_requested)
    {
      g_publish_trace_requested = false;
      ec.publishTrace(-1,"",0,0);
    }
    g_halt_motors = false;
    double after_ec = now();
    ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
    cm.update(this_moment, durp);
    double end = now();

    g_stats.ec_acc(after_ec - start);
    g_stats.cm_acc(end - after_ec);

    if ((end - last_published) > 1.0)
    {
      publishDiagnostics(publisher);
      last_published = end;
    }

    // Realtime loop should run about 1000Hz.
    // Missing timing on a control cycles usually causes a controller glitch and actuators to jerk.
    // When realtime loop misses a lot of cycles controllers will perform poorly and may cause robot to shake.
    // Halt motors if realtime loop does not run enough cycles over a given period.
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
        g_halt_motors = true;
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

    // Compute end of next period
    timespecInc(tick, period);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) > (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND))
    {
      // Total amount of time the loop took to run
      g_stats.overrun_loop_sec = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -
        (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);

      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period) * period;
      timespecInc(tick, period);

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

    // Sleep until end of period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

    // Calculate RT loop jitter
    struct timespec after;
    clock_gettime(CLOCK_REALTIME, &after);
    double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec-tick.tv_nsec)/NSEC_PER_SECOND);

    g_stats.jitter_acc(jitter);

    // Publish realtime loops statistics, if requested
    if (rtpublisher)
    {
      if (rtpublisher->trylock())
      {
        rtpublisher->msg_.data  = jitter;
        rtpublisher->unlockAndPublish();
      }
    }

    // Halt the motors, if requested by a service call
    if (g_halt_requested)
    {
      g_halt_motors = true;
      g_halt_requested = false;
    }
  }

  /* Shutdown all of the motors on exit */
  for (pr2_hardware_interface::ActuatorMap::const_iterator it = ec.hw_->actuators_.begin(); it != ec.hw_->actuators_.end(); ++it)
  {
    it->second->command_.enable_ = false;
    it->second->command_.effort_ = 0;
  }
  ec.update(false, true);

  publisher.stop(); delete rtpublisher; ros::shutdown(); return (void *)rv;
}

void quitRequested(int sig)
{
  g_quit = 1;
}

bool resetMotorsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_reset_motors = true;
  return true;
}

bool haltMotorsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_halt_requested = true;
  return true;
}

bool publishTraceService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_publish_trace_requested = true;
  return true;
}

static int
lock_fd(int fd)
{
  struct flock lock;
  int rv;

  lock.l_type = F_WRLCK;
  lock.l_whence = SEEK_SET;
  lock.l_start = 0;
  lock.l_len = 0;

  rv = fcntl(fd, F_SETLK, &lock);
  return rv;
}


static const char* PIDDIR = "/var/tmp/run/";

string generatePIDFilename(const char* interface)
{
  string filename;
  if (interface != NULL)
  {
    // There should a lock file for each EtherCAT interface instead of for entire computer
    // It is entirely possible to have different EtherCAT utilities operating indepedantly
    //  on different interfaces
    filename = string(PIDDIR) + "EtherCAT_" +  string(interface) + ".pid";
  }
  else
  {
    filename = string(PIDDIR) + string("pr2_etherCAT.pid");
  }
  return filename;
}


static int setupPidFile(const char* interface)
{
  int rv = -1;
  pid_t pid;
  int fd;
  FILE *fp = NULL;

  string filename = generatePIDFilename(interface);

  umask(0);
  mkdir(PIDDIR, 0777);
  fd = open(filename.c_str(), O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH);
  if (fd == -1)
  {
    if (errno != EEXIST)
    {
      ROS_FATAL("Unable to create pid file '%s': %s", filename.c_str(), strerror(errno));
      return rv;
    }

    if ((fd = open(filename.c_str(), O_RDWR)) < 0)
    {
      ROS_FATAL("Unable to open pid file '%s': %s", filename.c_str(), strerror(errno));
      return rv;
    }

    if ((fp = fdopen(fd, "rw")) == NULL)
    {
      ROS_FATAL("Can't read from '%s': %s", filename.c_str(), strerror(errno));
      return rv;
    }
    pid = -1;
    if ((fscanf(fp, "%d", &pid) != 1) || (pid == getpid()) || (lock_fd(fileno(fp)) == 0))
    {
      int rc;

      if ((rc = unlink(filename.c_str())) == -1)
      {
        ROS_FATAL("Can't remove stale pid file '%s': %s", filename.c_str(), strerror(errno));
        return rv;
      }
    } else {
      ROS_FATAL("Another instance of pr2_ethercat is already running with pid: %d", pid);
      return rv;
    }
  }

  unlink(filename.c_str());
  fd = open(filename.c_str(), O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH);

  if (fd == -1)
  {
    ROS_FATAL("Unable to open pid file '%s': %s", filename.c_str(), strerror(errno));
    return rv;
  }

  if (lock_fd(fd) == -1)
  {
    ROS_FATAL("Unable to lock pid file '%s': %s", filename.c_str(), strerror(errno));
    return rv;
  }

  if ((fp = fdopen(fd, "w")) == NULL)
  {
    ROS_FATAL("fdopen failed: %s", strerror(errno));
    return rv;
  }

  fprintf(fp, "%d\n", getpid());

  /* We do NOT close fd, since we want to keep the lock. */
  fflush(fp);
  fcntl(fd, F_SETFD, (long) 1);
  rv = 0;

  return rv;
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
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }

  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv, "realtime_loop");

  // Parse options
  g_options.program_ = argv[0];
  g_options.xml_ = NULL;
  g_options.rosparam_ = NULL;
  while (1)
  {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"stats", no_argument, 0, 's'},
      {"allow_unprogrammed", no_argument, 0, 'u'},
      {"interface", required_argument, 0, 'i'},
      {"xml", required_argument, 0, 'x'},
      {"rosparam", required_argument, 0, 'r'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hi:usx:r:", long_options, &option_index);
    if (c == -1) break;
    switch (c)
    {
      case 'h':
        Usage();
        break;
      case 'u':
        g_options.allow_unprogrammed_ = 1;
        break;
      case 'i':
        g_options.interface_ = optarg;
        break;
      case 'x':
        g_options.xml_ = optarg;
        break;
      case 'r':
        g_options.rosparam_ = optarg;
        break;
      case 's':
        g_options.stats_ = 1;
        break;
    }
  }
  if (optind < argc)
  {
    Usage("Extra arguments");
  }

  if (!g_options.interface_)
    Usage("You must specify a network interface");
  if (!g_options.xml_ && !g_options.rosparam_)
  {
    Usage("You must specify either an XML file or rosparam for robot description");
  }
  if (g_options.xml_ && g_options.rosparam_)
  {
    Usage("You must not specify both a rosparm and XML file for robot description");
  }

  // The previous EtherCAT software created a lock for any EtherCAT master.
  // This lock prevented two EtherCAT masters from running on the same computer.
  // However, this locking scheme was too restrictive.  
  // Two EtherCAT masters can run without conflicting with each other
  // as long as they are communication with different sets of EtherCAT devices.
  // A better locking scheme is to prevent two EtherCAT 
  // masters from running on same Ethernet interface.  
  // Therefore in the Groovy Galapagos ROS release, the global EtherCAT lock has been removed 
  // and only the per-interface lock will remain.
  if (setupPidFile(g_options.interface_) < 0) return -1;

  ros::NodeHandle node(name);

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  ros::ServiceServer reset = node.advertiseService("reset_motors", resetMotorsService);
  ros::ServiceServer halt = node.advertiseService("halt_motors", haltMotorsService);
  ros::ServiceServer publishTrace = node.advertiseService("publish_trace", publishTraceService);

  //Start thread
  int rv;
  if ((rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0)) != 0)
  {
    ROS_FATAL("Unable to create control thread: rv = %d", rv);
    exit(EXIT_FAILURE);
  }

  ros::spin();
  pthread_join(controlThread, (void **)&rv);

  // Cleanup pid files
  cleanupPidFile(NULL);
  cleanupPidFile(g_options.interface_);

  return rv;
}


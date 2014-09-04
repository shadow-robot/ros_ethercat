/**
 * This code is taken from pr2-grant
 * https://code.ros.org/svn/pr2debs/trunk/packages/pr2/pr2-grant/pr2_grant.cpp
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/capability.h>
#include <string>
#include <cstdlib>
#include <sys/prctl.h>

using namespace std;

#define EXECUTABLE "/var/tmp/granted"

int main(int argc, char *argv[])
{
  // Remove old executable, if it exists
  unlink(EXECUTABLE);

  // Copy new executable to /var/tmp
  string cmd;
  cmd = string("cp ") + string(argv[1]) + string(" " EXECUTABLE);
  if (system(cmd.c_str()) == -1) {
    perror("cp");
    return -1;
  }
  if (chown(EXECUTABLE, getuid(), getgid()) < 0) {
    perror("chown");
    return -1;
  }

  // Create capability set
  const char *cap_text = "cap_ipc_lock=ep cap_net_raw=ep cap_sys_nice=ep cap_net_admin=ep";
  cap_t cap_d = cap_from_text(cap_text);
  if (cap_d == NULL) {
    perror("cap_from_text");
    return -1;
  }

  // Set file capability
  int retval = cap_set_file(EXECUTABLE, cap_d);
  if (retval != 0) {
    fprintf(stderr, "Failed to set capabilities on file `%s' (%s)\n", argv[1], strerror(errno));
    return -1;
  }

  // Free capability
  if (cap_d) {
    cap_free(cap_d);
  }


  // Drop privileges
  setuid(getuid());
  setgid(getgid());

  // Allow core dumps
  prctl(PR_SET_DUMPABLE, 1, 0, 0, 0);

  // Exec executable
  if (execv(EXECUTABLE, argv + 1) < 0) {
    perror("execv");
    return -1;
  }

  return 0;
}

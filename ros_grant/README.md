# ros_grant

This is a for of the pr2-grant utility. It makes it possible to run the *ros_ethercat_loop* without the need of being root.

## Installation

If you installed this package from *apt-get* then you're good to go. (This is definitely the recommended way)

If you compiled this package from source, then you'll need to copy the resulting `ros_grant` executable to `/usr/local/bin` and add the sticky bit to it: `sudo chmod +s /usr/local/bin/ros_grant`. Then you can

## Use
Using ros_grant makes it possible to not use *sudo* anymore for running the ethercat main loop. Just use `launch-prefix="ros_grant"` in your launch files for the *ros_ethercat_loop*.

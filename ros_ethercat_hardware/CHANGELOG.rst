^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ethercat_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2015-04-07)
------------------
* Delete motor halted message
* Make the start address non-static (each ethercat port will have its own separate logical address space)
* Add run dependency, otherwise catkin complains.
* Enable rpath for ros_ethercat_hardware. We add a build dep on ros_ethercat_model to have the new cmake function available.
* bug fixes in initialization


0.1.8 (2014-07-18)
------------------

0.1.7 (2014-05-23)
------------------

0.1.6 (2014-05-15)
------------------

0.1.5 (2014-05-14)
------------------

0.1.4 (2014-05-14)
------------------

0.1.3 (2014-05-13)
------------------

0.1.2 (2014-05-13)
------------------

0.1.1 (2014-05-12)
------------------
* first hydro release

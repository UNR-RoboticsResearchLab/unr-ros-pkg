^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_object_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2014-03-30)
------------------
* changed distance for negative update
* renamed hector_object_tracker::param() method to hector_object_tracker::parameter()
  This will hopefully fix the build failures on Jenkins for lucid binaries.
  See http://jenkins.ros.org/job/ros-fuerte-hector-worldmodel_binarydeb_lucid_amd64/132/console
* added missing target dependency to hector_nav_msgs_generate_messages_cpp
* Contributors: Johannes Meyer

0.3.0 (2013-09-03)
------------------
* catkinized stack hector_worldmodel

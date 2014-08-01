^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package message_to_tf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2014-07-09)
------------------
* message_to_tf: fixed base_link transform publishing (was removed in 77f2cc2334d15fc0e9395ceb9b40cd4601448289)
* Contributors: Johannes Meyer

0.1.2 (2014-06-02)
------------------
* Add parameter for optionally not publishing roll/pitch to tf
* Don´t publish roll/pitch (to be parametrized soon)
* Contributors: Stefan Kohlbrecher, hector1

0.1.1 (2014-03-30)
------------------
* added missing dependency to roscpp
* Contributors: Johannes Meyer

0.1.0 (2013-09-03)
------------------
* catkinized stack hector_localization
* readded tf_prefix support (deprecated feature in ROS hydro beta)
* added ShapeShifter subscriber which accepts multiple message types
* added euler angle publisher (Vector3Stamped)

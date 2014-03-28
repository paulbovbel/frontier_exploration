^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package frontier_exploration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2014-03-28)
------------------
* revert launch file to general form
* document helper functions
* Contributors: Paul Bovbel

0.1.4 (2014-03-27)
------------------
* fix properly transforming robot position into goal frame
* Contributors: Paul Bovbel

0.1.3 (2014-03-27)
------------------
* revert redundant dependency
* Contributors: Paul Bovbel

0.1.2 (2014-03-27)
------------------
* fix message dependencies
* fix goal aliasing corner case
* fix empty frame name
* Update README.md
* accomodate for empty boundaries
* parametrize goal aliasing for redundant move base goals
* Contributors: Paul Bovbel

0.1.1 (2014-03-26)
------------------
* fix build dependency
* eliminate redundant move_base goal updates, fix locking
* externalize static for geometric calculations
* refactor action server with callbacks
* fix pre-emption with continuous goal updating
* implement continuous frontier/goal updating
* Contributors: Paul Bovbel

0.1.0 (2014-03-25)
------------------
* added install target for client
* Contributors: Paul Bovbel

0.0.4 (2014-03-25)
------------------
* remove robot specific names
* remove raytrace and obstacle range params
* reintroduce frontier selection parametrization
* added client for rviz interaction via point tool
* Contributors: Paul Bovbel

0.0.3 (2014-03-25)
------------------
* parametrize method of selecting frontier point to travel to
* use ros msg for Frontier structure
* remove unnecessary costmap copying
* remove debug messages, update comments
* update package name
* Contributors: Paul Bovbel

0.0.2 (2014-03-21)
------------------
* fix locking issue
* rename package
* update dependencies
* move sample parameters to launch file
* remove temp files
* refactoring
* fix off-map error and add costmap locking
* added resize parameter for working with external maps
* refactor message fields
* refactored message names
* add resize parameter for using layer with external maps
* refactor names
* remove even more debug code
* remove debug code
* remove debug points
* Updated maintainer info
* Update documentation and comments
* move test portion
* refactor for pre-emption
* clean up temp files
* expanded comments, cleaned up temp files
* Initial documentation
* Initial commit
* Contributors: Paul Bovbel
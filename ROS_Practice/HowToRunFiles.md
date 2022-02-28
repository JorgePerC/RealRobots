# How to run files


## Dependencies

## Steps:

1. Open a terminal
1. Source your ros setup file if not on path
1. Run `roscore`
1. Create a workspace `catkin_create pkgName libs rospy`

    * The package.xml file contains:   
        * Is a file that executes a command to run the complete proyect and build
        * Running mode
        * Running another file or launch file
    * Make your files C++ & Python executables. 
    * Or you can create the executable from the file properties. and configure it as c++
        
        * `chmod -x pythonFile.py`
        * Build it.
        * Source the devel carpet to update ROS list of runnable nodes. `source devel/setup.bash` 
1. Add the corresponding file names to your _CMakeLists.txt_ file and also _package.xml_
    * This deppends on wheater you are using Python or [C++](http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile). So use the desired one
    * This might not be necessary if using catkin tools.
1. Build project `catkin build`
    * If you don't have catkin tools, install it, or:
        * Run `catkin_make`
1. Run node `rosrun pkgName nodeName`. Note that the file sould be an executable. 
    * An alternative is `roslaunch pkgName nodeName.launch`

## Notes:
* EVERY PYTHON FILE YOU SOULD HAVE AT THE BEGINNING:
            
        `#!/usr/bin/env python`
        `# -*- coding: utf-8 -*-`

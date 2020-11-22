This is an example of exchanging messages between reactors using ROS2.

There is a MessageGenerator reactor that publishes String messages
on 'topic' and a MessageReceiver reactor that subscribes to 'topic'.

1- To get this example working, install ROS 2
   (https://index.ros.org/doc/ros2/Installation/Foxy/).

2- Follow the instruction in 
   https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/
   **section 1** to create a 'cpp_pubsub' package in the current (example/ROS) folder.

3- Follow section 2.2 and 2.3 to modify the CMakeLists.txt and package.xml.

4- Replace the default C++14 standard in CMakeLists.txt (i.e., set(CMAKE_CXX_STANDARD 14)) 
   with:

      # Default to C++20
      if(NOT CMAKE_CXX_STANDARD)
          set(CMAKE_CXX_STANDARD 20)
      endif()

and rename the source in 'add_executable' to reflect

  add_executable(talker src/BasicROS.cpp)

5- Run the provided build-ROS-node.sh:

      ./build-ROS-node.sh BasicROS cpp_pubsub

This will create a 'talker' node in the package cpp_pubsub (these names can be changed in 
CMakeLists.txt and in the argument to build-ROS-node.sh).

6- Source the appropriate setup.bash and run the node:

      source cpp_pubsub/install/setup.bash
      ros2 run cpp_pubsub talker

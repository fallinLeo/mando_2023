# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/leo/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/leo/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leo/Documents/GitHub/mando_2023/ws_mando/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/Documents/GitHub/mando_2023/ws_mando/build

# Include any dependencies generated for this target.
include usb_cam/CMakeFiles/usb_cam.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include usb_cam/CMakeFiles/usb_cam.dir/compiler_depend.make

# Include the progress variables for this target.
include usb_cam/CMakeFiles/usb_cam.dir/progress.make

# Include the compile flags for this target's objects.
include usb_cam/CMakeFiles/usb_cam.dir/flags.make

usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o: usb_cam/CMakeFiles/usb_cam.dir/flags.make
usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o: /home/leo/Documents/GitHub/mando_2023/ws_mando/src/usb_cam/src/usb_cam.cpp
usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o: usb_cam/CMakeFiles/usb_cam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/Documents/GitHub/mando_2023/ws_mando/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o"
	cd /home/leo/Documents/GitHub/mando_2023/ws_mando/build/usb_cam && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o -MF CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o.d -o CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o -c /home/leo/Documents/GitHub/mando_2023/ws_mando/src/usb_cam/src/usb_cam.cpp

usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/usb_cam.dir/src/usb_cam.cpp.i"
	cd /home/leo/Documents/GitHub/mando_2023/ws_mando/build/usb_cam && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/Documents/GitHub/mando_2023/ws_mando/src/usb_cam/src/usb_cam.cpp > CMakeFiles/usb_cam.dir/src/usb_cam.cpp.i

usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/usb_cam.dir/src/usb_cam.cpp.s"
	cd /home/leo/Documents/GitHub/mando_2023/ws_mando/build/usb_cam && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/Documents/GitHub/mando_2023/ws_mando/src/usb_cam/src/usb_cam.cpp -o CMakeFiles/usb_cam.dir/src/usb_cam.cpp.s

# Object files for target usb_cam
usb_cam_OBJECTS = \
"CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o"

# External object files for target usb_cam
usb_cam_EXTERNAL_OBJECTS =

/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: usb_cam/CMakeFiles/usb_cam.dir/build.make
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libcv_bridge.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libimage_transport.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libmessage_filters.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libclass_loader.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libroslib.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/librospack.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libcamera_info_manager.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libcamera_calibration_parsers.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libroscpp.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/librosconsole.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/librostime.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /opt/ros/noetic/lib/libcpp_common.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libv4l_driver.so
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so: usb_cam/CMakeFiles/usb_cam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/Documents/GitHub/mando_2023/ws_mando/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so"
	cd /home/leo/Documents/GitHub/mando_2023/ws_mando/build/usb_cam && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/usb_cam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
usb_cam/CMakeFiles/usb_cam.dir/build: /home/leo/Documents/GitHub/mando_2023/ws_mando/devel/lib/libusb_cam.so
.PHONY : usb_cam/CMakeFiles/usb_cam.dir/build

usb_cam/CMakeFiles/usb_cam.dir/clean:
	cd /home/leo/Documents/GitHub/mando_2023/ws_mando/build/usb_cam && $(CMAKE_COMMAND) -P CMakeFiles/usb_cam.dir/cmake_clean.cmake
.PHONY : usb_cam/CMakeFiles/usb_cam.dir/clean

usb_cam/CMakeFiles/usb_cam.dir/depend:
	cd /home/leo/Documents/GitHub/mando_2023/ws_mando/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/Documents/GitHub/mando_2023/ws_mando/src /home/leo/Documents/GitHub/mando_2023/ws_mando/src/usb_cam /home/leo/Documents/GitHub/mando_2023/ws_mando/build /home/leo/Documents/GitHub/mando_2023/ws_mando/build/usb_cam /home/leo/Documents/GitHub/mando_2023/ws_mando/build/usb_cam/CMakeFiles/usb_cam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : usb_cam/CMakeFiles/usb_cam.dir/depend

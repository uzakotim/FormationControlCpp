# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/timur/FormationControlCpp/CppVersion/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/timur/FormationControlCpp/CppVersion/build

# Include any dependencies generated for this target.
include orb_detector/CMakeFiles/orb_detector.dir/depend.make

# Include the progress variables for this target.
include orb_detector/CMakeFiles/orb_detector.dir/progress.make

# Include the compile flags for this target's objects.
include orb_detector/CMakeFiles/orb_detector.dir/flags.make

orb_detector/CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.o: orb_detector/CMakeFiles/orb_detector.dir/flags.make
orb_detector/CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.o: /home/timur/FormationControlCpp/CppVersion/src/orb_detector/src/orb_detector_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/timur/FormationControlCpp/CppVersion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object orb_detector/CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.o"
	cd /home/timur/FormationControlCpp/CppVersion/build/orb_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.o -c /home/timur/FormationControlCpp/CppVersion/src/orb_detector/src/orb_detector_node.cpp

orb_detector/CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.i"
	cd /home/timur/FormationControlCpp/CppVersion/build/orb_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/timur/FormationControlCpp/CppVersion/src/orb_detector/src/orb_detector_node.cpp > CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.i

orb_detector/CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.s"
	cd /home/timur/FormationControlCpp/CppVersion/build/orb_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/timur/FormationControlCpp/CppVersion/src/orb_detector/src/orb_detector_node.cpp -o CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.s

# Object files for target orb_detector
orb_detector_OBJECTS = \
"CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.o"

# External object files for target orb_detector
orb_detector_EXTERNAL_OBJECTS =

/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: orb_detector/CMakeFiles/orb_detector.dir/src/orb_detector_node.cpp.o
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: orb_detector/CMakeFiles/orb_detector.dir/build.make
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libcv_bridge.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libimage_transport.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libmessage_filters.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libclass_loader.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libdl.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libroslib.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/librospack.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libroscpp.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/librosconsole.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/librostime.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /opt/ros/noetic/lib/libcpp_common.so
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector: orb_detector/CMakeFiles/orb_detector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/timur/FormationControlCpp/CppVersion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector"
	cd /home/timur/FormationControlCpp/CppVersion/build/orb_detector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/orb_detector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
orb_detector/CMakeFiles/orb_detector.dir/build: /home/timur/FormationControlCpp/CppVersion/devel/lib/orb_detector/orb_detector

.PHONY : orb_detector/CMakeFiles/orb_detector.dir/build

orb_detector/CMakeFiles/orb_detector.dir/clean:
	cd /home/timur/FormationControlCpp/CppVersion/build/orb_detector && $(CMAKE_COMMAND) -P CMakeFiles/orb_detector.dir/cmake_clean.cmake
.PHONY : orb_detector/CMakeFiles/orb_detector.dir/clean

orb_detector/CMakeFiles/orb_detector.dir/depend:
	cd /home/timur/FormationControlCpp/CppVersion/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/timur/FormationControlCpp/CppVersion/src /home/timur/FormationControlCpp/CppVersion/src/orb_detector /home/timur/FormationControlCpp/CppVersion/build /home/timur/FormationControlCpp/CppVersion/build/orb_detector /home/timur/FormationControlCpp/CppVersion/build/orb_detector/CMakeFiles/orb_detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : orb_detector/CMakeFiles/orb_detector.dir/depend


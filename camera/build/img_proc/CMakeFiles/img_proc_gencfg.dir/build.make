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
CMAKE_SOURCE_DIR = /home/ubuntu/Software/people_counter/camera/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Software/people_counter/camera/build

# Utility rule file for img_proc_gencfg.

# Include the progress variables for this target.
include img_proc/CMakeFiles/img_proc_gencfg.dir/progress.make

img_proc/CMakeFiles/img_proc_gencfg: /home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h
img_proc/CMakeFiles/img_proc_gencfg: /home/ubuntu/Software/people_counter/camera/devel/lib/python3/dist-packages/img_proc/cfg/img_procConfig.py


/home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h: /home/ubuntu/Software/people_counter/camera/src/img_proc/cfg/img_proc.cfg
/home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.py.template
/home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/Software/people_counter/camera/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dynamic reconfigure files from cfg/img_proc.cfg: /home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h /home/ubuntu/Software/people_counter/camera/devel/lib/python3/dist-packages/img_proc/cfg/img_procConfig.py"
	cd /home/ubuntu/Software/people_counter/camera/build/img_proc && ../catkin_generated/env_cached.sh /home/ubuntu/Software/people_counter/camera/build/img_proc/setup_custom_pythonpath.sh /home/ubuntu/Software/people_counter/camera/src/img_proc/cfg/img_proc.cfg /opt/ros/noetic/share/dynamic_reconfigure/cmake/.. /home/ubuntu/Software/people_counter/camera/devel/share/img_proc /home/ubuntu/Software/people_counter/camera/devel/include/img_proc /home/ubuntu/Software/people_counter/camera/devel/lib/python3/dist-packages/img_proc

/home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig.dox: /home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig.dox

/home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig-usage.dox: /home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig-usage.dox

/home/ubuntu/Software/people_counter/camera/devel/lib/python3/dist-packages/img_proc/cfg/img_procConfig.py: /home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/Software/people_counter/camera/devel/lib/python3/dist-packages/img_proc/cfg/img_procConfig.py

/home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig.wikidoc: /home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig.wikidoc

img_proc_gencfg: img_proc/CMakeFiles/img_proc_gencfg
img_proc_gencfg: /home/ubuntu/Software/people_counter/camera/devel/include/img_proc/img_procConfig.h
img_proc_gencfg: /home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig.dox
img_proc_gencfg: /home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig-usage.dox
img_proc_gencfg: /home/ubuntu/Software/people_counter/camera/devel/lib/python3/dist-packages/img_proc/cfg/img_procConfig.py
img_proc_gencfg: /home/ubuntu/Software/people_counter/camera/devel/share/img_proc/docs/img_procConfig.wikidoc
img_proc_gencfg: img_proc/CMakeFiles/img_proc_gencfg.dir/build.make

.PHONY : img_proc_gencfg

# Rule to build all files generated by this target.
img_proc/CMakeFiles/img_proc_gencfg.dir/build: img_proc_gencfg

.PHONY : img_proc/CMakeFiles/img_proc_gencfg.dir/build

img_proc/CMakeFiles/img_proc_gencfg.dir/clean:
	cd /home/ubuntu/Software/people_counter/camera/build/img_proc && $(CMAKE_COMMAND) -P CMakeFiles/img_proc_gencfg.dir/cmake_clean.cmake
.PHONY : img_proc/CMakeFiles/img_proc_gencfg.dir/clean

img_proc/CMakeFiles/img_proc_gencfg.dir/depend:
	cd /home/ubuntu/Software/people_counter/camera/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Software/people_counter/camera/src /home/ubuntu/Software/people_counter/camera/src/img_proc /home/ubuntu/Software/people_counter/camera/build /home/ubuntu/Software/people_counter/camera/build/img_proc /home/ubuntu/Software/people_counter/camera/build/img_proc/CMakeFiles/img_proc_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : img_proc/CMakeFiles/img_proc_gencfg.dir/depend


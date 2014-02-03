cmake_minimum_required(VERSION 2.8.3)
project(rviz)

find_package(Boost REQUIRED
  COMPONENTS
  filesystem
  program_options
  signals
  system
  thread
)

find_package(Eigen REQUIRED)


message("AAAAAAAAAAA z1 OGRE_FOUND=${OGRE_FOUND}")
message("AAAAAAAAAAA z1 OGRE_OGRE=${OGRE_FOUND}")
message("AAAAAAAAAAA z1 OGRE_LIBRARIES=${OGRE_LIBRARIES}")
message("AAAAAAAAAAA z1 OGRE_OGRE_LIBRARIES=${OGRE_OGRE_LIBRARIES}")
message("AAAAAAAAAAA x1 OGRE_Overlay_FOUND=${OGRE_Overlay_FOUND}")
message("AAAAAAAAAAA x1 OGRE_Overlay_INCLUDE_DIRS=${OGRE_Overlay_INCLUDE_DIRS}")
message("AAAAAAAAAAA x1 OGRE_Overlay_LIBRARIES=${OGRE_Overlay_LIBRARIES}")

if (0)
  find_package(OGRE REQUIRED)
endif()

message("AAAAAAAAAAA z2 OGRE_FOUND=${OGRE_FOUND}")
message("AAAAAAAAAAA z2 OGRE_OGRE=${OGRE_FOUND}")
message("AAAAAAAAAAA z2 OGRE_LIBRARIES=${OGRE_LIBRARIES}")
message("AAAAAAAAAAA z2 OGRE_OGRE_LIBRARIES=${OGRE_OGRE_LIBRARIES}")
message("AAAAAAAAAAA x2 OGRE_Overlay_FOUND=${OGRE_Overlay_FOUND}")
message("AAAAAAAAAAA x2 OGRE_Overlay_INCLUDE_DIRS=${OGRE_Overlay_INCLUDE_DIRS}")
message("AAAAAAAAAAA x2 OGRE_Overlay_LIBRARIES=${OGRE_Overlay_LIBRARIES}")



message("AAAAAAAAAAA z3 OGRE_OV_FOUND=${OGRE_OV_FOUND}")
message("AAAAAAAAAAA z3 OGRE_OV_OGRE=${OGRE_OV_FOUND}")
message("AAAAAAAAAAA z3 OGRE_OV_LIBRARIES=${OGRE_OV_LIBRARIES}")
message("AAAAAAAAAAA z3 OGRE_OV_OGRE_LIBRARIES=${OGRE_OV_OGRE_LIBRARIES}")
message("AAAAAAAAAAA x3 OGRE_OV_Overlay_FOUND=${OGRE_OV_Overlay_FOUND}")
message("AAAAAAAAAAA x3 OGRE_OV_Overlay_INCLUDE_DIRS=${OGRE_OV_Overlay_INCLUDE_DIRS}")
message("AAAAAAAAAAA x3 OGRE_OV_Overlay_LIBRARIES=${OGRE_OV_Overlay_LIBRARIES}")

if (1)
  find_package(PkgConfig REQUIRED)

  # TODO: OGRE-Overlay is new in OGRE 1.9
  pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

  message("AAAAAAAAAAA x4 OGRE_OV_FOUND=${OGRE_OV_FOUND}")
  message("AAAAAAAAAAA x4 OGRE_OV_OGRE=${OGRE_OV_FOUND}")
  message("AAAAAAAAAAA x4 OGRE_OV_LIBRARIES=${OGRE_OV_LIBRARIES}")
  message("AAAAAAAAAAA x4 OGRE_OV_OGRE_LIBRARIES=${OGRE_OV_OGRE_LIBRARIES}")
  message("AAAAAAAAAAA x4 OGRE_OV_Overlay_FOUND=${OGRE_OV_Overlay_FOUND}")
  message("AAAAAAAAAAA x4 OGRE_OV_Overlay_INCLUDE_DIRS=${OGRE_OV_Overlay_INCLUDE_DIRS}")
  message("AAAAAAAAAAA x4 OGRE_OV_Overlay_LIBRARIES=${OGRE_OV_Overlay_LIBRARIES}")

  # Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
  # so try looking for just OGRE
  if(NOT OGRE_OV_FOUND)
    message("AAAAAAAAAAA 1 OGRE_OV NOT FOUND - TRY AGAIN")
    pkg_check_modules(OGRE_OV REQUIRED OGRE)
    message("AAAAAAAAAAA x5 OGRE_OV_FOUND=${OGRE_OV_FOUND}")
    message("AAAAAAAAAAA x5 OGRE_OV_OGRE=${OGRE_OV_FOUND}")
    message("AAAAAAAAAAA x5 OGRE_OV_LIBRARIES=${OGRE_OV_LIBRARIES}")
    message("AAAAAAAAAAA x5 OGRE_OV_OGRE_LIBRARIES=${OGRE_OV_OGRE_LIBRARIES}")
    message("AAAAAAAAAAA x5 OGRE_OV_Overlay_FOUND=${OGRE_OV_Overlay_FOUND}")
    message("AAAAAAAAAAA x5 OGRE_OV_Overlay_INCLUDE_DIRS=${OGRE_OV_Overlay_INCLUDE_DIRS}")
    message("AAAAAAAAAAA x5 OGRE_OV_Overlay_LIBRARIES=${OGRE_OV_Overlay_LIBRARIES}")
  endif(NOT OGRE_OV_FOUND)

  ## Find OGRE Plugin path (not necessarily platform-independent, I guess)
  execute_process(COMMAND
    pkg-config --variable=plugindir OGRE
    OUTPUT_VARIABLE OGRE_PLUGIN_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})

  message("AAAAAAAAAAA x6 OGRE_OV_LIBRARIES_ABS=${OGRE_OV_LIBRARIES_ABS}")
  message("AAAAAAAAAAA x6 OGRE_OV_LIBRARIES_ABS_TMP=${OGRE_OV_LIBRARIES_ABS_TMP}")

  # find absolute path of ogre libraries
  # This is stored in the cache to save time on cmake re-run.
  if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
    unset(OGRE_OV_LIBRARIES_ABS_TMP)
    set(OGRE_OV_NEED_ABS_LIBS FALSE)
    foreach(_lib ${OGRE_OV_LIBRARIES})
      set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")
      message("AAAAAAAAAAA ===_lib=${_lib}")
      message("AAAAAAAAAAA    OGRE_OV_LIB_TAG=${OGRE_OV_LIB_TAG}")
      message("AAAAAAAAAAA    OGRE_OV_LIB_TAG-val=${${OGRE_OV_LIB_TAG}}")
      message("AAAAAAAAAAA    PATHS=${OGRE_OV_LIBRARY_DIRS}")

      find_library(${OGRE_OV_LIB_TAG}_S
              NAMES ${_lib}
      )
      message("AAAAAAAAAAA    OGRE_OV_LIB_TAG_S=${OGRE_OV_LIB_TAG}_S")
      message("AAAAAAAAAAA    OGRE_OV_LIB_TAG_S-val=${${OGRE_OV_LIB_TAG}_S}")

      find_library(${OGRE_OV_LIB_TAG}
              NAMES ${_lib}
              HINTS ${OGRE_OV_LIBRARY_DIRS}
              PATHS ${OGRE_OV_LIBRARY_DIRS}
      )
      message("AAAAAAAAAAA    OGRE_OV_LIB_TAG=${OGRE_OV_LIB_TAG}")
      message("AAAAAAAAAAA    OGRE_OV_LIB_TAG-val=${${OGRE_OV_LIB_TAG}}")

      if(NOT ${${OGRE_OV_LIB_TAG}_S} STREQUAL ${${OGRE_OV_LIB_TAG}})
        message("AAAAAAAAAAA    NOT EQUAL")
        set(OGRE_OV_NEED_ABS_LIBS TRUE)
      endif()

      set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
      message("AAAAAAAAAAA    OGRE_OV_LIBRARIES_ABS_TMP=${OGRE_OV_LIBRARIES_ABS_TMP}")
    endforeach(_lib)

    # if the libraries found with pkg-config do not match the ones found with
    # system paths, then use the full path names so they are found correctly by
    # dependent projects.  Otherwise use the short names (better for building
    # deb packages).
    if(OGRE_OV_NEED_ABS_LIBS)
      set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
          CACHE FILEPATH "Pathname of library ${_lib}")
    else()
      set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES}
          CACHE FILEPATH "Pathname of library ${_lib}")
    endif()
  endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
  
  message("AAAAAAAAAAA x7 OGRE_OV_LIBRARIES_ABS=${OGRE_OV_LIBRARIES_ABS}")
  
endif()



message("AAAAAAAAAAA x6 OGRE_OV_FOUND=${OGRE_OV_FOUND}")
message("AAAAAAAAAAA x6 OGRE_OV_OGRE=${OGRE_OV_FOUND}")
message("AAAAAAAAAAA x6 OGRE_OV_LIBRARIES=${OGRE_OV_LIBRARIES}")
message("AAAAAAAAAAA x6 OGRE_OV_LIBRARY_DIRS=${OGRE_OV_LIBRARY_DIRS}")
message("AAAAAAAAAAA x6 OGRE_OV_OGRE_LIBRARIES=${OGRE_OV_OGRE_LIBRARIES}")
message("AAAAAAAAAAA x6 OGRE_OV_Overlay_FOUND=${OGRE_OV_Overlay_FOUND}")
message("AAAAAAAAAAA x6 OGRE_OV_Overlay_INCLUDE_DIRS=${OGRE_OV_Overlay_INCLUDE_DIRS}")
message("AAAAAAAAAAA x6 OGRE_OV_Overlay_LIBRARIES=${OGRE_OV_Overlay_LIBRARIES}")

message("AAAAAAAAAAA OGRE_OV_LIBRARIES=${OGRE_OV_LIBRARIES}")
message("AAAAAAAAAAA OGRE_OV_LIBRARY_DIRS=${OGRE_OV_LIBRARY_DIRS}")
message("AAAAAAAAAAA OGRE_OV_LDFLAGS=${OGRE_OV_LDFLAGS}")
message("AAAAAAAAAAA OGRE_OV_LDFLAGS_OTHER=${OGRE_OV_LDFLAGS_OTHER}")
message("AAAAAAAAAAA OGRE_OV_INCLUDE_DIRS=${OGRE_OV_INCLUDE_DIRS}")
message("AAAAAAAAAAA OGRE_OV_CFLAGS=${OGRE_OV_CFLAGS}")
message("AAAAAAAAAAA OGRE_OV_CFLAGS_OTHER=${OGRE_OV_CFLAGS_OTHER}")



#
#message("AAAAAAAAAAA x10 ACORN_TST_OGREMAIN_LIB=${ACORN_TST_OGREMAIN_LIB}")
#message("AAAAAAAAAAA x10 ACORN_TST_OGREOVERLAY_LIB=${ACORN_TST_OGREOVERLAY_LIB}")
#find_library(ACORN_TST_OGREMAIN_LIB NAMES OgreMain)
#find_library(ACORN_TST_OGREOVERLAY_LIB NAMES OgreOverlay)
#message("AAAAAAAAAAA x11 ACORN_TST_OGREMAIN_LIB=${ACORN_TST_OGREMAIN_LIB}")
#message("AAAAAAAAAAA x11 ACORN_TST_OGREOVERLAY_LIB=${ACORN_TST_OGREOVERLAY_LIB}")
#
#
#link_directories(${OGRE_OV_LIBRARY_DIRS})
#message("AAAAAAAAAAA x12 ACORN_TST_OGREMAIN_LIB_2=${ACORN_TST_OGREMAIN_LIB_2}")
#message("AAAAAAAAAAA x12 ACORN_TST_OGREOVERLAY_LIB_2=${ACORN_TST_OGREOVERLAY_LIB_2}")
#find_library(ACORN_TST_OGREMAIN_LIB_2 NAMES OgreMain)
#find_library(ACORN_TST_OGREOVERLAY_LIB_2 NAMES OgreOverlay)
#message("AAAAAAAAAAA x13 ACORN_TST_OGREMAIN_LIB_2=${ACORN_TST_OGREMAIN_LIB_2}")
#message("AAAAAAAAAAA x13 ACORN_TST_OGREOVERLAY_LIB_2=${ACORN_TST_OGREOVERLAY_LIB_2}")
#


if(APPLE)
  FIND_LIBRARY(Cocoa_LIBRARIES Cocoa)
  set(rviz_ADDITIONAL_LIBRARIES ${Cocoa_LIBRARIES})
endif()

find_package(OpenGL REQUIRED)

find_package(PythonLibs REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)

find_package(urdfdom REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
  angles
  geometry_msgs
  image_geometry
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  message_filters
  nav_msgs
  pluginlib
  python_qt_binding
  resource_retriever
  rosbag
  rosconsole
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  urdf
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    src
    ${EIGEN_INCLUDE_DIRS}
    ${OGRE_OV_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${urdfdom_INCLUDE_DIRS}
  LIBRARIES
    rviz
    default_plugin
    ${OGRE_OV_LIBRARIES_ABS}
    ${rviz_ADDITIONAL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${urdfdom_LIBRARIES}
  CATKIN_DEPENDS
    geometry_msgs
    image_geometry
    image_transport
    interactive_markers
    laser_geometry
    map_msgs
    message_filters
    nav_msgs
    pluginlib
    resource_retriever
    roscpp
    roslib
    sensor_msgs
    std_msgs
    tf
    urdf
    visualization_msgs
)

include_directories(SYSTEM
  ${EIGEN_INCLUDE_DIRS}
  ${OGRE_OV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${PYTHON_INCLUDE_PATH}
  ${urdfdom_INCLUDE_DIRS}
)
include_directories(src ${catkin_INCLUDE_DIRS})

#link_directories(${OGRE_OV_LIBRARY_DIRS})

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

#### If gtk ends up being the best way to get the correct window
#### position under X11, this is how to compile it in.
#
# find_package(GTK2)
# include_directories(${GTK2_INCLUDE_DIRS})
# include_directories(/usr/include/gdk-pixbuf-2.0)

add_subdirectory(src)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES default.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

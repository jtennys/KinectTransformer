// Filename: kinect_transformer.h
// Author: Jason Tennyson
// Date: 4-25-11
// 
// This file contains all of the pre-defined values of the
// kinect transformer package.

#ifndef kinect_transformer_h_
#define kinect_transformer_h_

#include <ros/ros.h>
#include <fstream>
#include <cstdlib>
#include <sys/stat.h>
#include <math.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/algorithm/string.hpp>
#include <Eigen/Geometry>
#include <pcl/common/transform.hpp>

// The file that the file names and parameters are read from.
#define PTS_INFO_FILE ("pts_info_file.txt")

// The folders that all of the files are separated into.
// These file folders are assumed to be in the root package directory.
#define PTS_IN_FOLDER ("individual_pts")
#define PTS_OUT_FOLDER ("combined_pts")
#define VRML_FOLDER ("vrml_files")
#define IMG_FOLDER ("image_files")
#define PTS_INFO_FOLDER (".")

// These are our file extensions
#define VRML_EXTENSION ("wrl")
#define PTS_EXTENSION ("pts")
#define IMG_EXTENSION ("bmp")

// These defines are used to convert units so users can input familiar units.
#define DEG_TO_RAD (0.0174533)

// This is the size of the tabbed white space that we use
// to signify that we are inside of a certain vrml scope.
#define	TAB_SIZE (2)

// The delay time in seconds between when the program starts and when a cloud is taken.
#define CAM_DELAY (5)

// The templated pcl object type.
typedef pcl::PointXYZRGB PointT;

#endif

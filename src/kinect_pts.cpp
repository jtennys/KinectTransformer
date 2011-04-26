// Filename: kinect_pts.cpp
// Author: Jason Tennyson
// Date: 4-25-11
// 
// This file is designed to be used after the openni_camera driver is launched
// for use with the Microsoft Kinect.  It reads a single point cloud message
// from the Kinect, formats the data, and saves it with a .pts file extension.

#include "kinect_transformer.h"

// This boolean value is set to true when we get a point cloud.
// The program loops until it gets a point cloud and sets this to true.
bool DATA_FULL = false;

// The user-specified filename that we are going to save the point cloud data to.
std::string FILE_NAME = "";

// The file path that we extract from the launch file.
std::string FILE_PATH = "";

// Accepts a point cloud of type XYZRGB from the callback function and saves
// its RGB data to a file with the defined image extension.
void saveImage(const pcl::PointCloud<PointT>::Ptr cloud);

// This function is called when the kinect driver pushes out a point cloud message.
void pcdCallback(const sensor_msgs::PointCloud2::Ptr msg);

int main(int argc, char** argv)
{
  // Store the file name given by the launch file.
  FILE_NAME += argv[1];

  // Store the path to the root package directory given by the launch file.
  FILE_PATH = argv[2];

  // This time delay allows you to start the program and walk away if it's in view of the Kinect.
  ROS_INFO("You now have %d seconds to back away!",CAM_DELAY);
  sleep(CAM_DELAY);
  ROS_INFO("Time is up!");

  // Initializes the node as a kinect listener.
  ros::init(argc, argv, "kinect_to_pcd");

  // Create an empty node handle.
  ros::NodeHandle n;

  // Create a subscriber on the depth topic.
  ros::Subscriber sub = n.subscribe("camera/depth/points2", 1000, pcdCallback);

  // Waits for new data. Then, the callback function is executed.
  while((!DATA_FULL) && ros::ok())
  {
    ros::spinOnce();
  }

  return (0);
}

void saveImage(const pcl::PointCloud<PointT>::Ptr cloud)
{
  // Integer return value when attempting to change a directory.
  int good_dir = 0;

  // Change directories to the root folder.
  if(chdir(FILE_PATH.c_str()) == 0)
  {
    good_dir = chdir(IMG_FOLDER);

    // If we can't change directories to the image folder, we assume it doesn't exist and make it.
    if(good_dir != 0)
    {
      // Make the image folder.
      if(mkdir(IMG_FOLDER, 0755) == 0)
      {
        // If successful, change directories into this image folder.
        good_dir = chdir(IMG_FOLDER);
      }
      else
      {
        ROS_INFO("Program failed, likely due to improper read/write permissions in this directory.");
      }
    }

    // If we are in the right directory, save the image file.
    if(good_dir == 0)
    {
      // Create a blank cv image object.
      IplImage* img = cvCreateImage(cvSize(cloud->width,cloud->height), IPL_DEPTH_8U, 3);

      // Populate the image object with rgb data from the point cloud we received.
      for(unsigned int i = 0; i < cloud->height; i++)
      {
        for(unsigned int j = 0; j < cloud->width; j++)
        {
          // Store the floating point rgb value as an integer for bit masking.
          int rgb = *reinterpret_cast<int*>(&cloud->points[i*cloud->width + j].rgb);
          ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = (rgb & 0xff);
          ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = ((rgb >> 8) & 0xff);
          ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = ((rgb >> 16) & 0xff);
        }
      }

      // Create a string and add the file name and its desired image extension type.
      std::string img_filename = FILE_NAME;
      img_filename += ".";
      img_filename += IMG_EXTENSION;

      // Save the image object to a file with the given name.
      cvSaveImage(img_filename.c_str(),img);

      // Delete the image object.
      cvReleaseImage(&img);
    }
    else
    {
      ROS_INFO("Program failed! Could not change directories to %s/%s!",FILE_PATH.c_str(),IMG_FOLDER);
    }
  }
}

void pcdCallback(const sensor_msgs::PointCloud2::Ptr msg)
{
  // The return value of an attempt to change or make directory is stored in this.
  int good_dir = 0;

  // We first check if data is full because even if we finish and save the file,
  // queued callback data continues to execute and save files.
  if(!DATA_FULL)
  {
    // Create an empty cloud.
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());

    // Fill the empty cloud with data from the kinect.
    pcl::fromROSMsg(*msg,*cloud);

    // If our root path is correct, continue.
    if(chdir(FILE_PATH.c_str()) == 0)
    {
      good_dir = chdir(PTS_IN_FOLDER);

      // If we aren't in the right directory, make the directory.
      if(good_dir != 0)
      {
        // If we are successful in making the directory, change directories to that directory.
        if(mkdir(PTS_IN_FOLDER, 0755) == 0)
        {
          good_dir = chdir(PTS_IN_FOLDER);
        }
        else
        {
          ROS_INFO("Program failed, likely due to improper read/write permissions in this directory.");
        }
      }

      // If we are now in the correct directory, write the data to disk.
      if(good_dir == 0)
      {
        // Create the file name with extension.
        std::string pts_file = FILE_NAME;
        pts_file += ".";
        pts_file += PTS_EXTENSION;

        // Open the output file stream.
        std::ofstream pts_out(pts_file.c_str());

        // Write the point cloud data to disk.
        for(unsigned int i = 0; i < cloud->points.size(); i++)
        {
          // If x y and z are numbers, we print the line.
          if(!((cloud->points[i].x != cloud->points[i].x) ||
               (cloud->points[i].y != cloud->points[i].y) ||
               (cloud->points[i].z != cloud->points[i].z)))
          {
            // Print the distance information to the pts file followed by a trash buffer value.
            pts_out << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " 0 ";

            // Print the RGB information and finish with a new line character.
            int rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
            pts_out << ((rgb >> 16) & 0xff) << " " << ((rgb >> 8) & 0xff) << " " << (rgb & 0xff) << "\n";
          }
        }

        // Print out one last new line at the end of the pts file.
        pts_out << "\n";

        // Write the RGB image to disk.
        saveImage(cloud);

        // We are done running the program.
        ROS_INFO("Done!");
        ROS_INFO("Point data was saved at %s/%s/%s.%s!",FILE_PATH.c_str(),PTS_IN_FOLDER,FILE_NAME.c_str(),PTS_EXTENSION);
        ROS_INFO("An image was saved at %s/%s/%s.%s!",FILE_PATH.c_str(),IMG_FOLDER,FILE_NAME.c_str(),IMG_EXTENSION);
        ROS_INFO("The point collection process has shut down!");
        ROS_INFO("Press Ctrl+C to shut down the Kinect driver...");

        // We have received our data and are done here.
        DATA_FULL = true;
      }
      else
      {
        ROS_INFO("Program failed! Could not change directories to %s/%s!",FILE_PATH.c_str(),PTS_IN_FOLDER);
      }
    }
    else
    {
      ROS_INFO("Incorrect root path given!");
    }
  }

  // Make the info file path if necessary.
  if(chdir(FILE_PATH.c_str()) == 0)
  {
    good_dir = chdir(PTS_INFO_FOLDER);

    // If we aren't in the right directory, make the directory.
    if(good_dir != 0)
    {
      // If we are successful in making the directory, change directories to that directory.
      if(mkdir(PTS_INFO_FOLDER, 0755) == 0)
      {
        good_dir = chdir(PTS_INFO_FOLDER);
      }
      else
      {
        ROS_INFO("Program failed, likely due to improper read/write permissions in this directory.");
      }
    }

    // If we're in the right directory, check to see if we can read the pts info file.
    // If we can't, create a placeholder file of the correct name with placeholder values.
    if(good_dir == 0)
    {
      // Open an input stream to the pts info file.
      std::ifstream pts_info_check(PTS_INFO_FILE);

      if(pts_info_check.is_open())
      {
        // If opening the file for reading was a success, close it.
        pts_info_check.close();
      }
      else
      {
        // If we could not open the file, we assume it's missing, and we create it.
        std::ofstream pts_info_check(PTS_INFO_FILE);

        // Fill the pts info file with a placeholder data set. The format is:
        // File name without extension, x, y, z in meters, x angle, y angle, z angle in degrees.
        pts_info_check << "placeholder,0,0,0,0,0,0\n";

        // Close the file.
        pts_info_check.close();
      }
    }
  }
}
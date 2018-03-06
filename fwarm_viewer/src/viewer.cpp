/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_definitions.h>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "sensor_sim/sensor_data.h"
#include "std_msgs/String.h"


class Receiver
{
public:
	enum Mode
	{
	  IMAGE = 0
	};

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;
	const bool rangeset = false;
	const int inputrange = 0.0f;

  bool updateImage, updateCloud;
  bool save;
  bool running;

  size_t frame;
  const size_t queueSize;


  int safeDistance = 5000.0f;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo,sensor_sim::sensor_data> ExactSyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo,sensor_sim::sensor_data> ApproximateSyncPolicy;

  ros::NodeHandle nh;

  //Sensor node handle, simulates sensor data
  ros::NodeHandle snh;

//Asynchoros spinner for mutlithreaded spinning i.e when callbacks are allowed to run, defaults to use as many threads as the hardware allows
  ros::AsyncSpinner spinner;
//Image_transport subrcibers is the equivalent of using message filters but for ros image msgs
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
//Message_filters subrcibers, from ros wiki  "A message filter is defined as something which a message arrives into and may or may not be spit back out of at a later point in time.". Used to be able to let through only msgs that are synched.
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
  message_filters::Subscriber<sensor_sim::sensor_data> *subVelocity;
  
//Message_filters synchronizer with exact and approximate policies, from ros wiki "The Synchronizer filter synchronizes incoming channels by the timestamps contained in their headers, and outputs them in the form of a single callback that takes the same number of channels."
  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed,const bool rangeset, const int inputrange)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),rangeset(rangeset),inputrange(inputrange),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(IMAGE)
  {

    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;
		if(rangeset){
			OUT_INFO("Maxrange(mm): " << inputrange);
			this->safeDistance=inputrange;
		}

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
	  subVelocity = new message_filters::Subscriber<sensor_sim::sensor_data> (snh, "wheeler_velocity", 10);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

	

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth, *subVelocity);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4,_5));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth, *subVelocity);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4,_5));
    }


    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    createLookup(this->color.cols, this->color.rows);

      imageViewer();
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth,  const sensor_sim::sensor_data::ConstPtr vel)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

		if(not this->rangeset){
			this->safeDistance = (vel->velocity*12000.0f)/50;
		}
    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }

  void imageViewer()
  {
    cv::Mat color, depth, depthDisp, combined, depthResize, eightBitDepth, detectMask;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::namedWindow("Image Viewer", cv::WindowFlags::WINDOW_NORMAL);

    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateImage = false;
        lock.unlock();

        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if(elapsed >= 1.0)
        {
          fps = frameCount / elapsed;
          oss.str("");
          oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)   Maxdistance: " << this->safeDistance/1000.0 
					<< " m Velocity: " << this->safeDistance*50/12000.0 << " km/h";
          start = now;
          frameCount = 0;
        }

	//floatvalue is lentgh in millimeters for the lidar
        dispDepth(depth, depthDisp, eightBitDepth, this->safeDistance);
				humanDetector(color, detectMask);

	//resize(640,480,color,depthDisp,color,depthDisp);
        combine(color, depthDisp, eightBitDepth, detectMask, combined);

        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
        cv::imshow("Image Viewer", combined);
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        if(mode == IMAGE)
        {
          saveImages(color, depth, depthDisp);
        }
        break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }

//Resize(outputdwidth,outputheight,input_imagecolor,input_imagedepth,output_imagecolor,output_imagedepth)
  void resize(int width, int height,const cv::Mat &inC, const cv::Mat &inD, cv::Mat &outC,cv::Mat &outD)
  {
	// For enlarging use interpolation INTER_CUBIC
	if(width*height>inC.cols*inC.rows){
		//resize(InputArray src, OutputArray dst, Size dsize, double fx=0, double fy=0, int interpolation )
		//fx,fy = scaling parameters see opencv doc resize	
		cv::resize(inC,outC,cv::Size(width,height),0.0,0.0,cv::INTER_CUBIC);
	}else{
		// For shrinking use interpolation INTER_AREA
		cv::resize(inC,outC,cv::Size(width,height),0.0,0.0,cv::INTER_AREA);
	}
	if(width*height>inD.cols*inD.rows){
		cv::resize(inD,outD,cv::Size(width,height),0.0,0.0,cv::INTER_CUBIC);
	}else{
		cv::resize(inD,outD,cv::Size(width,height),0.0,0.0,cv::INTER_AREA);
	}
}

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, cv::Mat &monoOut, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }
	monoOut = tmp;
    cv::applyColorMap(tmp, out, cv::COLORMAP_AUTUMN);
  }
  
  /* Function takes in color, depthDisp,combined, (color according to comment above) */
  void combine(const cv::Mat &inC, const cv::Mat &inD, const cv::Mat &inM, const cv::Mat &inH, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r){
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
      *itD = inD.ptr<cv::Vec3b>(r);
	  	const uint8_t 
			*itM = inM.ptr<uint8_t>(r),
			*itH = inH.ptr<uint8_t>(r);
	  
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO, ++itM, ++itH){
				if(*itM > 0.0f && *itM < 255.0f && *itH == 255.0f){
					itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
				  itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
				  itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
				}
				else{
					itO->val[0] = itC->val[0];
					itO->val[1] = itC->val[1];
					itO->val[2] = itC->val[2];
				}
    	}
    }
  }

  void humanDetector(const cv::Mat &videoIn, cv::Mat &maskOut){
	// initialize Histogram Of Oriented Gradients detector
	// SVM = support vector machine
	// detectMultiscale does all the magic. This can be optimized.
	// search for explanations of the parameters
	cv::HOGDescriptor hog;
	hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
	
	std::vector<cv::Rect> found, found_filtered;	
	hog.detectMultiScale(videoIn, found, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
	
	maskOut = cv::Mat(videoIn.rows, videoIn.cols, CV_8U, 0.0f);

	size_t i, j;
	for(i=0; i<found.size(); i++)
        {
            cv::Rect r = found[i];  
            for (j=0; j<found.size(); j++)   
                if (j!=i && (r & found[j])==r)
		break;
            if (j==found.size())
                found_filtered.push_back(r);    
        }
        for(i=0; i<found_filtered.size(); i++)
        {
	    cv::Rect r = found_filtered[i];
            r.x += cvRound(r.width*0.1);
	    r.width = cvRound(r.width*0.8);
	    r.y += cvRound(r.height*0.06);
	    r.height = cvRound(r.height*0.9);  
	    //rectangle(videoIn, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
	    rectangle(maskOut, r.tl(), r.br(), 255.0f, CV_FILLED);
	}	
}

  void saveImages(const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    OUT_INFO("saving color: " << colorName);
    cv::imwrite(colorName, color, params);
    OUT_INFO("saving depth: " << depthName);
    cv::imwrite(depthName, depth, params);
    OUT_INFO("saving depth: " << depthColoredName);
    cv::imwrite(depthColoredName, depthColored, params);
    OUT_INFO("saving complete!");
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};

void help(const std::string &path)
{
  std::cout << path << FG_BLUE " [options]" << std::endl
            << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the topic namespace" << std::endl
            << FG_GREEN "  image topic path" NO_COLOR ": " FG_YELLOW "'first string beginning with /'" NO_COLOR " equals to the full image topic path" << std::endl
            << FG_GREEN "  depth topic path" NO_COLOR ": " FG_YELLOW "'second string beginning with /'" NO_COLOR " equals to the full depth topic path" << std::endl
            << FG_GREEN "  kinect mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
            << FG_GREEN "  options" NO_COLOR ":" << std::endl
            << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
            << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl
						<< FG_YELLOW "    'setrange x'" NO_COLOR " sets the maxrange to x where x is distance in mm " << std::endl;


}

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  ros::init(argc, argv, "fwarm_viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  bool useExact = true;
  bool useCompressed = false;
  Receiver::Mode mode = Receiver::IMAGE;
  bool firstDefined = false;

	int inputrange=0.0f;
	bool rangeset=false;

  
  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
	else if(param.at(0)=='/') 
	{
	  ns = "";
	  if(firstDefined==false){
		topicColor = param;
		firstDefined=true;
      }else{
	    topicDepth = param;
	  }
	}

    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }
    else if(param == "compressed")
    {
      useCompressed = true;
    }
	else if(rangeset) 
	{
		inputrange=std::stoi(param);
	}
	else if(param == "setrange" ) 
	{
		rangeset=true;
	}
    else
    {
      ns = param;
    }
  }
if(firstDefined==true){
  topicColor = topicColor;
  topicDepth = topicDepth;
}else{
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
}
  OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
  OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed,rangeset,inputrange);

  OUT_INFO("starting receiver...");
  receiver.run(mode);

  ros::shutdown();
  return 0;
}

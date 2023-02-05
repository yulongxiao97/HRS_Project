/* INCLUDES */
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/ColorRGBA.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "std_msgs/Float32.h"
// OPEN CV libraries
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// Service messages
#include "nao_control/LookAround.h"
#include "nao_control/TurnAround.h"
#include "nao_control/NaoWalk.h"
#include "nao_control/NaoWalkWithArms.h"
#include "nao_control/BottleDetector.h"
#include "nao_control/detection.h"
#include "nao_control/ArucoDetector.h"
#include "nao_control/ArucoNavigation.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
/* NAME SPACES */
using namespace std;
using namespace cv;

/* DEFINES */
#define SEARCH_LABEL_TIME 10
#define FLIP_90_LEFT -1.57
#define NON_RECYCLE_ARUCO_ID 15
#define RECYCLE_ARUCO_ID 10

static const std::string OPENCV_WINDOW1 = "Detection";
static const std::string OPENCV_WINDOW2 = "raw";

/* GLOBAL VARIABLES */
bool stop_thread=false;

// Main Control Loop States
enum STATES { IDLE, BOTTLE_DETECTION, BOTTLE_NAVIGATION, CLASSIFY_BOTTLE, BIN_DETECTION, BIN_NAVIGATION, DEPOSIT_BOTTLE };
int STATE;

void spinThread()
{
	while(!stop_thread)
	{
		ros::spinOnce();
		//ROS_INFO_STREAM("Spinning the thing!!");
	}
}

class Bottle
{
	// client for bottle detector node
	ros::NodeHandle nh_;
	ros::ServiceClient bottle_detector_client;

	public:
		// Attributes
		bool bottle_detected;
		bool recyclable;
		float detected_blob_size;
		float angle;
		float real_blob_size;
		bool logo_detected;
		int rotate_times;

		// logo detection
		ros::Subscriber image_sub_top;
		ros::ServiceClient logo_detection_client;
		cv_bridge::CvImagePtr cv_ptr;
		nao_control::detection img;
		string text;
		cv::Scalar color;

		// Constructor
		Bottle()
		{
			bottle_detected = false;
			recyclable= false;
			// recyclable= true;
			angle= 0;
			detected_blob_size= 0;
			real_blob_size= 160; //pixels (at a distance of 22cm = grasping distance)
			logo_detected= false;
<<<<<<< HEAD
			rotate_times = 0;
			
=======

>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
			// service for bottle detector node
			bottle_detector_client = nh_.serviceClient <nao_control::BottleDetector>("bottledetector");

			// service for bottle detector node
			image_sub_top = nh_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Bottle::symbol_Detection, this);

			// service for bottle detector node
			logo_detection_client = nh_.serviceClient <nao_control::detection>("logo_detection");
		}

		// Destructor
		~Bottle()
		{
			cv::destroyWindow(OPENCV_WINDOW1);
    		cv::destroyWindow(OPENCV_WINDOW2);
		}

		// Methods
		void bottle_find(void)
		{
			/* THIS TRIGGERS THE BOTTLE DETECTOR NODE */
			nao_control::BottleDetector srv;
			srv.request.time = SEARCH_LABEL_TIME;
			if (bottle_detector_client.call(srv))
			{
				if(srv.response.reply)
				{
					detected_blob_size= srv.response.blob_size;
					angle= srv.response.angle;
					bottle_detected= true;
				}
				else
				{
					ROS_INFO("[MAIN] BOTTLE NOT DETECTED");
					bottle_detected= false;
				}
			}
			else
			{
				ROS_ERROR("[MAIN] Failed to call bottle detector service");
				bottle_detected= false;
			}
		}

		void symbol_Detection(const sensor_msgs::Image& msg)
		{
			if(STATE == CLASSIFY_BOTTLE)
			{
				// img.request.img.header.seq = msg.header.seq;
				// img.request.img.header.stamp = msg.header.stamp;
				// img.request.img.header.frame_id = msg.header.frame_id;
				// img.request.img.height = msg.height;
				// img.request.img.width = msg.width;
				// img.request.img.encoding = msg.encoding;
				// img.request.img.is_bigendian = msg.is_bigendian;
				// img.request.img.step = msg.step;
				// img.request.img.data = msg.data;

				try
				{
					cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				}
				catch (cv_bridge::Exception& e)
				{
					ROS_ERROR("[MAIN] cv_bridge exception: %s", e.what());
					return;
				}

				imshow(OPENCV_WINDOW2, cv_ptr->image);
				cv::moveWindow(OPENCV_WINDOW2, 100, 400);
				waitKey(10);

				// convert to HSV color space and split channels
				cv::Mat hsvImage;
				cv::cvtColor(cv_ptr->image, hsvImage, CV_BGR2HSV);
				cv::Mat hsv_channels[3];
				cv::split(hsvImage, hsv_channels);

				blobDetection(hsvImage);
				if(detected_blob_size == 0)
				{
					logo_detected = true;
					ROS_INFO("[MAIN] LOGO DETECTED");
				}
				else
				{
					logo_detected = false;
					ROS_INFO("[MAIN] LOGO NOT DETECTED");
				}
				// call_logo_detection();
			}
		}

		void blobDetection(cv::Mat hsvImage)
		{
			cv::Mat binaryImage1, binaryImage2, binaryImage;

			// color extraction
			cv::inRange(hsvImage, cv::Scalar(0, 100, 20), cv::Scalar(10,255,255), binaryImage1);
			cv::inRange(hsvImage, cv::Scalar(160,100,20), cv::Scalar(179,255,255), binaryImage2);
			binaryImage = binaryImage1 + binaryImage2;

			// dilate/erodate
			cv::Mat elementErode, erodedImage, elementDilate, dilatedImage;
			elementErode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2));
			elementDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6,6));
			cv::erode(binaryImage, erodedImage, elementErode);
			cv::dilate(erodedImage, dilatedImage, elementDilate);

			// Set up the detector with default parameters.
			cv::SimpleBlobDetector::Params params;
			// Change thresholds
			params.minThreshold = 10;
			params.maxThreshold = 200;
			params.filterByColor = true;
			params.blobColor = 255;

			// Filter by Area.
			params.filterByArea = true;
			params.minArea = 200;
			params.maxArea = 10000000;

			cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

			// Detect blobs.
			std::vector<cv::KeyPoint> keypoints;
			detector->detect(dilatedImage, keypoints);

			int max_index = 0;
			for(int i = 0; i < keypoints.size(); i++) {
				if(keypoints[max_index].size < keypoints[i].size){
					max_index = i;
				}
			}

			cv::Mat imageWithKeypoints;
			std::vector<cv::KeyPoint> maxKeypoints;

			if(keypoints.size()){
				maxKeypoints.push_back(keypoints[max_index]);
				detected_blob_size = keypoints[max_index].size;
			}
			else
			{
				detected_blob_size= 0;
			}

			return;
		}

<<<<<<< HEAD

		// void call_logo_detection(void)
		// {						
		// 	if (logo_detection_client.call(img))
		// 	{
		// 		ROS_INFO("[MAIN] logo detection service successfully called");
		// 		if (img.response.get==true)
		// 		{					
		// 			if (times == 20)
		// 			{
		// 				// if(detected_blob_size == 0)							
		// 				// {

		// 				logo_detected = true;						
		// 				// }
		// 			}						
		// 			text = "pfand symbol detected"; 
		// 			color = Scalar(0,255,0);
		// 			times++;					
		// 		}            
		// 		else
		// 		{
		// 			text = "no pfand symbol";
		// 			logo_detected = false;
		// 			color = Scalar(0,0,255);
		// 		}
		// 		putText(cv_ptr->image, text, Point(10,30),FONT_HERSHEY_COMPLEX_SMALL,1, color, 1, LINE_AA);
		// 		imshow(OPENCV_WINDOW1, cv_ptr->image);
		// 		cv::resizeWindow("Detection", 1000, 1000);
		// 		cv::moveWindow(OPENCV_WINDOW1, 400, 600);
		// 		waitKey(10);
		// 	}
		// 	else
		// 	{
		// 		ROS_ERROR("[MAIN] failed to call logo detection service");
		// 	} 
		// 	// pfand = pfand*(srv.response.get);
		// 	// return  pfand;        
		// }

=======
		/*
		void call_logo_detection(void)
		{
			if (logo_detection_client.call(img))
			{
				ROS_INFO("[MAIN] logo detection service successfully called");
				if (img.response.get==true)
				{
					if (times == 10)
					{
						if(detected_blob_size == 0)
						{
							text = "pfand symbol detected";
							color = Scalar(0,255,0);
							logo_detected = true;
						}
					}

					times++;
				}
				else
				{
					text = "no pfand symbol";
					logo_detected = false;
					color = Scalar(0,0,255);
				}
				putText(cv_ptr->image, text, Point(10,30),FONT_HERSHEY_COMPLEX_SMALL,1, color, 1, LINE_AA);
				imshow(OPENCV_WINDOW1, cv_ptr->image);
				cv::moveWindow(OPENCV_WINDOW1, 500, 400);
				waitKey(10);
			}
			else
			{
				ROS_ERROR("[MAIN] failed to call logo detection service");
			}
			// pfand = pfand*(srv.response.get);
			// return  pfand;
		}
		*/
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
};

class Bin
{
	ros::NodeHandle nh_;

	public:
	// Atributes
	bool aruco_detected;
	ros::ServiceClient aruco_detector_client;
	float aruco_angle;
	float detected_aruco_area;
	float real_aruco_area;

	// Constructor
	Bin()
	{
		aruco_angle= 0;
		detected_aruco_area= 0;
		real_aruco_area= 3800; // in px for a distance of 40cm

		aruco_detected= false;
		// aruco_detector_client = nh_.serviceClient <nao_control::ArucoDetector>("arucodetector"); // ARUCO DETECTOR
		aruco_detector_client = nh_.serviceClient <nao_control::ArucoDetector>("arucobroadcast"); // ARUCO DETECTOR
	}

	// Destructor
	~Bin()
	{
	}

	void aruco_find(int id)
	{
		/* THIS TRIGGERS THE BOTTLE DETECTOR NODE */
		nao_control::ArucoDetector srv;
		srv.request.time = SEARCH_LABEL_TIME/2;
		srv.request.id = id;

		if (aruco_detector_client.call(srv))
		{
			if(srv.response.new_pos.linear.x != 0 && srv.response.new_pos.linear.y != 0 && srv.response.new_pos.linear.z != 0)
			{
				ROS_INFO("[MAIN] Successfully call aruco detector service");
				// We will not use this ones
				// aruco_x= srv.response.new_pos.linear.x;
				// aruco_y= srv.response.new_pos.linear.y;
				// aruco_z= srv.response.new_pos.linear.z;
				// aruco_a_x= srv.response.new_pos.angular.x;
				// aruco_a_y= srv.response.new_pos.angular.y;
				// aruco_a_z= srv.response.new_pos.angular.z;
				// --------------------------------
				aruco_angle= srv.response.angle;
				detected_aruco_area= srv.response.aruco_area;
				aruco_detected= true;
				cout << "[MAIN] x: " << aruco_x << endl;
				cout << "[MAIN] y: " << aruco_y << endl;
				cout << "[MAIN] z: " << aruco_z << endl;
				cout << "[MAIN] ax: " << aruco_a_x << endl;
				cout << "[MAIN] ay: " << aruco_a_y << endl;
				cout << "[MAIN] az: " << aruco_a_z << endl;
			}
			else
			{
				aruco_detected= false;
			}
		}
		else
		{
			ROS_ERROR("[MAIN] Failed to call aruco detector service");
			aruco_detected= false;
		}
	}
};

class Environment
{
	public:
	// Atributes
	Bin bins;
	Bottle bottle;

	// Constructor
	Environment()
	{
	}

	// Destructor
	~Environment()
	{
	}

	void reset_parameters(void)
	{
		bins.aruco_angle= 0;
		bins.aruco_detected= false;
		bins.detected_aruco_area= 0;
		bottle.angle= 0;
		bottle.bottle_detected = false;
		bottle.detected_blob_size= 0;
		bottle.logo_detected= false;
		bottle.recyclable= false;
		bottle.rotate_times= 0;
	}


};

class Nao_control
{
	ros::NodeHandle nh_;

	public:
		/* SPEECH */
		vector<string> words;
		string voc_id;
		string speech_id;
		naoqi_bridge_msgs::SpeechWithFeedbackActionGoal fbag;
		string say;

		// bool pfand;

		// Environment
		Environment lab2223;

		// TIMER
		// uint8_t TIMER;
		// ros::Timer timer;

		// Buttons
		bool BUTTON_PRESSED;

		// client for look around node
		ros::ServiceClient nao_turn_head_client;

		// client for turn around node
		ros::ServiceClient nao_turn_body_client;

<<<<<<< HEAD
		ros::ServiceClient nao_walk_client;	
		ros::ServiceClient nao_walk_with_arms_client;
		ros::ServiceClient nao_walk_backwards_client;
=======
		ros::ServiceClient nao_walk_client;
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
		ros::ServiceClient nao_stand_client;
		ros::ServiceClient nao_rotate_bottle_client;
		ros::ServiceClient nao_open_hand_client;
		ros::ServiceClient nao_grasp_bottle_client;
		ros::ServiceClient nao_hand_down_client;

		ros::ServiceClient aruco_navigation_client;

		// subscriber to head tactile states
		ros::Subscriber tactile_sub;

		/* SPEECH */
		//publisher for nao speech
		ros::Publisher speech_pub;
		//publisher for nao vocabulary parameters
		ros::Publisher voc_params_pub;
		//client for starting speech recognition
		ros::ServiceClient recog_start_srv;
		//client for stoping speech recognition
		ros::ServiceClient recog_stop_srv;
		// subscriber to speech recognition
		ros::Subscriber recog_sub;
		/*-----------------------------*/

		//publisher for nao leds
		ros::Publisher leds_pub, leds_canc;

		// Thread Nao Control
		boost::thread *spin_thread;

		// Colors
		std_msgs::ColorRGBA RED;
		std_msgs::ColorRGBA GREEN;
		std_msgs::ColorRGBA BLUE;

		// Constructor
		Nao_control()
		{
			// subscribe to topic tactile_touch and specify that all data will be processed by function nao_tactile_callback
			tactile_sub= nh_.subscribe("/tactile_touch",1, &Nao_control::nao_tactile_callback, this);

			// Create a timer for having control of the blinking time
			// timer= nh_.createTimer(ros::Duration(1.0), &Nao_control::timerCallback, this);

			// publisher for look around node
			nao_turn_head_client = nh_.serviceClient <nao_control::LookAround>("lookaround");

			// publisher for look around node
			nao_turn_body_client = nh_.serviceClient <nao_control::TurnAround>("turnaround");

			nao_walk_client = nh_.serviceClient <nao_control::NaoWalk>("naowalk");
			nao_walk_with_arms_client = nh_.serviceClient <nao_control::NaoWalkWithArms>("naowalkwitharms");
			nao_walk_backwards_client = nh_.serviceClient <std_srvs::Empty>("naowalkbackwards");

			nao_stand_client = nh_.serviceClient<std_srvs::Empty>("StandUp");
			nao_rotate_bottle_client = nh_.serviceClient<std_srvs::Empty>("MoveArmsRotate");
			nao_grasp_bottle_client = nh_.serviceClient<std_srvs::Empty>("MoveArmsGrasp");
			nao_hand_down_client = nh_.serviceClient<std_srvs::Empty>("MoveArmsDown");
			nao_open_hand_client = nh_.serviceClient<std_srvs::Empty>("MoveArmsOpen");

			aruco_navigation_client = nh_.serviceClient <nao_control::ArucoNavigation>("aruco_navigation_service"); // WALK TO ARUCO

			/* SPEECH */
			// Publisher speech
			speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
			// Publisher vocabulary
			voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);
			// Client to start recognition
			recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");
			// Client to stop recognition
			recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");
			// suscribe to the topic word_recognized to get the speech that it was sayed
			recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCB, this);
			/*-------------------------------------------------*/

			// Publisher turn on blinking leds
			leds_pub= nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

			// Publisher turn off blinking leds
			leds_canc= nh_.advertise<actionlib_msgs::GoalID>("/blink/cancel", 1);


			// Control states
			STATE= IDLE;

			// Timer
			// TIMER= 0;
			// timer.stop(); // Must start in OFF

			// Colors
			RED.r= 1.0;
			RED.g= 0.0;
			RED.b= 0.0;
			RED.a= 1.0;

			// Colors
			GREEN.r= 0.0;
			GREEN.g= 1.0;
			GREEN.b= 0.0;
			GREEN.a= 1.0;

			// Colors
			BLUE.r= 0.0;
			BLUE.g= 0.0;
			BLUE.b= 1.0;
			BLUE.a= 1.0;

			/* SPEECH */
			// Vocabulary
			speech_id = "Speech";
			/*------------------------------------*/

			stop_thread = false;
			spin_thread = new boost::thread(&spinThread);
		}

		// Destructor
		~Nao_control()
		{
			free(&lab2223);

			stop_thread=true;
			sleep(1);
			spin_thread->join();
		}

		// Methods
		/* SPEECH */
		void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
		{
			ROS_INFO("[MAIN] start recognition");
			fbag.goal_id.id = speech_id;
			say += msg->words[0];
			say += " ";

			fbag.goal.say = say;
			std::cout << "[MAIN] " << fbag.goal.say << endl;
		}
		void pub_voc()
		{
			naoqi_bridge_msgs::SetSpeechVocabularyActionGoal msg;
			msg.goal_id.id = voc_id;
			msg.goal.words =  words;
			voc_params_pub.publish(msg);
			ROS_INFO("[MAIN] publish voc");
		}
		/*-------------------------------------------------*/

<<<<<<< HEAD
		// void timerCallback(const ros::TimerEvent&)
		// {
		// 	ROS_INFO("[MAIN] %d seconds", TIMER);
		// 	TIMER++;
		// }
=======
		void nao_blinking_leds(string name, std_msgs::ColorRGBA color)
		{
			naoqi_bridge_msgs::BlinkActionGoal msg;

			msg.goal_id.id = name;
			msg.goal.colors.push_back(color);
			msg.goal.blink_duration.sec = 0;
			msg.goal.blink_duration.nsec = 50000000;
			msg.goal.blink_rate_mean = 0.10;
			msg.goal.blink_rate_sd = 0.0001;
			msg.goal.bg_color = BLUE;

			leds_pub.publish(msg);
		}

		void cancelBlinkingLEDs(string name)
		{
			actionlib_msgs::GoalID msg;
			msg.id = name;
			leds_canc.publish(msg);
		}

		void timerCallback(const ros::TimerEvent&)
		{
			ROS_INFO("[MAIN] %d seconds", TIMER);
			TIMER++;
		}
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6

		void nao_head_pitch(float theta)
		{
			/* THIS TRIGGERS THE LOOK AROUND NODE */
			nao_control::LookAround srv;
			srv.request.name= "HeadPitch";
			srv.request.theta= theta;

			if (nao_turn_head_client.call(srv))
			{
				ROS_INFO("[HEAD] succesfully to call head position service");
			}
			else
			{
				ROS_ERROR("[HEAD] Failed to call head position service");
			}
		}

		bool nao_body_rotation(float angle)
		{
			/* THIS TRIGGERS THE TURN AROUND NODE */
			nao_control::TurnAround srv;
			srv.request.mode = angle;

			if (nao_turn_body_client.call(srv))
			{
				ROS_INFO("[MAIN] succesfully to call turn around service");
			}
			else
			{
				ROS_ERROR("[MAIN] Failed to call turn around service");
				return false;
			}

			return true;
		}

		void nao_walk_to_aruco(void)
		{
			/* THIS TRIGGERS THE BOTTLE DETECTOR NODE */
			nao_control::ArucoNavigation srv;
			// srv.request.new_pos.linear.x = lab2223.bins.aruco_x;
			// srv.request.new_pos.linear.y = lab2223.bins.aruco_y;
			// srv.request.new_pos.linear.z = lab2223.bins.aruco_z;
			// srv.request.new_pos.angular.x = lab2223.bins.aruco_a_x;
			// srv.request.new_pos.angular.y = lab2223.bins.aruco_a_y;
			// srv.request.new_pos.angular.z = lab2223.bins.aruco_a_z;

			if (aruco_navigation_client.call(srv))
			{
				ROS_INFO("[ARUCO] succesfully to call walk to aruco service");
<<<<<<< HEAD
=======
				lab2223.bins.navigation_stage= srv.response.new_stage;
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
			}
			else
			{
				ROS_ERROR("Failed to call service");
			}
		}

		void nao_tactile_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState)
		{
			// Start recycling process
			if(tactileState->button == 1 && tactileState->state == 1)
			{
				ROS_INFO("[MAIN] Front button pressed, start PFAND recycling");
				BUTTON_PRESSED = true;
			}
			else if(tactileState->button == 3 && tactileState->state == 1) // Stop recycling process
			{
				ROS_INFO("[MAIN] Rear button pressed, stop PFAND recycling");
				BUTTON_PRESSED = false;
			}
		}

		 void nao_walk_to(float dist,float theta)
    	{
			nao_control::NaoWalk srv;

			srv.request.dist = dist;
			srv.request.theta = theta;


			if (nao_walk_client.call(srv))
			{
				ROS_INFO("[MAIN] succesfully to call walk service");

			}
			else
			{
				ROS_ERROR("[MAIN] Failed to call walk service");
			}

    	}

		 void nao_walk_with_arms(float x,float y,float theta)
    	{
			nao_control::NaoWalkWithArms srv;

			srv.request.x= x;
			srv.request.y= y;
			srv.request.theta = theta;
		

			if (nao_walk_with_arms_client.call(srv))
			{
				ROS_INFO("[MAIN] succesfully to call walk with arms service");

			}
			else
			{
				ROS_ERROR("[MAIN] Failed to call walk with arms service");
			}
        
    	}

		void nao_walk_backwards()
    	{
			std_srvs::Empty emp;

			if (nao_walk_backwards_client.call(emp))
			{
				ROS_INFO("[MAIN] succesfully to call walk backwards service");

			}
			else
			{
				ROS_ERROR("[MAIN] Failed to call walk backwards service");
			}
        
    	}

		void nao_stand_up()
		{
			std_srvs::Empty emp;

			if (nao_stand_client.call(emp))
			{
				ROS_INFO("[MAIN] succesfully to call stand up service");
			}
			else
			{
				ROS_ERROR("Failed to call stand up service");
			}

		}

		void nao_grasp_hand()
		{
			std_srvs::Empty emp;

			if (nao_grasp_bottle_client.call(emp))
			{
				ROS_INFO("[MAIN] succesfully to call grasp service");
			}
			else
			{
				ROS_ERROR("Failed to call grasp service");
			}
		}

		void nao_hand_down()
		{
			std_srvs::Empty emp;
			
			if (nao_hand_down_client.call(emp))
			{
				ROS_INFO("[MAIN] succesfully to call hand down service");
			}
			else
			{
				ROS_ERROR("Failed to call hand down service");
			}
		}

		void nao_rotate_bottle()
		{
			std_srvs::Empty emp;

			if (nao_rotate_bottle_client.call(emp))
			{
				ROS_INFO("[MAIN] succesfully to call nao_rotate_bottle service");
			}
			else
			{
				ROS_ERROR("Failed to call nao_rotate_bottle service");
			}
		}

		void nao_open_hand()
		{
			std_srvs::Empty emp;

			if (nao_open_hand_client.call(emp))
			{
				ROS_INFO("[MAIN] succesfully to call open service");
			}
			else
			{
				ROS_ERROR("Failed to call open service");
			}
		}

		void main_loop()
		{
			// Main Control Loop Rate
			ros::Rate rate_sleep(1);

			while(nh_.ok())
			{

				if(!BUTTON_PRESSED)
					STATE= IDLE;


				ROS_INFO("[MAIN] STATE: %d", STATE);

				switch (STATE)
				{
					case IDLE:
						/* Wait to detect the touchable head button */
<<<<<<< HEAD
						if(BUTTON_PRESSED)		
						{
							fbag.goal_id.id = speech_id;
							fbag.goal.say = "Starting the bottle recycle process";
							speech_pub.publish(fbag);
							STATE= BOTTLE_DETECTION;	
						}					
							//STATE=CLASSIFY_BOTTLE;		
							// STATE=BOTTLE_NAVIGATION;
							// lab2223.bottle.recyclable= true;	
							// STATE= BIN_DETECTION;
=======
						if(BUTTON_PRESSED)

							// STATE= BOTTLE_DETECTION;
							//STATE=CLASSIFY_BOTTLE;
							// STATE=BOTTLE_NAVIGATION;
							STATE= BIN_DETECTION;
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
						break;

					case BOTTLE_DETECTION:
						/* Wait to detect a bottle */
						if(!lab2223.bottle.bottle_detected) // IF BOTTLE WAS NOT DETECTED
						{
							if(lab2223.bottle.detected_blob_size > 140)
								nao_head_pitch(0.1);

							lab2223.bottle.bottle_find();

							if (lab2223.bottle.bottle_detected) // IF BOTTLE WAS DETECTED
<<<<<<< HEAD
							{						
								ROS_INFO("[MAIN] BOTTLE DETECTED");								
								ROS_INFO("[MAIN] BOTTLE BLOB SIZE: %f", lab2223.bottle.detected_blob_size);
								ROS_INFO("[MAIN] BOTTLE ANGLE: %f", lab2223.bottle.angle);
=======
							{
								ROS_INFO("[MAIN] BOTTLE 1 DETECTED");
								ROS_INFO("[MAIN] BOTTLE 1 BLOB SIZE: %f", lab2223.bottle.detected_blob_size);
								ROS_INFO("[MAIN] BOTTLE 1 ANGLE: %f", lab2223.bottle.angle);
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
								if(lab2223.bottle.angle<-0.05 || lab2223.bottle.angle>0.05)
									nao_body_rotation(lab2223.bottle.angle);
							}
							else // IF BOTTLE
							{
								ROS_INFO("[MAIN] BOTTLE 1 NOT DETECTED");
								nao_body_rotation(FLIP_90_LEFT/2);
							}
						}
						else
						{
							STATE= BOTTLE_NAVIGATION;
							/* delete this below */
							// cout << "[MAIN] The detected bottle blob size is: " << lab2223.bottle.detected_blob_size << " px and the real is: " << lab2223.bottle.real_blob_size  << " px" << endl;
							// lab2223.bottle.bottle_detected= false;
						}

						break;

					case BOTTLE_NAVIGATION:

						// OPTION 1: COMPARING THE DETECTED BOTTLE SIZE WITH THE REAL
						if(lab2223.bottle.detected_blob_size < lab2223.bottle.real_blob_size)
						{
							cout << "[MAIN] The detected bottle blob size is: " << lab2223.bottle.detected_blob_size << " px and the real is: " << lab2223.bottle.real_blob_size  << " px" << endl;
							nao_walk_to((lab2223.bottle.real_blob_size-lab2223.bottle.detected_blob_size)/1500,0);
							STATE = BOTTLE_DETECTION;
							lab2223.bottle.bottle_detected= false;
						}
						else
						{
							// STATE= CLASSIFY_BOTTLE;
							ROS_INFO("[MAIN] WE REACHED THE BOTTLE");
							STATE = CLASSIFY_BOTTLE;
							nao_open_hand();
						}

						break;

					case CLASSIFY_BOTTLE:
						/* Rotating, sorting, notifying and grasping */
						ROS_INFO("[MAIN] Logo detected: %d" , lab2223.bottle.logo_detected);

						if(lab2223.bottle.logo_detected) //If there is a bottle and it has a recyclable label
						{
							nao_rotate_bottle();
							lab2223.bottle.recyclable= true;
<<<<<<< HEAD
							fbag.goal_id.id = speech_id;
							fbag.goal.say = "This is a recyclable bottle";
							speech_pub.publish(fbag);
							nao_grasp_hand();				
							// nao_blinking_leds("notifying bottle" , GREEN);
							ros::Duration(1).sleep();
							nao_walk_with_arms(0 , 0, -1.57); //FLIP_90_LEFT
							nao_hand_down();
							STATE= BIN_DETECTION;
						}
						else if(!lab2223.bottle.logo_detected && lab2223.bottle.rotate_times<6) // If there is a bottle and NAO doesn’t find recyclable label
						{														
							nao_rotate_bottle();
							lab2223.bottle.rotate_times++;
						}						
						else if(!lab2223.bottle.logo_detected && lab2223.bottle.rotate_times>=6)
=======
							nao_grasp_hand();
							nao_blinking_leds("notifying bottle" , GREEN);
							// speechGeneration();
							STATE= BIN_NAVIGATION;
						}
						else if(!lab2223.bottle.logo_detected && rotate_times<20) // If there is a bottle and NAO doesn’t find recyclable label
						{
							nao_rotate_bottle();
							rotate_times++;
						}
						else if(!lab2223.bottle.logo_detected && rotate_times>=20)
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
						{
							lab2223.bottle.recyclable= false;
							fbag.goal_id.id = speech_id;
							fbag.goal.say = "This is not a recyclable bottle";
							speech_pub.publish(fbag);
							nao_grasp_hand();
							// nao_blinking_leds("notifying bottle" , RED);
							ros::Duration(1).sleep();
							nao_walk_with_arms(0, 0, -1.57); //FLIP_90_LEFT
							nao_hand_down();
							STATE= BIN_DETECTION;
							
						}

						break;

					case BIN_DETECTION:

						/* Wait to detect aruco */
						if(!lab2223.bins.aruco_detected) // IF ARUCO WAS NOT DETECTED
<<<<<<< HEAD
						{					
							if(lab2223.bottle.recyclable)								
								lab2223.bins.aruco_find(RECYCLE_ARUCO_ID);
							else
								lab2223.bins.aruco_find(NON_RECYCLE_ARUCO_ID);

							if (lab2223.bins.aruco_detected) // IF ARUCO WAS DETECTED
							{						
								ROS_INFO("[MAIN] ARUCO DETECTED");
								ROS_INFO("[MAIN] ARUCO AREA: %f", lab2223.bins.detected_aruco_area);
								ROS_INFO("[MAIN] ARUCO ANGLE: %f", lab2223.bins.aruco_angle);
								if(lab2223.bins.aruco_angle<-0.05 || lab2223.bins.aruco_angle>0.05)
									nao_body_rotation(lab2223.bins.aruco_angle);												
=======
						{
							lab2223.bins.aruco_find();

							if (lab2223.bins.aruco_detected) // IF BOTTLE WAS DETECTED
							{
								ROS_INFO("[MAIN] ARUCO DETECTED");
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
							}
							else // IF ARUCO
							{
								ROS_INFO("[MAIN] ARUCO NOT DETECTED");
<<<<<<< HEAD
								nao_walk_with_arms(0,0, FLIP_90_LEFT/2);
								// nao_hand_down();																					
							}												
=======
								nao_body_rotation(FLIP_90_LEFT/2);
							}
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6
						}
						else
						{
							STATE= BIN_NAVIGATION;
							/* delete this below */
							// cout << "[MAIN] The detected aruco area is: " << lab2223.bins.detected_aruco_area << " px and the real is: " << lab2223.bins.real_aruco_area  << " px" << endl;							
							// lab2223.bins.aruco_detected= false;
						}
<<<<<<< HEAD
						
						break;		
							

					case BIN_NAVIGATION:
						
						// OPTION 1: COMPARING THE DETECTED BOTTLE SIZE WITH THE REAL
						if(lab2223.bins.detected_aruco_area < lab2223.bins.real_aruco_area)
						{
							cout << "[MAIN] The detected aruco area is: " << lab2223.bins.detected_aruco_area << " px and the real is: " << lab2223.bins.real_aruco_area  << " px" << endl;
							nao_walk_with_arms((lab2223.bins.real_aruco_area-lab2223.bins.detected_aruco_area)/10000,0,0);
							STATE = BIN_DETECTION;
							lab2223.bins.aruco_detected= false;
						}
						else
						{
							// STATE= CLASSIFY_BOTTLE;	
							ROS_INFO("[MAIN] WE REACHED THE ARUCO");
							STATE = DEPOSIT_BOTTLE;				
						}

						break;				

					case DEPOSIT_BOTTLE:
						/* Aruco 1 (recycle bin) & Aruco 2 (trash bin) */
						// nao_grasp_hand();
						// nao_hand_down();
						nao_stand_up();
						// ros::Duration(2).sleep();
						nao_walk_with_arms(0,0, FLIP_90_LEFT*2);

    					ros::Duration(2).sleep(); // We rest a little bit
						lab2223.reset_parameters();
						STATE= BOTTLE_DETECTION;					

=======

						break;

					case BIN_NAVIGATION:

						// // OPTION 1: COMPARING THE DETECTED BOTTLE SIZE WITH THE REAL
						// if(lab2223.bins.navigation_stage < 1)
						// {
						// 	ROS_INFO_STREAM("[MAIN] Aruco navigation stage: " << lab2223.bins.navigation_stage);
						// 	nao_walk_to_aruco();
						// 	STATE = BIN_DETECTION;
						// 	lab2223.bins.aruco_detected= false;
						// 	// nao_head_pitch(0.2);
						// }
						// else
						// {
							nao_walk_to_aruco();
							ROS_INFO_STREAM("[MAIN] Aruco navigation stage: " << lab2223.bins.navigation_stage);
							ROS_INFO("[MAIN] WE REACHED THE ARUCO");
							STATE = DEPOSIT_BOTTLE;
						// }

						break;

					case DEPOSIT_BOTTLE:
						/* Aruco 1 (recycle bin) & Aruco 2 (trash bin) */
						// lab2223.bottle.bottle_deposit();
						// STATE= BOTTLE_NAVIGATION;
>>>>>>> 41b2e6078a3a1ce5d4d7b91f43849fc55d4be0a6

						break;

					default:
						break;
				}

				rate_sleep.sleep();
			}
		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "project");

	Nao_control ic;
	ic.nao_stand_up();
	// ic.nao_open_hand();
	// ic.nao_grasp_hand();
	// ros::Duration(2).sleep();
	// ic.nao_walk_backwards();
	// ic.nao_hand_down();
	ic.main_loop();

	return 0;
}

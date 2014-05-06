 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include "std_msgs/Int32.h"
 #include "opencv2/objdetect/objdetect.hpp"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include "sensor_msgs/PointCloud2.h"
 #include <pcl/point_types.h>
 #include <pcl_ros/transforms.h>
 #include <pcl/conversions.h>
 #include <pcl/PCLPointCloud2.h>
 #include <cstdio>
 #include <tf/transform_broadcaster.h>
 #include <pcl_conversions/pcl_conversions.h>

 #include <iostream>
 #include <stdio.h>

 using namespace std;
 using namespace cv;

 /** Function Headers */
 void detectAndDisplay( Mat frame, int argc, char** argv  );

 /** Global variables */
 String face_cascade_name = "/src/haarcascade_frontalface_alt.xml";
 String eyes_cascade_name = "/src/haarcascade_eye_tree_eyeglasses.xml";
 CascadeClassifier face_cascade;
 CascadeClassifier eyes_cascade;
 string window_name = "Capture - Face detection";
 RNG rng(12345);
 int counter=0;

 /** @function main */
 int main( int argc, char** argv )
 {
	 CvCapture* capture;
	 Mat frame;

   int i=0;
   for(i=0; i<argc; i++){
      cout << argv[i] << "/n";
   }
	 //-- 1. Load the cascades
	 if( !face_cascade.load( argv[1] ) ){ printf("--(!)Error loading\n"); return -1; };
	 if( !eyes_cascade.load( argv[2] ) ){ printf("--(!)Error loading\n"); return -1; };

	 //-- 2. Read the video stream
	 capture = cvCaptureFromCAM(-1 );
	 if( capture )
	 {
		 while( true )
		 {
	 frame = cvQueryFrame( capture );

	 //-- 3. Apply the classifier to the frame
			 if( !frame.empty() )
			 { detectAndDisplay( frame, argc, argv ); }
			 else
			 { printf(" --(!) No captured frame -- Break!"); break; }

			 int c = waitKey(10);
			 if( (char)c == 'c' ) { break; }
			}
	 }
	 return 0;
 }

/** @function detectAndDisplay */
void detectAndDisplay( Mat frame, int argc, char** argv  )
{	
	
	
	std::vector<Rect> faces;
	Mat frame_gray;
	//ROS initiated
	ros::init(argc, argv, "detect_faces");
	ros::NodeHandle n;
	ros::Publisher face_pub = n.advertise<std_msgs::Int32>("num_faces",1000);
	std_msgs::Int32 msg;

	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );

	//-- Detect faces
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
	if(counter%5 == 0)
	{
		//ROS initiated
		ros::init(argc, argv, "detect_faces");
		ros::NodeHandle n;
		ros::Publisher face_pub = n.advertise<std_msgs::Int32>("num_faces",1000);
		std_msgs::Int32 msg;

		//store data to msg and publish it
		msg.data = faces.size();	
		face_pub.publish(msg);
	}
	
	



	for( size_t i = 0; i < faces.size(); i++ )
	{

		Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

		Mat faceROI = frame_gray( faces[i] );
		std::vector<Rect> eyes;

		//-- In each face, detect eyes
		eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

		for( size_t j = 0; j < eyes.size(); j++ )
		 {
			 Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
			 int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
			 circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
		 }
	}
	//-- Show what you got
	imshow( window_name, frame );
 }

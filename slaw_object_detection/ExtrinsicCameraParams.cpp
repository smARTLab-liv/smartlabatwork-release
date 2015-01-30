#ifndef EXTRINC_C
#define EXTRINC_C


#include <stdio.h>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>




using namespace cv;
using namespace std;



// Globals ----------------------------------------------------------------------------------------

int boardHeight = 6;
int boardWidth = 8;
Size cbSize = Size(boardHeight,boardWidth);



//string filename = "camera_info/rgb_1206090086.yaml";
string filename = "out_camera_data.xml";

bool doneYet = false;

//default image size
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//function prototypes
//void generate_boardPoints();
Mat intrinsics, distortion;

Mat webcamImage, gray, one;

//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
vector<Point2d> imagePoints, imageFramePoints, imageOrigin;
vector<Point3d> boardPoints, framePoints;
void processImage(const sensor_msgs::ImageConstPtr &ms);
// Main -------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Extrinsic");
    
    ros::NodeHandle nh("~");
    //set up a FileStorage object to read camera params from file
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    // read camera matrix and distortion coefficients from file
  //  fs["Camera_Matrix"] >> intrinsics;
    if(fs.isOpened()){
        cout << "Config is open " << endl;
    }else
    {
        cout << "not open " << endl;
    }

    fs["Distortion_Coefficients"] >> distortion;
    // close the input file
    fs.release();




    //set up matrices for storage


    //generate vectors for the points on the chessboard
    for (int i=0; i<boardWidth; i++)
    {
        for (int j=0; j<boardHeight; j++)
        {
            boardPoints.push_back( Point3d( double(i) * 2.5, double(j) * 2.5, 0.0) );
        }
    }
    //generate points in the reference frame
    framePoints.push_back( Point3d( 0.0, 0.0, 0.0 ) );
    framePoints.push_back( Point3d( 5.0, 0.0, 0.0 ) );
    framePoints.push_back( Point3d( 0.0, 5.0, 0.0 ) );
    framePoints.push_back( Point3d( 0.0, 0.0, 5.0 ) );


    //set up VideoCapture object to acquire the webcam feed from location 0 (default webcam location)
    //VideoCapture capture;
    //capture.open(0);
    //set the capture frame size
    //capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    //capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    

    int widthTest;
    if (nh.getParam("image_width", widthTest))
    {
        cout << " width 1: " << widthTest << endl;
    }
    else
    {
        cout << "not there" << endl;
    }

//    Mat mat;
  //  mat << nh.getParam("camera_matrix" ,);

    //Mat(

    XmlRpc::XmlRpcValue camera_matrix;
    int rows  , cols  ;

    if(nh.getParam("camera_matrix/data", camera_matrix) && nh.getParam("camera_matrix/cols", cols) && nh.getParam("camera_matrix/rows", rows))
    {
        intrinsics = Mat(rows, cols, CV_64F);
        for (int y = 0; y < rows; y++)
        {
            for( int x = 0; x < cols; x++)
            {


            //    cout << "value: " << (camera_matrix[(y*cols) + x]) << endl;
                camera_matrix.size();
                XmlRpc::XmlRpcValue rpcvalue = camera_matrix[(y*cols) + x];

                if(rpcvalue.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    intrinsics.at<double>(y,x) = rpcvalue;
                }
                if(rpcvalue.getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    intrinsics.at<double>(y,x) = (int)rpcvalue;
                }

            }
        }
    }


    if (camera_matrix.hasMember("rows"))
    {
       // intrinsics = Mat(static_cast<int>(camera_matrix.asStruct->), static_cast<int>(camera_matrix[1]),static_cast<);
        cout << "test" << endl;
    }
/*    for (int32_t i = 0; i < camera_matrix.size(); ++i)
     {
       ROS_ASSERT(camera_matrix[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
       //sum += static_cast<double>(my_list[i]);
     }
    */
    
    image_transport::ImageTransport img_transport_(nh);
    image_transport::Subscriber image_sub_;

    image_sub_ = img_transport_.subscribe("/camera/rgb/image_rect_color", 1, processImage);

    ros::spin();


    return 0;
}


void processImage(const sensor_msgs::ImageConstPtr &ms)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(ms, ms->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat webcamImage = cv_ptr->image;


    Mat rvec = Mat(Size(3,1), CV_64F);
    Mat tvec = Mat(Size(3,1), CV_64F);
    //store image to matrix
    //capture.read(webcamImage);

    //make a gray copy of the webcam image
    cvtColor(webcamImage,gray,COLOR_BGR2GRAY);


    //detect chessboard corners
    bool found = findChessboardCorners(gray, cbSize, imagePoints, CALIB_CB_FAST_CHECK);
    //drawChessboardCorners(webcamImage, cbSize, Mat(imagePoints), found);

    //find camera orientation if the chessboard corners have been found
    if ( found )
    {
        //find the camera extrinsic parameters
        solvePnP( Mat(boardPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false );

        //project the reference frame onto the image
        projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints );


        //DRAWING
        //draw the reference frame on the image
        circle(webcamImage, imagePoints[0], 4 ,CV_RGB(255,0,0) );

        Point one, two, three;
        one.x=10; one.y=10;
        two.x = 60; two.y = 10;
        three.x = 10; three.y = 60;

        line(webcamImage, one, two, CV_RGB(255,0,0) );
        line(webcamImage, one, three, CV_RGB(0,255,0) );


        line(webcamImage, imageFramePoints[0], imageFramePoints[1], CV_RGB(255,0,0), 2 );
        line(webcamImage, imageFramePoints[0], imageFramePoints[2], CV_RGB(0,255,0), 2 );
        line(webcamImage, imageFramePoints[0], imageFramePoints[3], CV_RGB(0,0,255), 2 );



        //show the pose estimation data
        cout << fixed << setprecision(2) << "rvec = ["
             << rvec.at<double>(0,0) << ", "
             << rvec.at<double>(1,0) << ", "
             << rvec.at<double>(2,0) << "] \t" << "tvec = ["
             << tvec.at<double>(0,0) << ", "
             << tvec.at<double>(1,0) << ", "
             << tvec.at<double>(2,0) << "]" << endl;

    }

    //show the image on screen
    namedWindow("OpenCV Webcam", 0);
    imshow("OpenCV Webcam", webcamImage);


    //show the gray image
    //namedWindow("Gray Image", CV_WINDOW_AUTOSIZE);
    //imshow("Gray Image", gray);


    waitKey(10);

}


#endif

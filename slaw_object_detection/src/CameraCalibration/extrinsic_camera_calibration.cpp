#include "CameraCalibration/extrinsic_camera_calibration.h"
namespace slaw_object_detection{

ExtrinsicCameraCalibration::ExtrinsicCameraCalibration(int boardRow,
                                                       int boardCols,
                                                       double fieldSize) :
    m_boardSize(boardRow, boardCols), m_fieldSize(fieldSize), m_nh("~"),m_image_transport(m_nh),
    m_isConfigured(false)
{
    m_boardPoints3D = CameraCalibrationUtils::generate3DChessboard(boardCols, boardRow, m_fieldSize);
    m_distortion = CameraCalibrationUtils::getMat64F_FromParam(m_nh, CameraCalibrationUtils::DISTORTION_STR);
    m_intrinsics = CameraCalibrationUtils::getMat64F_FromParam(m_nh, CameraCalibrationUtils::CAMERA_MATRIX_STR);


    string img_topic;
    if(!m_nh.getParam("image_topic", img_topic))
    {
         ROS_ERROR("PARAMETER : image_topic is not specified");
         exit(1);
    }

    if(!m_nh.getParam("config_file", m_yamlFileName))
    {
         ROS_ERROR("PARAMETER : config_file is not specified");
         exit(1);
    }

    m_callback = boost::bind(&ExtrinsicCameraCalibration::updateConfig, this,_1,_2);
    m_dyn_conf_server.setCallback(m_callback);

    initializeCameraParameter();



    m_imgIndex.push_back(0);
    m_imgIndex.push_back(m_boardSize.width - 1);
    m_imgIndex.push_back((m_boardSize.height - 1) * (m_boardSize.width));
    m_imgIndex.push_back((m_boardSize.height - 1) * (m_boardSize.width) + m_boardSize.width - 1);

    m_image_sub = m_image_transport.subscribe(img_topic, 1, &ExtrinsicCameraCalibration::processImage, this);



    ros::Rate r(40);
    while(ros::ok())
    {
        //processKeyInput();
        ros::spinOnce();
        r.sleep();
    }

}

void ExtrinsicCameraCalibration::updateConfig(CamCalibrExtrConfig &config, uint32_t level)
{
    if(!m_isConfigured){
        readConfigFile(config);
        m_isConfigured = true;
    }
    m_dynConfig = config;
    updateCameraParameters();
    if(config.save){
        saveYamlFile();
    }
    config.save = false;
    m_dynConfig.save = false;
}



void ExtrinsicCameraCalibration::initializeCameraParameter()
{
    //initialize dynamic config
    cout << "initializeCameraParameter!" <<endl;
    //FileStorage fs(m_yamlFileName, FileStorage::READ);

    readConfigFile(m_dynConfig);


    //initCameraParameter
    updateCameraParameters();

    //setup intrinsic camera matrix
    m_cameraInfo.focalLength = Point2d(m_intrinsics.at<double>(0,0), m_intrinsics.at<double>(1,1));
    m_cameraInfo.opticalCenter = Point2d(m_intrinsics.at<double>(0,2), m_intrinsics.at<double>(1,2));



    cout << "opticalCenter.x " << m_cameraInfo.opticalCenter.x << endl;
    cout << "opticalCenter.y " << m_cameraInfo.opticalCenter.y << endl;
    cout << "focalLength.x " << m_cameraInfo.focalLength.x << endl;
    cout << "focalLength.y " << m_cameraInfo.focalLength.y << endl;


}

void ExtrinsicCameraCalibration::readConfigFile(CamCalibrExtrConfig &config)
{
    //check for file
    ifstream check(m_yamlFileName.c_str());
    if(check.good())check.close();
    else return;

    FileStorage fs(m_yamlFileName, FileStorage::READ);

    fs["cameraHeight"] >> config.cameraHeight;
    fs["pitch"] >> config.pitch;
    fs["yaw"] >> m_dynConfig.yaw;
    fs["imageWidth"] >> config.imageWidth;
    fs["imageHeight"] >> config.imageHeight;

    fs["ipmWidth"] >> config.ipmWidth;
    fs["ipmHeight"] >> config.ipmHeight;

    fs["ipmLeft"] >> config.ipmLeft;
    fs["ipmRight"] >> config.ipmRight;
    fs["ipmTop"] >> config.ipmTop;
    fs["ipmButtom"] >> config.ipmBottom;
    fs["ipmInterpolation"] >> m_dynConfig.ipmInterpolation;
    fs["alpha"] >> m_dynConfig.alpha;
    fs["beta"] >> m_dynConfig.beta;
    fs["gamma"] >> m_dynConfig.gamma;
    fs["dx"] >> m_dynConfig.dx;
    fs["dy"] >> m_dynConfig.dy;
    fs["dz"] >> m_dynConfig.dz;
    fs["f"] >> m_dynConfig.f;
}

void ExtrinsicCameraCalibration::saveYamlFile()
{
    FileStorage fs(m_yamlFileName, FileStorage::WRITE);

    fs << "cameraHeight" << m_dynConfig.cameraHeight;
    fs << "pitch" << m_dynConfig.pitch;
    fs << "yaw" <<m_dynConfig.yaw;
    fs << "imageWidth" << m_dynConfig.imageWidth;
    fs << "imageHeight" << m_dynConfig.imageHeight;

    fs << "ipmWidth" << m_dynConfig.ipmWidth;
    fs << "ipmHeight" << m_dynConfig.ipmHeight;

    fs << "ipmLeft" << m_dynConfig.ipmLeft;
    fs << "ipmRight" << m_dynConfig.ipmRight;
    fs << "ipmTop" << m_dynConfig.ipmTop;
    fs << "ipmButtom" << m_dynConfig.ipmBottom;
    fs << "ipmInterpolation" << m_dynConfig.ipmInterpolation;

    fs << "alpha" << m_dynConfig.alpha;
    fs << "beta" << m_dynConfig.beta;
    fs << "gamma" << m_dynConfig.gamma;
    fs << "dx" << m_dynConfig.dx;
    fs << "dy" << m_dynConfig.dy;
    fs << "dz" << m_dynConfig.dz;
    fs << "f" << m_dynConfig.f;
}

void ExtrinsicCameraCalibration::updateCameraParameters()
{
    cout << "updateCameraParameters " << endl;
    m_ipmInfo.height = m_dynConfig.ipmHeight;
    cout << "m_ipmInfo.height " << m_ipmInfo.height <<endl;
    m_ipmInfo.width = m_dynConfig.ipmWidth;
    cout << "m_ipmInfo.width " << m_ipmInfo.width<<endl;
    m_ipmInfo.ipmLeft = m_dynConfig.ipmLeft;
    cout << "m_ipmInfo.ipmLeft " << m_ipmInfo.ipmLeft<<endl;
    m_ipmInfo.ipmRight = m_dynConfig.ipmRight;
    cout << "m_ipmInfo.ipmRight " << m_ipmInfo.ipmRight<<endl;
    m_ipmInfo.ipmTop = m_dynConfig.ipmTop;
    cout << "m_ipmInfo.ipmTop " << m_ipmInfo.ipmTop<<endl;
    m_ipmInfo.ipmBottom = m_dynConfig.ipmBottom;
    cout << "m_ipmInfo.ipmBottom " << m_ipmInfo.ipmBottom<<endl;
    m_ipmInfo.ipmInterpolation = m_dynConfig.ipmInterpolation;
    cout << "m_ipmInfo.ipmInterpolation " << m_ipmInfo.ipmInterpolation<<endl;


    m_cameraInfo.imageHeight = m_dynConfig.imageHeight;
    cout << "m_cameraInfo.cameraHeight " << m_cameraInfo.cameraHeight<<endl;
    m_cameraInfo.imageWidth = m_dynConfig.imageWidth;
    m_cameraInfo.cameraHeight = m_dynConfig.cameraHeight;
    m_cameraInfo.pitch = m_dynConfig.pitch;
    m_cameraInfo.yaw = m_dynConfig.yaw;

    m_alpha = m_dynConfig.alpha;
    m_beta = m_dynConfig.beta;
    m_gamma = m_dynConfig.gamma;
    m_dx = m_dynConfig.dx;
    m_dy = m_dynConfig.dy;
    m_dz = m_dynConfig.dz;
    m_f = m_dynConfig.f;
}

void ExtrinsicCameraCalibration::processImage(const sensor_msgs::ImageConstPtr &ms)
{
    if(!m_isConfigured) return;
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

    Mat image = cv_ptr->image;
    Mat image64f(image.rows, image.cols, IPL_DEPTH_32F);
    image.convertTo(image64f, IPL_DEPTH_32F);
    IplImage ipl = image;


    CvMat *raw_mat;
    CvMat *inImage;

    CameraCalibrationUtils::mcvIplImageToCvMat(&ipl, &raw_mat, &inImage);
    cvReleaseMat(&raw_mat);


    vector<Point2d> outP;
    CvMat *ipm = cvCreateMat(m_ipmInfo.height, m_ipmInfo.width, inImage->type);
   // cout << "mcvGetIPM start " << endl;

    //perspectiveTransform();

    //CameraCalibrationUtils::mcvGetIPM(inImage, ipm, &m_ipmInfo, &m_cameraInfo);
    //cout << "mcvGetIPM end" << endl;
    Mat rvec = Mat(Size(3,1), CV_64F);
    Mat tvec = Mat(Size(3,1), CV_64F);

    Mat gray;
    vector<Point2d> imagePoints;

    cvtColor(image, gray, COLOR_BGR2GRAY);

    bool found = findChessboardCorners(image, m_boardSize, imagePoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    Mat birdEye, gray32f;
    cvtColor(image64f, gray32f, COLOR_BGR2GRAY);
    gray32f.copyTo(birdEye);

    if(found)
    {
        cv::circle(image,imagePoints[m_imgIndex[0]],2,Scalar(255,0,0),2);
        cv::circle(image,imagePoints[m_imgIndex[1]],2,Scalar(0,255,0),2);
        cv::circle(image,imagePoints[m_imgIndex[2]],2,Scalar(0,0,255),2);
        cv::circle(image,imagePoints[m_imgIndex[3]],2,Scalar(255,255,0),2);
        //cornerSubPix(gray, imagePoints, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1) );
        calcChessPosition( &m_boardPoints3D, &imagePoints, rvec, tvec);
        drawAxisOnChessBoard(image, rvec, tvec, 4 * m_fieldSize, 0 , 0);
        drawTransRotText(image,rvec, tvec);
        getBirdEyeImage(gray32f, birdEye, m_boardPoints3D, imagePoints);
    }




   // cout << "mcvGetIPM end 2" << endl;
    namedWindow("OpenCV Webcam", 0);
    imshow("OpenCV Webcam", image);
    imshow("IMP", birdEye);
   // cout << "mcvGetIPM end 3" << endl;

    waitKey(10);

    //image->release();
    cvReleaseMat(&ipm);
    cvReleaseMat(&inImage);
    cvReleaseMat(&inImage);

}

void ExtrinsicCameraCalibration::rotateImage(const Mat &input, Mat &output, double alpha, double beta, double gamma, double dx, double dy, double dz, double f)
{

  alpha = (alpha - 90.)*CV_PI/180.;
  beta = (beta - 90.)*CV_PI/180.;
  gamma = (gamma - 90.)*CV_PI/180.;

  // get width and height for ease of use in matrices

  double w = (double)input.cols;
  double h = (double)input.rows;

  // Projection 2D -> 3D matrix

  Mat A1 = (Mat_<double>(4,3) <<
            1, 0, -w/2,
            0, 1, -h/2,
            0, 0, 0,
            0, 0, 1);
  // Rotation matrices around the X, Y, and Z axis

  Mat RX = (Mat_<double>(4, 4) <<
            1,          0,           0, 0,
            0, cos(alpha), -sin(alpha), 0,
            0, sin(alpha),  cos(alpha), 0,
            0,          0,           0, 1);

  Mat RY = (Mat_<double>(4, 4) <<
            cos(beta), 0, -sin(beta), 0,
            0, 1,          0, 0,
            sin(beta), 0,  cos(beta), 0,
            0, 0,          0, 1);

  Mat RZ = (Mat_<double>(4, 4) <<
            cos(gamma), -sin(gamma), 0, 0,
            sin(gamma),  cos(gamma), 0, 0,
            0,          0,           1, 0,
            0,          0,           0, 1);

  // Composed rotation matrix with (RX, RY, RZ)
  Mat R = RX * RY * RZ;

  // Translation matrix
  /*Mat T = (Mat_<double>(4, 4) <<
           1, 0, 0, dx,
           0, 1, 0, dy,
           0, 0, 1, dz,
           0, 0, 0, 1);*/
  Mat T = (Mat_<double>(2, 3) <<
           1, 0, dx,
           0, 1, dy);



  // 3D -> 2D matrix
  Mat A2 = (Mat_<double>(3,4) <<
            f, 0, w/2, 0,
            0, f, h/2, 0,
            0, 0,   1, 0);

  // Final transformation matrix
  //Mat trans = A2 * (T * (R * A1));
  //Mat trans = T * R;
  // Apply matrix transformation
  //warpPerspective(input, output, trans, input.size(), INTER_LANCZOS4);
  R = getRotationMatrix2D(Point2f(output.cols /2, output.rows/2) ,gamma, 1);
  warpAffine(input, output, T, Size(output.cols, output.rows));
  warpAffine(input, output, R, Size(output.cols, output.rows));
}



void ExtrinsicCameraCalibration::getBirdEyeImage(Mat &src, Mat &dst, vector<Point3d> &object, vector<Point2d> &image)
{
    Mat H(3,3, CV_32F);
    vector<Point2f> im;
    im.push_back(Point2f(image[m_imgIndex[0]].x, image[m_imgIndex[0]].y));
    im.push_back(Point2f(image[m_imgIndex[1]].x, image[m_imgIndex[1]].y));
    im.push_back(Point2f(image[m_imgIndex[2]].x, image[m_imgIndex[2]].y));
    im.push_back(Point2f(image[m_imgIndex[3]].x, image[m_imgIndex[3]].y));

    vector<Point2f> o;
    o.push_back(Point2f(0,0));
    o.push_back(Point2f(0, m_boardSize.height * m_fieldSize));
    o.push_back(Point2f(m_boardSize.width * m_fieldSize, 0));
    o.push_back(Point2f(m_boardSize.width * m_fieldSize, m_boardSize.height * m_fieldSize));

    for (int i=0; i < 4; i++){
        cout << i << " object( "<<o[i].x << ", "<<o[i].y<<")"<<endl;
        cout << i <<"im( "<<im[i].x << ", "<<im[i].y<<")"<<endl;
    }



    H = getPerspectiveTransform(Mat(o), Mat(im));
    Mat T = (Mat_<double>(3, 3) <<
             1, 0, -m_dx,
             0, 1, m_dy,
             0, 0, 1);

   // H.at<float>(2,2) = m_cameraInfo.cameraHeight;
    H = T * H;
    warpPerspective(src, dst, H, Size(dst.cols, dst.rows), INTER_LINEAR | WARP_INVERSE_MAP /*| WARP_FILL_OUTLIERS*/);
    //rotateImage(dst, dst, m_alpha, m_beta, m_gamma, m_dx, m_dy, m_dz, m_f);

}


void ExtrinsicCameraCalibration::drawTransRotText(Mat &mat, Mat &rvec, Mat &tvec)
{
    string rotMsg, transMsg;
    Mat rotM = Mat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rotM);
    double yaw = atan(rotM.at<double>(1,0) / rotM.at<double>(0,0));
    double pitch = atan(-rotM.at<double>(2,0) / sqrt(pow(rotM.at<double>(2,1),2) * pow(rotM.at<double>(2,2),2)));
    double roll = atan(rotM.at<double>(2,1) / rotM.at<double>(2,2));


   /* rotation << fixed << setprecision(2) << "rvec = ["
         << rvec.at<double>(0,0) << ", "
         << rvec.at<double>(1,0) << ", "
         << rvec.at<double>(2,0) << "]";*/
    stringstream rotation, translation;
    rotation << fixed << setprecision(2) << "yaw = "
             << yaw << " pitch "
             << pitch << " roll "
             << roll;
    translation << "tvec = ["
         << tvec.at<double>(0,0) << ", "
         << tvec.at<double>(1,0) << ", "
         << tvec.at<double>(2,0) << "]";
    rotMsg = rotation.str();
    transMsg = translation.str();
    // cout << msg;

    int baseLine = 0;
    Size rotTxtSize = getTextSize(rotMsg, 1, 1, 1, &baseLine);
    Point textOrigin(mat.cols - 2 * rotTxtSize.width - 10, mat.rows - 2 * baseLine - 10);
    Point textOriginRot(10, rotTxtSize.height * 2);
    Point textOriginTrans(10, textOriginRot.y + rotTxtSize.height * 2);

    putText( mat, rotMsg, textOriginRot, 1, 1, Scalar(0,255,0));
    putText( mat, transMsg, textOriginTrans, 1, 1, Scalar(0,255,0));
}



void ExtrinsicCameraCalibration::calcChessPosition(vector<Point3d> *boardPoints3D, vector<Point2d> *imagePoints, Mat &rvec, Mat &tvec)
{

    solvePnP( Mat(*boardPoints3D), Mat(*imagePoints), m_intrinsics, m_distortion, rvec, tvec, false );
}


void ExtrinsicCameraCalibration::drawAxisOnChessBoard(Mat &img, Mat &rvec, Mat &tvec, double axisLength, double offsetX, double offsetY)
{

        vector<Point3d> framePoints;
        framePoints.push_back(Point3d(offsetX, offsetY, 0));
        framePoints.push_back(Point3d(offsetX + axisLength, offsetY, 0));
        framePoints.push_back(Point3d(offsetX, offsetY + axisLength, 0));
        framePoints.push_back(Point3d(offsetX, offsetY, axisLength));

        vector<Point2d> imageFramePoints;


        projectPoints(framePoints, rvec, tvec, m_intrinsics, m_distortion, imageFramePoints );


        //DRAWING
        //draw the reference frame on the image
        //circle(img, imagePoints[0], 4 ,CV_RGB(255,0,0) );

        line(img, imageFramePoints[0], imageFramePoints[1], CV_RGB(255,0,0), 2 );
        line(img, imageFramePoints[0], imageFramePoints[2], CV_RGB(0,255,0), 2 );
        line(img, imageFramePoints[0], imageFramePoints[3], CV_RGB(0,0,255), 2 );
}

}// end namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Intrinsic Cameracalibration");
    ros::NodeHandle nh("~");

    int boardRows, boardCols;
    double cellSize;

    if(!nh.getParam("BoardCols", boardCols))
    {
         ROS_ERROR("PARAMETER : BoardCols is not specified");
         exit(1);
    }

    if(!nh.getParam("BoardRows", boardRows))
    {
         ROS_ERROR("PARAMETER : BoardRows is not specified");
         exit(1);
    }

    if(!nh.getParam("CellSize", cellSize))
    {
         ROS_ERROR("PARAMETER : CellSize is not specified");
         exit(1);
    }

    slaw_object_detection::ExtrinsicCameraCalibration(boardRows, boardCols, cellSize);



    return 0;
}

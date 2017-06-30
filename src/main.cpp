#include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <image_transport/camera_publisher.h>
// #include <camera_info_manager/camera_info_manager.h>
// #include <std_srvs/Empty.h>
// #include <string>
// #include <iostream>
// #include <sstream>
// #include <opencv2/core/persistence.hpp> // Need this for cv::FileStorage?


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
// #include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseArray.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
// #include <aruco_msgs/kastanMarker.h>



class ImageClass
{
public:
    void chatterCallback( const sensor_msgs::ImageConstPtr& msg );
    void arucoDetect(  );
    void initAruco( int argc, char** argv, const char* keys  );
    cv::Mat copyMatLoc( cv::Mat image );

    ros::Subscriber  _subscriber_key;
    cv::Mat          _classImage;
    ImageClass();
    ~ImageClass();

private:
    cv::VideoCapture     _inputVideo;
    bool             _showRejected;
    bool             _estimatePose;
    float            _markerLength;
    int              _markDim;
    std::vector< int > _ids; // marker ids
    std::vector < cv::Vec3d > _rvecs, _tvecs;
    cv::Ptr<cv::aruco::DetectorParameters>    _detectorParams;
    cv::Mat _camMatrix, _distCoeffs;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    
    aruco_msgs::MarkerArray::Ptr _marker_msg;
    // TODO remove this
    // aruco_msgs::kastanMarker::Ptr kastan_message;
    // cv::aruco::MarkerDetector _mDetector;  This is from aruco_ros
};

ImageClass::ImageClass()
{
    std_msgs::String outputMsg;
    geometry_msgs::PoseArray::Ptr   poseMessage;
    poseMessage = geometry_msgs::PoseArray::Ptr(new geometry_msgs::PoseArray());

    _marker_msg = aruco_msgs::MarkerArray::Ptr(new aruco_msgs::MarkerArray());

    // TODO remove
    // kastan_message = aruco_msgs::kastanMarker::Ptr(new geometry_msgs::PoseArray());
    // kastan_message->header.frame_id = "TESTING THIS SHIITT";
    // std::cout << "HEADER FRAME " << kastan_message->header.frame_id << std::endl;

    _marker_msg->header.frame_id = "world_frame";
    _marker_msg->header.seq = 0;

    _marker_msg->markers.resize(10);
    //_marker_msg->markers.at(0).MARKERID = 10;
    //

    
    // TODO 
    //      this is the way to do it (markers[x].pose.pose.position.x
    //      orientation is orientation and is x y z w 
    //      orientatin is in Quarternion form.
    _marker_msg->markers[2].pose.pose.position.x = 1;
}

ImageClass::~ImageClass()
{


}


// A dictionary of parameters.  Adding more is easy - just follow the pattern.
// Using OpenCV's command line parser.
namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{numMark  |       | number of markers in custom dictionary}" // currently set to 64
        "{markDim  |       | dimension of markers (ex: 4x4) in dict}"
        ;
}

/**
  *  Read camera parameters from file.
 */
static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


/**
  * read and populate the detector params file.
 */
static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}


/* Goal: Fetch parameters from OpenCV parser (command line inputs) 
 *       Initialize the dictionary of AR markers to be read
 * Params: argc and argv for cmd line parameters, and OpenCV Keys from main.
 * Returns: void
 */
void ImageClass::initAruco( int argc, char** argv, const char* keys ) {

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    _showRejected = parser.has("r");
    _estimatePose = parser.has("c");
    _markerLength = parser.get<float>("l");
    _markDim = parser.get<int>("markDim");

    // set detector parameters from file
    _detectorParams = cv::aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<std::string>("dp"), _detectorParams);
        std::cout << "detector params filename: " <<  parser.get<std::string>("dp") << std::endl;
        if(!readOk) {
            std::cerr << "Invalid detector parameters file" << std::endl;
            return;
        }
    }
    _detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    if(_estimatePose) {
        bool readOk = readCameraParameters(parser.get<std::string>("c"), _camMatrix, _distCoeffs);
        if(!readOk) {
            std::cerr << "Invalid camera file" << std::endl;
            return;
        }
    }
    // create custom dictionary
    _dictionary = cv::aruco::generateCustomDictionary(64, 4); // 64 markers in dict, all 4x4
}

/* DETECT MARKERS and DISPLAY MARKERS
 * Params: none
 * Returns: void
 */
void ImageClass::arucoDetect( ) 
{
    // create std::vectors of data holding marker info
    std::vector< std::vector < cv::Point2f > > corners, rejected; // marker corners and rejected candidates

    // detect markers
    cv::aruco::detectMarkers(_classImage, _dictionary, corners, _ids, _detectorParams, rejected);

    // estimate pose
    if (_estimatePose && _ids.size() > 0) {
        cv::aruco::estimatePoseSingleMarkers(corners, _markerLength, _camMatrix, _distCoeffs, 
                                                _rvecs, _tvecs);
    }
    



    // -----------------------------------------------------
    // ------------------- POPULATE MSGS -------------------
    // -----------------------------------------------------
    
    _marker_msg->markers.clear();


    try { 
    std::cout << "VECTORS " << _tvecs[0].position.x << std::endl;
    } catch (int e)
    {
        std::cout << "PRINTING VECTORS FAILED!!" << std::endl;
    }
    // OHHH just do:
    
    // TODO
    // _marker_msg->markers.header = // confidence = 20;

    // _marker_msg->markers.pose.pose.position.x = _tvecs[0].x;

    // _marker_msg->markers.pose.translation.x = _tvecs[0].x;
    // _marker_msg->markers.pose.rotation.x = _rvecs[0].x;  // need to figure out how these are indexed...
                                    // is it by detected markers or is marker 42 always in position 42?
    



    // -----------------------------------------------------
    // ------------------- DRAW RESULTS --------------------
    // -----------------------------------------------------


    // draw results
    if (_ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(_classImage, corners, _ids);
    }  
    // draw pose axis
    if (_estimatePose && _ids.size() > 0 ) {
        for (int i = 0; i < _ids.size(); i++) {   
            cv::aruco::drawAxis( _classImage, _camMatrix, _distCoeffs, _rvecs[i], _tvecs[i],
                                    _markerLength);
        }
    }

    cv::imshow("ourGreatWindow", _classImage);
    cv::waitKey(1);

    return;
}

// Input: image_transport message
// Return: a cv::Mat object
cv::Mat rgbToMat( const sensor_msgs::ImageConstPtr& msg ){

    // convert from ROS array to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return cv_ptr->image; // not trying to return anything, just break after error
    }
    return cv_ptr->image; // Return a cv::Mat object
}

void ImageClass::chatterCallback( const sensor_msgs::ImageConstPtr& msg)
{
    // this will be called whenver a new message arrives.
    // ROS_INFO("val 1 = %d, val 2 = %d, val 3 = %d, val 4 = %d", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    // ROS_INFO("width = %d, height = %d, step = %d", msg->width, msg->height, msg->step);
    // ROS_INFO("encoding = %s", msg->encoding.c_str());
    
    _classImage = rgbToMat( msg );

    arucoDetect( );
    
    return;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "arucoMain");

    ros::NodeHandle node;
    ImageClass image_class;
    std::cout<< "create image_class" << std::endl;

    image_class.initAruco( argc, argv, keys );

    // subscribe to publishing camera
    image_class._subscriber_key = node.subscribe("/usb_cam/image_raw", 1, &ImageClass::chatterCallback, &image_class); //  /gazebo/camera0/image_raw

    ros::spin();

    std::cout << "\nProgram terminated." << std::endl;

    return EXIT_SUCCESS;
}

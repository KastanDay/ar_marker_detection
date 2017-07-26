#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

// tf2 (transform) includes 
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>

// Messages
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/TransformStamped.h>
#include <fantastic_marker_detection/MarkerArray.h>
// #include <fantastic_marker_detection/Marker.h>



class ImageClass
{
private:
    cv::VideoCapture     _inputVideo;
    

    std::vector < int > _ids; // marker ids
    std::vector < cv::Vec3d > _rvecs, _tvecs;
    cv::Mat _camMatrix, _distCoeffs;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    
    tf2_ros::Buffer               _tfBuffer;
    tf2_ros::TransformListener*   _tfListener; 

    ros::Publisher _marker_pub;
 
    fantastic_marker_detection::MarkerArray::Ptr _marker_msg;
    
public:
    // function prototypes 
    ImageClass();
    ~ImageClass();
    void chatterCallback( const sensor_msgs::ImageConstPtr& msg );
    void msgCallback ( const geometry_msgs::TransformStamped &msg);
    void arucoDetect(  );
    void initAruco(   );
    geometry_msgs::TransformStamped  getTransform(std::string& refFrame,
                                                  std::string& childFrame,
                                                  geometry_msgs::TransformStamped transform);


    cv::Mat copyMatLoc( cv::Mat image ); // TODO: REMOVE??

    ros::NodeHandle _node;
    ros::Subscriber  _subscriber_key;
    cv::Mat          _classImage;

    // .launch (launch file) params
    std::string _source_frame;
    std::string _target_frame;
    std::string _camera_params;
    std::string _detector_params_filename;
    std::string _video_input_sub_rostopic;
    std::string _marker_pub_rostopic;
    double _marker_length;
    double _mark_dim;
    cv::Ptr<cv::aruco::DetectorParameters>    _detectorParams;
    bool draw_markers_on_screen_;



};

ImageClass::ImageClass()
{
    // -----------------------------------------------------
    // ---------------- READ .launch PARAMS ----------------
    // -----------------------------------------------------
    

    // Read params from "fantastic_marker_detection.launch" file
    ros::NodeHandle private_node("~");
                     // ("name in .launch", name_of_variable, "default value")
    private_node.param<std::string>("source_frame",    _source_frame,  "");
    private_node.param<std::string>("target_frame",    _target_frame,  "");
    private_node.param<std::string>("marker_pub_rostopic", _marker_pub_rostopic, "markerskdcpp");
    private_node.param<std::string>("video_input_sub_rostopic", _video_input_sub_rostopic, "");
    private_node.param<double>("marker_length",        _marker_length, 0.05);
    private_node.param<double>("marker_dimension",     _mark_dim, 4);
    private_node.param<std::string>("camera_params", _camera_params, "/home/kastan/catkin_ws/src/fantastic_marker_detection/bin/camCalib.txt"); // defult calib file location
    private_node.param<std::string>("detector_params", _detector_params_filename, "");
    private_node.param<bool>("draw_markers_on_screen", draw_markers_on_screen_, "true");

   
    // initialize transform listener 
    _tfListener = new tf2_ros::TransformListener(_tfBuffer);
    
    // create _marker_msg
    _marker_msg = fantastic_marker_detection::MarkerArray::Ptr
        (new fantastic_marker_detection::MarkerArray());

    // publish MarkerArray of marker messages on rostopic /markers
    _marker_pub = _node.advertise<fantastic_marker_detection::MarkerArray>(_marker_pub_rostopic, 100);
}

ImageClass::~ImageClass()
{

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


/* Purpose: Fetch parameters from OpenCV parser (command line inputs) 
 *          Initialize the dictionary of AR markers to be read
 * Params: argc and argv for cmd line parameters, and OpenCV Keys from main.
 * Returns: void
 */
void ImageClass::initAruco(  ) {
    
    // set detector params from file
    _detectorParams = cv::aruco::DetectorParameters::create(); 
    std::cout << "$$$$$ Detector params filename: " << _detector_params_filename << std::endl;
    if(! ( _detector_params_filename == "") ) {  // if a file name is specified, enter this if statement.
        bool readOk = readDetectorParameters( _detector_params_filename, _detectorParams );
        if(!readOk) {
            std::cerr << "Invalid detector parameters file with filename: " << _detector_params_filename << std::endl;
            return;
        }
    }
    _detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
    
    // set camera params from file
    if(! ( _camera_params == "") ) { // private_node.hasParam("camera_params") 
                                       // "_camera_params" is from .launch parameters
        bool readOk = readCameraParameters(_camera_params, _camMatrix, _distCoeffs);
        if(!readOk) {
            std::cerr << "Invalid camera file" << std::endl;
            return;
        }
    }

    // create custom dictionary
    // NOTE: IF YOU NEED TO CHANGE THE MARKERS BEING DETECTED, THIS IS WHRE YOU DO IT
    _dictionary = cv::aruco::generateCustomDictionary(64, _mark_dim); // 64 markers in dict, all 4x4
}

/* Purpose: Run ArUco marker detection and (optionally) display markers
 * Params: none
 * Returns: void
 */
void ImageClass::arucoDetect( ) 
{
    _marker_msg->markers.clear();
    _tvecs.clear();
    _rvecs.clear();

    std::vector< std::vector < cv::Point2f > > corners, rejected; // marker corners and rejected candidates

    // detect markers
    cv::aruco::detectMarkers(_classImage, _dictionary, corners, _ids, _detectorParams, rejected);

    // estimate pose
    if (_ids.size() > 0) {
        cv::aruco::estimatePoseSingleMarkers(corners, _marker_length, _camMatrix, _distCoeffs, 
                                                _rvecs, _tvecs);
    }

    // -----------------------------------------------------
    // ------------------- POPULATE MSGS -------------------
    // -----------------------------------------------------
    
    _marker_msg->markers.resize(_ids.size());
    
    // convert _tvecs and _rvecs (CV standard) to a geometry_msg/PoseStamped.msg format 
    //     we need a stamped format to be transformed bec. it has a message.frame_id data field.

    for(int i = 0; i < _ids.size(); i++) {

        _marker_msg->markers[i].poseStamped.header.frame_id = _source_frame;

        // marker id is stored for combination by the conssolidator node.
        _marker_msg->markers[i].marker_id = _ids[i];
        std::cout << "current marker id = " << _ids[i] << std::endl;
        sleep(1); // QUESTION
        
        _marker_msg->markers[i].poseStamped.pose.position.x = _tvecs[i][0];
        _marker_msg->markers[i].poseStamped.pose.position.y = _tvecs[i][1];
        _marker_msg->markers[i].poseStamped.pose.position.z = _tvecs[i][2];
        // std::cout << "~MarkerMsg~[" << i << "] " 
        //      << _marker_msg->markers[i].pose.pose.position << std::endl;
        
        _marker_msg->markers[i].poseStamped.pose.orientation.x = _rvecs[i][0];
        _marker_msg->markers[i].poseStamped.pose.orientation.y = _rvecs[i][1];
        _marker_msg->markers[i].poseStamped.pose.orientation.z = _rvecs[i][2];
        _marker_msg->markers[i].poseStamped.pose.orientation.w = _rvecs[i][3];
        // std::cout << "~MarkerMsg~[" << i << "] " 
        //      << _marker_msg->markers[i].pose.pose.orientation << std::endl;
    }

    // -----------------------------------------------------
    // ------------------- TRANSFORM MSGS ------------------
    // -----------------------------------------------------
    
    
    // Ensure transformation is possible
    try{ 
    _tfBuffer.lookupTransform(_target_frame, _source_frame, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    
    
    // transform messages into "world" frame from a "camera" frame
    for(int i = 0; i < _ids.size(); ++i) {
        // transform (input message, output message, frame to be translated into)
        // transforms from input.frame_id to "frame to be translated into"
        _tfBuffer.transform(_marker_msg->markers[i].poseStamped, _marker_msg->markers[i].poseStamped, _target_frame);
        _marker_msg->markers[i].header.frame_id = _target_frame;
    }
    
    // publish marker array
    _marker_pub.publish(_marker_msg);

    
    // -----------------------------------------------------
    // ------------------- DRAW RESULTS --------------------
    // -----------------------------------------------------

    if(draw_markers_on_screen_) {

        // draw results
        if (_ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(_classImage, corners, _ids);
        }  
        // draw pose axis
        if (_ids.size() > 0 ) {
            for (int i = 0; i < _ids.size(); i++) {   
                cv::aruco::drawAxis( _classImage, _camMatrix, _distCoeffs, _rvecs[i], _tvecs[i], _marker_length);
            }
        }

        cv::imshow("ourGreatWindow", _classImage);
        cv::waitKey(1);
    }

    return;
}

/* Purpose: convert a ROS image message to a cv::Mat object
 *
 * Parameter: sensor_msgs::ImageConstPtr (usually from callback function)
 *
 * Return: cv::Mat object
 */
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

/* Purpose: Get the transofmr (the difference) between two frames
 *
 *
 */
// get the transform (the difference) between two frames.
// Copied from marker_publish.cpp in aruco_ros
geometry_msgs::TransformStamped ImageClass::getTransform(std::string& refFrame,
                                                         std::string& childFrame,
                                                         geometry_msgs::TransformStamped transform)
{
    try{ 
    transform = _tfBuffer.lookupTransform(_source_frame, _target_frame, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    return transform;
} // end getTransform
 

void ImageClass::chatterCallback( const sensor_msgs::ImageConstPtr& msg)
{
    // this will be called whenver a new message arrives
    
    _classImage = rgbToMat( msg );

    arucoDetect( );
    
    return;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "arucoMain");

    ImageClass image_class;
    std::cout<< "create image_class" << std::endl;
    
    image_class.initAruco(   );

    // subscribe to publishing camera
    image_class._subscriber_key = image_class._node.subscribe(image_class._video_input_sub_rostopic, 1, &ImageClass::chatterCallback, &image_class); 
    //  Derrek's cam feeds: /gazebo/camera0/image_raw

    ros::spin();

    std::cout << "\nArUco node terminated." << std::endl;

    return 0;
}

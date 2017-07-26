// ros include
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
// std include
#include <iostream>
#include <string>
#include <vector>
// signal catching 
#include <signal.h>
// Vimba (gigE cams) include
#include "VimbaCPP.h"
// open cv include
#include <opencv2/highgui.hpp>
// convert cv::Mat object to ROS message
#include <cv_bridge/cv_bridge.h>

using namespace AVT::VmbAPI; 

// globals
bool SIGINTrecieved = false;


// GETTING NOTIFIED ABOUT CAMERA LIST CHANGES
// define observer that reacts on camera list changes
class CamObserver : public ICameraListObserver
{
public:
    void CameraListChanged( CameraPtr pCam, UpdateTriggerType reason ) {
        if ( UpdateTriggerPluggedIn == reason || UpdateTriggerPluggedOut == reason ) {
            // Put code here to react on the changed camera list (gained or lost)
            ROS_ERROR("THE NUMBER OF CONNECTED CAMERAS HAS CHANGED SINCE START OF PROGRAM");
        }
    }
};

class VimbaData
{
public:
    // funciton prototypes
    VimbaData( );
    void initVimbaCams( );
    void cameraCapture( );
    
    // Vimba (gigE) data structures
    CameraPtrVector cameras_;
    VimbaSystem &system_ = VimbaSystem::GetInstance();
    std::vector<int> openedCams_;
    
    // ros data
    ros::NodeHandle node_;
    std::vector<image_transport::Publisher> vectorOfPubs_;
    image_transport::ImageTransport it_;

    // .launch params
    std::string pub_topic_template_;
    std::string cam_settings_xml_path_;
    bool display_images_on_screen_;

private:

};

// Purpose: link ImageTransport with NodeHandle.  Also read .launch params
VimbaData::VimbaData( ) : it_(node_) {
    // in the top line of the constructor, link the ImageTransport object to the ros::NodeHandle object

    // Read .launch params
    ros::NodeHandle private_node("~");
                                // ("name_of_launch_var", local_var, default value);
    private_node.param<std::string>("gigE_cam_pub_topic_template", pub_topic_template_, "gigE_cam_0");
    private_node.param<std::string>("path_to_cam_settings_xml", cam_settings_xml_path_, "");
    private_node.param<bool>("display_images_on_screen", display_images_on_screen_, "");
}

/* Purpose: 1. Initialize all cameras detected on the local network.
 *          2. Load camera parameters from file.
 *          3. Register a camera_list observer which will throw a (non-lethal) error 
 *             if a camera is added or removed from the network during runtime.
 *
 * Parameters:
 */
void VimbaData::initVimbaCams(  ) {
    

    // initalize Vimba API   
    if ( VmbErrorSuccess == system_.Startup()) { 
        ROS_INFO("Opening all detected, network-connected cameras");
        // find all connected cameras
        if ( VmbErrorSuccess == system_.GetCameras ( cameras_ ) ) {
            std::string camName;
            int numCam = 0;
            for (   CameraPtrVector::iterator iter = cameras_.begin();
                    cameras_.end() != iter; 
                    ++iter ) 
            {
                // attempt to open (prepare for use) all detected cameras
                if ( VmbErrorSuccess  == (*iter)->Open( VmbAccessModeFull ) ) { 

                    if (VmbErrorSuccess != (*iter)->Camera::GetID( camName )) {
                        ROS_ERROR("Could not get camera name for cam %d", numCam);
                    }

                    // load camera settings form .xml
                    // NOTE: Use VimbaViewer to create profiles, found in Vimba examples dir.
                    if (VmbErrorSuccess != (*iter)->Camera::LoadCameraSettings(cam_settings_xml_path_) ) 
                    {
                        ROS_ERROR("FAILED to load camera settings for cam %d.", numCam);
                    }

                    ROS_INFO("Camera %d opened!", numCam);
                    // add cam to vector of openedCams
                    openedCams_.push_back( numCam );
                    ++numCam;

                    // you could also open specific cameras by id (ip address) with OpenCameraByID("{ip_address}", access mode, CameraPtr)
                } else { // this camera FAILED to open
                    ROS_ERROR("Camera %d, named %s, FAILED to open", numCam, camName.c_str());
                    ++numCam;
                }
            }
            // check if all detected cameras were opened.
            if ( openedCams_.size() != cameras_.size() ) {
                ROS_ERROR("Not all cameras detected on the network were able to be opened! Maybe they are already in use.");
            }
        }
    }
    
    //// CAMERA_LIST OBSERVER ////
    // Register the observer; automatic discovery for GigE is turned on
    VmbErrorType res;
    CamObserver *_pCamObserver = new CamObserver();
    res = system_.RegisterCameraListObserver( ICameraListObserverPtr( _pCamObserver ) ); // TODO Do error checking
    //// end CAMERA_LIST OBSERVER ////
    

    // create a vector of publisher topics, one topic for each gigE camera feed
    std::string pub_topic = pub_topic_template_;
    for (int i = 0; i < openedCams_.size(); ++i) {
        // @debug
        ROS_INFO("pub_topic: %s", pub_topic.c_str() );
        
        // need a new ros::Publisher for EACH camera.
        image_transport::Publisher cam_publisher = it_.advertise(pub_topic, 2);  /*<sensor_msgs::ImagePtr>*/
        vectorOfPubs_.push_back(cam_publisher);
        
        // iterate "pub_topic" string.
        // Usually starts at 0, ex: "gigE_cam_0" --> "gigE_cam_1" 
        pub_topic.pop_back();
        pub_topic += std::to_string(i+1); // +1 so that the 

    }
}

/* Purpose: 
 *      1. Detects all cameras on network.
 *      2. Attempts to open all of them - throws (non-lethal) errors if it cannot.
 *      3. Publishes the images recieved to *seperate* rosTopics, 1 per camera. Message type: sensor_msgs::ImagePtr
 *
 * Parameters:
 *      All data is shared by VimbaData class.
 */
void VimbaData::cameraCapture(  ) {
    FramePtr pFrame;  // frame data
    VmbUint32_t timeout = 500;
    VmbFrameStatusType status;
    VmbUint32_t nWidth = 0;
    VmbUint32_t nHeight = 0;
    VmbUchar_t *pImage = NULL; // frame data will be put here to be converted to cv::Mat
    sensor_msgs::ImagePtr ROS_image_ptr; // message to be published to ROS_Topic

    // for each camera {
    //    grame frame, convert it to ros::image_transport, publish it on its own topic
    // }
    
    while ( openedCams_.size() > 0 && SIGINTrecieved == false ) { // could use a global variable.  Maybe change this to "total_currently_open_cams > 0"
        // only grab frames from cams that were sucessfully opened
        for (int i = 0; i < openedCams_.size(); ++i) {
            if (VmbErrorSuccess == cameras_[ openedCams_[i] ]->Camera::AcquireSingleImage(pFrame, timeout) ) {
                // check if frame was recieved properly
                if (VmbErrorSuccess == pFrame->Frame::GetReceiveStatus( status )) {
                    // frame recieved, now ~do everything necessary with it inside this scope.~

                    ROS_INFO("Frame recieved from cam %d ", openedCams_[i]);
                        
                    // grab width and height of frame
                    if (VmbErrorSuccess != pFrame->GetWidth ( nWidth ) )
                        ROS_ERROR("FAILED to aquire width of frame!");
                    if (VmbErrorSuccess != pFrame->GetHeight( nHeight ) )
                        ROS_ERROR("FAILED to aquire height of frame!");
                    if (VmbErrorSuccess != pFrame->GetImage ( pImage ) )
                        ROS_ERROR("FAILED to acquire image data of frame!");

                    // convert to cv::Mat 
                    cv::Mat cvMat = cv::Mat(nHeight, nWidth, CV_8UC1, pImage ); 
                    cv::cvtColor(cvMat, cvMat, CV_BayerBG2RGB);
                    
                    if(display_images_on_screen_) { 
                        // @debug: display cam feeds in seperate windows 
                        cv::imshow("cvMat"+std::to_string( openedCams_[i] ), cvMat);
                        cv::waitKey(1);
                    }
                    
                    // convert from cv::Mat to ROS image message
                    sensor_msgs::ImagePtr ROSmsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvMat).toImageMsg();

                    // Publish to ROS_Topic
                    // There will never be an empty, or skipped, topic. "gigE_cam_0" will always be filled before "gigE_cam_1".
                    vectorOfPubs_[i].publish( ROSmsg ); 
                }
                else { // frame not recieved properly
                    ROS_ERROR("FRAME FROM CAM %d NOT RECIEVED PROPERLY", openedCams_[i]);
                }
            }
            else { // frame not aquired at all
                ROS_ERROR("Frame from camera %d not aquired", openedCams_[i]);
            }
        } 
        std::cout << std::endl; // space separating each batch of captures
    }
}

/* Purpose: Catch SIGINT (typically generated by user).  Gracefully exit the otherwise infinate loop of camera capture.
 *
 * Parameters: SIGINT integer from signal.  ctrl+c has value of 2.
 */
void signalHandler( int signum ) {
    ROS_INFO("Interrupt signal (%d) received", signum);
    
    SIGINTrecieved = true; // global var
}

int main(int argc, char **argv) {

    ROS_INFO("Running Vimba.cpp!");

	ros::init(argc, argv, "vimba");

    ros::NodeHandle sigNode;
    signal(SIGINT, signalHandler);

    VimbaData vimba_data;

    vimba_data.initVimbaCams( );

    // infinite loop of camera capture
    vimba_data.cameraCapture( );

    // shutdown API & destroy all objects used
    vimba_data.system_.Shutdown(); 
    ros::shutdown();
 
    return 0;
} 


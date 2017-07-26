#include <ros/ros.h>
#include <ros/console.h>
#include <ros/console.h>

// messages
#include <fantastic_marker_detection/MarkerArray.h>
#include <fantastic_marker_detection/Marker.h>

class JoinAndPublish
{
private:

    int _cams_processed = 0;
    fantastic_marker_detection::MarkerArray::Ptr _final_marker_msg;

    ros::Publisher _marker_pub;

public:
    // fucntion prototypes
    JoinAndPublish();
    ~JoinAndPublish();
    void markersCallback( const fantastic_marker_detection::MarkerArray& msg );
    void consolidation( const fantastic_marker_detection::MarkerArray& msg );

    // ros declarations
    ros::NodeHandle _node;
    
    // .launch (launch file) params
    int _num_cameras;
    int _num_markers;
    std::string _sub_topic_template;
};

JoinAndPublish::JoinAndPublish() {
    
    // Read .launch params
    ros::NodeHandle private_node("~");
                 // ("name_of_launch_var, "local_var", default_value)
    private_node.param<int>("num_cameras",  _num_cameras, 2);
    private_node.param<int>("num_markers",  _num_markers, 64);
    private_node.param<std::string>("markers_sub_topic_template", _sub_topic_template, "markers_node_0");

    // create marker_msg big enough to hold all markers
    _final_marker_msg = fantastic_marker_detection::MarkerArray::Ptr (new fantastic_marker_detection::MarkerArray() );


    _final_marker_msg->markers.resize(_num_markers);

    // publish MarkerArray - the one big array that all other users will care about!
    // TODO: Make the pub topic a .launch param
    _marker_pub = _node.advertise<fantastic_marker_detection::MarkerArray>("FANTASTIC_MARKERS", 100);
}

JoinAndPublish::~JoinAndPublish() {

}

// This function is called once for each camera feed.  After every camera feed has been consolidated
// the consolidated array is published.
// @breif take in a MarkerArray and add it to our master marker array
// @param the (pointer!) MarkerArray message for the current camera feed being consolidated
// @return void
void JoinAndPublish::consolidation( const fantastic_marker_detection::MarkerArray& msg  ) {

     // Go through and put markers in their proper place in the array.
     // Invariant: marker with id 4, will be in array position 4.  id 0 in position 0.
     // if data already exists there, overwrite it.  (@future, consider taking the average?)

     for (int i = 0; i < msg.markers.size(); ++i) {
         _final_marker_msg->markers[msg.markers[i].marker_id] = msg.markers[i];
     }
    return;
}

void JoinAndPublish::markersCallback( const fantastic_marker_detection::MarkerArray& msg ) {

    consolidation( msg );

    _cams_processed++;
    
    // publish consolidated markers once all camera feeds are combined
    // Enter this if statment after all camera feeds combined, publish a single frame.
    // Should end up publishing many times per second (this is, basically, our fps).
    if ( _cams_processed == _num_cameras ) {
        _marker_pub.publish(_final_marker_msg);
        _cams_processed = 0;
    }

}

int main (int argc, char **argv) {

    ros::init(argc, argv, "fmd_consolidation");

    std::cout << "This consoloidation node launched!" << std::endl;
    
    // declare class
    JoinAndPublish join_and_publish;
    
    // create sufficient subscribers for all of our camera feeds
    std::vector<ros::Subscriber> vectorOfSubs;
    for(int i = 0; i < join_and_publish._num_cameras; ++i) {  
        ros::Subscriber msgs_subscriber;
        vectorOfSubs.push_back(msgs_subscriber);
    }
    
    // subscribe to all of our topics publishing MarkerArrays from camera feeds 
    std::string sub_topic = join_and_publish._sub_topic_template;
    for(int i = 0; i < join_and_publish._num_cameras; ++i) {  
        vectorOfSubs[i] = join_and_publish._node.subscribe(sub_topic,
                1, &JoinAndPublish::markersCallback, &join_and_publish); // "markers_node_%d", i
        
        // increment the string "sub_topic_#" by 1 (to subscribe to all camera topics)
        // Usually starts at 0, ex: "sub_topic_0" --> "sub_topic_1"
        sub_topic.pop_back();
        sub_topic += std::to_string(i);
    }

    ros::spin();

    std::cout << "\nConsolidation node terminated." << std::endl;
    
    return 0;
}


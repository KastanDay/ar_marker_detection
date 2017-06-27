// Written by Kastan Day
// Purpose: Create a custom dictinary of ArUco markers.
// 3 necessary parameters: bb, ms, and save location (in a string using quotes) "~/location/here"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

using namespace cv; // don't use

namespace {
const char* about = "Create an ArUco marker image";
const char* keys  =
        "{@outfile |<none> | Output image location. Provide directory only, will output set of png files. }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{id       |       | Marker id in the dictionary }"
        "{ms       | 200   | Marker size in pixels }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }"
        "{numMark  |       | number of markers desired in dictionary (keep as small as reasonable}"
        "{markDim  |       | dimension of marker, only provide a single number (ex: '4' means 4x4}" ;
}

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 3) {
        parser.printMessage();
        return 0;
    }

    int borderBits = parser.get<int>("bb"); // ex: -bb=1 (1 byte of border)
    int markerSize = parser.get<int>("ms"); // ex: -ms=500 (500px x 500px)
    String saveLocation = parser.get<String>(0); // ex: "home/kastan/workspace/ArUcoMarkers/" (quotes necessary, cannot use ~/workspace (must be full path) MUST INCLUDE TRAILING '/')
    int numMark = parser.get< int >("numMark"); // ex: -numMark=64, create 64 markers
    int markDim = parser.get< int >("markDim"); // ex: -markDim=4, create 4x4 markers

    Ptr<aruco::Dictionary> kdayDictionary = aruco::generateCustomDictionary(numMark, markDim);
    Mat markerImg;
    std::string saveFilename;
    // draw and save (imwrite) all 64 markers to a specified folder.
    for( int i = 0; i < 64; i++){
        
        aruco::drawMarker(kdayDictionary, /*markerId*/ i, markerSize, markerImg, borderBits);
        
        saveFilename = saveLocation + std::to_string(i) + ".png";
    
        std::cout << "Writing marker #" + std::to_string(i) + " to " + saveFilename << std::endl;
        // imshow("marker", markerImg);
        // waitKey(0);
        try {
            imwrite(saveFilename, markerImg);
        }
        catch (std::runtime_error& ex) {
            fprintf(stderr, "Error writing image to file in 'imwrite()': %s\n", ex.what() );
            return 1;
        }
    }
    return 0;
}


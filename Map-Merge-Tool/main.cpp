#include <string>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui_c.h"
#include "mapmerge.h"
#include <cstdlib>
#include <unistd.h>
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  while(true){

    bool verbose = true;
    string outfile = "final.pgm";
    vector<string> infiles;

    /*
      About max_pairwise_distance
      In the default image, anything less than 0.5 will cause a core dump or an error.
      On the other hand, there is no upper limit and it can be as large as you like.

      About matches_threshold
      In the default image, anything less than 20 will cause a core dump or an error.
      On the other hand, there is no upper limit and it can be as large as you want.
    */
    //float max_pairwise_distance = 5.0;
    //float matches_threshold = 30.0;
    
    // Parameters for map1 and map2 integration
    float max_pairwise_distance = 35.0;
    float matches_threshold = 35.0;

    if(argc <= 3){
      cerr << "error: format <inputfile1> <inputfile2> <outputfile>" << endl;
      exit(-1);
    }
    
    infiles.push_back(argv[1]);
    infiles.push_back(argv[2]);
    outfile = argv[3];
   
    // load the images
    vector<Mat> images;
    for (size_t i = 0; i < infiles.size(); i++) {
      Mat image = imread(infiles[i].c_str(), 0); // 0=grayscale
      if (!image.data) {
        cerr << "error: image " << infiles[i] << " not loadable." << endl;
        exit(-1);
      }
      images.push_back(image);
    }

    // create the stitched map
    StitchedMap map(images[0],images[1], max_pairwise_distance, matches_threshold);
    
    // write to outfile if applicable
    if (outfile.size() != 0) {
      imwrite(outfile, map.get_stitch());
    }

    if (outfile.size() == 0 || verbose) { // generate some output
      cout << "rotation: "          << map.rotation << endl
           << "translation (x,y): " << map.transx   << ", " << map.transy << endl
           << "matrix: "            << map.H        << endl;
    }

    if (verbose) {
      namedWindow("wrap"); imshow("wrap", map.get_stitch()); imwrite("stitch.pgm", map.get_stitch());
      namedWindow("debug"); imshow("debug", map.get_debug()); imwrite("debug.pgm", map.get_debug());
      waitKey(5000);
    }
  }
  return 0;
}

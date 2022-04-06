#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui_c.h"
#include <cstdlib>
#include <unistd.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{ 
    string outfile        = "final.pgm";
    string infiles        = "figure/room3.pgm";
    string target_inflies = "figure/room4.pgm";

    // Loading images
    cv::Mat image = imread(infiles.c_str(), 0); // 0 = grayscale, 1 = colorscale
    cv::Mat target_image = imread(target_inflies.c_str(), 0); // 0 = grayscale, 1 = colorscale
    
    // Image loading judgment
    if (!image.data) {
        cerr << "error: image " << infiles << " not loadable." << endl;
        exit(-1);
    }

    /* 
      Generate an image with all surfaces in grayscale.
      <Reference> : https://shimikel.hatenablog.com/entry/2015/07/25/190139
    */
    int width    = max(image.cols, target_image.cols) + 1;
    int height   = max(image.rows, target_image.rows) + 1;
    int channels = target_image.channels();
    cv::Mat grayscale_image(Size(width, height), CV_8U, Scalar::all(205)); 
    //cv::Mat grayscale_image(Size(237, 199), CV_8U, Scalar::all(205)); 
    
    /*
      Embed the pixel values of the target image into a grayscale image.
      <Rederence> : https://d.akiroom.com/2012-06/opencv-pixel-read-write-get-set/
    */
    for( int y = 0; y < image.rows; y++ ) {
        // Getting pointers
        cv::Vec3b* ptr = image.ptr<cv::Vec3b>( y );
        cv::Vec3b* gray_ptr = grayscale_image.ptr<cv::Vec3b>( y );

        for( int x = 0; x < (int) (image.cols / 3); x++ ) {
            gray_ptr[x] = ptr[x];
        }
    }

    cv::imshow("risize map", grayscale_image);
    //waitKey(5000);
    
    imwrite(outfile, grayscale_image);

    return 0;
}
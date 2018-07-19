#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>
using namespace cv;
using namespace cv::ximgproc;
using namespace std;

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);
const String keys =

    "{help h usage ? |                  | print this message                                                }"
    "{vis_mult       |1.0               | coefficient used to scale disparity map visualizations            }"
    "{max_disparity  |160               | parameter of stereo matching                                      }"
    "{window_size    |-1                | parameter of stereo matching                                      }"
    "{wls_lambda     |8000.0            | parameter of post-filtering                                       }"
    "{wls_sigma      |1.5               | parameter of post-filtering                                       }"
    ;

// ^ These are parameters that can be used to modify the process to some degree. There used to be more,
// so these are the vestiges of a larger program that allowed changing the algorthim etc. We are only using
// SGBM, only filtering with confidence, etc so all extraneous options were removed. 

int main(int argc, char** argv){
    CommandLineParser parser(argc,argv,keys);
    parser.about("Disparity Filtering Demo");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    string left_input;
    string right_input;
    double disparity;
    int max_disp = parser.get<int>("max_disparity");
    double lambda = parser.get<double>("wls_lambda");
    double sigma  = parser.get<double>("wls_sigma");
    double vis_mult = parser.get<double>("vis_mult");

    int wsize;
    if(parser.get<int>("window_size")>=0) //user provided window_size value
    {
        wsize = parser.get<int>("window_size");
    }
    else 
    {
        wsize = 3; //default window size for SGBM
    }    
    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }

    //! [load_views]

    cout << endl;
    cout <<"Enter left image file name and extension [.jpg, .tif, .png]" << endl;
    getline( cin, left_input);
	Mat left = imread(left_input);
    if ( left.empty() )
    {
        cout<<"Cannot read image file: "<<left;
        return -1;
    }


    cout << endl;
    cout <<"Enter right image file name and extension [.jpg, .tif, .png]" << endl;
    getline(cin, right_input);
	Mat right = imread(right_input);

    if ( right.empty() )
    {
        cout<<"Cannot read image file: "<<right;
        return -1;
    }
    //! [load_views]


    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;

    if(max_disp<=0 || max_disp%16!=0)
    {
        cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
        return -1;
    }
    if(wsize<=0 || wsize%2!=1)
    {
        cout<<"Incorrect window_size value: it should be positive and odd";
        return -1;
    }
    
        
    left_for_matcher  = left.clone();
    right_for_matcher = right.clone();
        
    Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
    left_matcher->setP1(24*wsize*wsize);
    left_matcher->setP2(96*wsize*wsize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
    wls_filter = createDisparityWLSFilter(left_matcher);
    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
            
    left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            
    

        //! [filtering]
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    wls_filter->filter(left_disp,left,filtered_disp,right_disp);
        //! [filtering]


        // Get the ROI that was used in the last filter call:
    ROI = wls_filter->getROI();
        

        //! [visualization]
        
    double min, max;
    minMaxLoc(filtered_disp, &min, &max);
    disparity = max/16;
	
    // ^ finds disparity of the highest point. due to file
    // type, value must be divided by 16 to get actual
    // disparity value. This value can be used in the 
    // triangulation equation to calculate distance to 
    // target
        
    cout << endl;
    cout << disparity << endl;

    Mat raw_disp_vis;
    getDisparityVis(left_disp,raw_disp_vis,vis_mult);
    namedWindow("raw disparity", WINDOW_AUTOSIZE);
    imshow("raw disparity", raw_disp_vis);
    Mat filtered_disp_vis;
    getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    imshow("filtered disparity", filtered_disp_vis);
    imwrite("disparity.tif", filtered_disp_vis);

     waitKey();

        //! [visualization]
	
    return 0;
}

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();
    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;
    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;


    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);

    return r;

}

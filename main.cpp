#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace std;
using namespace cv;
/// Global Variables
bool use_mask;
Mat img; Mat templ; Mat mask; Mat result; Mat base; Mat test;
const char* image_window = "Source Image";
const char* result_window = "Result window";

int match_method = 3;
int max_Trackbar = 5;
//! [declare]
/// Function Headers

void MatchingMethod(int, void*);
// This function resizes the target image to a scale of the template example input: .5
void resize() {

	float ratio;
	cout << "Enter ratio: ";
	cin >> ratio;
	Mat resized_image;
	//This is the original target image
	Mat og_image = imread("filename");
	resize(og_image, resized_image, Size(), ratio, ratio);
	imwrite("filename", resized_image);

}
double rotate() {
	//This fuction rotates the large map to the angle that the drones are flying
	double ang;
	cout << "Degrees off of North: ";
	//positive 0-90 clockwise and negative 0-90 counter-clockwise
	cin >> ang;
	//This file is the template, the image that you will be searching on, the larger one
	Mat pic = imread("filename");
	Mat rot;
	Point2f pc(pic.cols / 2., pic.rows / 2.);
	Mat r = getRotationMatrix2D(pc, ang, 1.0);

	Rect2f box = RotatedRect(Point2f(), pic.size(), ang).boundingRect2f();
	r.at<double>(0, 2) += box.width / 2.0 - pic.cols / 2.0;
	r.at<double>(1, 2) += box.height / 2.0 - pic.rows / 2.0;

	warpAffine(pic, rot, r, box.size());
	//rotated image
	imwrite("filename", rot);
	return ang;
	return 1;
}
void rotate2(double ang) {

	// This function rotates the base map back to its original orientation
	//This image is the rotated map with the location of the two drones
	Mat pic = imread("filename");
	Mat rot;
	ang *= -1;
	Point2f pc(pic.cols / 2., pic.rows / 2.);
	Mat r = getRotationMatrix2D(pc, ang, 1.0);

	Rect2f box = RotatedRect(Point2f(), pic.size(), ang).boundingRect2f();
	r.at<double>(0, 2) += box.width / 2.0 - pic.cols / 2.0;
	r.at<double>(1, 2) += box.height / 2.0 - pic.rows / 2.0;

	warpAffine(pic, rot, r, box.size());
	//This file is the final output, the image back in its original orientation with the locations of the drones in red
	imwrite("filename", rot);
}

int main(int argc, char** argv)
{
	/*	if (argc < 3)
	{
	cout << "Not enough parameters" << endl;
	cout << "Usage:\n./MatchTemplate_Demo <image_name> <template_name> [<mask_name>]" << endl;
	return -1;
	}*/
	//! [load_image]
	/// Load image and template    img = base templ = test

	double ang = rotate();
	resize();
	//This is the rotated image that gets written in the rotate function
	img = imread("filename");// big picture
	//This is the resized file
	templ = imread("filename");// Find



	if (argc > 3) {
		use_mask = true;
		mask = imread(argv[3], IMREAD_COLOR);
	}
	if (img.empty() || templ.empty() || (use_mask && mask.empty()))
	{
		cout << "Can't read one of the images" << endl;
		return -1;
	}
	//! [load_image]
	//! [create_windows]
	/// Create windows
	namedWindow(image_window, WINDOW_AUTOSIZE);
	namedWindow(result_window, WINDOW_AUTOSIZE);
	//! [create_windows]
	//! [create_trackbar]
	/// Create Trackbar
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//const char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
	//createTrackbar(trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod);
	//! [create_trackbar]
	MatchingMethod(0, 0);
	rotate2(ang);
	//! [wait_key]
	waitKey(0);
	return 0;
	//! [wait_key]
}
/**
* @function MatchingMethod
* @brief Trackbar callback
*/
void MatchingMethod(int, void*)
{
	//! [copy_source]
	/// Source image to display
	Mat img_display;
	img.copyTo(img_display);
	//! [copy_source]
	//! [create_result_matrix]
	/// Create the result matrix
	int result_cols = img.cols - templ.cols + 1;
	int result_rows = img.rows - templ.rows + 1;
	result.create(result_rows, result_cols, CV_32FC1);
	//! [create_result_matrix]
	//! [match_template]
	/// Do the Matching and Normalize
	bool method_accepts_mask = (TM_SQDIFF == match_method || match_method == TM_CCORR_NORMED);
	if (use_mask && method_accepts_mask)
	{
		matchTemplate(img, templ, result, match_method, mask);
	}
	else
	{
		matchTemplate(img, templ, result, match_method);
	}
	//! [match_template]
	//! [normalize]
	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	//! [normalize]
	//! [best_match]
	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
	//! [best_match]
	//! [match_loc]
	/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
	if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED)
	{
		matchLoc = minLoc;
	}
	else
	{
		matchLoc = maxLoc;
	}
	//! [match_loc]
	//! [imshow]
	/// Show me what you got
	Size sizel(1280, 960);
	rectangle(img_display, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);
	rectangle(result, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);
	resize(result, base, sizel);
	resize(img_display, test, sizel);
	imshow(image_window, test);
	imshow(result_window, base);
	if (match_method == 3) {
		int x = matchLoc.x;
		int y = matchLoc.y;
		int x1 = templ.cols / 2;
		int y1 = templ.rows / 2;
		int j = x + x1;
		int k = y + y1;
		Point p(j - 7, k);
		Point q(j + 7, k);
		circle(img, p, 2, Scalar(0, 0, 255), 2, 8, 0);
		circle(img, q, 2, Scalar(0, 0, 255), 2, 8, 0);
		//This file is an image of the location of the two drones on the rotated image
		imwrite("filename", img);
	}


	//imshow(image_window, img_display);
	//imshow(result_window, result);
	//! [imshow]
	return;
}

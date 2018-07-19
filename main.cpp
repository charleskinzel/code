#include "stdio.h"
#include "opencv2/highgui/highgui_c.h"
#include "ASICamera2.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types_c.h>
#include <cstdlib>
#include <fstream>
#include <opencv/cv.h>
#include <Windows.h>

using namespace std;
using namespace cv;

int main() {

	IplImage *img = 0;
	IplImage *other = 0;

	int imgtype = ASI_IMG_RAW8;
	int num = ASIGetNumOfConnectedCameras();

	cout << "Number of cameras: " << num << endl << endl;

	ASI_CAMERA_INFO caminfo;
	for (int i = 0; i < num; ++i) {
		ASIGetCameraProperty(&caminfo, i);
		cout << i << " " << caminfo.Name << endl;
	}
	for (int i = 0; i < num; ++i) {
		ASIOpenCamera(i);
		if (ASIOpenCamera(i) != ASI_SUCCESS) {
			cout << "Camera failed to open..." << endl;
		}
		ASIInitCamera(i);
	}
	cout << endl << "These are the controls: " << endl;
	ASI_CONTROL_CAPS asicon;
	int numcon;
	ASIGetNumOfControls(0, &numcon);
	for (int i = 0; i < numcon; ++i) {
		ASIGetControlCaps(0, i, &asicon);
		cout << asicon.Name << endl;
	}

	int w = 1280, h = 960, g = 0, opt = 0;
	int e = 2;
	while (opt != 9) {
		cout << endl << endl << "Choose a number to change settings '9' to continue..." << endl;
		cout << "1. Width  = " << w << endl;
		cout << "2. Height = " << h << endl;
		cout << "3. Gain   = " << g << endl;
		cout << "4. Expose = " << e << "ms" << endl;
		cout << "Choice: ";
		cin >> opt;
		switch (opt) {
		case 1:
			cout << "Enter width: ";
			cin >> w;
			cout << endl << "Settings changed... " << endl << endl << endl;
			break;
		case 2:
			cout << "Enter height: ";
			cin >> h;
			cout << endl << "Settings changed... " << endl << endl << endl;
			break;
		case 3:
			cout << "Enter gain: ";
			cin >> g;
			cout << endl << "Settings changed... " << endl << endl << endl;
			break;
		case 4:
			cout << "Enter exposure: ";
			cin >> e;
			cout << endl << "Settings changed... " << endl << endl << endl;
			break;

		}


	}
	int l = 0;
	int r = 1;
	cout << "Exposure is about to begin, do not move cameras..." << endl;
	Sleep(3000);
	img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	other = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);

	ASISetROIFormat(r, w, h, 1, ASI_IMG_RAW8);
	ASISetROIFormat(l, w, h, 1, ASI_IMG_RAW8);
	long imgSize = w * h*(1 + (imgtype == ASI_IMG_RAW8));

	ASISetControlValue(r, ASI_GAIN, g, ASI_FALSE);
	ASISetControlValue(r, ASI_EXPOSURE, e * 1000, ASI_FALSE);
	ASISetControlValue(r, ASI_BANDWIDTHOVERLOAD, 40, ASI_FALSE);

	ASISetControlValue(l, ASI_GAIN, g, ASI_FALSE);
	ASISetControlValue(l, ASI_EXPOSURE, e * 1000, ASI_FALSE);
	ASISetControlValue(l, ASI_BANDWIDTHOVERLOAD, 40, ASI_FALSE);
	char cap = 'c';
	int count = 0;
	for (int i = 0; i < 20; ++i) {
		//while (cap == 'c') {
		ASI_EXPOSURE_STATUS status;
		ASIStartExposure(r, ASI_FALSE);
		ASIStartExposure(l, ASI_FALSE);

		status = ASI_EXP_WORKING;

		while (status == ASI_EXP_WORKING) {
			ASIGetExpStatus(r, &status);
		}

		if (status == ASI_EXP_SUCCESS) {
			cout << "Left Image....Success!" << endl;
			ASIGetDataAfterExp(r, (unsigned char*)img->imageData, imgSize);

		}

		status = ASI_EXP_WORKING;
		while (status == ASI_EXP_WORKING) {
			ASIGetExpStatus(l, &status);
		}

		if (status == ASI_EXP_SUCCESS) {
			cout << "Right image....Success!" << endl;
			ASIGetDataAfterExp(l, (unsigned char*)other->imageData, imgSize);
		}


		Mat qwe = cvarrToMat(img);
		Mat asd = cvarrToMat(other);

		stringstream l_name, r_name;
		l_name << "left" << setw(2) << setfill('0') << count << ".tif";
		r_name << "right" << setw(2) << setfill('0') << count << ".tif";
		imwrite(l_name.str(), qwe);
		imwrite(r_name.str(), asd);
		cout << "Saved set " << count << endl;
		count++;

		Beep(750, 300);
		Sleep(3000);
		cout << "Enter 'c' to take another picture: " << endl;
		//cin >> cap;
		system("CLS");
	}
	Sleep(1000);

	ASIStopExposure(r);
	ASICloseCamera(r);
	ASIStopExposure(l);
	ASICloseCamera(l);

}
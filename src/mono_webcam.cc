/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include<time.h>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

struct ocam_model
{
	double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
	int length_pol;                // length of polynomial
	double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
	int length_invpol;             // length of inverse polynomial
	double xc;         // row coordinate of the center
	double yc;         // column coordinate of the center
	double c;          // affine parameter
	double d;          // affine parameter
	double e;          // affine parameter
	int width;         // image width
	int height;        // image height
};

int get_ocam_model(struct ocam_model *myocam_model, char *filename);
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model);
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model);
void create_perspecive_undistortion_LUT(CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf);

int get_ocam_model(struct ocam_model *myocam_model, char *filename)
{
	double *pol = myocam_model->pol;
	double *invpol = myocam_model->invpol;
	double *xc = &(myocam_model->xc);
	double *yc = &(myocam_model->yc);
	double *c = &(myocam_model->c);
	double *d = &(myocam_model->d);
	double *e = &(myocam_model->e);
	int    *width = &(myocam_model->width);
	int    *height = &(myocam_model->height);
	int *length_pol = &(myocam_model->length_pol);
	int *length_invpol = &(myocam_model->length_invpol);
	FILE *f;
	char buf[CMV_MAX_BUF];
	int i;

	//Open file
	if (!(f = fopen(filename, "r")))
	{
		printf("File %s cannot be opened\n", filename);
		return -1;
	}

	//Read polynomial coefficients
	fgets(buf, CMV_MAX_BUF, f);
	fscanf(f, "\n");
	fscanf(f, "%d", length_pol);
	for (i = 0; i < *length_pol; i++)
	{
		fscanf(f, " %lf", &pol[i]);
	}

	//Read inverse polynomial coefficients
	fscanf(f, "\n");
	fgets(buf, CMV_MAX_BUF, f);
	fscanf(f, "\n");
	fscanf(f, "%d", length_invpol);
	for (i = 0; i < *length_invpol; i++)
	{
		fscanf(f, " %lf", &invpol[i]);
	}

	//Read center coordinates
	fscanf(f, "\n");
	fgets(buf, CMV_MAX_BUF, f);
	fscanf(f, "\n");
	fscanf(f, "%lf %lf\n", xc, yc);

	//Read affine coefficients
	fgets(buf, CMV_MAX_BUF, f);
	fscanf(f, "\n");
	fscanf(f, "%lf %lf %lf\n", c, d, e);

	//Read image size
	fgets(buf, CMV_MAX_BUF, f);
	fscanf(f, "\n");
	fscanf(f, "%d %d", height, width);

	fclose(f);
	return 0;
}



using namespace std;

// From http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

int main(int argc, char **argv)
{

	cv::Mat source, destination;

	struct ocam_model o;
	char str[] = "./calib_results_fisheye.txt";
	get_ocam_model(&o, str);

	string vocabPath = "ORBvoc.txt";
	string settingsPath = "webcam.yaml";
	if (argc == 1)
	{

	}
	else if (argc == 2)
	{
		vocabPath = argv[1];
	}
	else if (argc == 3)
	{
		vocabPath = argv[1];
		settingsPath = argv[2];
	}
    else
    {
        cerr << endl << "Usage: mono_webcam.exe path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocabPath, settingsPath,ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

	cv::VideoCapture cap(2);
	cv::VideoCapture stereo(0);



	// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
	__int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	__int64 last = now;
	int id = 0;
	std::list<cv::Mat> traj;

	std::list<cv::Mat> mono_images;
	std::list<cv::Mat> stereo_images;

	// Main loop
	cv::Mat Tcw;
    while (true)
    {

		cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		// cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.25);

		stereo.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
		stereo.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

		cv::Mat stereo_image;
		stereo >> stereo_image;
		stereo_images.push_back(std::move(stereo_image));

		cv::Mat src;
		cap >> src;
		mono_images.push_back(src.clone());

		IplImage copy;
		copy = src;
		IplImage *src1 = &copy;
		IplImage *dst_persp = cvCreateImage(cvGetSize(src1), 8, 3);


		CvMat* mapx_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);
		CvMat* mapy_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);

		float sf = 1.5;
		create_perspecive_undistortion_LUT(mapx_persp, mapy_persp, &o, sf);

		cvRemap(src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

		source = cv::cvarrToMat(src1);
		destination = cv::cvarrToMat(dst_persp);

		__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		// Pass the image to the SLAM system
		Tcw = SLAM.TrackMonocular(destination, curNow / 1000.0);

		if (!Tcw.empty()) {
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
			std::ostringstream stream;
			stream << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2);
			string fileName = stream.str();
			if ((curNow - last) > 1000) {
				std::cout << fileName << std::endl;
				last = curNow;
				traj.push_back(twc);
			}
		}
		/* This can write each image with its position to a file if you want
		if (!Tcw.empty())
		{
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
			std::ostringstream stream;
			//stream << "imgs/" << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " <<
			//	Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " <<
				//Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << ".jpg";
			stream << "imgs/" << curNow << ".jpg";
			string fileName = stream.str();
			cv::imwrite(fileName, im);
		}
		*/

		// This will make a third window with the color images, you need to click on this then press any key to quit
		cv::namedWindow("Undistorted Perspective Image", 1);
		cv::imshow("Undistorted Perspective Image", destination);


		if (cv::waitKey(1) >= 0)
			break;
    }

	ofstream os;
	os.open("traj.txt");
	for (auto& pose : traj) {
		os << pose.at<float>(0) << " " << pose.at<float>(1) << " " << pose.at<float>(2) << "\n";
	}
	os.close();

	id = 0;
	for (auto& image : mono_images) {
		++id;
		if (!image.empty()) {
			cv::imwrite(("./images/mono_" + std::to_string(id) + ".jpg").c_str(), image);
		}
	}
	id = 0;
	for (auto& image : stereo_images) {
		++id;
		if (!image.empty()) {
			cv::imwrite(("./images/stereo_" + std::to_string(id) + ".jpg").c_str(), image);
		}
	}

    // Stop all threads
    SLAM.Shutdown();
	cap.release();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
	double *pol = myocam_model->pol;
	double xc = (myocam_model->xc);
	double yc = (myocam_model->yc);
	double c = (myocam_model->c);
	double d = (myocam_model->d);
	double e = (myocam_model->e);
	int length_pol = (myocam_model->length_pol);
	double invdet = 1 / (c - d * e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

	double xp = invdet * ((point2D[0] - xc) - d * (point2D[1] - yc));
	double yp = invdet * (-e * (point2D[0] - xc) + c * (point2D[1] - yc));

	double r = sqrt(xp*xp + yp * yp); //distance [pixels] of  the point from the image center
	double zp = pol[0];
	double r_i = 1;
	int i;

	for (i = 1; i < length_pol; i++)
	{
		r_i *= r;
		zp += r_i * pol[i];
	}

	//normalize to unit norm
	double invnorm = 1 / sqrt(xp*xp + yp * yp + zp * zp);

	point3D[0] = invnorm * xp;
	point3D[1] = invnorm * yp;
	point3D[2] = invnorm * zp;
}

void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
	double *invpol = myocam_model->invpol;
	double xc = (myocam_model->xc);
	double yc = (myocam_model->yc);
	double c = (myocam_model->c);
	double d = (myocam_model->d);
	double e = (myocam_model->e);
	int    width = (myocam_model->width);
	int    height = (myocam_model->height);
	int length_invpol = (myocam_model->length_invpol);
	double norm = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
	double theta = atan(point3D[2] / norm);
	double t, t_i;
	double rho, x, y;
	double invnorm;
	int i;

	if (norm != 0)
	{
		invnorm = 1 / norm;
		t = theta;
		rho = invpol[0];
		t_i = 1;

		for (i = 1; i < length_invpol; i++)
		{
			t_i *= t;
			rho += t_i * invpol[i];
		}

		x = point3D[0] * invnorm*rho;
		y = point3D[1] * invnorm*rho;

		point2D[0] = x * c + y * d + xc;
		point2D[1] = x * e + y + yc;
	}
	else
	{
		point2D[0] = xc;
		point2D[1] = yc;
	}
}

void create_perspecive_undistortion_LUT(CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
	int i, j;
	int width = mapx->cols; //New width
	int height = mapx->rows;//New height     
	float *data_mapx = mapx->data.fl;
	float *data_mapy = mapy->data.fl;
	float Nxc = height / 2.0;
	float Nyc = width / 2.0;
	float Nz = -width / sf;
	double M[3];
	double m[2];

	for (i = 0; i<height; i++)
		for (j = 0; j<width; j++)
		{
			M[0] = (i - Nxc);
			M[1] = (j - Nyc);
			M[2] = Nz;
			world2cam(m, M, ocam_model);
			*(data_mapx + i * width + j) = (float)m[1];
			*(data_mapy + i * width + j) = (float)m[0];
		}
}
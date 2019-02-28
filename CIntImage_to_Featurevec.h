/*
 * David Fischinger -TUW
 * 18.11.2011
 *
 * CLASS for calculating Feature Vector from Integral images
 *
 * OUTDATED!!!!!
 *
 * USAGE:
 * 	- generate ListOfIIFilenames.txt in correct folder
 * 	- change all Parameters
 * 	- execute
 *
 *
 * input:
 *   Filenames (generated with: "ls pcd* -1 > ListOfIIFilenames.txt"
 *   executed in IntegralImages folder)
 *   with integral images and filename with features
 *
 * output:
 *   One file for each integral images including all feature values (1 value for each feature) is saved in the given output folder.
 *
 *   PARAMETERS:
 *
 *    HEIGHT 15 (14+1)
 *    WIDTH 15 (14+1)
 *    pathout = "/home/grasp/David/GPDatabase/badgps/twocams/featurevec/";
 *    goodgps		indicates if features for good or bad GPs are calculated => label +1/-1 in output .txt
 *	  path =    "/opt/ros/privat/stacks/toolsdf/CeditFeatures_input.txt";       //file with Haar-Features
 */


#ifndef CINTIMAGE_TO_FEATUREVEC_H_
#define CINTIMAGE_TO_FEATUREVEC_H_

//#include <boost/thread/thread.hpp>
#include <iostream>
#include <string.h>
#include <sstream>
#include <fstream>

//#include "CIntegralImage.cpp"
#include <time.h>
#include <opencv/cv.h>
#include <CHaarFeature.h>

#define HEIGHT 15
#define WIDTH 15

using namespace std;
using namespace cv;



class CIntImage_to_Featurevec
{
public:
	CHaarFeature * currentfeature;	//number of regions
	vector<CHaarFeature> allfeatures;
	float intimagemat[HEIGHT][WIDTH];
	//PARAMETERS
	string path;
	string pathfull_list_of_filenames;
	string pathout;
	bool goodgps;


	void print_heights();
	void read_features();
	void print_features();
	void write_featurevector(string outputpath);
	float calc_featurevalue(int nr_feat);


	CIntImage_to_Featurevec();
};


#endif /* CINTIMAGE_TO_FEATUREVEC_H_ */

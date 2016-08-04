
 /*
  * Clusters.h
  *
  * Copyright 2016  <cagatay.odabasi@boun.edu.tr>
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
  * MA 02110-1301, USA.
  *
  *
  */

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <vector>
#include <string>

class Clusters
{
	public:
		Clusters(float, char*, int, float);

		// similarity threshold
		float similarity_threshold;

		// Defines the space between sift extraction points
		int sift_density;

		// Histogram score threshold
		float hist_score_thresh;

		// Filenames of fixation points
		std::vector<std::string> filenames;

		// Sift Feature vectors of fixation points
		std::vector<cv::Mat> feature_array_vector;

		// Hist Feature Vector
		std::vector<cv::MatND> hist_array_vector;

		// Labels of files in filenames vector
		// Do not touch this variable
		std::vector<int> labels;

    // how many clusters do we have? Don't touch this!
    int number_of_clusters;

		// Run clustering algorithm, this will automatically update both labels and
		// number_of_clusters variables
		void runClustering(std::string filename);

		// save current labels and filenames to a file
		void write2TXT();

	private:
		// Computes the sift densely and returns a feature array
		cv::Mat computeDenseSIFT(cv::Mat im, int step_size);

		// Computes the HUE and Saturation histograms of BGR image
		cv::MatND computeHSHist(cv::Mat im);

    // txt file used for saving the labels
	  char* txtfile_name;
};

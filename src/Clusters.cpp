/*
 * Clusters.cpp
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
#include "Clusters.h"

// Constructor
Clusters::Clusters(float _similarity_threshold,
	 char* filename,
	 int _sift_density,
   float _hist_score_threshold)
{
	Clusters::similarity_threshold = _similarity_threshold;

  Clusters::number_of_clusters = 0;

	Clusters::txtfile_name = filename;

	Clusters::sift_density = _sift_density;

	Clusters::hist_score_thresh = _hist_score_threshold;
}


void Clusters::runClustering(std::string filename)
{

	// Read the image
	cv::Mat im = cv::imread(filename, CV_LOAD_IMAGE_COLOR);

	// Resize the image
	cv::Mat resized_im;

	cv::resize(im, resized_im, cv::Size(150,150));

	// Compute the histogram of H and S channels
	cv::MatND hist = Clusters::computeHSHist(resized_im);

	// Push histogram to vector
	Clusters::hist_array_vector.push_back(hist);

	std::cout << "Hist size: " << hist.size() << std::endl;

	// if there is only one image
	if(Clusters::filenames.size() < 1)
	{
		// Assign the first label to it
		Clusters::labels.push_back(Clusters::number_of_clusters);

		// increase the number_of_clusters
		Clusters::number_of_clusters++;

		// extract the sift feature and push it
		Clusters::feature_array_vector.
		push_back(Clusters::computeDenseSIFT(resized_im, 5));

		// Push filename to the list
    Clusters::filenames.push_back(filename);

		return;
	}

	// Push the initial label to the cluster
	Clusters::labels.push_back(Clusters::number_of_clusters);

  // Extracts the feature array
	cv::Mat feature_array =
	Clusters::computeDenseSIFT(resized_im, Clusters::sift_density);

	// Flag
	int is_similar = 0;

	// hist max
	float hist_max = -99;
	int max_index;

  // iterate over all images and compare the Feature array with them
	for(int i = 0; i < Clusters::filenames.size(); i++)
	{
    std::cout << "Image " << i << std::endl;

		// Compute SIFT score
		float score = cv::norm(feature_array-Clusters::feature_array_vector[i],
		cv::NORM_L1);

		std::cout << "SIFT Score: " << score << std::endl;

		double hist_score = cv::compareHist(hist, hist_array_vector[i],
				0);

		std::cout << "Hist Score " << ": " << hist_score << std::endl;

		if(hist_score > hist_max)
		{
			hist_max = hist_score;
			max_index = i;
		}

		if (hist_max > Clusters::hist_score_thresh)
		{
			// Change the label
			Clusters::labels.back() =
			Clusters::labels[max_index];

			// set the flag
			is_similar = 1;

		}
	}

	if (is_similar == 0)
	{
		// increse the number_of_clusters by 1
		Clusters::number_of_clusters++;
	}

	// Push the feature array
	Clusters::feature_array_vector.push_back(feature_array);

	// Push filename to the list
	Clusters::filenames.push_back(filename);

	// Debug
	std::cout  <<
		"Filename: " << filename << std::endl <<
		"Label: " <<  labels.back() <<
		std::endl;


	return;
}

// Compute sift densely
// im : image
// step_size : spacing between keypoints in number of pixel
cv::Mat Clusters::computeDenseSIFT(cv::Mat im, int step_size)
{
  // SIFT extractor
  cv::SiftFeatureDetector detector;

  // Descriptor storage
  cv::Mat descriptors;

  // Keypoints vector
  std::vector<cv::KeyPoint> keypoints;

  // Define keypoints
  for(int i = 0; i < im.rows-step_size; i+=step_size)
  {
    for(int j = 0; j < im.cols-step_size; j+=step_size)
    {
        keypoints.push_back(cv::KeyPoint(float(j), float(i), float(step_size)));
    }
  }

  // compute the SIFT
  detector.compute(im, keypoints, descriptors);

  // Reshape to make it a vector
  cv::Mat feature_array = descriptors.reshape(0,1);

  return feature_array;
  }

	cv::MatND Clusters::computeHSHist(cv::Mat im)
	{
    cv::MatND hist;

    // Change the color space
		cv::Mat im_hsv;
		cv::cvtColor(im, im_hsv, CV_BGR2HSV);

		// Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };

    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

		// Calculate histogram
    cv::calcHist( &im_hsv, 1, channels, cv::Mat(), // do not use mask
             hist, 2, histSize, ranges,
             true, // the histogram is uniform
             false );

    // Normalize the histograms to 0,1
		cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1 , cv::Mat());

		return hist;
	}

	void Clusters::write2TXT()
	{
    std::cout << std::endl << "Saving to file!" << std::endl;

		std::ofstream myfile;
		myfile.open (Clusters::txtfile_name, std::ios::out|std::ios::app);

		std::cout << "Size: " << Clusters::filenames.size() << std::endl;

		for(int i = 0; i < Clusters::labels.size(); i++)
		{
			std::cout << Clusters::filenames[i] << " "
			<< Clusters::labels[i] << std::endl;
			myfile << Clusters::filenames[i] << " "
			<< Clusters::labels[i] << std::endl;
		}

		myfile.close();

		return;
	}

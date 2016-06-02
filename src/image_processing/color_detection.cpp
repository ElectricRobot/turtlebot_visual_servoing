
#include <color_detection.hpp>

namespace ml
{
	/*!
	* author : Matheus Laranjeira
	* date   : 03/2016
	* 
	* \brief find the non oriented bounding boxe that fits with the larger area contour
	* \param contours, contour set of countour points grouped by connexity (see opencv findContour fonction) 
	* \return bRect, a non oriented rectangle  
	*/
	cv::Rect findBoundingBoxe(const std::vector<std::vector<cv::Point> > &contours)
	{
		cv::Rect bRect(0,0,0,0); 
		double largest_area = 0.0; 
		for( int i = 0; i< contours.size(); i++ )
		{
			double a = contourArea( contours[i],false);  //  Find the area of contour
			if(a>largest_area)
			{
				largest_area = a;
				bRect        = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
			}
		}
		return bRect;
	}


	/*!
	* author : Claire Dune
	* date   : 11/03/2016
	* \brief find the oriented boxe that fits with the largest area contour
	* \param a contour (see findContour)
	* \return an oriented rectangle
	*/
	cv::RotatedRect findOrientedBoxe(const std::vector<std::vector<cv::Point> > &contours)
	{
		/// Find the rotated rectangles the biggest contour
		cv::RotatedRect minRect;
		double largest_area=0.0; 
		for( int i = 0; i < contours.size(); i++ )
		{ 
			double area = cv::contourArea(contours[i],false);
			if( area > largest_area )
			{
				largest_area = area;
				minRect      = cv::minAreaRect( cv::Mat(contours[i]) );
			}
		}
		return minRect;
	}


	/*!
	* author : Matheus Laranjeira
	* date   : 03/2016
	* 
	* \brief detects contours with color defined by Hue = [h_min, h_max]. Some morphological operations to filter the noise
	* \param an opencv image
	* \return a set of closed contours  
	*/
	std::vector<std::vector<cv::Point> > findColoredContour(const cv::Mat imgSrc, /* image source */
														cv::Mat& imgMapColor, /* retour binary image */
														cv::Mat& imgMapMorpho, /* retour binary image after erode/dilate */
														const double& h_min = 5,
														const double& h_max = 30, 
														const double& m_e = 20,
														const double& m_d = 20)
	{
	
		// init the images
		cv::Mat imgHsv;
		imgMapColor  = cv::Mat::zeros( imgSrc.size(), imgSrc.type() );
		imgHsv       = cv::Mat::zeros( imgSrc.size(), imgSrc.type() );
	
		// convertion from color to hsv
		cv::cvtColor(imgSrc, imgHsv, cv::COLOR_BGR2HSV); 
	
		// Select the desired color and build the binary image map
		cv::inRange(imgHsv, cv::Scalar(h_min,0,0), cv::Scalar(h_max,255,255), imgMapColor);
		
		// dilate the detected area and build the map
		cv::erode(imgMapColor , imgMapMorpho , getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_e,m_e)));
		cv::dilate(imgMapMorpho, imgMapMorpho, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_d,m_d)));

		// Find contours of the white area in the image map
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(imgMapMorpho.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		return contours;	
	}


	/*!
	* author : Matheus Laranjeira
	* date   : 05/2016
	* 
	* \brief rotates a given image "src" by "angles" in degrees. Positive angles for clockwise rotation
	* \param  an opencv image and rotation angle in degrees (integer)
	* \return an image rotated by "angles" degrees
	*/
	cv::Mat rotateImg (cv::Mat src, int angle)
	{
		cv::Point2f center(src.cols/2.0F, src.rows/2.0F);
		cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
		cv::Mat dst;
		cv::Rect bbox = cv::RotatedRect(center,src.size(), angle).boundingRect();
		// adjust transformation matrix
		rot.at<double>(0,2) += bbox.width/2.0 - center.x;
		rot.at<double>(1,2) += bbox.height/2.0 - center.y;
		cv::warpAffine(src, dst, rot, bbox.size());
		return dst;
	}

}

#include "color_interesting_points_tableware_alg.h"

ColorInterestingPointsTablewareAlgorithm::ColorInterestingPointsTablewareAlgorithm
    (void)
{
	pthread_mutex_init(&this->access_, NULL);
}

ColorInterestingPointsTablewareAlgorithm::~ColorInterestingPointsTablewareAlgorithm
    (void)
{
	pthread_mutex_destroy(&this->access_);
}

void ColorInterestingPointsTablewareAlgorithm::config_update(Config & new_cfg,
							     uint32_t level)
{
	this->lock();

	// save the current configuration
	this->config_ = new_cfg;

	this->unlock();
}

// ColorInterestingPointsTablewareAlgorithm Public API

/*
  Algorithm to extract all the interest points of an image. This algorithm also draws
  all the interest points in the image publisher to obtain a visual feedback.
  This is the main function of this node
*/
void ColorInterestingPointsTablewareAlgorithm::
extract_interest_points(const sensor_msgs::Image::ConstPtr & rgb_msg,
			const sensor_msgs::PointCloud2::ConstPtr & points_msg,
			sensor_msgs::Image & img_publish,
			std::vector < interest_point > &v)
{

	/// OpenCV bridge for input and output images
	cv_bridge::CvImagePtr cv_ptr =
	    cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
	cv_bridge::CvImagePtr cv_ptr_detected =
	    cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);

	/// Get the original Image
	cv::Mat imgOriginal = cv_ptr->image;

	/// from ROS msg to PLC XYZ
	pcl::PointCloud < pcl::PointXYZ > cloud;
	pcl::fromROSMsg(*points_msg, cloud);

	/// Crop the image to delete some rubbish that appear in the image borders. 
	// Crop from the Origin values to (the original size - the_same_origin_value - Offset)
	// Croping Values in Pixels
	int OriginCropX = 50;
	int OriginCropY = 50;
	int SizeX = imgOriginal.size().width - OriginCropX - OriginCropX;
	int SizeY = imgOriginal.size().height - OriginCropY - OriginCropY;
	//Show the region we crop, in the original image, so add offset size 
	rectangle(imgOriginal, cv::Point(OriginCropX, OriginCropY),
		  cv::Point(SizeX + OriginCropX, SizeY + OriginCropY),
		  cv::Scalar(0), 2, 8, 0);

	/// Known colors that are going to be detected
	/* HUE range --> Color
	   [0]: iLowH = 0
	   [1]: iHighH = 179
	   Stauration range --> like brightness, 0 is white
	   [2]: iLowS = 0 
	   [3]: iHighS =255
	   Value range --> like darkness, 0 always black
	   [4]: iLowV = 0
	   [5]: iHighV =255
	 */
	const int NUM_OF_COLORS=4;
	int MyColors[NUM_OF_COLORS][6] = { 0, 19, 110, 255, 120, 255,	//OrangePlateHSV
		65, 95, 10, 150, 60, 200,						//GreenFIBGlassHSV
		35, 60, 50, 255, 100, 200,						//GreenSFGlassHSV
		100, 140, 90, 255, 0, 150						//BlueCupHSV
	};			

	/// Once the color is detected a similar color is drawn in the pub img to have some feedback 
	int MyColorsBGR[NUM_OF_COLORS][3] = { 0, 0, 255,	//Red
		0, 255, 0,							//Green
		128, 128, 128,						//Gray
		255, 0, 0							//Blue
	};			

	/// Converting image from BRG to HSV for the color detection 
	cv::Mat imgHSV;
	cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

	/// Trying to find a BoundingBox and Centroid for each color
	// var to store BoundingBoxes
	std::vector < cv::Rect > List_BoundingBox(4);
	// var to store Centroides
	std::vector < cv::Point > List_Centroids(4);

	int i = 0;
	while (i < NUM_OF_COLORS) {
		compute_color_BoundingBox(imgHSV, OriginCropX, OriginCropY,	// Image and cropping
					  MyColors[i],			// HSV values
					  List_BoundingBox[i]);	// Return values

		compute_color_centroid(imgHSV, OriginCropX, OriginCropY,	// Image and cropping
				       MyColors[i],			// HSV values
				       List_Centroids[i]);	// Return values
		i++;
	}

	/// Find 3d Position, 4 points per color
	// Find Centroids 3d Position
	i = 0;
	while (i < NUM_OF_COLORS) {

		if (List_BoundingBox[i].width > 0
		    && List_BoundingBox[i].height > 0) {
			//EXPAND BB a little bit
			List_BoundingBox[i].x = List_BoundingBox[i].x - 10;
			List_BoundingBox[i].y = List_BoundingBox[i].y - 10;
			List_BoundingBox[i].width =
			    List_BoundingBox[i].width + 20;
			List_BoundingBox[i].height =
			    List_BoundingBox[i].height + 20;
			get_interest_points_3D(cloud, List_BoundingBox[i], v,
					       i);
		}

		if (List_Centroids[i].x > 0 && List_Centroids[i].y > 0)
			obtain_3D(cloud, List_Centroids[i], v, i);

		i++;
	}

//////Trick to change centroide in plate with red-orange color
//There are some problems due to the range of red and orange Hue
	i = 0;
	while (i < v.size()) {
		if (v[i].id_color == 0 && v[i].type == 'C') {
			v[i].U =
			    List_BoundingBox[0].x +
			    List_BoundingBox[0].width / 2;
			v[i].V =
			    List_BoundingBox[0].y +
			    List_BoundingBox[0].height / 2;
		}
		if (v[i].id_color == 4 && v[i].type == 'C') {
			v[i].U =
			    List_BoundingBox[4].x +
			    List_BoundingBox[4].width / 2;
			v[i].V =
			    List_BoundingBox[4].y +
			    List_BoundingBox[4].height / 2;
		}
		i++;
	}
///////////End of rubish trick

	/// DRAWING STUFF
	// Drawing a BB and number
	i = 0;
	while (i < NUM_OF_COLORS) {
		// If the color is detected we draw the BB 
		if (List_BoundingBox[i].width > 0
		    && List_BoundingBox[i].height > 0) {
			// Draw a Bounding Box
			cv::rectangle(imgOriginal, List_BoundingBox[i].tl(),
				      List_BoundingBox[i].br(),
				      cv::Scalar(MyColorsBGR[i][0],
						 MyColorsBGR[i][1],
						 MyColorsBGR[i][2]), 2, 8, 0);
			// Drawing number of color
			char text[10];
			std::sprintf(text, " #:%d", i);
			cv::putText(imgOriginal, text, List_BoundingBox[i].br(),
				    cv::FONT_HERSHEY_SIMPLEX, 0.5,
				    cv::Scalar(MyColorsBGR[i][0],
					       MyColorsBGR[i][1],
					       MyColorsBGR[i][2]), 2, 8);
		}

		i++;
	}

	// Drawing a circle in the centroids and centroids grasping position U V, BGR
	i = 0;
	while (i < v.size()) {
		if (v[i].type == 'C')	//Centroide
			cv::circle(imgOriginal, cv::Point(v[i].U, v[i].V), 2,
				   cv::Scalar(MyColorsBGR[v[i].id_color][0],
					      MyColorsBGR[v[i].id_color][1],
					      MyColorsBGR[v[i].id_color][2]), 2,
				   8, 0);
		else		//Grasp Point
			cv::circle(imgOriginal, cv::Point(v[i].U, v[i].V), 10,
				   cv::Scalar(MyColorsBGR[v[i].id_color][0],
					      MyColorsBGR[v[i].id_color][1],
					      MyColorsBGR[v[i].id_color][2]), 2,
				   8, 0);

		i++;
	}

	/// From OpenCV to ROS Image msg. img_publish is the image message to publish
	cv_ptr_detected->image = imgOriginal;
	copy_image(cv_ptr_detected->toImageMsg(), img_publish);

}

/* 
  Algorithm that computes the Bounding Box of a color in HSV range  
*/
void ColorInterestingPointsTablewareAlgorithm::
compute_color_BoundingBox(const cv::Mat & imgHSV, int OriginCropX,
			  int OriginCropY, int RangeHSV[6],
			  cv::Rect & BoundingBox)
{

	/// Threshold the image
	cv::Mat imgDetect;	//B&W image where the color detected is white
	cv::Mat imgCropped;	//B&W image, color detected and cropped
	cv::Mat imgThresholded;	//B&W image where the centroids are computed
	//                   (iLowH, iLowS, iLowV), (iHighH, iHighS, iHighV) 
	cv::inRange(imgHSV, cv::Scalar(RangeHSV[0], RangeHSV[2], RangeHSV[4]),
		    cv::Scalar(RangeHSV[1], RangeHSV[3], RangeHSV[5]),
		    imgDetect);

	/// Crop the image, from the Origin to (the original size - the same origin value - Offset)
	int SizeX = imgDetect.size().width - OriginCropX - OriginCropX;
	int SizeY = imgDetect.size().height - OriginCropY - OriginCropY;
	cv::Rect extractROI(OriginCropX, OriginCropY, SizeX, SizeY);	//originX, originY  width, height
	cv::Mat(imgDetect, extractROI).copyTo(imgCropped);

	/// morphological opening (remove small objects from the foreground)
	cv::erode(imgCropped, imgThresholded,
		  getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded,
		   getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	/// morphological closing (fill small holes in the foreground)
	cv::dilate(imgThresholded, imgThresholded,
		   getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::erode(imgThresholded, imgThresholded,
		  getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	/// Calc Bounding Box
	// Once we have a B&W image we extract the contours to obtain a vector of points.
	std::vector < std::vector < cv::Point > >contours;
	std::vector < cv::Vec4i > hierarchy;

	// Find contours
	findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE,
		     CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// Merging diferent lists of points
	std::vector < cv::Point > My_contours;
	int i = 0;
	while (i < contours.size()) {
		int j = 0;
		while (j < contours[i].size()) {
			My_contours.push_back(contours[i][j]);
			j++;
		}
		i++;
	}
	if (My_contours.size() > 0) {
		// With a vector of points, we can obtain the bounding box
		//BoundingBox=boundingRect( cv::Mat(contours[0]) );
		BoundingBox = boundingRect(cv::Mat(My_contours));

		//The image has been cropped, here we add that part
		BoundingBox.x += OriginCropX;
		BoundingBox.y += OriginCropY;
	}

}

/* 
  Algorithm that computes the centroid of a color in HSV range  
*/
void ColorInterestingPointsTablewareAlgorithm::
compute_color_centroid(const cv::Mat & imgHSV, int OriginCropX, int OriginCropY,
		       int RangeHSV[6], cv::Point & Centroid)
{

	/// Threshold the image
	cv::Mat imgDetect;	//B&W image where the color detected is white
	cv::Mat imgCropped;	//B&W image, color detected and cropped
	cv::Mat imgThresholded;	//B&W image where the centroids are computed
	//                   (iLowH, iLowS, iLowV), (iHighH, iHighS, iHighV) 
	cv::inRange(imgHSV, cv::Scalar(RangeHSV[0], RangeHSV[2], RangeHSV[4]),
		    cv::Scalar(RangeHSV[1], RangeHSV[3], RangeHSV[5]),
		    imgDetect);

	/// Crop the image, from the Origin to (the original size - the same origin value - Offset)
	int SizeX = imgDetect.size().width - OriginCropX - OriginCropX;
	int SizeY = imgDetect.size().height - OriginCropY - OriginCropY;
	cv::Rect extractROI(OriginCropX, OriginCropY, SizeX, SizeY);	//originX, originY  width, height
	cv::Mat(imgDetect, extractROI).copyTo(imgCropped);

	/// morphological opening (remove small objects from the foreground)
	cv::erode(imgCropped, imgThresholded,
		  getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded,
		   getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	/// morphological closing (fill small holes in the foreground)
	cv::dilate(imgThresholded, imgThresholded,
		   getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::erode(imgThresholded, imgThresholded,
		  getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	/// Calc moments of the image to find centroid. 
	cv::Moments ObjMoments = cv::moments(imgThresholded);
	//Position of the cropped image, so offset added
	Centroid.x = (int)(ObjMoments.m10 / ObjMoments.m00 + OriginCropX);	//U 
	Centroid.y = (int)(ObjMoments.m01 / ObjMoments.m00 + OriginCropY);	//V

}

/*
  Algorithm that given a Bounding Box and a point cloud returns some interested points
  to grasp tableware. In this case we are going to obtain 4 IP, one for each quadrant
*/
void ColorInterestingPointsTablewareAlgorithm::
get_interest_points_3D(const pcl::PointCloud < pcl::PointXYZ > &cloud,
		       const cv::Rect & BoundingBox,
		       std::vector < interest_point > &v, const int color)
{

	/// Checking the entrance, just in case..
	if (BoundingBox.width == 0)
		return;
	if (BoundingBox.height == 0)
		return;

	int i = 0;		//U iterator
	int j = 0;		//V iterator
	//Nearest robot point: U=points_msg->width/2, V=0

	/// 2nd Q
	i = BoundingBox.x;
	interest_point IP_Q1;
	if (color == 0)
		IP_Q1.type = 'p';	//plate
	else
		IP_Q1.type = 'g';	//glass
	IP_Q1.Z = 999999;
	IP_Q1.id_color = color;

	while (i < BoundingBox.x + BoundingBox.width / 2) {
		j = BoundingBox.y;	//V iterator    
		while (j < BoundingBox.y + BoundingBox.height / 2) {

			if (cloud(i, j).z < IP_Q1.Z) {
				// if new z is taller 
				IP_Q1.U = i;
				IP_Q1.V = j;
				IP_Q1.X = cloud(i, j).x;
				IP_Q1.Y = cloud(i, j).y;
				IP_Q1.Z = cloud(i, j).z;
			}

			j++;
		}
		i++;
	}

	/// 1st Q
	i = BoundingBox.x + BoundingBox.width / 2 + 1;	//U iterator
	interest_point IP_Q2;
	if (color == 0)
		IP_Q2.type = 'p';	//plate
	else
		IP_Q2.type = 'g';	//glass
	IP_Q2.Z = 999999;
	IP_Q2.id_color = color;

	while (i < BoundingBox.x + BoundingBox.width) {
		j = BoundingBox.y;	//V iterator
		while (j < BoundingBox.y + BoundingBox.height / 2) {

			if (cloud(i, j).z < IP_Q2.Z) {
				// if new z is taller
				IP_Q2.U = i;
				IP_Q2.V = j;
				IP_Q2.X = cloud(i, j).x;
				IP_Q2.Y = cloud(i, j).y;
				IP_Q2.Z = cloud(i, j).z;
			}

			j++;
		}
		i++;
	}

	/// 3rd Q
	i = BoundingBox.x;	//U iterator
	interest_point IP_Q3;
	if (color == 0)
		IP_Q3.type = 'p';	//plate
	else
		IP_Q3.type = 'g';	//glass
	IP_Q3.Z = 999999;
	IP_Q3.id_color = color;

	while (i < BoundingBox.x + BoundingBox.width / 2) {
		j = BoundingBox.y + BoundingBox.height / 2 + 1;	//V iterator
		while (j < BoundingBox.y + BoundingBox.height) {

			if (cloud(i, j).z < IP_Q3.Z) {
				// if new z is taller 
				IP_Q3.U = i;
				IP_Q3.V = j;
				IP_Q3.X = cloud(i, j).x;
				IP_Q3.Y = cloud(i, j).y;
				IP_Q3.Z = cloud(i, j).z;
			}

			j++;
		}
		i++;
	}

	/// 4Q
	i = BoundingBox.x + BoundingBox.width / 2 + 1;	//U iterator
	interest_point IP_Q4;
	if (color == 0)
		IP_Q4.type = 'p';	//plate
	else
		IP_Q4.type = 'g';	//glass
	IP_Q4.Z = 999999;
	IP_Q4.id_color = color;

	while (i < BoundingBox.x + BoundingBox.width) {
		j = BoundingBox.y + BoundingBox.height / 2 + 1;	//V iterator
		while (j < BoundingBox.y + BoundingBox.height) {

			if (cloud(i, j).z < IP_Q4.Z) {
				// if new z is taller 
				IP_Q4.U = i;
				IP_Q4.V = j;
				IP_Q4.X = cloud(i, j).x;
				IP_Q4.Y = cloud(i, j).y;
				IP_Q4.Z = cloud(i, j).z;
			}

			j++;
		}
		i++;
	}

	/// Insert IP in the vector 
	v.push_back(IP_Q1);
	v.push_back(IP_Q2);
	v.push_back(IP_Q3);
	v.push_back(IP_Q4);

}

/* 
  Algorithm that uses PointCloud2 to obtain the Z axis (3D) form a list of 2D positions
*/
void ColorInterestingPointsTablewareAlgorithm::obtain_3D(const pcl::PointCloud <
							 pcl::PointXYZ > &cloud,
							 const cv::Point &
							 Centroid,
							 std::vector <
							 interest_point > &v,
							 const int color)
{

	// Usings pcl::PointXYZRGB points to calc 3D coordinates
	if (Centroid.x > 0 && Centroid.y > 0) {

		interest_point IPoint;
		IPoint.U = Centroid.x;
		IPoint.V = Centroid.y;
		IPoint.X = cloud(Centroid.x, Centroid.y).x;
		IPoint.Y = cloud(Centroid.x, Centroid.y).y;
		IPoint.Z = cloud(Centroid.x, Centroid.y).z;
		IPoint.type = 'C';	//Centroid
		IPoint.id_color = color;

		/// Insert IP in the vector 
		v.push_back(IPoint);

	}

}

/* 
  Algorithm that copies an image from a subscriber message format to a publisher message format
*/
void ColorInterestingPointsTablewareAlgorithm::
copy_image(const sensor_msgs::Image::ConstPtr & msg,
	   sensor_msgs::Image & cp_img)
{

	cp_img.data = msg->data;
	cp_img.height = msg->height;
	cp_img.width = msg->width;
	cp_img.encoding = msg->encoding;
	cp_img.is_bigendian = msg->is_bigendian;
	cp_img.step = msg->step;
	cp_img.header = msg->header;

	//ROS_INFO("[COPY_IMAGE]: OK");

}

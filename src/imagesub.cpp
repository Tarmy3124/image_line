#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgcodecs/imgcodecs.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>
#include<math.h>
#include<vector>
using namespace std;  
using namespace cv;  
//opencv官方参考
//https://docs.opencv.org/3.0.0/d8/dd4/lsd_lines_8cpp-example.html
void imageCalllback(const sensor_msgs::ImageConstPtr& msg)
{
        bool useRefine = false;  //是否平滑精细化处理
        bool useCanny = false; //不使用canny  
	ROS_INFO("Received \n");
        cv::Mat m_color_image;
	try{
        //cv::imshow( "video", cv_bridge::toCvShare(msg, "bgr8")->image );
        m_color_image = cv_bridge::toCvShare(msg, "mono8")->image;  // single gray
        cv::waitKey(30);
        if( m_color_image.empty() )  
         {  
           cout << "Unable to load Image" << endl;  
           return ;  
         } 
        if (useCanny)  
        Canny(m_color_image, m_color_image, 50, 200, 3);  
  
        Ptr<LineSegmentDetector> ls = useRefine ? createLineSegmentDetector(LSD_REFINE_STD) : createLineSegmentDetector(LSD_REFINE_NONE);  
  
        vector<Vec4f> lines_std;  

        ls->detect(m_color_image, lines_std);  
         /*param
        1, image：A grayscale (CV_8UC1) input image. If only a roi needs to be selected, use: lsd_ptr->detect(image(roi), lines, …); lines += Scalar(roi.x, roi.y, roi.x, roi.y);
        2, lines：A vector of Vec4i or Vec4f elements specifying the beginning and ending point of a line. Where Vec4i/Vec4f is (x1, y1, x2, y2), point 1 is the start, point 2 - end. Returned lines are strictly oriented depending on the gradient.
       */
        // Show found lines  
        if (useCanny)  
        m_color_image = Scalar(0, 0, 255);  
  
        ls->drawSegments(m_color_image, lines_std);   //lines_std为上步直线检测的结果
  
        imshow("video", m_color_image);  
    }
    catch( cv_bridge::Exception& e )
    {
        ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str() );
    }
}

int main(int argc, char** argv)
{


	ros::init(argc, argv, "image_listener");
	ros::NodeHandle n;
    cv::namedWindow("video");
    cv::startWindowThread();

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe( "/camera/color/image_raw", 1, imageCalllback );

	
	ros::spin();
    cv::destroyWindow("video");
	return 0;
}


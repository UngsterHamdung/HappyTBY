```c++

```







```c++

```





```c++
#include <ros/ros.h> 
#include <sensor_msgs/Image.h> 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h> 
#include "geometry_msgs/Twist.h" 



using namespace cv; 

using namespace std; 

geometry_msgs::Twist msg; 

Mat Draw_Guide_Line(Mat img)
{
    /*Mat result_img;*/
    img.copyTo(img);

    line(img, Point(0, 420), Point(640, 420), Scalar(0, 255, 255), 1, 0);
   


    return img;

}

Mat Region_of_Interest_crop_L(Mat image, Point* points)
{
    Mat img_roi_crop;

    Rect bounds(0, 0, image.cols/2, image.rows);
    Rect r(points[0].x, points[0].y, image.cols, points[2].y - points[0].y);


    img_roi_crop = image(r & bounds);

    return img_roi_crop;
}

Mat Region_of_Interest_crop(Mat image, Point* points)
{
    Mat img_roi_crop;

    Rect bounds(0, 0, image.cols, image.rows);
    Rect r(points[0].x, points[0].y, image.cols, points[2].y - points[0].y);


    img_roi_crop = image(r & bounds);

    return img_roi_crop;
}
Mat Canny_Edge_Detection(Mat img)
{
    Mat mat_blur_img, mat_canny_img;
    blur(img, mat_blur_img, Size(3, 3));
    Canny(mat_blur_img, mat_canny_img, 70, 150, 3);

    return mat_canny_img;
}




void ImageCallbak(const sensor_msgs::Image::ConstPtr &img) 
{
  double spdCurve = 0; 
  cv_bridge::CvImagePtr cv_ptr; 

  ROS_INFO("Image(%d, %d)", img->width, img->height);

  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Error to convert!");
    return;
  }
 
  Mat mat_image_org_color = cv_ptr->image; 
  Mat mat_image_roi, mat_image_canny_edge,mat_image_org_gray, mat_image_roiL, mat_image_roiR; 
 
  
  int count = 0;
    int line_count = 0;
    int circle_L = 0;
    int circle_R = 0;
    float  c[NO_LINE] = { 0.0, };
    float  d[NO_LINE] = { 0.0, };
    float  line_center_x = 0.0;
    float inter_sect_x[NO_LINE] = { 0, };
    double cnt = 0;
    double sum = 0;
    points[0] = Point(0, 360);
    points[1] = Point(0, 480);
    points[2] = Point(640, 480);
    points[3] = Point(640, 360);

    points_L[0] = Point(0, 360);
    points_L[1] = Point(0, 480);
    points_L[2] = Point(320, 480);
    points_L[3] = Point(320, 360);

    points_R[0] = Point(320, 360);
    points_R[1] = Point(320, 480);
    points_R[2] = Point(640, 480);
    points_R[3] = Point(640, 360);  
  cvtColor(mat_image_org_color, mat_image_org_gray, cv::COLOR_RGB2GRAY);       
  mat_image_roi = Region_of_Interest_crop(mat_image_org_gray, points);       
  mat_image_canny_edge = Canny_Edge_Detection(mat_image_roi);


  vector<Vec4i> linesP;
  HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI / 180, 30, 15, 10);

    for (int i = 0; i < linesP.size(); i++)
                  {
                      float* intersect = new float[20];
                      if (i >= NO_LINE) break;
                      Vec4i L = linesP[i];
                      /*
                      int cx1 = linesP[i][0];
                      int cy1 = linesP[i][1];
                      int cx2 = linesP[i][2];
                      int cy2 = linesP[i][3];
                      */
                      c[i] = ((float)L[2] - (float)L[0]) / ((float)L[3] - (float)L[1]); // gradient

                      d[i] = (float)L[0] - c[i] * (float)L[1]; // intersect
                      if (fabs(c[i]) < DEG2RAD(65))
                      {

                          intersect[i] = c[i] * (float)ASSIST_BASE_WIDTH + d[i];
                         

                          line(mat_image_org_color, Point(L[0], L[1] + ASSIST_BASE_LINE - ASSIST_BASE_WIDTH), Point(L[2], L[3] + ASSIST_BASE_LINE - ASSIST_BASE_WIDTH), Scalar(0, 0, 255), 2, LINE_AA);
                          circle(mat_image_org_color, Point(intersect[i], ASSIST_BASE_LINE), 20, Scalar(255, 0, 0), -1);
                      }
 
    



            //    }
            Draw_Guide_Line(mat_image_org_color);
            //    line(mat_image_org_color, Point(0, 520), Point(640, 520), Scalar(0, 255, 255), 3, 0);
           
            mat_image_roiR = Region_of_Interest_crop(mat_image_org_color, points_R);
            mat_image_roiL = Region_of_Interest_crop_L(mat_image_org_color, points_L);
  
            
            vector<Vec3f> circles_R, circles_L;
            cvtColor(mat_image_roiR, mat_image_roiR, cv::COLOR_RGB2GRAY);
            cvtColor(mat_image_roiL, mat_image_roiL, cv::COLOR_RGB2GRAY);
            HoughCircles(mat_image_roiR, circles_R, HOUGH_GRADIENT, 1, 100, 50, 35, 0, 0);
            circle_R = circles_R.size();
            HoughCircles(mat_image_roiL, circles_L, HOUGH_GRADIENT, 1, 100, 50, 35, 0, 0);
            circle_L = circles_L.size();

            if (circle_R > circle_L)
            {
                printf("\ngo right\n");
                msg.angular.z = -1;
                
            }
            else if (circle_L > circle_R)
            {
                printf("\ngo Left\n");
                msg.angular.z = +1;
            }
            else
            {
                printf("\nAnyway\n");
                
            }
            imshow("original_image", mat_image_org_color);
            imshow("R_image", mat_image_roiR); imshow("L_image", mat_image_roiL);
            imwrite("mola.jpg", mat_image_org_color);


            if (waitKey(25) == 27)
                break;




        }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "overtaking"); 
  ros::NodeHandle nh; 

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("/main_camera/image-raw", 1, ImageCallbak);

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Rate rate(10);.
  ROS_INFO("Starting to move forward"); 
  while (ros::ok()) { 
          pub.publish(msg); 
          ros::spinOnce(); 
          rate.sleep();
          }
}
```


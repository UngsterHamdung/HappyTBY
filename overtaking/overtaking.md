```c++

```

이거 잘못됐어요




```c++

```





```c++
#include <ros/ros.h> 
#include <sensor_msgs/Image.h> 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h> 
#include "geometry_msgs/Twist.h" 
#define ASSIST_BASE_LINE 390 
#define ASSIST_BASE_WIDTH 90


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
  float intersect_R[20] = { 0, };
  float intersect_L[20] = { 0, };
  float intersect_R1[20] = { 0, };
  float intersect_L1[20] = { 0, };
  Mat mat_image_org_gray_L, mat_image_org_gray_R, mat_image_canny_edge_R, mat_image_canny_edge_L, mat_image_org_color, mat_image_org_gray, mat_image_roi, mat_image_roiL,   mat_image_roiR, mat_image_canny_edge; 
 
  
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
  mat_image_roiR = Region_of_Interest_crop(mat_image_org_color, points_R);
        cvtColor(mat_image_roiR, mat_image_org_gray_R, cv::COLOR_RGB2GRAY);// ROI 영역을 추출함      
        mat_image_canny_edge_R = Canny_Edge_Detection(mat_image_org_gray_R);

        mat_image_roiL = Region_of_Interest_crop_L(mat_image_org_color, points_L);    // ROI 영역을 추출함      
        cvtColor(mat_image_roiL, mat_image_org_gray_L, cv::COLOR_RGB2GRAY);
        mat_image_canny_edge_L = Canny_Edge_Detection(mat_image_org_gray_L);
        vector<Vec4i> linesP_L, linesP_R;
        HoughLinesP(mat_image_canny_edge_R, linesP_R, 1, CV_PI / 180, 30, 15, 10);
        HoughLinesP(mat_image_canny_edge_L, linesP_L, 1, CV_PI / 180, 30, 15, 10);

        //printf("Line Number : %3d\n", (int)linesP.size());
        //mat_image_roiR = Region_of_Interest_crop(mat_image_org_color, points_R);
        //mat_image_roiL = Region_of_Interest_crop_L(mat_image_org_color, points_L);
        for (int i = 0; i < linesP_L.size(); i++)
        {
           
            if (i >= NO_LINE) break;
            Vec4i LL = linesP_L[i];
            /*
            int cx1 = linesP[i][0];
            int cy1 = linesP[i][1];
            int cx2 = linesP[i][2];
            int cy2 = linesP[i][3];
            */
            //printf("드감?");
            c_L[i] = ((float)LL[2] - (float)LL[0]) / ((float)LL[3] - (float)LL[1]); // gradient

            d_L[i] = (float)LL[0] - c_L[i] * (float)LL[1]; // intersect
            if (fabs(c_L[i]) < DEG2RAD(65))
            {

                intersect_L[i] = c_L[i] * (float)10 + d_L[i];
                intersect_L1[i] = c_L[i] * (float)110 + d_L[i];
                //intersect_L[i] = c[i] * (float)ASSIST_BASE_WIDTH + d[i];

                if (intersect_L)
                {
                    circle_L++;
                }
                if (intersect_L1)
                {
                    circle_L++;
                }
                line(mat_image_org_color, Point(LL[0], LL[1] + 300), Point(LL[2], LL[3] + 300), Scalar(0, 0, 255), 2, LINE_AA);
                circle(mat_image_org_color, Point(intersect_L1[i], 420), 10, Scalar(255, 0, 0), -1);
                circle(mat_image_org_color, Point(intersect_L[i], 310), 10, Scalar(255, 0, 0), -1);

            }

        }
        for (int j = 0; j < linesP_R.size(); j++)
        {

            if (j >= NO_LINE) break;
            Vec4i L = linesP_R[j];
            /*
            int cx1 = linesP[i][0];
            int cy1 = linesP[i][1];
            int cx2 = linesP[i][2];
            int cy2 = linesP[i][3];
            */
            //printf("드감?");
            c_R[j] = ((float)L[2] - (float)L[0]) / ((float)L[3] - (float)L[1]); // gradient

            d_R[j] = (float)L[0] - c_R[j] * (float)L[1]; // intersect
            if (fabs(c_R[j]) < DEG2RAD(65))
            {
                //printf("\nin there??\n");
                intersect_R[j] = c_R[j] * (float)10 + d_R[j];
                intersect_R1[j] = c_R[j] * (float)110 + d_R[j];
                //intersect_L[i] = c[i] * (float)ASSIST_BASE_WIDTH + d[i];
                if (intersect_R)
                {
                    circle_R++;
                }
                if (intersect_R1)
                {
                    circle_R++;
                }
                
                line(mat_image_org_color, Point(320+L[0], L[1] + 300), Point(320+L[2], L[3] + 300), Scalar(0, 0, 255), 2, LINE_AA);
                circle(mat_image_org_color, Point(320+intersect_R[j], 310), 10, Scalar(255, 0, 0), -1);
                circle(mat_image_org_color, Point(320+intersect_R1[j], 420), 10, Scalar(255, 0, 0), -1);

            }





        }
            Draw_Guide_Line(mat_image_org_color);
            //    line(mat_image_org_color, Point(0, 520), Point(640, 520), Scalar(0, 255, 255), 3, 0);



            /*vector<Vec3f> circles_R, circles_L;
            cvtColor(mat_image_roiR, mat_image_roiR, cv::COLOR_RGB2GRAY);
            cvtColor(mat_image_roiL, mat_image_roiL, cv::COLOR_RGB2GRAY);
            HoughCircles(mat_image_roiR, circles_R, HOUGH_GRADIENT, 1, mat_image_roiR.rows/6, 50, 35, 0, 0);
            circle_R = circles_R.size();
            HoughCircles(mat_image_roiL, circles_L, HOUGH_GRADIENT, 1, 100, 50, 35, 0, 0);
            circle_L = circles_L.size();
            */


            //imshow("original_image", mat_image_org_color);
            //imshow("R_image", mat_image_roiR); imshow("L_image", mat_image_roiL);
            //imwrite("mola.jpg", mat_image_org_color);
            //imwrite("mola1.jpg", mat_image_roiR);


            /*if (waitKey(25) == 27)
                break;*/
            imshow("ori", mat_image_org_color);

            
            

        }

        printf("count : L:%d, R:%d ", circle_L, circle_R);
        if (circle_R > circle_L)
        {
            printf("\ngo Left\n");
            msg.angular.z=-1;
        }
        else if (circle_L > circle_R)
        {
            printf("\ngo Right\n");
            msg.angular.z=1;
        }
        else
        {
            printf("\nAnyway\n");
        }
        circle_L = 0;
        circle_R = 0;;


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


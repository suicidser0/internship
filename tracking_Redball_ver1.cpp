#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O


using namespace std;
using namespace cv;

class ColorDetector {
private :
  int minDist;
  cv::Vec3b target;
  cv::Mat result;

public:
  ColorDetector() : minDist(100) {
    target[0] = target[1] = target[2] = 0;
  }

  void setColorDistanceThreshold(int distance) {
    if(distance < 0)
      distance = 0;
    minDist = distance;
  }

  int getColorDistanceThreshold() const {
    return minDist;
  }

  void setTargetColor(unsigned char red, unsigned char green, unsigned char blue) {
    target[2] = red;
    target[1] = green;
    target[0] = blue;
  }
  
  void setTargetColor(cv::Vec3b color) {
    target = color;
  }

  int getDistance(const cv::Vec3b& color) const {
    return abs(color[0]-target[0])+abs(color[1]-target[1])+abs(color[2]-target[2]);
  }

  cv::Vec3b getTargetColor() const {
    return target;
  }
  cv::Mat process(const cv::Mat &image) {
    result.create(image.rows, image.cols, CV_8U);
    cv::Mat_<cv::Vec3b>::const_iterator it= image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend= image.end<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itout = result.begin<uchar>();

    for(; it!=itend; ++it, ++itout) {
      if(getDistance(*it)<minDist) {
        *itout = 255;
      } else {
        *itout = 0;
      }
    }
    return result;
  }

};
/**
 * function main
 *
 */

int main(int argc, char *argv[], char *window_name)
{

  ColorDetector cdetect;
  cv::Mat image = cv::imread("lena.jpg");
  cv::VideoCapture cap(0);
  cv::Mat frame;
  cv::Mat result;
  cv::Mat draw_line;
  cv::Mat hsv;

  if(!cap.isOpened())  // check if we succeeded
      return -1;

  for(;;) {
    cap >> frame;

    cdetect.setTargetColor(190,20,20);
    result = cdetect.process(frame);

    cv::cvtColor(result, hsv, CV_BGR2HSV);

    cv::Moments mom = cv::moments(result);
    cv::circle(frame, cv::Point(mom.m10/mom.m00, mom.m01/mom.m00), 15, cv::Scalar(0.8, 0.2, 0.2), 2);
    std::cout << "position : " << mom.m10/mom.m00 << ", " << mom.m01/mom.m00 << std::endl;
    static int posX = 0;
    static int posY = 0;

    int lastX = posX;
    int lastY = posY;

    posX = mom.m10/mom.m00;
    posY = mom.m01/mom.m00;

    if(lastX>0 && lastY>0 && posX>0 && posY>0) {
      cv::line(frame, cv::Point(posX, posY), cv::Point(lastX, lastY), cv::Scalar(0.8,0.2,0.2),5);
    }
    //cv::add(frame, draw_line, frame);
    cv::imshow("original",frame);
    cv::imshow("result", result);
    cv::imshow("test", hsv);

    cv::waitKey(3);
  }

  return 0;

}

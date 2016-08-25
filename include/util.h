

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libfreenect2/frame_listener.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// using namespace cv;


// cv::Mat createChessboard(int columns, int rows);
static const double PI = 3.1415;

int doSomething(int s);

void showMat(cv::Mat image);

std::string intToStr(int n);

cv::Mat frameToMat(std::string type, libfreenect2::Frame *frame);




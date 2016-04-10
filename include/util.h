

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libfreenect2/frame_listener.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// using namespace cv;


// cv::Mat createChessboard(int columns, int rows);


int doSomething(int s);

void showMat(cv::Mat image);

cv::Mat frameToMat(std::string type, libfreenect2::Frame *frame);




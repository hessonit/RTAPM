#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

class Intrinsics {
	public:
		void	setup(Mat cameraMatrix, Size imageSize, Size sensorSize = Size(0, 0));
		Mat		getCameraMatrix() const;
		Size	getImageSize() const;
		Size	getSensorSize() const;
		Point2d getFov() const;
		double	getFocalLength() const;
		double	getAspectRatio() const;
		Point2d getPrincipalPoint() const;
        
         
	protected:
		Mat		cameraMatrix;
		Size	imageSize; 
        Size	sensorSize;
		Point2d fov;
		double	focalLength, aspectRatio;
		Point2d principalPoint;
	};











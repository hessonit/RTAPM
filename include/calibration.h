#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

/// Calibration class
class Calibration
{
public:
	Calibration();
	void					 readSettings(std::string path);
	void					 saveSettings(std::string path);

	cv::Mat					 createChessboard(int cx, int cy, int blockSize);
	std::pair<int, int>		 projectChessbooard(int blockSize = 25, int x = -1, int y = -1);
	bool					 detectChessboard(cv::Mat image, std::vector<cv::Point2f> output);
	
	void					 setKinect(libfreenect2::SyncMultiFrameListener *listener);
	void					 collectPoints(libfreenect2::Freenect2Device *dev);
	void					 calibrate();
	void					 calibrate2();
	
	void					 projectFindResult(bool flag);
	void					 projectFindResult(bool flag, std::vector<cv::Point2f> points);
	
	bool					 calibrationEnded();
	void					 setProjectorResolution(int rx, int ry);
	
	std::vector<cv::Vec2f>	 project(std::vector<cv::Point3f> wrldSrc, int i);
	std::vector<cv::Point2f> projectPoints2(std::vector<cv::Point3f> wrldSrc);

	void					 printCalibration();
 
	int						 projectorResolutionX, projectorResolutionY;
	std::vector<cv::Mat>	 boardRotations, boardTranslations;
	cv::Mat					 cameraMatrix, distCoeffs;
private:
	int										numberOfIterations = 5;
	bool									calibrationEnd;
	libfreenect2::SyncMultiFrameListener*	_listener;
	cv::Mat									_chessBoard;
	std::vector<cv::Point2f>				internalPoints;
	std::vector<std::vector <cv::Point3f> >	worldCoordinatesChessboardBuffer;
	std::vector<std::vector <cv::Point2f> >	kinectCoordinatesChessboardBuffer;
	std::vector<std::vector <cv::Point2f> >	imageCoordinatesChessboardBuffer;
	
	
	std::vector<double> x;
	// cv::Settings s; // TODO read&save calibration

};
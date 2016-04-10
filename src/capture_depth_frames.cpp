#include "libfreenect2/libfreenect2.hpp"
#include "libfreenect2/frame_listener_impl.h"
#include "libfreenect2/registration.h"
#include <libfreenect2/logger.h>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include <GLFW/glfw3.h>
using namespace cv;

int main(int argc, char ** argv) {
	int frames_to_capture = 30;
	if (argc >= 2)
		frames_to_capture = std::stoi(argv[1]);

	libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));

	libfreenect2::Freenect2 freenect2;
	libfreenect2::PacketPipeline *pipeline = new libfreenect2::OpenGLPacketPipeline;
	libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice(pipeline);

	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
	libfreenect2::FrameMap frames;

	dev->setIrAndDepthFrameListener(&listener);

	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

	Mat frame;

	namedWindow("FaceTrack", CV_WINDOW_NORMAL);
	moveWindow("FaceTrack", 0, 0);
	setWindowProperty("FaceTrack", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	int guard = 0;
	while (guard < 20) {
		guard++;
		listener.waitForNewFrame(frames);
		 
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		Mat image_depth((int)depth->height, (int)depth->width, CV_32F, depth->data);
		imshow("FaceTrack", image_depth);
		int op = waitKey(100);
		// if(op != -1)
		// std::cout<<"OP: "<<(char)(op)<<" "<<op<<"\n";

		if (op == 1113997 || op == 1048586 || op == 1048608 || op == 10 || op == 32)
		{
			//shutdown = true;
			destroyWindow("FaceTrack");
			break;
		}
		listener.release(frames);
	}


	/*int frame_id = 0;

    cv::FileStorage fs("depth.yml", cv::FileStorage::WRITE);
    fs << "frame_count" << frames_to_capture;
    fs << "frames" << "[";
	while (frame_id < frames_to_capture) {
		listener.waitForNewFrame(frames);

		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		
		cv::Mat image_depth(depth->height, depth->width, CV_32F, depth->data);

		fs <<  image_depth;
		std::cout << "saved frame " << frame_id << "\n";
		
		listener.release(frames);
		frame_id++;
	}
	fs << "]";*/

	dev->stop();
	dev->close();

	return 0;
}
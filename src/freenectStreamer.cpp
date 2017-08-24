
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
#include <Ole2.h>
#include <Kinect.h>

#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <fstream>
#include <cstdlib>
#include <chrono>

#include <asio.hpp>
#include <turbojpeg.h>
#include "IMPRESS_UDPClient.cpp"


#define STB_DXT_IMPLEMENTATION
#include "stb_dxt.h"

#define depthWidth 512
#define depthHeight 424
#define colorwidth 1920
#define colorheight 1080

using namespace std::chrono;
using asio::ip::udp;

bool stream_shutdown = false;
void sigint_handler(int s) {
	stream_shutdown = true;
}

IKinectSensor* sensor;         // Kinect sensor
IMultiSourceFrameReader* reader;   // Kinect data source
ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates

asio::io_service io_service;
IMPRESS_UDPClient client(io_service);

ColorSpacePoint depth2rgb[depthWidth*depthHeight];     // Maps depth pixels to rgb pixels
unsigned char rgbimage[colorwidth*colorheight * 4];    // Stores RGB color image
unsigned char mappedRGBImage[depthWidth*depthHeight * 3];    // Stores RGB color image

int streamFrame(UINT16 *depthData, unsigned char *RGBImage, uint32_t sequence);
int openAndStream(std::string serial, std::string pipelineId);
int sendConfig();

struct DEPTH_DATA_HEADER {
	unsigned char msgType = 0x03;   // 00 // 0x03 = Depth; 0x04 = Color
	unsigned char deviceID = 0x00;  // 01
	unsigned char unused1 = 0x00;   // 02 => for consistent alignment
	unsigned char unused2 = 0x00;   // 03 => for consistent alignment
	uint32_t sequence = 0;          // 04-07
	unsigned short startRow = 0;    // 08-09
	unsigned short endRow = 0;      // 10-11
};

struct CONFIG_MSG {
	unsigned char msgType = 0x01;    // 00
	unsigned char deviceID = 0x00;   // 01
	unsigned char deviceType = 0x02; // 02
	unsigned char unused1 = 0x00;    // 03 => for consistent alignment
	unsigned short frameWidth = 0;   // 04-05
	unsigned short frameHeight = 0;  // 06-07
	unsigned short maxLines = 0;     // 08-09
	unsigned short unused2 = 0;      // 10-11 => for consistent alignment
	float Cx = 0.0f;                 // 12-15
	float Cy = 0.0f;                 // 16-19
	float Fx = 0.0f;                 // 20-23
	float Fy = 0.0f;                 // 24-27
	float DepthScale = 0.0f;         // 28-31
	char guid[33] = "00000000000000000000000000000000"; // 32-...
};

DEPTH_DATA_HEADER depth_header;
CONFIG_MSG stream_config;

unsigned int headerSize = sizeof(depth_header);
unsigned int linesPerMessage;
unsigned int sendThrottle;
unsigned char * frameStreamBufferDepth;
unsigned char * frameStreamBufferColor;
int frameStreamBufferDepthSize;
int frameStreamBufferColorSize;
char config_msg_buf[sizeof(stream_config)];

// JPEG COMPRESSION START ==================================
const int JPEG_QUALITY = 50;
// =========================================================

int main(int argc, char *argv[]) {
	std::string socketIDParam("-i");
	std::string portParam("-p");
	std::string ipParam("-d");
	std::string serialParam("-s");
	std::string linesParam("-l");
	std::string sendThrottleParam("-t");
	std::string pipelineParam("-q");

	std::string socketIDValue = "kinect1";
	std::string ipValue = "impress.mooo.com";
	std::string portValue = "6312";
	std::string serialValue = "";
	std::string linesValue = "-1";
	std::string sendThrottleValue = "10";
	std::string pipelineValue = "opengl";

	if (argc % 2 != 1) {
		std::cout << "Usage:\n  freenectStreamer "
			<< socketIDParam << " [socketID] "
			<< portParam << " [port] "
			<< ipParam << " [ip] "
			<< serialParam << " [serial] "
			<< linesParam << " [maxLines] "
			<< sendThrottleParam << " [sendThrottle] "
			<< pipelineParam << " [pipeline] "
			<< "\nDefaults"
			<< "\n  socketID:     " << socketIDValue
			<< "\n  port:     " << portValue
			<< "\n  ip:       " << ipValue
			<< "\n  serial:   " << "\"\" (empty = first available)"
			<< "\n  maxLines:   " << linesValue << " (max. block size in lines; -1 = auto)"
			<< "\n  sendThrottle:   " << sendThrottleValue << " (us delay between sending line blocks; 0 = no limit)"
			<< "\n  pipeline: " << pipelineValue << " (one of cpu,opengl,cuda,opencl)"
			<< "\n";
		return -1;
	}

	for (int count = 1; count < argc; count += 2) {
		if (socketIDParam.compare(argv[count]) == 0) {
			socketIDValue = argv[count + 1];
		}
		else if (ipParam.compare(argv[count]) == 0) {
			ipValue = argv[count + 1];
		}
		else if (portParam.compare(argv[count]) == 0) {
			portValue = argv[count + 1];
		}
		else if (serialParam.compare(argv[count]) == 0) {
			serialValue = argv[count + 1];
		}
		else if (linesParam.compare(argv[count]) == 0) {
			linesValue = argv[count + 1];
		}
		else if (sendThrottleParam.compare(argv[count]) == 0) {
			sendThrottleValue = argv[count + 1];
		}
		else if (pipelineParam.compare(argv[count]) == 0) {
			pipelineValue = argv[count + 1];
		}
		else {
			std::cout << "Unknown Parameter: " << argv[count] << std::endl;
		}
	}

	sendThrottle = std::stoi(sendThrottleValue);
	int parsedLines = std::stoi(linesValue);

	int bytesPerDepthLine = 512 * 2;
	int maximumLinesPerDepthPacket = (65506 - headerSize) / bytesPerDepthLine;

	if (parsedLines > maximumLinesPerDepthPacket) {
		std::cout << "Invalid value for -p. Try >= 4 and < " << maximumLinesPerDepthPacket << std::endl;
		return -1;
	}

	if (parsedLines == -1) {
		linesPerMessage = maximumLinesPerDepthPacket;
	}
	else if (parsedLines < 4) {
		linesPerMessage = 4;
	}
	else {
		linesPerMessage = parsedLines;
	}

	// Round to multiple of 4 for DTX compression
	//int remainderM4 = linesPerMessage % 4;
	//linesPerMessage = linesPerMessage - remainderM4;

	std::cout << linesPerMessage << "\n";

	frameStreamBufferDepthSize = bytesPerDepthLine * linesPerMessage + headerSize;
	frameStreamBufferDepth = new unsigned char[frameStreamBufferDepthSize];
	frameStreamBufferColorSize = tjBufSize(depthWidth, depthHeight, JPEG_QUALITY) + headerSize;
	frameStreamBufferColor = new unsigned char[frameStreamBufferDepthSize]; // TODO: what is a good estimate maximum?

	std::string program_path(argv[0]);
	size_t executable_name_idx = program_path.rfind("freenectStreamer");

	std::string binpath = "/";
	if (executable_name_idx != std::string::npos) {
		binpath = program_path.substr(0, executable_name_idx);
	}
	std::cout << "Running in: " << binpath << std::endl;

	client.init(socketIDValue, true, ipValue, portValue);
	return openAndStream(serialValue, pipelineValue);
}


int openAndStream(std::string serial, std::string pipelineId) {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
			&reader);
	}
	else {
		return false;
	}


	unsigned int fpsCounter = 0;
	milliseconds lastFpsAverage = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	milliseconds interval = milliseconds(2000);

	//s.native_non_blocking(true);

#if !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
	asio::socket_base::send_low_watermark slwoption(frameStreamBufferSize);
	s.set_option(slwoption);
#endif



	//std::cout << "Device started." << std::endl;
	//std::cout << "\tSerial:\t " << dev->getSerialNumber() << std::endl;
	//std::cout << "\tFirmware:\t " << dev->getFirmwareVersion() << std::endl << std::endl;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler);
#endif
	stream_shutdown = false;

	// Setup Registration
	//libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	//libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

	CameraIntrinsics cameraIntrinsics;

	//wait a moment; GetDepthCameraIntrinsics can be unreliable at startup
	std::this_thread::sleep_for(std::chrono::microseconds(2500000));


	WCHAR uniqueKinectId[256];
	if (FAILED(sensor->get_UniqueKinectId(256, uniqueKinectId))) {
		std::cout << "NO ID" << std::endl;
	}
	for (int i = 0; i < sizeof(stream_config.guid) / sizeof(stream_config.guid[0]); i++) {
		stream_config.guid[i] = uniqueKinectId[i];
	}


	if (FAILED(mapper->GetDepthCameraIntrinsics(&cameraIntrinsics))) {
		std::cout << "NO intrinsics" << std::endl;
	}

	stream_config.frameWidth = depthWidth;
	stream_config.frameHeight = depthHeight;
	stream_config.maxLines = (unsigned short)linesPerMessage;

	float depthScale = 0.001f; // TODO: is this correct for kinect?!
	stream_config.Cx = cameraIntrinsics.PrincipalPointX;
	stream_config.Cy = cameraIntrinsics.PrincipalPointY;
	stream_config.Fx = cameraIntrinsics.FocalLengthX;
	stream_config.Fy = cameraIntrinsics.FocalLengthY;
	stream_config.DepthScale = depthScale;

	std::cout << "Cx: "
		<< stream_config.Cx << ", Cy: "
		<< stream_config.Cy << ", Fx: "
		<< stream_config.Fx << ", Fy: "
		<< stream_config.Fx << std::endl;


	uint32_t sequence = 0;

	while (!stream_shutdown) {

		UINT depthCapacity = 0;
		UINT16 *depthData = NULL;

		IMultiSourceFrame* frame = NULL;
		if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
			fpsCounter++;

			IDepthFrame* depthFrame;
			IDepthFrameReference* depthframeref = NULL;
			frame->get_DepthFrameReference(&depthframeref);
			depthframeref->AcquireFrame(&depthFrame);
			if (depthframeref) depthframeref->Release();

			IColorFrame* colorFrame;
			IColorFrameReference* colorframeref = NULL;
			frame->get_ColorFrameReference(&colorframeref);
			colorframeref->AcquireFrame(&colorFrame);
			if (colorframeref) colorframeref->Release();

			if (depthFrame) {
				//depthWidth = depthFrame->get_FrameDescription.get_Width();
				depthFrame->AccessUnderlyingBuffer(&depthCapacity, &depthData);
				mapper->MapDepthFrameToColorSpace(
					depthCapacity, depthData,        // Depth frame data and size of depth frame
					depthWidth*depthHeight, depth2rgb);		// Output ColorSpacePoint array and size

			}

			if (depthFrame && colorFrame) {
				colorFrame->CopyConvertedFrameDataToArray(colorwidth*colorheight * 4, rgbimage, ColorImageFormat_Rgba);

				for (int i = 0; i < depthWidth*depthHeight; i++) {
					ColorSpacePoint p = depth2rgb[i];

					if (p.X < 0 || p.Y < 0 || p.X > colorwidth || p.Y > colorheight) {
						mappedRGBImage[3 * i + 0] = 128;
						mappedRGBImage[3 * i + 1] = 128;
						mappedRGBImage[3 * i + 2] = 128;
					}
					else {
						int idx = (int)p.X + colorwidth*(int)p.Y;
						mappedRGBImage[3 * i + 0] = rgbimage[4 * idx + 0];
						mappedRGBImage[3 * i + 1] = rgbimage[4 * idx + 1];
						mappedRGBImage[3 * i + 2] = rgbimage[4 * idx + 2];
					}
				}

				if (streamFrame(depthData, mappedRGBImage, sequence++) == -1) {
					stream_shutdown = true;
				}


				depthFrame->Release();
				colorFrame->Release();
			}

		}




		//IColorFrame* colorFrame = NULL;
		//if (SUCCEEDED(colorReader->AcquireLatestFrame(&colorFrame))) {
		//	colorFrame->CopyConvertedFrameDataToArray(width*height * 4, data, ColorImageFormat_Bgra);
		//}
		//if (frame) frame->Release();


		milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

		if (lastFpsAverage + interval <= ms) {
			lastFpsAverage = ms;
			std::cout << "Average FPS: " << fpsCounter / 2.0 << std::endl;
			fpsCounter = 0;
			sendConfig();
		}

		//listener.release(frames);
	}

	return 0;
}

int sendConfig() {

	memcpy(config_msg_buf, &stream_config, sizeof(stream_config));

	try {
		client.SendData(config_msg_buf, sizeof(stream_config));
		//s.send_to(asio::buffer(config_msg_buf, sizeof(stream_config)), endpoint);
	}
	catch (std::exception& e) {
		std::cerr << "Exception: " << e.what() << "\n";
		return -1;
	}

	return 0;
}

int streamFrame(UINT16 *depthData, unsigned char *RGBImage, uint32_t sequence) {
	depth_header.sequence = sequence;

	depth_header.msgType = 0x04;
	depth_header.startRow = (unsigned short)0;
	depth_header.endRow = (unsigned short)depthHeight;
	memcpy(&frameStreamBufferColor[0], &depth_header, headerSize);

	// COMPRESS COLOR
	long unsigned int _jpegSize = frameStreamBufferColorSize - headerSize;
	unsigned char* _compressedImage = &frameStreamBufferColor[headerSize];

	// replace  _complressedImage with &frameStreamBufferColor[headerSize]
	tjhandle _jpegCompressor = tjInitCompress();
	tjCompress2(_jpegCompressor, RGBImage, depthWidth, 0, depthHeight, TJPF_RGB,
		&_compressedImage, &_jpegSize, TJSAMP_444, JPEG_QUALITY,
		TJFLAG_FASTDCT);

	//std::cout << _jpegSize << "\n";
	//memcpy(&frameStreamBufferColor[headerSize], _compressedImage, _jpegSize);

	try {
		client.SendData(frameStreamBufferColor, headerSize + _jpegSize);
		//s.send_to(asio::buffer(frameStreamBufferColor, headerSize + _jpegSize), endpoint);
	}
	catch (std::exception& e) {
		std::cerr << "Exception: " << e.what() << "\n";
	}

	//tjFree(_compressedImage);
	tjDestroy(_jpegCompressor);


	// Send Depth
	depth_header.msgType = 0x03;
	for (unsigned int startRow = 0; startRow < depthHeight; startRow += linesPerMessage) {

		size_t endRow = startRow + linesPerMessage;
		if (endRow >= depthHeight) endRow = depthHeight;
		if (startRow >= endRow) break;

		size_t totalLines = endRow - startRow;

		depth_header.startRow = (unsigned short)startRow;
		depth_header.endRow = (unsigned short)endRow;
		memcpy(&frameStreamBufferDepth[0], &depth_header, headerSize);
		size_t writeOffset = headerSize;

		size_t depthLineSizeR = depthWidth * 2 /*regdepth->bytes_per_pixel*/;
		size_t depthLineSizeW = depthWidth * 2;
		size_t readOffset = startRow*depthLineSizeR;

		for (int line = startRow; line < endRow; line++) {
			for (int i = 0; i < depthWidth; i++) {
				//UINT16 depthValue = depthData[readOffset/2 + i];
				//memcpy(&depthValue, &depthData[readOffset + i * 4], sizeof(depthValue));
				//unsigned short depthValueShort = (unsigned short)(depthValue);
				memcpy(&frameStreamBufferDepth[writeOffset + i * 2], &depthData[readOffset / 2 + i], 2);
			}

			writeOffset += depthLineSizeW;
			readOffset += depthLineSizeR;
		}

		//size_t colorLineSizeR = regrgb->width * regrgb->bytes_per_pixel;
		//readOffset = startRow*colorLineSizeR;

		//stb_compress_dxt(&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], 512, (int)totalLines, 0);

		try {
			//s.send_to(asio::buffer(frameStreamBufferDepth, writeOffset), endpoint);
			client.SendData(frameStreamBufferDepth, writeOffset);
		}
		catch (std::exception& e) {
			std::cerr << "Exception: " << e.what() << "\n";
		}

		//std::this_thread::sleep_for(std::chrono::microseconds(sendThrottle));
	}

	return 0;
}


/*

# Protocol

OUTDATED!!!!!!!!!!!

## Registered Frame Data (0x03):

,----------------------------- Msg Type
|    ,------------------------ Device ID
|    |    ,------------------- Start Row (0 = first row)
|    |    |    ,--------------
|    |    |    |    ,--------- End Row
|    |    |    |    |    ,----
|    |    |    |    |    |        ,---- Frame Data. 2 * (EndRow-StartRow) * 4 bytes. First half is depth, second is rgb
|    |    |    |    |    |        |         ....
V    V    V    V    V    V    <=========>
0x03 0x01 0x00 0x00 0x00 0x05 0xab ... 0xef


## Config Frame (0x04):

,---------------------------- Msg Type
|    ,----------------------- Parameter 1 (0x01 = device id)
|    |    ,------------------ Parameter 2 (...)
|    |    |    ,------------- Parameter 3 (...)
|    |    |    |
|    |    |    |        ,---- Payload (could be guid)
|    |    |    |        |           ....
V    V    V    V    <=========>
0x04 0x01 0x0b 0x0c 0xab ... 0xef

TODO: other frame data we have available are: timestamp, sequenceNr, exposure, gain, gamma, status

*/



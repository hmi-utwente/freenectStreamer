#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <fstream>
#include <cstdlib>
#include <chrono>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <asio.hpp>

#define STB_DXT_IMPLEMENTATION
#include "stb_dxt.h"

using namespace std::chrono;
using asio::ip::udp;

bool stream_shutdown = false;
void sigint_handler(int s) {
	stream_shutdown = true;
}

bool kinect_paused = false;
libfreenect2::Freenect2Device *devtopause;
void sigusr1_handler(int s) {
	if (devtopause == 0)  return;
	if (kinect_paused)    devtopause->start();
	else                  devtopause->stop();
	kinect_paused = !kinect_paused;
}

asio::io_service io_service;
udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
udp::resolver resolver(io_service);
udp::endpoint endpoint;

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence);
int openAndStream(std::string serial, std::string pipelineId);
int sendConfig();

struct DEPTH_DATA_HEADER {
    unsigned char msgType = 0x03;   // 00
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
unsigned int linesPerMessage = 32;
unsigned int maxFramesPerSeconds = -1;
unsigned char * frameStreamBuffer;
int freamStreamBufferSize;
char config_msg_buf[sizeof(stream_config)];

int main(int argc, char *argv[]) {
	std::string portParam("-p");
	std::string ipParam("-d");
	std::string serialParam("-s");
	std::string linesParam("-l");
	std::string maxFramesParam("-m");
	std::string pipelineParam("-q");

	std::string ipValue = "127.0.0.1";
	std::string portValue = "1339";
	std::string serialValue = "";
	std::string linesValue = "32";
	std::string maxFramesValue = "-1";
	std::string pipelineValue = "opengl";

	if (argc % 2 != 1) {
		std::cout << "Usage:\n  freenectStreamer "
			<< portParam << " [port] "
			<< ipParam << " [ip] "
			<< serialParam << " [serial] "
			<< linesParam << " [maxLines] "
			<< maxFramesParam << " [maxFPS] "
			<< pipelineParam << " [pipeline] "
			<< "\nDefaults"
			<< "\n  port:     " << portValue
			<< "\n  ip:       " << ipValue
			<< "\n  serial:   " << "\"\" (empty = first available)"
			<< "\n  maxLines: " << linesValue << " (-1 = no limit)"
			<< "\n  maxFPS:   " << maxFramesValue
			<< "\n  pipeline: " << pipelineValue << " (one of cpu,opengl,cuda,opencl)"
			<< "\n";
		return -1;
	}

	for (int count = 1; count < argc; count += 2) {
		if (ipParam.compare(argv[count]) == 0) {
			ipValue = argv[count + 1];
		} else if (portParam.compare(argv[count]) == 0) {
			portValue = argv[count + 1];
		} else if (serialParam.compare(argv[count]) == 0) {
			serialValue = argv[count + 1];
		} else if (linesParam.compare(argv[count]) == 0) {
			linesValue = argv[count + 1];
		} else if (maxFramesParam.compare(argv[count]) == 0) {
			maxFramesValue = argv[count + 1];
		} else if (pipelineParam.compare(argv[count]) == 0) {
			pipelineValue = argv[count + 1];
		} else {
			std::cout << "Unknown Parameter: " << argv[count] << std::endl;
		}
	}
	
	int parsedLines = std::stoi(linesValue);
	if (parsedLines > 44 || parsedLines < 1) {
		std::cout << "Invalid value for -p. Try > 0 and < 45" << std::endl;
		return -1;
	}

	int parsedMaxFrames = std::stoi(maxFramesValue);
	maxFramesPerSeconds = parsedMaxFrames;

	linesPerMessage = parsedLines;
	freamStreamBufferSize = (256 * linesPerMessage) + (512 * linesPerMessage * 2) + headerSize;
	frameStreamBuffer = new unsigned char[freamStreamBufferSize];

	endpoint = *resolver.resolve({ udp::v4(), ipValue, portValue });

	libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Info));

	std::string program_path(argv[0]);
	size_t executable_name_idx = program_path.rfind("freenectStreamer");

	std::string binpath = "/";
	if (executable_name_idx != std::string::npos) {
		binpath = program_path.substr(0, executable_name_idx);
	}
	std::cout << "Running in: " << binpath << std::endl;

	return openAndStream(serialValue, pipelineValue);
}

int openAndStream(std::string serial, std::string pipelineId) {
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	if (freenect2.enumerateDevices() == 0) {
		std::cout << "E: No device connected." << std::endl;
		return -1;
	}

	unsigned int fpsCounter = 0;
	milliseconds lastFpsAverage = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
	milliseconds lastFrameSent = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
	milliseconds fpsInterval = milliseconds((int) 1000.0 / maxFramesPerSeconds);
	milliseconds interval = milliseconds(2000);
	
	if (pipelineId.compare("cpu") == 0) {
		pipeline = new libfreenect2::CpuPacketPipeline();
	} else if (pipelineId.compare("opengl") == 0) {
		pipeline = new libfreenect2::OpenGLPacketPipeline();
	} else if (pipelineId.compare("cuda") == 0) {
		//pipeline = new libfreenect2::CudaPacketPipeline(-1);
		// THIS WILL NOT COMPILE IF LIBFREENECT IS NOT COMPILED WITH CUDA!!!
		// set flags...
	} else if (pipelineId.compare("opencl") == 0) {
		pipeline = new libfreenect2::OpenCLPacketPipeline(1);
		return -1;
	} else {
		std::cout << "Unknown pipeline: " << pipelineId << std::endl;
		return -1;
	}

	//	std::cout << freenect2.getDefaultDeviceSerialNumber() << std::endl;


	// Default device serial (if not specified)
	if (serial == "") serial = freenect2.getDefaultDeviceSerialNumber();

	// Listeners
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth); // | libfreenect2::Frame::Ir
	libfreenect2::FrameMap frames;

	// Open Device & Register listenres
	dev = freenect2.openDevice(serial, pipeline);
	if (!dev->start()) {
		std::cout << "E: Failed to start device." << std::endl;
		return -1;
	}

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	devtopause = dev;

	std::cout << "Device started." << std::endl;
	std::cout << "\tSerial:\t " << dev->getSerialNumber() << std::endl;
	std::cout << "\tFirmware:\t " << dev->getFirmwareVersion() << std::endl << std::endl;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler);
#endif
	stream_shutdown = false;

	// Setup Registration
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

	// Configure Socket:
	asio::socket_base::send_buffer_size soption(freamStreamBufferSize);
	s.set_option(soption);

    
    stream_config.frameWidth = undistorted.width;
    stream_config.frameHeight = undistorted.height;
    stream_config.maxLines = (unsigned short) linesPerMessage;
    libfreenect2::Freenect2Device::IrCameraParams i_d = dev->getIrCameraParams();
    float depthScale = 0.001f;
    stream_config.Cx = i_d.cx;
    stream_config.Cy = i_d.cy;
    stream_config.Fx = i_d.fx;
    stream_config.Fy = i_d.fy;
    stream_config.DepthScale = depthScale;
    std::strcpy(stream_config.guid, dev->getSerialNumber().c_str());
    
	while (!stream_shutdown) {

		milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

		if (maxFramesPerSeconds > 0 && (ms - lastFrameSent) < fpsInterval) {
			continue;
		}

		if (!listener.waitForNewFrame(frames, 5 * 1000)) {
			return -1;
		}

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		registration->apply(rgb, depth, &undistorted, &registered);

		if (streamFrame(&undistorted, &registered, rgb->sequence) == -1) {
			stream_shutdown = true;
		}

		fpsCounter++;
		lastFrameSent = ms;
		ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

		if (lastFpsAverage + interval <= ms) {
			lastFpsAverage = ms;
			std::cout << "Average FPS: " << fpsCounter / 2.0 << std::endl;
			fpsCounter = 0;
            sendConfig();
		}

		listener.release(frames);
	}

	dev->stop();
	dev->close();
	delete registration;

	return 0;
}

int sendConfig() {
    memcpy(config_msg_buf, &stream_config, sizeof(stream_config));
    
    try {
        s.send_to(asio::buffer(config_msg_buf, sizeof(stream_config)), endpoint);
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return -1;
    }
    
    return 0;
}

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence) {
    depth_header.sequence = sequence;

	for (unsigned int startRow = 0; startRow < regdepth->height; startRow += linesPerMessage) {

		size_t endRow = startRow + linesPerMessage;
		if (endRow >= regdepth->height) endRow = regdepth->height;
		if (startRow >= endRow) break;

		size_t totalLines = endRow - startRow;

        depth_header.startRow = (unsigned short) startRow;
        depth_header.endRow = (unsigned short) endRow;
        memcpy(&frameStreamBuffer[0], &depth_header, headerSize);
        size_t writeOffset = headerSize;

		size_t depthLineSizeR = regdepth->width * regdepth->bytes_per_pixel;
		size_t depthLineSizeW = regdepth->width * 2;
		size_t readOffset = startRow*depthLineSizeR;
		for (int line = startRow; line < endRow; line++) {
			for (int i = 0; i < regdepth->width; i++) {
				float depthValue = 0;
				memcpy(&depthValue, &regdepth->data[readOffset + i * 4], sizeof(depthValue));
				unsigned short depthValueShort = (unsigned short)(depthValue);
				memcpy(&frameStreamBuffer[writeOffset + i * 2], &depthValueShort, sizeof(depthValueShort));
			}

			writeOffset += depthLineSizeW;
			readOffset += depthLineSizeR;
		}

		size_t colorLineSizeR = regrgb->width * regrgb->bytes_per_pixel;
		readOffset = startRow*colorLineSizeR;

		stb_compress_dxt(&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], 512, (int)totalLines, 0);

		try {
			s.send_to(asio::buffer(frameStreamBuffer, freamStreamBufferSize), endpoint);
		} catch (std::exception& e) {
			std::cerr << "Exception: " << e.what() << "\n";
		}
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



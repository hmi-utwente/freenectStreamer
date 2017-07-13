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
udp::endpoint endpoint = *resolver.resolve({udp::v4(), "127.0.0.1", "1339"});

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence);
int openAndStream(std::string serial);

const unsigned int headerSize = 10;
const unsigned int linesPerMessage = 20;
// Buffer has size for 10 lines at 512 pixel a line and 4 byte a pixel, plus 10 bytes header
//(512*linesPerMessage*4)
const std::size_t freamStreamBufferSize = (256*linesPerMessage) + (512*linesPerMessage*2) + headerSize;
unsigned char frameStreamBuffer[freamStreamBufferSize];

int main(int argc, char *argv[]) {
    return openAndStream("");
}

int openAndStream(std::string serial) {
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    
	unsigned int fpsCounter = 0;
    milliseconds lastFpsAverage = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    milliseconds interval = milliseconds(2000);
    
    if(freenect2.enumerateDevices() == 0) {
        std::cout << "E: No device connected." << std::endl;
        return -1;
    }
    
    
    // Default device serial (if not specified)
    if (serial == "") serial = freenect2.getDefaultDeviceSerialNumber();
    
    // Pipeline - TODO: make different pipelines possible
    //pipeline = new libfreenect2::CpuPacketPipeline();
    //pipeline = new libfreenect2::OpenCLPacketPipeline(-1);
    pipeline = new libfreenect2::OpenGLPacketPipeline();
    
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
    
    signal(SIGINT,sigint_handler);
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
    
    
    while(!stream_shutdown) {
        if (!listener.waitForNewFrame(frames, 5*1000)) {
            std::cout << "Haven't received a frame for 5 seconds. Stopping." << std::endl;
            return -1;
        }
        
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        
        registration->apply(rgb, depth, &undistorted, &registered);
        
        if (streamFrame(&undistorted, &registered, rgb->sequence) == -1) {
            stream_shutdown = true;
        }
        fpsCounter++;
        milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
        if (lastFpsAverage+interval <= ms) {
            lastFpsAverage = ms;
            std::cout << "Average FPS: " << fpsCounter/2.0 << std::endl;
            fpsCounter = 0;
        }
        
        listener.release(frames);
    }
    
    dev->stop();
    dev->close();
    delete registration;

    return 0;
}

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence) {
    frameStreamBuffer[0] = 0x03;
    frameStreamBuffer[1] = 0x01;
    
    frameStreamBuffer[2] = (sequence & 0x000000ff);
    frameStreamBuffer[3] = (sequence & 0x0000ff00) >> 8;
    frameStreamBuffer[4] = (sequence & 0x00ff0000) >> 16;
    frameStreamBuffer[5] = (sequence & 0xff000000) >> 24;
    
    //std::cout << "#";
    for (unsigned int startRow = 0; startRow < regdepth->height; startRow += linesPerMessage) {
        
		unsigned int endRow = startRow + linesPerMessage;
        if (endRow >= regdepth->height) endRow = regdepth->height;
        if (startRow >= endRow) break;
        
        int totalLines = endRow - startRow;
        
        frameStreamBuffer[6] = startRow & 0xff;
        frameStreamBuffer[7] = (startRow >> 8) & 0xff;
        frameStreamBuffer[8] = endRow & 0xff;
        frameStreamBuffer[9] = (endRow >> 8) & 0xff;
        
        unsigned int writeOffset = headerSize;
        
        size_t depthLineSizeR = regdepth->width * regdepth->bytes_per_pixel;
        size_t depthLineSizeW = regdepth->width * 2;
		unsigned int readOffset = startRow*depthLineSizeR;
        for (int line=startRow; line < endRow; line++) {
            //memcpy (&frameStreamBuffer[writeOffset], &regdepth->data[readOffset], depthLineSize * sizeof(char));
            for (int i = 0; i < regdepth->width; i++) {
                float depthValue = 0;
                memcpy(&depthValue, &regdepth->data[readOffset+i*4], sizeof(depthValue));
				unsigned short depthValueShort = unsigned short(depthValue);
                memcpy(&frameStreamBuffer[writeOffset+i*2], &depthValueShort, sizeof(depthValueShort));
                //frameStreamBuffer[writeOffset+i*2]   = 0x00;
                //frameStreamBuffer[writeOffset+i*2+1] = 0x00;
            }
            
            writeOffset += depthLineSizeW;
            readOffset += depthLineSizeR;
        }
        
        size_t colorLineSizeR = regrgb->width * regrgb->bytes_per_pixel;
        //size_t colorLineSizeW = regrgb->width * 4;
        readOffset = startRow*colorLineSizeR;
        
        /* Line-by-line
         for (int line=startRow; line < endRow; line++) {
         memcpy (&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], colorLineSizeW * sizeof(char));
         writeOffset += colorLineSizeW;
         readOffset += colorLineSizeR;
         }
         */
        
        
        // DXT1
        stb_compress_dxt(&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], 512, totalLines, 0);
        
        try {
            s.send_to(asio::buffer(frameStreamBuffer, freamStreamBufferSize), endpoint);
            //std::cout << "=";
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
        }
        
        //usleep(1000);
    }
    //std::cout << "#" << std::endl;
    
    return 0;
}


/*
 
 # Protocol
 
 Msg Type:
 
 
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



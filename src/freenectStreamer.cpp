#define ASIO_STANDALONE 1
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <fstream>
#include <cstdlib>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include "asio.hpp"
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
udp::endpoint endpoint = *resolver.resolve({udp::v4(), "127.0.0.1", "1344"});

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


const uint headerSize = 6;
const uint linesPerMessage = 10;
// Buffer has size for 10 lines at 512 pixel a line and 4 byte a pixel, plus 4 bytes header
const std::size_t freamStreamBufferSize = 512*linesPerMessage*4*2+6;
char frameStreamBuffer[freamStreamBufferSize];

std::string getFormat(libfreenect2::Frame::Format format) {
    if (format == libfreenect2::Frame::Format::BGRX) {
        return "BGRX";
    } else if (format == libfreenect2::Frame::Format::Float) {
        return "Float";
    } else if (format == libfreenect2::Frame::Format::Gray) {
        return "Gray";
    } else if (format == libfreenect2::Frame::Format::Invalid) {
        return "Invalid";
    } else if (format == libfreenect2::Frame::Format::Raw) {
        return "Raw";
    } else if (format == libfreenect2::Frame::Format::RGBX) {
        return "RGBX";
    }
    
    return "FAIL";
}

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb) {
    /*
    std::cout << "RegDepth Info: " << std::endl;
    std::cout << "\tDim:\t\t" << regdepth->width << "x" << regdepth->height << std::endl;
    std::cout << "\tBPP:\t\t" << regdepth-> bytes_per_pixel<< std::endl;
    std::cout << "\tFormat:\t\t" << getFormat(regdepth->format) << std::endl;
    
    std::cout << "RegRgb Info: " << std::endl;
    std::cout << "\tDim:\t\t" << regrgb->width << "x" << regrgb->height << std::endl;
    std::cout << "\tBPP:\t\t" << regrgb->bytes_per_pixel<< std::endl;
    std::cout << "\tFormat:\t\t" << getFormat(regrgb->format) << std::endl;
    
    std::cout << "Stream Info: " << std::endl;
    std::cout << "\tfreamStreamBufferSize:\t\t" << freamStreamBufferSize << std::endl;
    
    std::cout << std::endl << std::endl;
    for (int y = 100; y < regrgb->height; y++) {
        for (int x = 0; x < regrgb->width; x++) {
            std::cout << "b: " << 0+regrgb->data[y*512*4+x*4] << "\tg: " << 0+regrgb->data[y*512*4+x*4+1] << "\tr: " << 0+regrgb->data[y*512*4+x*4+2] << "\tx: " << 0+regrgb->data[y*512*4+x*4+3] << std::endl;
        }
        break;
    }
     std::cout << std::endl << std::endl;
     */
    
    

    //std::cout << "#";
    for (uint startRow = 0; startRow < regdepth->height; startRow += linesPerMessage) {
        
        frameStreamBuffer[0] = 0x03;
        frameStreamBuffer[1] = 0x01;
        
        uint endRow = startRow + 10;
        if (endRow >= regdepth->height) endRow = regdepth->height;
        if (startRow == endRow) break;
        
        frameStreamBuffer[2] = startRow & 0xff;
        frameStreamBuffer[3] = (startRow >> 8) & 0xff;
        frameStreamBuffer[4] = endRow & 0xff;
        frameStreamBuffer[5] = (endRow >> 8) & 0xff;
        
        uint writeOffset = headerSize;
        
        size_t depthLineSize = regdepth->width * regdepth->bytes_per_pixel;
        uint readOffset = startRow*depthLineSize;
        for (int line=startRow; line < endRow; line++) {
            memcpy (&frameStreamBuffer[writeOffset], &regdepth->data[readOffset], depthLineSize * sizeof(char));
            writeOffset += depthLineSize;
            readOffset += depthLineSize;
        }
        
        size_t colorLineSize = regrgb->width * regrgb->bytes_per_pixel;
        readOffset = startRow*colorLineSize;
        for (int line=startRow; line < endRow; line++) {
            memcpy (&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], colorLineSize * sizeof(char));
            writeOffset += colorLineSize;
            readOffset += colorLineSize;
        }
        
        try {
            s.send_to(asio::buffer(frameStreamBuffer, freamStreamBufferSize), endpoint);
            //std::cout << "=";
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
        }
        
        //usleep(1000*5);
    }
    //std::cout << "#" << std::endl;
    
    return 0;
}


int OpenAndStream(std::string serial) {
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    
    
    if(freenect2.enumerateDevices() == 0) {
        std::cout << "E: No device connected." << std::endl;
        return -1;
    }
    
    
    // Default device serial (if not specified)
    if (serial == "") serial = freenect2.getDefaultDeviceSerialNumber();
    
    // Pipeline - TODO: make different pipelines possible
    //pipeline = new libfreenect2::CpuPacketPipeline();
    pipeline = new libfreenect2::OpenCLPacketPipeline(-1);
    
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
        /*
        std::cout << "Depth Info: " << std::endl;
        std::cout << "\tDim:\t\t" << depth->width << "x" << depth->height << std::endl;
        std::cout << "\tBPP:\t\t" << depth-> bytes_per_pixel<< std::endl;
        std::cout << "\tFormat:\t\t" << getFormat(depth->format) << std::endl;
        
        std::cout << "Rgb Info: " << std::endl;
        std::cout << "\tDim:\t\t" << rgb->width << "x" << rgb->height << std::endl;
        std::cout << "\tBPP:\t\t" << rgb->bytes_per_pixel<< std::endl;
        std::cout << "\tFormat:\t\t" << getFormat(rgb->format) << std::endl << std::endl;
        */
        registration->apply(rgb, depth, &undistorted, &registered);
        if (streamFrame(&undistorted, &registered) == -1) {
            stream_shutdown = true;
        }
        
        //usleep(1000*10);
        
        listener.release(frames);
    }
    
    dev->stop();
    dev->close();
    delete registration;

    return 0;
}

int main(int argc, char *argv[]) {
    return OpenAndStream("");
}

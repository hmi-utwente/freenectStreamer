# freenectStreamer

Stream KinectV2 data using libfreenect2 through UDP accross the network.
Also see the [page on the HmiMR Wiki](https://github.com/hmi-utwente/HmiMixedRealityWiki/wiki/Streaming-Kinect-Data) for more information.

# Installation

## Windows (TODO!):
 - Place this in a folder next to the libfreenect2 folder, and make sure you can compile libfreenect first:
   - git clone https://github.com/hmi-utwente/freenectStreamer.git
 - Make a folder freenectStreamer/depends and clone asio into it(stay on the master branch for Windows!):
   - cd freenectStreamer && mkdir depends && cd depends
   - git clone https://github.com/chriskohlhoff/asio.git
 - make a folder "build" in freenectStreamer
   - cd build
   - cmake -G "Visual Studio 14 2015 Win64" .. 
   - (or your respective string for your version of visual studio)
 - Now you can open freenectStreamer.sln with Visual Studio

## macOS
 - Place this in a folder next to the libfreenect2 folder, and make sure you can compile libfreenect first:
   - git clone https://github.com/hmi-utwente/freenectStreamer.git
 - Make a folder freenectStreamer/depends and clone asio into it, checkout the asio-1-10-branch:
   - cd freenectStreamer && mkdir depends && cd depends
   - git clone https://github.com/chriskohlhoff/asio.git
   - cd asio
   - git checkout asio-1-10-branch
 - Go back to freenectStreamer folder and make a "build" folder, then cmake...:
   - cd build
   - cmake -G <YOUR IDE> .. 
   - i.e for xcode: cmake -G Xcode ..
 - Now you have a solution for your IDE with which you can compile and run freenectStreamer

## Linux (TODO)
 - Probably same as macOS, but also stay on the master branch of asio.

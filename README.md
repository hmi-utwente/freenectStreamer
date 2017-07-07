# freenectStreamer

NOTE: these are just the barebones to get started developing. This does nothing yet!

Preliminary setup instructions, only tested with gcc and Xcode:

 - Place this in a folder next to the libfreenect2 folder, and make sure you can compile libfreenect first.
 - Also next to libfreenect2 and freenectStreamer, clone asio and use the asio-1-10-branch:
   - git clone git@github.com:chriskohlhoff/asio.git
   - cd asio
   - git checkout asio-1-10-branch
 - make a folder "build" in freenectStreamer
   - cd build
   - cmake -G <YOUR IDE> .. 
   - i.e for xcode: cmake -G Xcode ..
 - Now you have a solution for your IDE with which you can compile and run freenectStreamer




# jetson-ffmpeg
L4T Multimedia API for ffmpeg

**1.build and install library**

    git clone https://github.com/jocover/jetson-ffmpeg.git
    cd jetson-ffmpeg
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig
	
**2.patch ffmpeg and build**

    git clone git://source.ffmpeg.org/ffmpeg.git -b release/4.2 --depth=1
    cd ffmpeg
    wget https://github.com/jocover/jetson-ffmpeg/raw/master/ffmpeg_nvmpi.patch
    git apply ffmpeg_nvmpi.patch
    ./configure --enable-nvmpi
    make

**3.using**

### Supports Decoding
  - MPEG2
  - H.264/AVC
  - HEVC
  - VP8
  - VP9
  
**example**

    ffmpeg -c:v h264_nvmpi -i input_file -f null -
	
### Supports Encoding
  - H.264/AVC
  - HEVC
  
**example**

    ffmpeg -i input_file -c:v h264_nvmpi <output.mp4>

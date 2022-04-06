cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
add_executable( DisplayImage main.cpp mapmerge.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )

add_executable( ResizeImage resize_image.cpp )
target_link_libraries( ResizeImage ${OpenCV_LIBS} )
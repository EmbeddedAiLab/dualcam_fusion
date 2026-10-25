# One static library that contains every module.
# Each module lives in its own folder with a .h and a .cpp.

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d)
find_package(Threads REQUIRED)

# Rockchip RGA (librga). Optional: without it the CPU fallback is used.
find_path(RGA_INCLUDE_DIR NAMES im2d.hpp PATH_SUFFIXES rga)
find_library(RGA_LIBRARY NAMES rga)

add_library(dualcam_lib STATIC
  dmabuf/dmabuf.cpp
  camera/camera.cpp
  preprocess/image_preprocess.cpp
  sync/frame_sync.cpp
  calibration/camera_calibration.cpp
  pyramid/laplacian_pyramid.cpp
  fusion/fusion_engine.cpp
  reconstruct/pyramid_reconstruct.cpp
  viewer/viewer.cpp
  stats/pipeline_stats.cpp
)

# Headers are included as "module/file.h", so lib/ is the include root
target_include_directories(dualcam_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(dualcam_lib PUBLIC ${OpenCV_LIBS} Threads::Threads)

target_compile_options(dualcam_lib PUBLIC -Wall -Wextra)

if(RGA_INCLUDE_DIR AND RGA_LIBRARY)
  message(STATUS "RGA found: ${RGA_LIBRARY}")
  target_compile_definitions(dualcam_lib PUBLIC HAVE_RGA=1)
  target_include_directories(dualcam_lib PUBLIC ${RGA_INCLUDE_DIR})
  target_link_libraries(dualcam_lib PUBLIC ${RGA_LIBRARY})
else()
  message(WARNING "librga not found - building with the slow OpenCV CPU fallback")
endif()

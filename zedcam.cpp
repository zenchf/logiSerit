#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

using namespace sl;
using namespace cv;
using namespace std;

// Initialize ZED camera
sl::Camera zed;
InitParameters init_parameters;

sl::Mat image;
cv::Mat cvImage;

// 10029681, //10029681,//

int serial = 10029681;
const int serialtabela = 14913510;

void opencamera()
{

  init_parameters.camera_resolution = RESOLUTION::HD1080; // 1920x1080 1280x720
  //init_parameters.camera_fps = 30;
  init_parameters.input.setFromSerialNumber(serialtabela); // Set the serial number here

  // Open the camera
  auto returned_state = zed.open(init_parameters);
  // int zed_serial = zed.getCameraInformation().serial_number;
  // printf("Hello! This is my serial number: %d\n", zed_serial);
  //  Capture new images until 'q' is pressed
  if (returned_state != ERROR_CODE::SUCCESS)
  {
    std::cout << "Camera Open: " << sl::toString(returned_state) << ". Exit program." << std::endl;
  }
}

void camsettings()
{
  if (zed.grab() == sl::ERROR_CODE::SUCCESS)
  {
    // Retrieve left image
    zed.retrieveImage(image, sl::VIEW::LEFT);
    cvImage = cv::Mat(image.getHeight(), image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM::CPU));
    // Get camera information (serial number)
  }
/*/
  zed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, 3); //0-8
  zed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, 4);//0-8
  zed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, 0);//0-11
  //zed.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, 4);//0-8
  zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, 0);//0-8
  zed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, 5);//0-8
  zed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, 4300);//2800-6500
 // zed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, 19);//0-100
  zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, 0);//0-100
*/
  //  	zed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, 0); 
  	zed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 1);
  // zed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, 5);
}

void goruntuler()
{
  if (!cvImage.empty())
  {
    cv::imshow("Image", cvImage);
  }
}

int main(int argc, char **argv)
{
  opencamera();
  while (true)
  {
    camsettings();
    goruntuler();
    if (cv::waitKey(1) == 27)
      break; // Exit loop if any key is pressed
  }
  return 0;
}

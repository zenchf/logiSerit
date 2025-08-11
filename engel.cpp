#include <iostream>
#include <numeric>
#include <iomanip>
#include <chrono>
#include <stdlib.h>
#include <vector>
#include <algorithm>
// Standard includes
#include <stdio.h>
#include <string.h>
#include <string>

// ZED include
#include <sl/Camera.hpp>

// OpenCV include (for display)
#include <opencv2/opencv.hpp>

//tcp socket kütüphanesi
#include "/home/legaca/async-tcp/async-sockets/include/tcpsocket.hpp"

using namespace sl;
using namespace cv;
using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std;

InitParameters init_parameters;
int64_t start, stop;
int interval = 1000;
// Create a ZED Camera object
sl::Mat image;
cv::Mat cvImage;

sl::Mat depth_map_i;
cv::Mat cvDepthMap_i;

cv::Mat cvDepthMap;

sl::Mat depth_map;
cv::Mat obstacleMap;
std::string pointNumber;
cv::Scalar pointColor;

int ort_depth = 10; // Her noktada kaç ölçüm yapılacağını belirtir
int pointCounter = 0;

const int ynoktasi = 50;

std::string distanceText;
float averageDistance;
float totalDistance;

const int roi_weight = 1280;
const int roi_height = 150;
const int roi_x = 0;
const int roi_y = 500;
char key = ' ';
int kirmizisayac;
bool working = true;

std::string mesaj;
int engelbilgisi_int;

string engelbilgisi; 
string yakinlik;

sl::Camera zed;
sl::ERROR_CODE returned_state;

bool isOpen = false;

cv::Scalar yellow = cv::Scalar(0, 255, 255);
cv::Scalar green = cv::Scalar(0, 255, 0);
cv::Scalar red = cv::Scalar(0, 0, 255);

std::vector<Scalar> colors = {red};

cv::Scalar target = red;
cv::Scalar target2 = yellow;
int serialengel = 14913510;

bool isConnected = false;

void opencamera()
{

  init_parameters.camera_resolution = RESOLUTION::HD720; // 1920x1080 1280x720
  //init_parameters.camera_fps = 30;
  init_parameters.input.setFromSerialNumber(serialengel); // Set the serial number here

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
/*
void opencamera()
{
    init_parameters.camera_resolution = RESOLUTION::HD1080; // Automatically select resolution
    //init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Use a more compatible depth mode
    //init_parameters.camera_fps = 30;
    init_parameters.input.setFromSerialNumber(serialengel); // Camera serial number

    // Open the camera
    returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        std::cout << "Camera Open Error: " << sl::toString(returned_state) 
                  << " (" << returned_state << ")" << std::endl;
        working = false;
    }
}
*/
void camsettings()
{
  // Retrieve left image
  zed.retrieveImage(image, VIEW::LEFT);
  zed.retrieveImage(depth_map_i, VIEW::DEPTH);
  zed.retrieveMeasure(depth_map, MEASURE::DEPTH);

  cvImage = cv::Mat(image.getHeight(), image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM::CPU));
  cvDepthMap_i = cv::Mat(depth_map_i.getHeight(), depth_map_i.getWidth(), CV_8UC4, depth_map_i.getPtr<sl::uchar1>(sl::MEM::CPU));
  cvDepthMap = cv::Mat(depth_map.getHeight(), depth_map.getWidth(), CV_32FC1, depth_map.getPtr<sl::uchar1>(sl::MEM::CPU));

  cv::Rect roi(roi_x, roi_y, roi_weight, roi_height); // x,y,w,h
  obstacleMap = cvDepthMap_i(roi);
}
void goruntuler()
{
  cv::imshow("Image", cvImage);
  cv::imshow("Depth Map i", cvDepthMap_i);
  // cv::imshow("Depth Map", cvDepthMap);
  // cv::imshow("Obstacle Map", obstacleMap);
}
void points()
{
  pointCounter = 0;
  colors.clear();

  for (int i = 400; i < obstacleMap.cols - 400 ; i += 40)
  {
    std::vector<float> distances;
    cv::Point point(i, ynoktasi); // x,y
    distances.clear();

    for (int j = 0; j < ort_depth; j++)
    {
      float distance = cvDepthMap.at<float>(500, i) / 1000.0f; // y,x // mesafe bilgisini al metreye çevir
      distances.push_back(distance);
    }

    // Ortalama mesafeyi hesapla
    totalDistance = std::accumulate(distances.begin(), distances.end(), 0.0);
    averageDistance = totalDistance / distances.size();
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << averageDistance; // 2 ondalık basamağa azalt
    distanceText = stream.str();

    //pointColor = (averageDistance < 1.0) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    pointColor = (4.0 > averageDistance) ? ((2 < averageDistance) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255)) : cv::Scalar(0, 255, 0);// furkimaaaaaa
    cv::circle(obstacleMap, point, 5, pointColor, -1); // image, center, radius, color, thickness

    colors.push_back(pointColor);
  
    // Nokta numarasını yazdır
    pointNumber = std::to_string(pointCounter); 

    cv::putText(obstacleMap, pointNumber, cv::Point(i, ynoktasi + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    cv::putText(obstacleMap, distanceText, cv::Point(i, ynoktasi - 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
    pointCounter++; // Nokta sayacını artırynoktasi

    goruntuler();
  }

  // Print which points are red
}

void gonder()
{
  int cnt = count(colors.begin(), colors.end(), target);
  if (cnt > 0) // bgr
  {
    engelbilgisi = "Engel var";

  }
  else
  {
    int cnt2 = count(colors.begin(), colors.end(), target2);
    if (cnt2 > 0)
    {
      engelbilgisi = "4 metre";
    }
    else
    {
      engelbilgisi = "Engel yok";
    }
  }

}

int main()
{
  // TCP Bağlantısı başlat
  TCPSocket tcpSocket([](int errorCode, std::string errorMessage){
      cout << "Socket creation error:" << errorCode << " : " << errorMessage << endl;
  });

  tcpSocket.onMessageReceived = [](string message) {
      cout << "Mesaj alindi: " << message << endl;
  };

  tcpSocket.onSocketClosed = [](int errorCode){
      cout << "Baglanti kapandi: " << errorCode << endl;
      isConnected = false;
  };

  tcpSocket.Connect("localhost", 8888, [&] {
    cout << "Sunucuya baglanti basarili!" << endl;
    isConnected = true;
    tcpSocket.Send("#engel*");
  },
  [](int errorCode, std::string errorMessage){
    cout << errorCode << " : " << errorMessage << endl;
  });

  while (working)
  {
    opencamera();
    returned_state = zed.grab();

    if (returned_state == ERROR_CODE::SUCCESS)
    {
      camsettings();
      points();
      gonder();

      key = (char)waitKey(1);
      if (key == 27 or key == 113)
      {
        destroyAllWindows();
        working = false;
        exit(1);
      }
    }
  }
  zed.close();
  return 0;
}//g++ -o engel engel.cpp -I/home/legaca/async-tcp/async-sockets/include -I/usr/local/zed/include -L/usr/local/zed/lib -lsl_zed -lopencv_core -opencv_highgui -lopencv_imgproc -lpthread
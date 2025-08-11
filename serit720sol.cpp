// B8:D6:1A:43:7A:0A ucube
#pragma region Kütüphaneler
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
// Araç kontrol sistemiyle haberleşmede kullanılacak kütüphane#include <CppLinuxSerial/SerialPort.hpp>
// Şerit takibi için kullanılacak OpenCV kütüphaneleri
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Hesaplama gibi işlemlerde kullanılacak kütüphaneler
#include <iterator>
#include <sstream>
#include <fstream>
#include <chrono>
#include <list>
#include <vector>
#include <iomanip>
#include <stdlib.h>
#include <bitset>
#include <filesystem>
#include <termios.h>

//tcp socket kütüphanesi
#include "/home/legaca/async-tcp/async-sockets/include/tcpsocket.hpp"

using namespace std::chrono;
using namespace cv;
using namespace std;

string gonder, serit_gonder;
bool isConnected = false;

cv::Mat blackFrame;

// map fonksiyonu, kullanımı: mapper(degisken, giris_min, giris_max, cikis_min, cikis_max), integer bir değer döndürür
int mapper(int x, int in_min, int in_max, int out_min, int out_max)
{
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// ends_with fonksiyonu, kullanımı: ends_with(string, sonunda olup olmadığı kontrol edilecek değer), bool bir değer döndürür
inline bool ends_with(std::string const &value, std::string const &ending)
{
    if (ending.size() > value.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

#pragma region Variables

// Hesaplama vb. işlerde kullanılacak değişkenler
int64_t start, stop;
int counter, to_send;
int interval = 600;
bool to_right, to_left, kavsak, ortala, park_edilebilir, dodge;
bool ileri, ready, right_lane, left_lane, dur = false;
char key;
const int sample_size = 100;
int minThresh = 150;
int maxThresh = 255;

// Şerit takibinde kullanılacak değişkenler

const int frameCenter = 520;
int linePos, laneCenter, result, mapped_result, durak_counter, durak_timer_start, durak_timer_last, durak_interval, durak_state, laneCenterAvg;
Point2f Source[] = {Point2f(0, 600), Point2f(1920, 600), Point2f(0, 1080), Point2f(1920, 1080)};
Point2f Destination[] = {Point2f(480, 0), Point2f(1440, 0), Point2f(480, 1080), Point2f(1440, 1080)};
cv::Mat frame, rgb, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDupe, ROILane;
vector<int> histogramLane;
VideoCapture cap(0);
const int minLine = 150;
stringstream ss;
string obstacle;
int countersteer, dodgeSteer;
bool working = true;
const int minCmd = 1201;
const int maxCmd = 1400;
const int roiHeight = 200;
#pragma endregion

// Moving average fonksiyonu, şerit takibi verisinin stabil olması için
template <typename T>
class MovingAverage
{
public:
    explicit MovingAverage(size_t size) : size_(size) {}

    T Next(T value)
    {
        window_.push_back(value);
        sum_ += value;
        if (window_.size() > size_)
        {
            sum_ -= window_.front();
            window_.pop_front();
        }
        return sum_ / window_.size();
    }

private:
    std::deque<T> window_;
    T sum_ = 0;
    size_t size_;
};

void calculateThreshold(cv::Mat inputFrameGRAY)
{
    blackFrame = cv::Mat::zeros(frame.size(), CV_8UC1);
    // Ensure inputFrameGRAY is a single-channel grayscale image
    if (inputFrameGRAY.channels() != 1)
    {
        cerr << "Error: inputFrameGRAY must be a single-channel grayscale image." << endl;
        return;
    }

    // Ensure frame is properly initialized
    if (frame.empty())
    {
        cerr << "Error: frame is not initialized." << endl;
        return;
    }

    vector<int> thresh(frame.size().height);
    for (int i = 0; i < frame.size().height; i++)
    {
        int sumCount = 0;
        int sumThresh = 0;
        for (int j = 0; j < frame.size().width; j++)
        {
            Vec3b bgrValue = inputFrameGRAY.at<Vec3b>(i, j);
            // Access grayscale pixel value correctly
            //cout << "Blue at coord: " << static_cast<int>(bgrValue[0]) << endl;
            if(inputFrameGRAY.at<uchar>(i, j) > 140){
                sumThresh += inputFrameGRAY.at<uchar>(i, j);
                sumCount++;
            }
        }
        //cout << "Sum Thresh: " << sumThresh << endl;
        //cout << "Divided By: " << sumCount << endl;
        if (sumCount > 0) // Sıfıra bölmeyi önle
        {
            thresh[i] = (sumThresh / sumCount);
        }
        else
        {
            thresh[i] = 0; // Varsayılan bir değer ata
        }
        //cout << "Thresh: " << thresh[i] << endl;
    }
    for (int i = 0; i < frame.size().height; i++)
    {
        for (int j = 0; j < frame.size().width; j++)
        {
            if (inputFrameGRAY.at<uchar>(i, j) < thresh[i] + 5)
            {
                blackFrame.at<uchar>(i, j) = 0;
            }
            else
            {
                blackFrame.at<uchar>(i, j) = 255;
            }
        }
    }
    minThresh = sum(thresh)[0] / frame.size().width + 10;

    // cout << "Min: " << minThresh << " Max: " << maxThresh << endl;
}

void threshold()
{
    cvtColor(frame, frameGray, COLOR_RGB2GRAY);
    calculateThreshold(frameGray);
    inRange(blackFrame, 240, 255, frameThresh);

    // Apply morphological operations to remove noise
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(frameThresh, frameThresh, element);
    dilate(frameThresh, frameThresh, element);

    Canny(frameGray, frameEdge, 400, 600, 3, false);
    add(frameThresh, frameEdge, frameFinal);
    cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
    cvtColor(frameFinal, frameFinalDupe, COLOR_RGB2BGR);
}

void histogram()
{
    histogramLane.resize(frame.size().width);
    histogramLane.clear();

    for (int i = 0; i < frame.size().width; i++)
    {
        ROILane = frameFinal(cv::Rect(i, roiHeight, 1, 50));
        divide(255, ROILane, ROILane);
        histogramLane.push_back((int)(sum(ROILane)[0]));
        //cout << "Pushed: " << (int)(sum(ROILane)[0]) << " at " << i << endl;
    }
    cout << endl;
}

void laneFinder()
{
    vector<int>::iterator linePtr;
    linePtr = max_element(histogramLane.begin(), histogramLane.end());
    linePos = distance(histogramLane.begin(), linePtr);

    // Ters konumlandırma
    linePos = frame.size().width - linePos;

    line(frame, Point2f(linePos, roiHeight), Point2f(linePos, roiHeight + 50), Scalar(0, 255, 255), 6);
}

void laneCenterer()
{
    / Ters konumlandırma
    laneCenter = frame.size().width - (linePos - 400);

    MovingAverage<double> avg(sample_size);
    laneCenterAvg = laneCenter;

    line(frame, Point2f(laneCenterAvg, roiHeight), Point2f(laneCenterAvg, roiHeight + 50), Scalar(255, 255, 0), 5);
    line(frame, Point2f(frameCenter, roiHeight), Point2f(frameCenter, roiHeight + 50), Scalar(255, 0, 0), 5);

    result = laneCenterAvg - frameCenter;

    if (result > 350) { result = 350; }
    if (result < -350) { result = -350; }

    mapped_result = mapper(result, -350, 350, minCmd, maxCmd);
    if (mapped_result < minCmd)
    {
        mapped_result = minCmd;
    }
    if (mapped_result > maxCmd)
    {
        mapped_result = maxCmd;
    }
}

void laneDetection()
{
    cvtColor(frame, rgb, COLOR_BGR2RGB);
    threshold();
    histogram();
    laneFinder();
    laneCenterer();

    ss.str(" ");
    ss.clear();
    ss << "Result = " << result;
    putText(frame, ss.str(), Point2f(1, 50), 0, 1, Scalar(0, 0, 255), 2);

    ss.str(" ");
    ss.clear();
    ss << "Mapped Result = " << mapped_result;
    putText(frame, ss.str(), Point2f(1, 150), 0, 1, Scalar(0, 0, 255), 2);
}

void setup()
{
    // Şerit takibi kamerası çözünürlük ayarı
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    // Şerit takibi kamerasının açıldığını kontrol et
    if (!cap.isOpened())
    {
        cout << "Can't open camera!" << endl;
    }
    else
    {
        cout << "Camera ready!" << endl;
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
        tcpSocket.Send("#serit*");
    },
    [](int errorCode, std::string errorMessage){
        cout << errorCode << " : " << errorMessage << endl;
    });

    setup();
    while (working)
    {
        cap >> frame;
        // Şerit takibi
        laneDetection();

        // Şerit takibi görüntülerini göster
        namedWindow("Frame", WINDOW_KEEPRATIO);
        resizeWindow("Frame", 1280, 720);
        imshow("Frame", frame);

        namedWindow("Thresh", WINDOW_KEEPRATIO);
        resizeWindow("Thresh", 1280, 720);
        imshow("Thresh", frameThresh);

        // TCP'ye gönderilecek komut
        gonder = "SN" + to_string(mapped_result);
        serit_gonder = "#" + gonder + "\n*";
        cout << "Gonderilen: " << serit_gonder << endl;

        if (isConnected) {
            tcpSocket.Send(serit_gonder);
        }

        // ESC tuşu basıldığında programı durdur
        if (waitKey(1) == 27)
        {
            destroyAllWindows();
            working = false;
            return 0;
        }
    }
    return 0;
}
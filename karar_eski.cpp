#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include "CppLinuxSerial/SerialPort.hpp"
#include <chrono>
#include <functional>
#include "std_msgs/msg/string.hpp"

using namespace mn::CppLinuxSerial;
using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;
using namespace std;

SerialPort autopilot("/dev/ttyUSB0", 500000);
SerialPort copilot("/dev/ttyUSB1", 500000);

bool sola_don, saga_don, geri, dur = false;
bool ileri = true;
int counter = 0;
bool right_lane, left_lane = false;

bool serittakibi = false;
// bool ort = true;
bool engel_var = false;
bool normalsurus = true;
bool isLaneDetected = false;

#define signCamHDefault 1650

int komut_publisher;
std::string buffer;
std::string message;
std::string data_gelen;
int engel_zaman = 0;
int64_t start, stop;
int interval = 1000;
string tabela_verisi, on_serit_verisi, sol_serit_verisi, arka_serit_verisi;
string engel_verisi;
int engel_verisi_int;

bool durakTamamlandi = true;
bool dortmetre = false;
int direksiyon_komut, on_direksiyon_komut, arka_direksiyon_komut, sol_direksiyon_komut, leftFirstLane;
bool onGonderildi = false;
string tekerTurRaw;
float currentPos, initialPos;

const int durak_bekleme = 5;

bool engel_birinci_durus = false;
bool engel_ikinci_geri = false;
bool engel_ucuncu_dur = false;
bool engel_dort_double_ack_sol = false;
bool engel_dort_double_ileri1 = false;
bool engel_dur = false;
bool engel_duzle = false;

bool engeldenKacti = true;
bool engel_yok = false;
bool cikarken_engel_gordu = false;
bool gonderildi = false;

const int durakThreshold = 40;
const int lampThreshold = 90;
int signSize, durakSize, lampSize = 0;
string signName = "NULL";

int checkStartingLane = 0;
string startingLane = "right";

int mapper(int value, int fromLow, int fromHigh, int toLow, int toHigh)
{
  if (fromHigh == fromLow)
  {
    std::cerr << "Error: Division by zero in mapper function" << std::endl;
    return toLow; // or some default value
  }
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void otopilotaGonder(int komut)
{
  cout << "Otopilota Gonderilen komut: " << komut << endl;
  autopilot.Write(std::to_string(komut));
  komut_publisher = komut;
}

void copilotaGonder(int selectedServo, int position)
{
  if (position > 2300)
  {
    position = 2300;
  }
  else if (position < 700)
  {
    position = 700;
  }
  ostringstream ss;
  ss << to_string(selectedServo) << "," << to_string(position) << ";";
  cout << "Copilota gönderilen komut: " << ss.str() << endl;
  copilot.Write(ss.str());
}

bool engelAdimlari[15] = {false};
bool rightDone = false;

void sagdaki_engel_alg()
{
  if (interval >= 1000)
  {
    start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    engel_zaman++;
  }
  stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  interval = stop - start;
  cout << "engel zaman: " << engel_zaman << endl;
  cout << "Moved: " << currentPos - initialPos << endl;

  if (engel_var and !engelAdimlari[0])
  {
    if (!gonderildi)
    {
      otopilotaGonder(500);
      engel_zaman = 0;
      gonderildi = true;
    }
    if (engel_zaman > 1 and engel_zaman <= 2)
    {
      otopilotaGonder(1300);
    }
    if (engel_zaman > 2)
    {
      engelAdimlari[0] = true;
      gonderildi = false;
    }
  }

  if (engelAdimlari[0] and !engelAdimlari[1])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(900);
      gonderildi = true;
    }
    if (engel_zaman > 1)
    {
      engelAdimlari[1] = true;
      gonderildi = false;
      initialPos = currentPos;
    }
  }
  if (engelAdimlari[1] and !engelAdimlari[2])
  {
    if (currentPos - initialPos >= 3)
    {
      if (!gonderildi)
      {
        engel_zaman = 0;
        otopilotaGonder(500);
        gonderildi = true;
      }
      if (engel_zaman > 1)
      {
        engel_zaman = 0;
        engelAdimlari[2] = true;
        gonderildi = false;
      }
    }
    else if (engel_zaman >= 1)
    {
      otopilotaGonder(mapper(arka_direksiyon_komut, 201, 401, 401, 201));
    }
  }
  if (engelAdimlari[2] and !engelAdimlari[3])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(15);
      gonderildi = true;
    }
    if (engel_zaman > 1 and engel_zaman <= 2)
    {
      otopilotaGonder(201);
    }
    if (engel_zaman > 2)
    {
      engelAdimlari[3] = true;
      gonderildi = false;
    }
  }
  if (engelAdimlari[3] and !engelAdimlari[4])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(600);
      gonderildi = true;
    }
    if (engel_zaman > 2)
    {
      engelAdimlari[4] = true;
      gonderildi = false;
      initialPos = currentPos;
    }
  }
  if (engelAdimlari[4] and !engelAdimlari[5])
  {
    if (currentPos - initialPos >= 4.5)
    {
      if (!gonderildi)
      {
        engel_zaman = 0;
        otopilotaGonder(500);
        gonderildi = true;
      }
      else if (engel_zaman > 1 and engel_zaman <= 2)
      {
        otopilotaGonder(1100);
      }
      else if (engel_zaman > 2)
      {
        gonderildi = false;
        engelAdimlari[5] = true;
        initialPos = currentPos;

        engelAdimlari[6] = true;
        engelAdimlari[7] = true;
      }
    }
  }
  if (engelAdimlari[7] and !engelAdimlari[8])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(200);
      gonderildi = true;
    }
    if (engel_zaman > 1 and engel_zaman <= 2)
    {
      otopilotaGonder(370);
    }
    if (engel_zaman > 2)
    {
      gonderildi = false;
      engelAdimlari[8] = true;
    }
  }
  if (engelAdimlari[8] and !engelAdimlari[9])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(580);
      gonderildi = true;
    }
    if (engel_zaman > 1)
    {
      gonderildi = false;
      engelAdimlari[9] = true;
      initialPos = currentPos;
    }
  }
  if (engelAdimlari[9] and !engelAdimlari[10])
  {
    if (currentPos - initialPos >= 2 and isLaneDetected)
    {
      rightDone = true;
    }
    if (rightDone)
    {
      if (!gonderildi)
      {
        engel_zaman = 0;
        otopilotaGonder(500);
        gonderildi = true;
      }
      else if (engel_zaman > 1 and engel_zaman <= 2)
      {
        otopilotaGonder(1100);
      }
      else if (engel_zaman > 2)
      {
        gonderildi = false;
        engelAdimlari[10] = true;
        engeldenKacti = true;
        normalsurus = true;
        engel_var = false;
        rightDone = false;
        return;
      }
    }
  }

  for (int i = 0; i < 11; i++)
  {
    cout << "Sag Adim " << i + 1 << ": " << engelAdimlari[i] << endl;
  }
  cout << "Engel var: " << engel_var << endl
       << endl
       << endl;
}

bool solEngelAdimlari[15] = {false};
bool leftDone = false;

void soldaki_engel_alg()
{
  if (interval >= 1000)
  {
    start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    engel_zaman++;
  }
  stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  interval = stop - start;
  cout << "engel zaman: " << engel_zaman << endl;
  cout << "Moved: " << currentPos - initialPos << endl;

  if (engel_var and !solEngelAdimlari[0])
  {
    if (!gonderildi)
    {
      otopilotaGonder(500);
      engel_zaman = 0;
      gonderildi = true;
    }
    if (engel_zaman > 1 and engel_zaman <= 2)
    {
      otopilotaGonder(1300);
    }
    if (engel_zaman > 2)
    {
      solEngelAdimlari[0] = true;
      gonderildi = false;
    }
  }

  if (solEngelAdimlari[0] and !solEngelAdimlari[1])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(900);
      gonderildi = true;
    }
    if (engel_zaman > 1)
    {
      solEngelAdimlari[1] = true;
      gonderildi = false;
      initialPos = currentPos;
    }
  }
  if (solEngelAdimlari[1] and !solEngelAdimlari[2])
  {
    if (currentPos - initialPos >= 3)
    {
      if (!gonderildi)
      {
        engel_zaman = 0;
        otopilotaGonder(500);
        gonderildi = true;
      }
      if (engel_zaman > 1)
      {
        engel_zaman = 0;
        solEngelAdimlari[2] = true;
        gonderildi = false;
      }
    }
    else if (engel_zaman >= 1)
    {
      otopilotaGonder(mapper(arka_direksiyon_komut, 201, 401, 401, 201));
    }
  }
  if (solEngelAdimlari[2] and !solEngelAdimlari[3])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(200);
      gonderildi = true;
    }
    if (engel_zaman > 1 and engel_zaman <= 2)
    {
      otopilotaGonder(370);
    }
    if (engel_zaman > 2)
    {
      solEngelAdimlari[3] = true;
      gonderildi = false;
    }
  }
  if (solEngelAdimlari[3] and !solEngelAdimlari[4])
  {
    if (!gonderildi)
    {
      engel_zaman = 0;
      otopilotaGonder(580);
      gonderildi = true;
    }
    if (engel_zaman > 2)
    {
      solEngelAdimlari[4] = true;
      gonderildi = false;
      initialPos = currentPos;
    }
  }
  if (solEngelAdimlari[4] and !solEngelAdimlari[5])
  {
    if (currentPos - initialPos >= 2 and isLaneDetected)
    {
      leftDone = true;
    }
    if (leftDone)
    {
      if (!gonderildi)
      {
        engel_zaman = 0;
        otopilotaGonder(500);
        gonderildi = true;
      }
      else if (engel_zaman > 1 and engel_zaman <= 2)
      {
        otopilotaGonder(1100);
      }
      else if (engel_zaman > 2)
      {
        gonderildi = false;
        solEngelAdimlari[5] = true;
        engeldenKacti = true;
        normalsurus = true;
        engel_var = false;
        leftDone = false;
        return;
      }
    }
  }
  for (int i = 0; i < 11; i++)
  {
    cout << "Sol Adim " << i + 1 << ": " << solEngelAdimlari[i] << endl;
  }
  cout << "Engel var: " << engel_var << endl
       << endl
       << endl;
}

bool durakAdimlar[10] = {false};
bool durak_tabelasi_goruldu = false;
int durak_zaman = 0;
bool durakDone = false;
int entranceDistance = 0;
bool adim5basladi = false;

void durakAlgoritmasi()
{
  if (interval >= 1000)
  {
    start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    durak_zaman++;
  }
  stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  interval = stop - start;
  cout << "durak zaman: " << durak_zaman << endl;
  cout << "Moved: " << currentPos - initialPos << endl;

  if (!durakAdimlar[0])
  {
    if (!gonderildi)
    {
      otopilotaGonder(500);
      durak_zaman = 0;
      gonderildi = true;
    }
    if (durak_zaman > 1 and durak_zaman <= 2)
    {
      otopilotaGonder(1300);
    }
    if (durak_zaman > 2)
    {
      durakAdimlar[0] = true;
      gonderildi = false;
      initialPos = currentPos;
    }
  }

  if (durakAdimlar[0] and !durakAdimlar[1])
  {
    if (!gonderildi)
    {
      durak_zaman = 0;
      otopilotaGonder(200);
      gonderildi = true;
    }
    if (durak_zaman > 1 and durak_zaman <= 2)

      otopilotaGonder(370);

    if (durak_zaman > 2)
    {
      durakAdimlar[1] = true;
      initialPos = currentPos;
      gonderildi = false;
    }
  }

  if (durakAdimlar[1] and !durakAdimlar[2])
  {
    if (!gonderildi)
    {
      durak_zaman = 0;
      otopilotaGonder(580);
      gonderildi = true;
    }
    if (durak_zaman > 2)
    {
      durakAdimlar[2] = true;
      initialPos = currentPos;
      gonderildi = false;
      initialPos = 0;
    }
  }
  if (durakAdimlar[2] and !durakAdimlar[3])
  {
    if (currentPos - initialPos >= 2 and isLaneDetected)
    {
      entranceDistance = currentPos - initialPos;
      durakDone = true;
    }
    if (durakDone)
    {
      if (!gonderildi)
      {
        durak_zaman = 0;
        otopilotaGonder(500);
        gonderildi = true;
      }
      else if (durak_zaman > 1 and durak_zaman <= 2)
      {
        otopilotaGonder(1100);
      }
      else if (durak_zaman > 2)
      {
        gonderildi = false;
        durakAdimlar[3] = true;
        initialPos = currentPos;
        durakDone = false;
      }
    }
  }

  if (durakAdimlar[3] and !durakAdimlar[4])
  {
    if (!gonderildi)
    {
      durak_zaman = 0;
      otopilotaGonder(15);
      gonderildi = true;
    }
    if (durak_zaman > durak_bekleme and durak_zaman <= durak_bekleme + 1)
    {
      otopilotaGonder(220);
    }
    if (durak_zaman > 6)
    {
      durakAdimlar[4] = true;
      initialPos = currentPos;
      gonderildi = false;
    }
  }

  if (durakAdimlar[4] and !durakAdimlar[5])
  {
    if (!gonderildi)
    {
      durak_zaman = 0;
      otopilotaGonder(580);
      gonderildi = true;
    }
    if (durak_zaman > 2)
    {
      durakAdimlar[5] = true;
      gonderildi = false;
      initialPos = currentPos;
    }
  }

  if (durakAdimlar[5] and !durakAdimlar[6])
  {
    if (!adim5basladi)
    {
      durak_zaman = 0;
      adim5basladi = true;
    }
    if (currentPos - initialPos > entranceDistance + 2 and !isLaneDetected)
    {
      if (durak_zaman > 1 and durak_zaman <= 2)
      {
        otopilotaGonder(500);
      }
      if (durak_zaman > 2 and durak_zaman <= 3)
      {
        otopilotaGonder(1100);
      }
      if (durak_zaman > 3 and durak_zaman <= 4)
      {
        otopilotaGonder(580);
      }
      if (durak_zaman > 4)
      {
        otopilotaGonder(on_direksiyon_komut);
      }
    }
    else
    {
      if (currentPos - initialPos >= 2 and isLaneDetected)
      {
        durakDone = true;
      }
      if (durakDone)
      {
        if (!gonderildi)
        {
          durak_zaman = 0;
          otopilotaGonder(500);
          gonderildi = true;
        }
        else if (durak_zaman > 1 and durak_zaman <= 2)
        {
          otopilotaGonder(1100);
        }
        else if (durak_zaman > 2)
        {
          gonderildi = false;
          durakAdimlar[6] = true;
          initialPos = currentPos;
          durakDone = false;
          durakTamamlandi = true;
          normalsurus = true;
        }
      }
    }
  }

  for (int i = 0; i < 7; i++)
  {
    cout << "Adim " << i + 1 << ": " << durakAdimlar[i] << endl;
  }
  cout << "Durak done: " << durakDone << endl;
  cout << "Entrance Dist: " << entranceDistance << endl;
}

void karar()
{
  // if(engel_verisi >= 0 && engel_verisi <31)
  if (engel_verisi == "Engel var")
  {
    normalsurus = false;
    engel_var = true;
    ileri = false;
    dortmetre = false;
    //otopilotaGonder(500);
    cout << "Engel var" << endl;
  }

  if (engel_verisi == "4 metre") /////////////////////////
  {
    dortmetre = true;
  }
  else
  {
    dortmetre = false;
  }
  if (engel_verisi == "Engel yok")
  {
    engel_var = false;
    dortmetre = false;
  }
  /* if (engel_verisi == "Engel yok" and normalsurus == true)
   {
     engel_var = false;
     dortmetre = false;
   }*/

  if (signName == "20_hiz_limiti" or signName == "trafik_isigi_yesil")
  {
    ileri = true;
  }

  /*if (signName == "durak" && signSize >= durakThreshold)
  {
    normalsurus = false;
    ileri = false;
    durakTamamlandi = false;
  }*/

  if (signName == "dur" or signName == "trafik_isigi_kirmizi")
  {
    ileri = false;
  }
}

void action()
{
  cout << "Durak tamamlandı: " << durakTamamlandi << endl;
  if (!durakTamamlandi)
  {
    durakAlgoritmasi();
  }

  cout << "ileri: " << ileri << endl;
  if (engel_var and !ileri and !normalsurus)
  {
    engeldenKacti = false;
    // ileri = false;
  }

  if (!engeldenKacti and checkStartingLane > 5)
  {
    sagdaki_engel_alg();
  }

  else if (!engel_var and normalsurus)
  {
    if (counter < 3)
    {
      counter++;
      // serit_verisi = "100";
      otopilotaGonder(direksiyon_komut); // mapper(direksiyon_komut, 0, 200, 1201, 1401)
    }

    else if (ileri)
    {
      cout << "ileri" << endl;
      otopilotaGonder(600);
      counter = 0;
    }
  }
  /* else if(geri)
   {
     gonder(900);
     geri = false;
   }
   if(dur and !normalsurus)
   {
     gonder(500);
     dur = false;
   }*/
  /* if(engel_var and !ileri)
   {
     gonder(500);
   }
   if(!engel_var and ileri)
   {
     gonder(600);
   }*/
}

void printCommands()
{
  cout << "İleri: " << ileri << endl;
  cout << "Sağa: " << saga_don << endl;
  cout << "Sola: " << sola_don << endl;
  cout << "Engel: " << engel_var << endl;
  //  cout << "normalsurus: " << normalsurus << endl;
  //  cout << "Kavşak: " << kavsak << endl;
  // cout << "Result: " << mapped_result << endl;
  // cout << "Right Lane: " << right_lane << endl;
  // cout << "Left Lane: " << left_lane << endl;
  cout << endl
       << "------------------------------------" << endl
       << endl;
}

std::vector<std::string> parseStringByComma(const std::string &str)
{
  std::vector<std::string> result;
  std::stringstream ss(str);
  std::string item;

  while (std::getline(ss, item, ','))
  {
    result.push_back(item);
  }

  return result;
}
/*
std::vector<float> parseStringByLine(const std::string &str)
{
  std::vector<float> result;
  std::stringstream ss(str);
  std::string item;

  while (std::getline(ss, item, '\n'))
  {
    // cout << "Item is: " << item << endl;
    result.push_back(stof(item));
  }

  return result;
}
*/
class KararSubscriber : public rclcpp::Node
{
public:
  KararSubscriber()
      : Node("Karar_Subscriber")
  {
    esubscription_ = this->create_subscription<std_msgs::msg::String>("engel", 10, std::bind(&KararSubscriber::topic_callback, this, _1));
    tsubscription_ = this->create_subscription<std_msgs::msg::String>("tabela", 10, std::bind(&KararSubscriber::ttopic_callback, this, _1));
    ssubscription_ = this->create_subscription<std_msgs::msg::String>("onSerit", 10, std::bind(&KararSubscriber::stopic_callback, this, _1));
    arkaSeritSubscription_ = this->create_subscription<std_msgs::msg::String>("arkaSerit", 10, std::bind(&KararSubscriber::arkaSeritTopic_callback, this, _1));
    solSeritSubscription_ = this->create_subscription<std_msgs::msg::String>("solSerit", 10, std::bind(&KararSubscriber::solSeritTopic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("lyeclabs", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&KararSubscriber::gomulu, this));
  }

private:
  void topic_callback(const std_msgs::msg::String &emsg) const
  {
    RCLCPP_INFO(this->get_logger(), "I Eheard:  '%s'", emsg.data.c_str());
    //  engel_verisi =stoi(emsg.data.c_str());
    engel_verisi = emsg.data.c_str();
  }

  void ttopic_callback(const std_msgs::msg::String &tmsg) const
  {
    RCLCPP_INFO(this->get_logger(), "I Theard: '%s'", tmsg.data.c_str());
    tabela_verisi = tmsg.data.c_str();
    vector<string> signData = parseStringByComma(tabela_verisi);
    signName = signData[0];
    cout << "Converting sign size: " << signData[1] << endl;
    signSize = stoi(signData[1]);
    cout << "Converted sign size" << endl;
    cout << "Sign Name: " << signName << endl;
    cout << "Sign Size: " << signSize << endl;
  }

  void stopic_callback(const std_msgs::msg::String &smsg) const
  {
    RCLCPP_INFO(this->get_logger(), "Sag serit verisi: '%s'", smsg.data.c_str());
    on_serit_verisi = smsg.data.c_str();

    // cout << "Direksiyon Komut: " << direksiyon_komut << endl;
    /*cout << "Left First Lane: " << leftFirstLane << endl;
    cout << "Durak Tamamlandı: " << durakTamamlandi << endl;
    cout << "Is lane detected: " << isLaneDetected << endl;*/

    std::vector<std::string> parsedStrings = parseStringByComma(on_serit_verisi);

    on_direksiyon_komut = stoi(parsedStrings[0]);
    isLaneDetected = stoi(parsedStrings[3]);
    cout << "Is lane detected: " << isLaneDetected << endl;

    // leftFirstLane = stoi(parsedStrings[1]);
    // durakTamamlandi = stoi(parsedStrings[2]);
  }

  void arkaSeritTopic_callback(const std_msgs::msg::String &smsg) const
  {
    RCLCPP_INFO(this->get_logger(), "Arka serit verisi: '%s'", smsg.data.c_str());
    arka_serit_verisi = smsg.data.c_str();

    // cout << "Direksiyon Komut: " << direksiyon_komut << endl;
    /*cout << "Left First Lane: " << leftFirstLane << endl;
    cout << "Durak Tamamlandı: " << durakTamamlandi << endl;
    cout << "Is lane detected: " << isLaneDetected << endl;*/

    std::vector<std::string> parsedStrings = parseStringByComma(arka_serit_verisi);

    arka_direksiyon_komut = stoi(parsedStrings[0]);
    /*leftFirstLane = stoi(parsedStrings[1]);
    durakTamamlandi = stoi(parsedStrings[2]);
    isLaneDetected = stoi(parsedStrings[3]);*/
  }

  void solSeritTopic_callback(const std_msgs::msg::String &smsg) const
  {
    RCLCPP_INFO(this->get_logger(), "Sol serit verisi: '%s'", smsg.data.c_str());
    sol_serit_verisi = smsg.data.c_str();

    // cout << "Direksiyon Komut: " << direksiyon_komut << endl;
    /*cout << "Left First Lane: " << leftFirstLane << endl;
    cout << "Durak Tamamlandı: " << durakTamamlandi << endl;
    cout << "Is lane detected: " << isLaneDetected << endl;*/

    std::vector<std::string> parsedStrings = parseStringByComma(sol_serit_verisi);

    sol_direksiyon_komut = stoi(parsedStrings[0]);
    /*leftFirstLane = stoi(parsedStrings[1]);
    durakTamamlandi = stoi(parsedStrings[2]);
    isLaneDetected = stoi(parsedStrings[3]);*/
  }

  void gomulu()
  {
    auto message = std_msgs::msg::String();
    message.data = to_string(komut_publisher);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    vector<float> tekerTurVec;
    copilot.Write("OK;");
    if (copilot.Available())
    {
      copilot.Read(tekerTurRaw);
      cout << "Data: " << tekerTurRaw << endl;
      currentPos = stof(tekerTurRaw);
      tekerTurRaw = "";
      /*tekerTurVec = parseStringByLine(tekerTurRaw);
      currentPos = tekerTurVec[tekerTurVec.size() - 1];*/
    }
    cout << "Teker tur: " << currentPos << endl;

    if (checkStartingLane < 3)
    {
      if (engel_verisi == "Engel var" or engel_verisi == "4 metre")
      {
        startingLane = "right";
      }
      else
      {
        startingLane = "left";
      }
      cout << "Starting from " << startingLane << endl;
      checkStartingLane++;
    }
    else if (checkStartingLane < 4)
    {
      checkStartingLane++;
      // copilotaGonder(5, 1450);
    }

    onGonderildi = false;

    /*if (startingLane == "right")
    {
      if (!onGonderildi)
      {
        direksiyon_komut = on_direksiyon_komut;
        onGonderildi = true;
      }
      else
      {
        direksiyon_komut = arka_direksiyon_komut;
        onGonderildi = false;
      }
    }
    else{
      direksiyon_komut = sol_direksiyon_komut;
    }*/
    direksiyon_komut = on_direksiyon_komut;

    karar();
    action();
    cout << endl
         << endl;
    // actionengel();
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esubscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tsubscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ssubscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arkaSeritSubscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr solSeritSubscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  autopilot.Open();
  copilot.Open();
  copilotaGonder(3, 1710);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KararSubscriber>());
  rclcpp::shutdown();
  autopilot.Close();
  copilot.Close();
  return 0;
}
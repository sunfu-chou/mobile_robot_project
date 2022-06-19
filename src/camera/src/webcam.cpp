#include <stdio.h>
#include <math.h>
#include <random>

#include <iostream>
#include <signal.h>
#include <stdlib.h> /* srand, rand */
#include <unistd.h>
#include <mutex>
#include "Yolo3Detection.h"
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>

#define PI acos(-1)

bool gRun;
bool SAVE_RESULT = false;

void sig_handler(int signo)
{
  std::cout << "request gateway stop\n";
  gRun = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "real_camera");                                           //////////
  ros::NodeHandle n;                                                              //////////
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("coneXP", 1000);  //////////
  ros::Rate loop_rate(30);                                                        //////////
  signal(SIGINT, sig_handler);
  std::string net = "/home/nvidia/tkDNN/build/yolo4tiny_obj_fp32.rt";
  char ntype = 'y';
  int n_classes = 2;
  int n_batch = 1;
  bool show = false;  // 選擇是否要開一個視窗顯示相機畫面
  float conf_thresh = 0.3;
  if (!show)
    SAVE_RESULT = true;

  tk::dnn::Yolo3Detection yolo;

  tk::dnn::DetectionNN* detNN;  // detnn相關的函式在tkDNN/include/DetctionNN.h裡
  detNN = &yolo;
  detNN->init(net, n_classes, n_batch, conf_thresh);

  gRun = true;
  int ind = 0;
  // std::string str="v4l2src device=/dev/video1  extra-controls=""s,exposure_auto=1,exposure_absolute=200"" ! video/x-raw , width=1280, height=720 ! videoconvert ! appsink";
  // system("v4l2-ctl --set-fmt-video=width=1280,height=720");

  // system("v4l2-ctl -d /dev/video1 -c backlight_compensation=1");
  // system("v4l2-ctl -d /dev/video1 -c brightness=100");
  // system("v4l2-ctl -d /dev/video1 -c exposure_absolute=100");
  // system("v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0");
  // system("v4l2-ctl -d /dev/video1 -c white_balance_temperature=10000");

  // cv::VideoCapture cap(str,cv::CAP_GSTREAMER);

  cv::VideoCapture cap(0);
  // system("v4l2-ctl -d /dev/video1 -c exposure_auto=1");
  // system("v4l2-ctl -d /dev/video1 -c exposure_absolute=200");
  // std::cout<<"exposure:"<<cap.get(cv::CAP_PROP_EXPOSURE)<<std::endl;
  // std::cout<<"width1: "<<cap.get(cv::CAP_PROP_FRAME_WIDTH)<<" height: "<<cap.get(cv::CAP_PROP_FRAME_HEIGHT)<<std::endl;
  // cap.set(cv::CAP_PROP_FRAME_WIDTH,1280.0);
  // cap.set(cv::CAP_PROP_FRAME_HEIGHT,720.0);
  // cap.set(cv::CAP_PROP_EXPOSURE,200);
  std::cout << "width2: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << " height: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

  if (!cap.isOpened())
    gRun = false;
  else
    std::cout << "camera started\n";

  cv::VideoWriter resultVideo;
  bool a = false;
  if (a)
  {
    int w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    resultVideo.open("/home/nvidia/test2/result2.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(w, h));
  }

  cv::Mat frame;
  if (show)
    cv::namedWindow("detection", cv::WINDOW_NORMAL);

  std::vector<cv::Mat> batch_frame;
  std::vector<cv::Mat> batch_dnn_input;
  //////////while(gRun) {//////////
  while (ros::ok() && gRun)
  {
    /// init array to save cone data
    std_msgs::Float64MultiArray status;  //////////
    int max_cone = 10;
    float cone_data[10][4] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
    float* p = &cone_data[0][0];
    ///
    batch_dnn_input.clear();
    batch_frame.clear();
    // std::cout<<cap.get(cv::CAP_PROP_EXPOSURE)<<std::endl;
    for (int bi = 0; bi < n_batch; ++bi)
    {                //每次處理的畫面數量迴圈,不過n_batch是設為1,所以可以不用管
      cap >> frame;  //從相機抓畫面出來
      if (!frame.data)
      {  //沒抓到畫面的處理,可以不用管
        std::cout << "no data" << std::endl;
        break;
      }

      batch_frame.push_back(frame);

      // this will be resized to the net format
      batch_dnn_input.push_back(frame.clone());
    }
    if (!frame.data)
    {
      break;
    }

    // inference
    detNN->update(batch_dnn_input, n_batch);  //這行不看應該也沒差
                                              ///
    detNN->print(batch_frame, p);             //參考tkDNN/include/DetctionNN.h //transfer pointer to DetctionNN.h
    for (int i = 0; i < max_cone; i++)
    {
      if (cone_data[i][0] != 0)
      {
        if (cone_data[i][2] >= 0.9)
        {
          float distance;
          float angle;
          // float radian = fabs(cone_data[i][1] * 100) * PI / 180;
          // float x_dis = 2.5 * (cone_data[i][0] * cos(radian) - 0.5) + 0.46;
          // float y_dis = 0.64 * cone_data[i][0] * sin(radian);
          // distance = sqrt(pow(x_dis, 2) + pow(y_dis, 2));
          // angle = atan(y_dis / x_dis) * 180 / PI;
          // angle = cone_data[i][1] < 0 ? angle : angle *= -1;+
            // distance = 36.0/11.0*(cone_data[i][0]-0.54)+0.44;
            // angle = 1.16*(-cone_data[i][1]+0.208)-0.233;
            // distance = cone_data[i][0];
            // angle = -cone_data[i][1];
          distance = sqrt(0.32 * 0.32 + cone_data[i][0] * cone_data[i][0] + 2 * cone_data[i][0] * 0.32 * cos(abs(cone_data[i][1])));
          angle = (0.32 * 0.32 + distance * distance - cone_data[i][0] * cone_data[i][0]) / (2 * 0.32 * distance);
          angle = acos(angle);
          angle = copysign(angle, -cone_data[i][1]);
          status.data.push_back(distance);
          status.data.push_back(angle);
          status.data.push_back(cone_data[i][3]);
          printf("\ncone:%d  distance:%f  degree:%f  prob:%f  color:%f\n", i, distance, angle, cone_data[i][2], cone_data[i][3]);
        }
      }
    }
    ///
    if (show)
    {
      for (int bi = 0; bi < n_batch; ++bi)
      {
        cv::imshow("detection", batch_frame[bi]);
        cv::waitKey(1);
      }
    }
    if (n_batch == 1 && a)
      resultVideo << frame;
    pub.publish(status);  ///////////
    ros::spinOnce();      //////////
    loop_rate.sleep();    //////////
  }
  //後面的不重要,可以不用管
  std::cout << "detection end\n";
  double mean = 0;

  std::cout << COL_GREENB << "\n\nTime stats:\n";
  std::cout << "Min: " << *std::min_element(detNN->stats.begin(), detNN->stats.end()) / n_batch << " ms\n";
  std::cout << "Max: " << *std::max_element(detNN->stats.begin(), detNN->stats.end()) / n_batch << " ms\n";
  for (int i = 0; i < detNN->stats.size(); i++)
    mean += detNN->stats[i];
  mean /= detNN->stats.size();
  std::cout << "Avg: " << mean / n_batch << " ms\t" << 1000 / (mean / n_batch) << " FPS\n" << COL_END;

  return 0;
}

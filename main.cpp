#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct StreamProp
{
    const int camera_id;             //Camera index
    const int api_id;                //API id
    const int frame_width;           //Frame width
    const int frame_height;          //Frame height
    const int camrea_exposure_mode;  /*Camera v4l2 exposure mode
                                      1->manually exposure
                                      3->auto exposure
                                      */
    const int camera_exposure;       //Camera v4l2 exposure value (78-10000)
    int frame_threshold[4];          //ROI rect (start_x, end_x, start_y, end_y)
    int camera_angle_V;              //Camera angle of vectical
    int waitkey_delay;               //Delay of waitkey between processing 2 frame
    int read_count;                  //Frame read count
};

struct RectProp
{
    int contour_depth;               //Contour detect depth
    double canny_threshold;          //Canny threshold max
    double area_threshold;           //Squart area threshold max
    double cosine_max_threshold;     //Cosine max threshold of each edge line
    double approx_epsilon;           //Approx epsilon of lenth calculating
};

struct ColorRange
{
    const std::string color_name;    //Color name
    int lowerb[3];                   //HSV lowerb
    int upperb[3];                   //HSV upperb
    int threshold;                   //Minimal area threshold
    int color_location;              //Color at what the crossing tag number is
};

struct LineData
{
  int area_threshold_min;            //Ball center vibrate (pixels), when larger then send ball location
  int area_threshold_max;            //Line area threshold max (pixels), larger as crossing
  int line_axis_H;                   //Line horizental axis (pixels), determined by camera's location
  int line_axis_H_vibrate;           //Line horizental axis vibrate (pixels), when larger then make movement
  int line_main_V_s;                 //ROI of 1st line follow area start y (up->down)
  int line_main_V_e;                 //ROI of 1st line follow area end y (up->down)
  int line_dist_V_s;                 //ROI of 2nd line follow area start y (up->down)
  int line_dist_V_e;                 //ROI of 2nd line follow area end y (up->down)
  int line_dist_R_vibrate;           //Line angle rotation vibrate (pixels), when larger then make rotatement
  int line_mark_cnt;                 //Count the crossing tag on the line (not including turns)
  int line_mark_cnt_limit;           //When crossing tag count eqaled, end the current function
};

void linePosition(cv::Mat *p_input_img, cv::Scalar *p_input_lowerb, cv::Scalar *p_input_upperb, cv::Point2f *p_line_ctr, double *p_area_max)
{
    double p_area_tmp;
    unsigned int area_max_tag = 0;

    cv::Mat p_input_img_tmp = p_input_img->clone();

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

//    cv::cvtColor(p_input_img_tmp, p_input_img_tmp, cv::COLOR_BGR2HSV_FULL);
//    cv::inRange(p_input_img_tmp, *p_input_lowerb, *p_input_upperb, p_input_img_tmp);
//    imshow("HSV", p_input_img_tmp);
    cvtColor(p_input_img_tmp, p_input_img_tmp, COLOR_BGR2GRAY);
    imshow("gray", p_input_img_tmp);
    threshold(p_input_img_tmp, p_input_img_tmp, 103, 255, THRESH_BINARY_INV);
    imshow("thre", p_input_img_tmp);
    medianBlur(p_input_img_tmp, p_input_img_tmp, 3);
    cv::findContours(p_input_img_tmp, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(contours.size() > 0)
    {
      *p_area_max = contourArea(contours[area_max_tag]);
    }

    for(unsigned int i = 0; i != contours.size(); i++)
    {
      p_area_tmp = contourArea(contours[i]);

      if(p_area_tmp > *p_area_max)
      {
        *p_area_max = p_area_tmp;
        area_max_tag = i;
      }
    }

    if(contours.size() > 0)
    {
      cv::Moments momentum = cv::moments(contours[area_max_tag], false);
      *p_line_ctr = cv::Point2f(float(momentum.m10 / momentum.m00), float(momentum.m01 / momentum.m00));
      cv::drawContours(*p_input_img, contours, int(area_max_tag), cv::Scalar(255, 0, 0), 20, 8, hierarchy, 0, cv::Point());
      cv::circle(*p_input_img, *p_line_ctr, 20, cv::Scalar(0, 255, 0), -1, 8, 0);
    }
    contours.clear();
    hierarchy.clear();
}

int main()
{
//    VideoCapture capture(0);
    VideoCapture capture("/home/xx/baiduwangpan/VID_20201024_213323.mp4");
    Mat frame;
    int Cross_num = 0;
    namedWindow("std", WINDOW_AUTOSIZE);
    int thre_num = 107;
    createTrackbar("thre = ", "std", &thre_num, 255, NULL);
    StreamProp stream_prop = {
        0,           //Camera index
        cv::CAP_ANY, //API id
        640,         //Frame width
        400,         //Frame height
        1,           /*Camera v4l2 exposure mode
                                                   1->manually exposure
                                                   3->auto exposure
                                                 */
        5000,        //Camera v4l2 exposure value (78-10000)
        {
            0,   //ROI rect strat x (left->rignt)
            640, //ROI rect end x (left->rignt)
            0,   //ROI rect strat y (up->down)
            400  //ROI rect end y (up->down)
        },
        90, /*Camera angle of vectical
                                                     0->camera face to the ground
                                                     90->camera face to the front
                                                 */
        1,  //Delay of waitkey between processing 2 frame
        80  //Fream read count, as same as triers
    };
    ColorRange input_color = {"black",
                                 {0, 50, 0},
                                 {360, 255, 90},

                                 /* Not used in lineFollow */
                                 0,
                                 0};
    LineData input_line_data = {
          1000,                             //Area threshold min (pixels), smaller as noisy
          13000,                            //Line area threshold max (pixels), larger as crossing
          int(stream_prop.frame_width / 2), //Line horizental axis (pixels), determined by camera's location
          30,                               //Line horizental axis vibrate (pixels), when larger then make movement
          0,                              //ROI of 1st line follow area start y (up->down)
          200,                              //ROI of 1st line follow area end y (up->down)
          300,                              //ROI of 2nd line follow area start y (up->down)
          400,                              //ROI of 2nd line follow area end y (up->down)
          20,                               //Line angle rotation vibrate (pixels), when larger then make rotatement
          0,                                //Count the crossing tag on the line (not including turns)
          6                                 //When crossing tag count eqaled, end the current function
      };
    int slider_threshold_limit = stream_prop.frame_width * stream_prop.frame_height;
    std::string slider_console_name = std::string(input_color.color_name + " slider");
    cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
    cv::namedWindow(slider_console_name, cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("line_area_threshold_min", slider_console_name, &input_line_data.area_threshold_min, slider_threshold_limit, NULL);
    cv::createTrackbar("line_area_threshold_max", slider_console_name, &input_line_data.area_threshold_max, slider_threshold_limit, NULL);
    cv::createTrackbar("line_width", slider_console_name, &input_line_data.line_axis_H, stream_prop.frame_width, NULL);
    cv::createTrackbar("line_axis_H_vibrate", slider_console_name, &input_line_data.line_axis_H_vibrate, stream_prop.frame_width, NULL);
    cv::createTrackbar("frame_threshold[0]", slider_console_name, &stream_prop.frame_threshold[0], stream_prop.frame_width, NULL);
    cv::createTrackbar("frame_threshold[1]", slider_console_name, &stream_prop.frame_threshold[1], stream_prop.frame_width, NULL);
    cv::createTrackbar("frame_threshold[2]", slider_console_name, &stream_prop.frame_threshold[2], stream_prop.frame_height, NULL);
    cv::createTrackbar("frame_threshold[3]", slider_console_name, &stream_prop.frame_threshold[3], stream_prop.frame_height, NULL);
    cv::createTrackbar("line_main_V_s", slider_console_name, &input_line_data.line_main_V_s, stream_prop.frame_height, NULL);
    cv::createTrackbar("line_main_V_e", slider_console_name, &input_line_data.line_main_V_e, stream_prop.frame_height, NULL);
    cv::createTrackbar("line_dist_V_s", slider_console_name, &input_line_data.line_dist_V_s, stream_prop.frame_height, NULL);
    cv::createTrackbar("line_dist_V_e", slider_console_name, &input_line_data.line_dist_V_e, stream_prop.frame_height, NULL);
//    cv::createTrackbar("H_L", slider_console_name, &input_color.lowerb[0], 360, NULL);
//    cv::createTrackbar("H_U", slider_console_name, &input_color.upperb[0], 360, NULL);
//    cv::createTrackbar("S_L", slider_console_name, &input_color.lowerb[1], 255, NULL);
//    cv::createTrackbar("S_U", slider_console_name, &input_color.upperb[1], 255, NULL);
//    cv::createTrackbar("V_L", slider_console_name, &input_color.lowerb[2], 255, NULL);
//    cv::createTrackbar("V_U", slider_console_name, &input_color.upperb[2], 255, NULL);
    while(1)
    {
        capture.read(frame);
        double t = cv::getTickCount();
        if(frame.empty())
        {
          continue;
        }
        bool crossing_flag = false;
        bool straight_flag = false;
        bool serial_action = false;


        cv::Mat frame_tmp[2];
        cv::Scalar input_lowerb_tmp = cv::Scalar(input_color.lowerb[0], input_color.lowerb[1], input_color.lowerb[2]);
        cv::Scalar input_upperb_tmp = cv::Scalar(input_color.upperb[0], input_color.upperb[1], input_color.upperb[2]);
        cv::Point2f line_ctr[2];
        double area_max[2];
        resize(frame, frame, Size(640, 400));
        frame = frame(cv::Range(stream_prop.frame_threshold[2], stream_prop.frame_threshold[3]), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));
        frame_tmp[1] = frame(cv::Range(input_line_data.line_dist_V_s, input_line_data.line_dist_V_e), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));
        frame_tmp[0] = frame(cv::Range(input_line_data.line_main_V_s, input_line_data.line_main_V_e), cv::Range(stream_prop.frame_threshold[0], stream_prop.frame_threshold[1]));
        linePosition(&frame_tmp[1], &input_lowerb_tmp, &input_upperb_tmp, &line_ctr[1], &area_max[1]);
        linePosition(&frame_tmp[0], &input_lowerb_tmp, &input_upperb_tmp, &line_ctr[0], &area_max[0]);
        int line_dist_R_displacement = int(line_ctr[0].x - line_ctr[1].x);
//        cout<<endl;
//        cout<<"area_max[0] = "<<area_max[0]<<endl;
//        cout<<"area_max[1] = "<<area_max[1]<<endl;
//        cout<<"转弯 = "<<line_dist_R_displacement<<endl;

        if((area_max[0] < area_max[1] ? area_max[0] : area_max[1])  > input_line_data.area_threshold_min)
        {
          if(area_max[0] > input_line_data.area_threshold_max && abs(line_dist_R_displacement) < 50)
          {
            if(crossing_flag == false)
            {
              crossing_flag = true;
              straight_flag = false;
              putText(frame, "crossing", Point(frame.cols/2, frame.rows/2), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2,8);
              std::cout << "crossing" << ++input_line_data.line_mark_cnt << "->" << area_max[0] << std::endl;
            }
          }
          else
          {
              crossing_flag = false;
              straight_flag = true;
              putText(frame, "straight", Point(frame.cols/2, frame.rows/2), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2,8);


              if(abs(line_dist_R_displacement) < 60)//不偏转
              {
                int line_axis_H_displacement = int(line_ctr[1].x - input_line_data.line_axis_H);
//                cout<<"车离黑线偏移 = "<<line_axis_H_displacement<<endl;
                if(line_axis_H_displacement > input_line_data.line_axis_H_vibrate)
                {
                    serial_action = true;
                    putText(frame, "left", Point(frame.cols/2, frame.rows/2+20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2,8);
                }

                if(line_axis_H_displacement < -input_line_data.line_axis_H_vibrate)
                {
                    serial_action = true;
                    putText(frame, "right", Point(frame.cols/2, frame.rows/2+20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2,8);
                }
              }
              else{//需要转弯
                  if(line_dist_R_displacement > 0)//右转
                  {
                      putText(frame, "->->->->", Point(frame.cols/2, frame.rows/2+20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2,8);
                  }
                  else{//左转
                      putText(frame, "<-<-<-<-", Point(frame.cols/2, frame.rows/2+20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2,8);
                  }
              }
          }
        }
        t = (cv::getTickCount() - t) / cv::getTickFrequency();
        double fps = 1.0 / t;
        char fps_string[10];

        std::sprintf(fps_string, "FPS:%.2f", fps);
//        resize(frame, frame, Size(frame.cols*0.15, frame.rows*0.15));
        cv::putText(frame, fps_string, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::line(frame, cv::Point(input_line_data.line_axis_H), cv::Point(input_line_data.line_axis_H, frame.rows), cv::Scalar(0, 0, 0), 10);

        cv::imshow("frame", frame);

        if(waitKey(30) == 'q')
        {
          break;
        }
    }
    capture.release();
    return 0;
}

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <ctime>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <sstream>
#include <string>
#include <fstream>
#include <numeric>
#include <array>
#include "TCP/tcpSend.h"

using namespace ARRC;

TCP tcp("172.16.84.224");

constexpr std::size_t WIDTH = 640;
constexpr std::size_t HEIGHT = 360;
constexpr double ratio = WIDTH / (double)HEIGHT;
constexpr int STRAP1_LINE = 90;
constexpr int STRAP2_LINE = 170;
constexpr int STRAP3_LINE = 250;
constexpr int STRAP4_LINE = 330;
constexpr int STRAP5_LINE = 410;
constexpr int STRAP6_LINE = 490;
constexpr int STRAP7_LINE = 570;
constexpr int STRAP8_LINE = 600;
constexpr bool filter = true;
constexpr short hole_fillter_mode = 1;

int orion_status;

struct  realsense_distance
{
    float rs_x;
    float rs_y;
    float rs_z;    
};

struct rgb_data
{
    float x;
    float y;
    float z;
    float yaw;
    float roll;
    float pitch;
};

int pole_interval = 500;
int horizontal_line = 0;

int main(int argc, char **argv) try
{

    rs2::colorizer cr;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);
    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2::hole_filling_filter hole_filling(hole_fillter_mode);
    rs2::spatial_filter spat_filling;
    // rs2::align align_to_depth(RS2_STREAM_DEPTH);

    auto intr = depth_stream.get_intrinsics();
    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);
    std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();

    // cv::Mat cameraMatrix, distCoeffs;
    std::array<std::array<float ,4>,5> distance_save;

    struct realsense_distance rdist;
    struct rgb_data rgb;
    //short count = 0;
    short start_count = 0;
    int numbering = 0;
    //std::time_t
    auto real_time = std::chrono::system_clock::now();
    std::time_t sys_time = std::chrono::system_clock::to_time_t(real_time);

    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 465.33068563,0,321.98961202,0,464.52224766 ,164.37790344,0,0,1);    

    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.07103579,  0.13770855, -0.00838435, -0.00454967, -0.81693468);


    while (true)
    { 
        //std::cout << "aa" << std::endl;
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        auto aligned_frames = align_to_color.process(frames);
        auto depth_map = aligned_frames.get_depth_frame();
        auto color_map = aligned_frames.get_color_frame();
        
        if (filter)
        {
            depth_map = hole_filling.process(depth_map);
            depth_map = spat_filling.process(depth_map);
        }

        cv::Mat color(cv::Size(color_map.get_width(), color_map.get_height()), CV_8UC3, (void *)color_map.get_data(), cv::Mat::AUTO_STEP);
        if(horizontal_line > 0){
            cv::line(color, cv::Point(0,horizontal_line), cv::Point(WIDTH,horizontal_line), cv::Scalar(0, 255, 100), 5, 10);
        }

        if(pole_interval < 1280 && pole_interval > 0){
            cv::line(color, cv::Point(WIDTH/2 + pole_interval / 2 ,0), cv::Point(WIDTH/2 + pole_interval / 2,HEIGHT), cv::Scalar(255, 200, 30), 5, 16);
            cv::line(color, cv::Point(WIDTH/2 - pole_interval / 2,0), cv::Point(WIDTH/2 - pole_interval / 2,HEIGHT), cv::Scalar(255, 200, 30), 5, 16);
        }
        
        cv::line(color, cv::Point(STRAP1_LINE, 0), cv::Point(STRAP1_LINE, 360), cv::Scalar(100, 100, 0), 2, 2);
        cv::line(color, cv::Point(STRAP2_LINE, 0), cv::Point(STRAP2_LINE, 360), cv::Scalar(110, 110, 0), 2, 2);
        cv::line(color, cv::Point(STRAP3_LINE, 0), cv::Point(STRAP3_LINE, 360), cv::Scalar(120, 120, 0), 2, 2);
        cv::line(color, cv::Point(STRAP4_LINE, 0), cv::Point(STRAP4_LINE, 360), cv::Scalar(130, 130, 0), 2, 2);
        cv::line(color, cv::Point(STRAP5_LINE, 0), cv::Point(STRAP5_LINE, 360), cv::Scalar(140, 140, 0), 2, 2);
        cv::line(color, cv::Point(STRAP6_LINE, 0), cv::Point(STRAP6_LINE, 360), cv::Scalar(150, 150, 0), 2, 2);
        cv::line(color, cv::Point(STRAP7_LINE, 0), cv::Point(STRAP7_LINE, 360), cv::Scalar(160, 170, 0), 2, 2);
        cv::line(color, cv::Point(STRAP8_LINE, 0), cv::Point(STRAP8_LINE, 360), cv::Scalar(170, 170, 0), 2, 2);
        
        
        cv::imshow("set", color);

        int input_key_num = cv::waitKey(5);
        std::cout << input_key_num << std::endl;

        switch(input_key_num){
            case 84:
                horizontal_line++;
                break; 
            case 82:
                horizontal_line--;
                break;
            case 83:
                pole_interval += 2;
                break;
            case 81:
                pole_interval -= 2;
                break;
        }

        std::cout << "pole interval:" << pole_interval << std::endl;

        if(input_key_num == 't'){
            cv::destroyAllWindows();
            break;
        }

    }

    while (true)
    {

        start_count = start_count + 1;
        distance_save[0] = distance_save[1];
        distance_save[1] = distance_save[2];
        distance_save[2] = distance_save[3];
        distance_save[3] = distance_save[4];

        rs2::frameset frames = pipe.wait_for_frames();
        //rs2::depth_frame depth_point = frames.get_depth_frame;
        rs2::align align_to_color(RS2_STREAM_COLOR);
        auto aligned_frames = align_to_color.process(frames);
        auto depth_map = aligned_frames.get_depth_frame();
        auto color_map = aligned_frames.get_color_frame();
        if (filter)
        {
            depth_map = hole_filling.process(depth_map);
        }

        cv::Mat color(cv::Size(color_map.get_width(), color_map.get_height()), CV_8UC3, (void *)color_map.get_data(), cv::Mat::AUTO_STEP);
        // マーカーの検出
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(color, dictionary, marker_corners, marker_ids, parameters);
        cv::aruco::drawDetectedMarkers(color, marker_corners, marker_ids);
        
        //--------------------pose estimation-------------------------------------//
        //        std::vector< cv::Vec3d > rvecs, tvecs;
        //        cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        //----------------------------------------------------------------------------//
        
        float sum_marker_coordinate_x = 0;
        float sum_marker_coordinate_z = 0;
        float marker_coodinate_y = 0;
        bool exist = false;        
        double send_data = 0;

        if (marker_ids.size() > 0 && marker_ids.size() < 4)
        {
            for (int marker_num_count = 0; marker_num_count < marker_ids.size(); marker_num_count++)
            {
                if (marker_ids.at(marker_num_count) == 0)
                {
                    exist = true;
                    for (int i = 0; i < 4; i++)
                    {
                        sum_marker_coordinate_x += marker_corners[marker_num_count][i].x;
                        sum_marker_coordinate_z += marker_corners[marker_num_count][i].y;
                    }

                    if(sum_marker_coordinate_x > STRAP1_LINE && sum_marker_coordinate_x <= STRAP2_LINE){
                        send_data += 10;
                    }else if(sum_marker_coordinate_x > STRAP2_LINE && sum_marker_coordinate_x <= STRAP3_LINE){
                        send_data += 20;
                    }else if(sum_marker_coordinate_x > STRAP3_LINE && sum_marker_coordinate_x <= STRAP4_LINE){
                        send_data += 30;
                    }else if(sum_marker_coordinate_x > STRAP4_LINE && sum_marker_coordinate_x <= STRAP5_LINE){
                        send_data += 40;
                    }else if(sum_marker_coordinate_x > STRAP5_LINE && sum_marker_coordinate_x < STRAP6_LINE){
                        send_data += 50;
                    }else if(sum_marker_coordinate_x > STRAP6_LINE && sum_marker_coordinate_x < STRAP7_LINE){
                        send_data += 60;
                    }else if(sum_marker_coordinate_x > STRAP7_LINE && sum_marker_coordinate_x < STRAP8_LINE){
                        send_data += 70;
                    }else{
                       // tcp.send(0);
                    }

                    cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.406, cameraMatrix, distCoeffs, rvecs,tvecs);
                    rgb.x = tvecs[marker_num_count].val[0];
                    rgb.y = tvecs[marker_num_count].val[2];
                    rgb.z = tvecs[marker_num_count].val[1];
                    rgb.yaw   = rvecs[marker_num_count].val[2] * 180 / M_PI;
                    rgb.roll  = rvecs[marker_num_count].val[0] * 180 / M_PI;
                    rgb.pitch = rvecs[marker_num_count].val[1] * 180 / M_PI;
                    cv::aruco::drawAxis(color, cameraMatrix, distCoeffs, rvecs[marker_num_count], tvecs[marker_num_count], 0.1);

                    //std::cout << rgb.x  << "   " << rgb.y << "   " << rgb.z << std::endl;
                    //std::cout << rgb.yaw << "   " << rgb.roll << "   " << rgb.pitch << std::endl;
                    std::chrono::steady_clock::time_point now_time = std::chrono::steady_clock::now();
                    std::chrono::milliseconds elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - previous_time);
                    if (elapsed_time.count() > 1)
                    {
                        float point_center_marker_x = sum_marker_coordinate_x / 4;
                        float point_center_marker_z = sum_marker_coordinate_z / 4;

                        std::array<float,4> marker_info_average;
                        float depth_average = (depth_map.get_distance(point_center_marker_x, point_center_marker_z) + depth_map.get_distance(point_center_marker_x + 1, point_center_marker_z) + depth_map.get_distance(point_center_marker_x - 1, point_center_marker_z) + depth_map.get_distance(point_center_marker_x, point_center_marker_z - 1) + depth_map.get_distance(point_center_marker_x, point_center_marker_z + 1)) / 5.0;
                        /*distance_save.at(4)[0] = depth_average;
                        distance_save.at(4)[1] = rgb.x;
                        distance_save.at(4)[2] = rgb.y;
                        distance_save.at(4)[3] = rgb.z;*/
                        distance_save.at(4) = {depth_average,rgb.x,rgb.y,rgb.z};

                        //std::cout << " qqq" << std::endl;
                        start_count++;
                        if (start_count > 8)
                        {
                            for(int k = 0; k < 4; k++){
                                marker_info_average.at(k) = (distance_save.at(0)[k]  + (distance_save.at(1)[k] * 0.4 + distance_save.at(2)[k] * 0.3 + distance_save.at(3)[k] * 0.2 + distance_save.at(4)[k] * 0.1)) / 2;
                            }
                            depth_average = marker_info_average.at(0);
                            rgb.x = marker_info_average.at(1);
                            rgb.y = marker_info_average.at(2);
                            rgb.z = marker_info_average.at(3);

                            start_count = 8;
                            float point[3] = {0, 0, 0};
                            float pixel[2] = {0, 0};
                            pixel[0] = point_center_marker_x;
                            pixel[1] = point_center_marker_z;
                            rs2_deproject_pixel_to_point(point, &intr, pixel, depth_average);
                            //double distance_sum = marker_distance + distance_sum;
                            previous_time = now_time;
                            rdist.rs_x = (point[0] * 1000 + rgb.x * 1000) / 2;
                            rdist.rs_y = (point[2] * 1000 + rgb.y * 1000) / 2;
                            rdist.rs_z = (point[1] * 1000 + rgb.z * 1000) / 2;
                            //std::cout << rdist.rs_x << "  " << rdist.rs_y << "  " << rdist.rs_z << std::endl;
                        }
                    }
                }
            }
        }
        else
        {
            float center_marker_x = -50000;
            float center_marker_y = -50000;
            float center_marker_z = -50000;
        }

        cv::Mat obstacle = cv::Mat::zeros(color.size(),CV_8UC3);
        for(int pix_count_x = WIDTH/2 - pole_interval / 2 ; pix_count_x < WIDTH/2 + pole_interval / 2; pix_count_x+=2){
            for(int pix_count_y = horizontal_line ; pix_count_y < HEIGHT / 2; pix_count_y+=2){
                if(depth_map.get_distance(pix_count_x, pix_count_y) < 2.5 && depth_map.get_distance(pix_count_x, pix_count_y) > 2.0){
                        //std::cout << pix_count_x  << ":" << pix_count_y << ":" << stability_count << std::endl;
                        //std::cout << WIDTH  << ":" << HEIGHT << ":" << stability_count << std::endl;
                    cv::circle(obstacle, cv::Point(pix_count_x,pix_count_y),1,cv::Scalar(255,255,255),-1);
                    //cv::erode(obstacle, obstacle, cv::Mat(), cv::Point(-1,-1), 1);
                }
            }
        }

//-----------------------------------------------------------------//
        cv::inRange(obstacle, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), obstacle);
        cv::Mat LabelImg;
        cv::Mat stats;
        cv::Mat centroids;

        static int prev_nLab = 0;
        int nLab = cv::connectedComponentsWithStats(obstacle, LabelImg, stats, centroids);
        //std::cout << "nLab:" << nLab << std::endl;        

        // 描画色決定

        //重心計算
        int centerX[nLab];
        int centerY[nLab];
        for (int i = 1; i < nLab; ++i)
        {
            double *param = centroids.ptr<double>(i);
            centerX[i] = static_cast<int>(param[0]);
            centerY[i] = static_cast<int>(param[1]);
            
        }

        int area_num = 0;
        static int prev_area_num;
            //座標

        if(nLab < 1){
            tcp.send(0);
        }

        for (int i = 1; i < nLab; ++i)
        {
            int *param = stats.ptr<int>(i);
            if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > 500 && param[cv::ConnectedComponentsTypes::CC_STAT_LEFT] <= 800)
            {
                std::cout << "centerX:" << centerX[i] << std::endl;

                area_num++;
                if(centerX[i] > STRAP1_LINE && centerX[i] <= STRAP2_LINE){
                    send_data += 1;
                }else if(centerX[i] > STRAP2_LINE && centerX[i] <= STRAP3_LINE){
                    send_data += 2;
                }else if(centerX[i] > STRAP3_LINE && centerX[i] <= STRAP4_LINE){
                    send_data += 3;
                }else if(centerX[i] > STRAP4_LINE && centerX[i] <= STRAP5_LINE){
                    send_data += 4;
                }else if(centerX[i] > STRAP5_LINE && centerX[i] <= STRAP6_LINE){
                    send_data += 5;
                }else if(centerX[i] > STRAP6_LINE && centerX[i] <= STRAP7_LINE){
                    send_data += 6;
                }else if(centerX[i] > STRAP7_LINE && centerX[i] <= STRAP1_LINE){
                    send_data += 7;
                }else{
                    //tcp.send(0);
                }
                cv::circle(color, cv::Point(centerX[i], centerY[i]), 3, cv::Scalar(0, 0, 255), -1);
                int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
                int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
                int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
                int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
                cv::rectangle(color, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
                std::stringstream num;
                num << area_num;
                putText(color, num.str(), cv::Point(x + 5, y + 20), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            }
        }

       // std::cout << "test" << std::endl;

        //std::cout << prev_area_num << ":::" << area_num << std::endl;
        if(area_num > prev_area_num){
            //system("play alart.wav&");
        }
        prev_area_num = area_num;
        

//------------------------------------------------------------------------//
         //std::cout << send_data << std::endl;
//        std::cout << "before" << std::endl;
        tcp.send(send_data);
        std::cout << send_data << std::endl;
       
        cv::Mat line_in = color;
        // cv::line(line_in,cv::Point(340/2,215/2),cv::Point(940/2,215/2), cv::Scalar(255,0,100), 5, 16);

        cv::imshow("marker_detection", line_in);
        cv::imshow("obstacle",obstacle);
        //cv::imshow("cr",depth);

        int key = cv::waitKey(10);
        if (key == 115)
        {
            cv::imwrite("data.jpg", color);
            std::cout << "cap" << std::endl;
            numbering++;
            std::ostringstream picture_name;
            picture_name << "data" << numbering << ".jpg" << std::flush;
            cv::imwrite(picture_name.str(), color);
        }
        else if (key == 27)
        {
            break;
        }

    }
    return 0;
}
catch (const rs2::error &e)
{
    std::cerr << "Realsense error calling" << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

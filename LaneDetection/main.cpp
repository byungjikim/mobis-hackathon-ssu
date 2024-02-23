/*TODO
 * improve edge linking
 * remove blobs whose axis direction doesnt point towards vanishing pt
 * Parallelisation
 * lane prediction
*/

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>
#include "curl/curl.h"
#include <sstream>
#include <chrono>
#include <ctime>
#include <nlohmann/json.hpp>
// #include <wiringPi.h> // raspberry pi library

using namespace std;
using namespace cv;
clock_t start, stop;

#define SPRING_SERVER_URL "http://13.209.80.41:8080/driving/embedded"

class LaneDetect
{
public:
    Mat currFrame; //stores the upcoming frame
    Mat temp;      //stores intermediate results
    Mat temp2;     //stores the final lane segments

    int diff, diffL, diffR;
    int laneWidth;
    int diffThreshTop;
    int diffThreshLow;
    int ROIrows;
    int vertical_left;
    int vertical_right;
    int vertical_top;
    int smallLaneArea;
    int longLane;
    int  vanishingPt;
    // LED 값 알면 수정 필요
    const int LeftLED = 17;
    const int RightLED = 5;
    // vector<float> center2line;
    float maxLaneWidth;
    

    //to store various blob properties
    Mat binary_image; //used for blob removal
    Mat birdeye_view; // 시점 변환
    Mat canny_edge;
    int minSize;
    int ratio;
    float  contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
    Size2f sz;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RotatedRect rotated_rect;


    LaneDetect(Mat startFrame)
    {
        // if (wiringPiSetup() == -1)
        // {
        //     std::cerr << "WiringPi 초기화 실패!" << std::endl;
        //     return 1;
        // }
        // pinMode(LeftLED, INPUT);
        // pinMode(RightLED, INPUT);
        //currFrame = startFrame;                                    //if image has to be processed at original size

        // currFrame = Mat(320,480,CV_8UC1,0.0);                        //initialised the image size to 320x480
        currFrame = Mat(720,1280,CV_8UC1,0.0);                        //initialised the image size to 320x480

        resize(startFrame, currFrame, currFrame.size());             // resize the input to required size

        temp      = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores possible lane markings
        temp2     = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores finally selected lane marks

        vanishingPt    = currFrame.rows/2;                           //for simplicity right now
        ROIrows        = currFrame.rows - vanishingPt;               //rows in region of interest
        minSize        = 0.00015 * (currFrame.cols*currFrame.rows);  //min size of any region to be selected as lane
        maxLaneWidth   = 0.025 * currFrame.cols;                     //approximate max lane width based on image size
        smallLaneArea  = 7 * minSize;
        longLane       = 0.3 * currFrame.rows;
        ratio          = 4;

        //these mark the possible ROI for vertical lane segments and to filter vehicle glare
        vertical_left  = 2*currFrame.cols/5;
        vertical_right = 3*currFrame.cols/5;
        vertical_top   = 2*currFrame.rows/3;

        // namedWindow("lane",2);
        namedWindow("midstep", 2);
        // namedWindow("currframe", 2);
        // namedWindow("laneBlobs",2);
        namedWindow("BirdEyeView",2);
        // namedWindow("Canny",2);

        getLane();
    }

    void updateSensitivity()
    {
        int total=0, average =0;
        for(int i= vanishingPt; i<currFrame.rows; i++)
            for(int j= 0 ; j<currFrame.cols; j++)
                total += currFrame.at<uchar>(i,j);
        average = total/(ROIrows*currFrame.cols);
        cout<<"average : "<<average<<endl;
    }

    void getLane()
    {
        //medianBlur(currFrame, currFrame,5 );
        // updateSensitivity();
        //ROI = bottom half
        for(int i=vanishingPt; i<currFrame.rows; i++)
            for(int j=0; j<currFrame.cols; j++)
            {
                temp.at<uchar>(i,j)    = 0;
                temp2.at<uchar>(i,j)   = 0;
            }
            
        imshow("currframe", currFrame);
        birdeyeview(currFrame);
        imshow("midstep", birdeye_view);
        blobRemoval();
    }

    void markLane()
    {
        for(int i=vanishingPt; i<currFrame.rows; i++)
        {
            //IF COLOUR IMAGE IS GIVEN then additional check can be done
            // lane markings RGB values will be nearly same to each other(i.e without any hue)

            //min lane width is taken to be 5
            laneWidth =5+ maxLaneWidth*(i-vanishingPt)/ROIrows;
            for(int j=laneWidth; j<currFrame.cols- laneWidth; j++)
            {

                diffL = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j-laneWidth);
                diffR = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j+laneWidth);
                diff  =  diffL + diffR - abs(diffL-diffR);

                //1 right bit shifts to make it 0.5 times
                diffThreshLow = currFrame.at<uchar>(i,j)>>1;
                //diffThreshTop = 1.2*currFrame.at<uchar>(i,j);

                //both left and right differences can be made to contribute
                //at least by certain threshold (which is >0 right now)
                //total minimum Diff should be atleast more than 5 to avoid noise
                //if (diffL>0 && diffR >0 && diff>5)
                if(diff>=diffThreshLow /*&& diff<= diffThreshTop*/ )
                    temp.at<uchar>(i,j)=255;
            }
        }

    }

    void blobRemoval()
    {
        markLane();

        // find all contours in the binary image
        temp.copyTo(binary_image);
        findContours(binary_image, contours,
                     hierarchy, cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_SIMPLE);

        // for removing invalid blobs
        if (!contours.empty())
        {
            for (size_t i=0; i<contours.size(); ++i)
            {
                //====conditions for removing contours====//

                contour_area = contourArea(contours[i]) ;

                //blob size should not be less than lower threshold
                if(contour_area > minSize)
                {
                    rotated_rect    = minAreaRect(contours[i]);
                    sz              = rotated_rect.size;
                    bounding_width  = sz.width;
                    bounding_length = sz.height;


                    //openCV selects length and width based on their orientation
                    //so angle needs to be adjusted accordingly
                    blob_angle_deg = rotated_rect.angle;
                    if (bounding_width < bounding_length)
                        blob_angle_deg = 90 + blob_angle_deg;

                    //if such big line has been detected then it has to be a (curved or a normal)lane
                    if(bounding_length>longLane || bounding_width >longLane)
                    {
                        drawContours(currFrame, contours,i, Scalar(255), cv::FILLED, 8);
                        drawContours(temp2, contours,i, Scalar(255), cv::FILLED, 8);
                    }

                    //angle of orientation of blob should not be near horizontal or vertical
                    //vertical blobs are allowed only near center-bottom region, where centre lane mark is present
                    //length:width >= ratio for valid line segments
                    //if area is very small then ratio limits are compensated
                    else if ((blob_angle_deg <-10 || blob_angle_deg >-10 ) &&
                             ((blob_angle_deg > -70 && blob_angle_deg < 70 ) ||
                              (rotated_rect.center.y > vertical_top &&
                               rotated_rect.center.x > vertical_left && rotated_rect.center.x < vertical_right)))
                    {

                        if ((bounding_length/bounding_width)>=ratio || (bounding_width/bounding_length)>=ratio
                                ||(contour_area< smallLaneArea &&  ((contour_area/(bounding_width*bounding_length)) > .75) &&
                                   ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)))
                        {
                            drawContours(currFrame, contours,i, Scalar(255), cv::FILLED, 8);
                            drawContours(temp2, contours,i, Scalar(255), cv::FILLED, 8);
                        }
                    }
                }
            }
        }
        // imshow("midstep", temp);
        // imshow("laneBlobs", temp2);
        birdeyeview(temp2);
        // Canny(birdeye_view, canny_edge, 50, 150);
        // imshow("Canny", canny_edge);
        analysis_Lanetype(birdeye_view);
        imshow("BirdEyeView", birdeye_view);

        // imshow("lane",currFrame);
    }


    void nextFrame(Mat &nxt)
    {
        //currFrame = nxt;                        //if processing is to be done at original size

        resize(nxt ,currFrame, currFrame.size()); //resizing the input image for faster processing
        getLane();
    }

    void birdeyeview(Mat frame)
    {
        vector<Point2f> src_points = {
            Point2f(580, 481),
            Point2f(766, 481),
            Point2f(1070, 656),
            Point2f(340, 656)
        };

        vector<Point2f> dest_points = {
            Point2f(320, 720),
            Point2f(960, 720),
            Point2f(960, 0),
            Point2f(320, 0)
        };

        Mat birdeye_tf = getPerspectiveTransform(src_points, dest_points);
        warpPerspective(frame, birdeye_view, birdeye_tf, frame.size());
    }

    void analysis_Lanetype(Mat bird_frame)
    {
        int height = bird_frame.rows;
        int width = bird_frame.cols;
        int window_height = 80;
        int window_width = 90;

        // vector<int> startPoints = {Point(320, height), Point(width / 2, height), Point(960, height)};
        vector<int> start_x = {320, width / 2, 960};

        vector<string> laneTypes;

        for (const auto& start_xpoint : start_x) 
        {
            string type = slidingWindow(bird_frame, start_xpoint, window_width, window_height);
            laneTypes.push_back(type);
        }

        cout << laneTypes[0] << " " << laneTypes[1] << " " << laneTypes[2] << endl;
        // cout << center2line[0] << " " << center2line[1] << " " << center2line[2] << endl;
        // 시나리오
        illegal_LaneChange(laneTypes);

    }

    string slidingWindow(Mat& image, int start_x, int window_width, int window_height)
    {
        float whitePixels = 0;
        int window_size = window_height * window_width;
        float whiteratio =0;
        float total_window = image.rows/window_height;
        float window_cnt =0;
        // float center_x=0;
        // float mean_center_x =0;

        for (int y=image.rows; y>0; y -=window_height)
        {
            // center_x =0;
            whitePixels =0;
            Point tl(start_x - window_width, y - window_height);
            Point br(start_x + window_width, y + window_height);
            rectangle(image, tl, br, Scalar(255, 255, 255), 2);

            for (int height=y; height > y-window_height; height--)
            {
                for (int x=start_x - window_width; x<start_x + window_width; x++)
                {
                    if(image.at<uchar>(height,x) >= 200)
                    {    
                        whitePixels++;
                        // center_x += x;
                    }
                }
            }

            whiteratio = whitePixels / window_size;
            // cout << "WhiteRatio: " << whiteratio << endl;

            if(whiteratio > 0.3)
            {
                window_cnt++;
                // start_x = center_x / whitePixels;
            }
            // mean_center_x += start_x;
        }
        // mean_center_x = mean_center_x / total_window;
        // mean_center_x = (image.cols/2 - mean_center_x);
        // center2line.push_back(mean_center_x);
        // cout << "window cnt: " << window_cnt << endl;
        // cout << "WhiteRatio: " << whiteratio << endl;
        if (window_cnt >= total_window *0.6)
            return "Solid";
        else if(window_cnt >= total_window *0.1)
            return "Dashed";
        else
            return "None";
    }
    
    using json = nlohmann::json;
    
    void illegal_LaneChange(vector<string> lanetype)
    {
        // bool LeftLED_state = digitalRead(LeftLED);
        // bool RightLED_state = digitalRead(RightLED);

        // if(lanetype[1] != "None" && (LeftLED_state || RightLED_state))
        if(lanetype[1] != "None" && lanetype[2] != "None")
        {
            string circumstance = "1";
            report_illegal(circumstance);
        }

        else if(lanetype[1] != "None")
        {
            string circumstance = "2";
            report_illegal(circumstance);
        }
    }

    void report_illegal(string circumstance)
    {
        curl_global_init(CURL_GLOBAL_ALL);

        CURL *curl = curl_easy_init();
        if (!curl) 
        {
            std::cerr << "Failed to initialize libcurl" << std::endl;
        }
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        char buffer[80]; 
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now_time));
        std::string formatted_time(buffer);

        json j_data;
        j_data[circumstance] = circumstance;
        j_data["createdAt"] = formatted_time;
        
        std::string s = j_data.dump();
        // JSON 형식의 데이터 생성
        //std::stringstream json_ss;
        //json_ss << "{";
        //json_ss << "\"1\": \"" << circumstance << ",";
        //json_ss << "\"2\": \"" << formatted_time << "\"";
        //json_ss << "}";
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 1L);
        
        curl_easy_setopt(curl, CURLOPT_URL, SPRING_SERVER_URL); // API 엔드포인트
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, s.c_str()/*json_ss.str().c_str()*/); // JSON 데이터 설정
        
        //curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json_ss.str().size());

        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK) 
        {
            std::cerr << "Failed to send HTTP request: " << curl_easy_strerror(res) << std::endl;
        }
        else
            cout << "success! " << endl;
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        curl_global_cleanup();
    }

    Mat getResult()
    {
        return temp2;
    }

};//end of class LaneDetect


void makeFromVid(string path)
{
    Mat frame;
    // VideoCapture cap(path); // open the video file for reading
    VideoCapture cap(0);  // 카메라 실시간 쓸 거면 이걸로

    if ( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

    //cap.set(cv::CAP_PROP_POS_MSEC, 300); //start the video at 300ms

    double fps = cap.get(cv::CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);
    LaneDetect detect(frame);

    while(1)
    {
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess)                   //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

        //start = clock();
        detect.nextFrame(frame);
        //stop =clock();
        // cout<<"fps : "<<1.0/(((double)(stop-start))/ CLOCKS_PER_SEC)<<endl;

        if(waitKey(10) == 27) //wait for 'esc' key press for 10 ms. If 'esc' key is pressed, break loop
        {
            cout<<"video paused!, press q to quit, any other key to continue"<<endl;
            if(waitKey(0) == 'q')
            {
                cout << "terminated by user" << endl;
                break;
            }
        }
    }
}

int main()
{
    makeFromVid("/home/user/LaneDetection/project_video.mp4");
    waitKey(0);
    destroyAllWindows();
}

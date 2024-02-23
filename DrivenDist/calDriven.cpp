#include <iostream>
#include <wiringPiI2C.h>
#include <cmath>
#include <unistd.h>
#include <curl/curl.h>
#include <sstream>
#include <chrono>
#include <ctime>
#include <time.h>
#include <nlohmann/json.hpp>

#define Device_Address 0x68

// MPU6050 레지스터 주소
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE   0x38

#define ACCEL_RANGE 2
#define SERVER_URL "http://13.209.80.41:8080/driving/embedded"

int fd;

using json = nlohmann::json;

void MPU6050_Init() {
    wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07);
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(fd, CONFIG, 0);
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);
    wiringPiI2CWriteReg8(fd, ACCEL_CONFIG, 24);
    wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);
}

int16_t read_raw_data(int addr) {
    int16_t high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr+1);
    value = (high_byte << 8) | low_byte;
    return value;
}

void calculate_velocity_distance(float ax,float ay, float dt, float &velocity, float &distance) {
    
    float vx = ax * dt;
    float vy = ay * dt;
    velocity = sqrt(vx*vx + vy*vy);
    
    if(velocity < 0.001)
    {
		velocity = 0;
		vx =0;
		vy =0;
	}
	
    float x = vx * dt;
    float y = vy * dt;
        
    distance += sqrt(x*x + y*y);
    
    std::cout << "Velocity: " << velocity << " m/s" << std::endl;
    std::cout << "Distance: " << distance << " m" << std::endl;
}

void send_to_server(std::string type, float value) {
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::time_t now_time = std::chrono::system_clock::to_time_t(now);
	std::string vel = std::to_string(value);
	char buffer[80]; 
	std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now_time));
	std::string formatted_time(buffer);
	
	json j_data;
	j_data[type] = vel;
	j_data["createdAt"] = formatted_time;
	//j_data["100"] = "value";
	//j_data["createdAt"] = "2024-02-141";
	std::string s = j_data.dump();

    CURL *curl;
    CURLcode res;    
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    struct curl_slist *headers = NULL;
	headers = curl_slist_append(headers, "Content-Type: application/json");
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        //curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
        //curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 1L);
        curl_easy_setopt(curl, CURLOPT_URL, SERVER_URL);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, s.c_str());
        res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        }
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();
}


int main() 
{
    int16_t Acc_x, Acc_y, Acc_z;
    float Ax = 0.0, Ay = 0.0, Az = 0.0;
    float velocity = 0.0;
    float distance = 0.0;
    float dt = 0.11; // 시간 간격
	int send_cnt = 0;
    fd = wiringPiI2CSetup(Device_Address);
    time_t start_time = 0;
    time_t current_time = time(NULL);
    MPU6050_Init();

    while(1) 
    {
        Acc_x = read_raw_data(0x3B);
        Acc_y = read_raw_data(0x3D);

        Ax = (float)Acc_x / 16384.0;
        Ay = (float)Acc_y / 16384.0;


        calculate_velocity_distance(Ax, Ay, dt, velocity, distance);
        
        if (std::difftime(std::time(NULL), start_time) >= 5 && send_cnt == 0) 
        {
            send_to_server("200", velocity);
            send_to_server("100", distance);
            send_cnt +=1;
        }
        
        if (std::difftime(std::time(NULL), start_time) >= 6) 
        {
            send_cnt = 0;
            start_time = std::time(NULL);
        }
		usleep(100000);
	}
        
    return 0;
}

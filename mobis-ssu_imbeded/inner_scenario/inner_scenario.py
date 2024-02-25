import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from threading import Lock
import json
import requests
import pytz

from datetime import datetime


# Spring 서버의 URL
SPRING_SERVER_URL = "http://15.164.253.57:8080/driving/embedded"

class driverStateSub(Node):

    def __init__(self):
        super().__init__('driver_state_subscriber')
        self.headpose_sub = self.create_subscription(String, '/head_pose', self.headpose_callback, 10)
        self.phone_sub = self.create_subscription(String, '/mobile_detection_topic', self.phone_callback, 10)
        self.eye_sub = self.create_subscription(String, '/eye_position', self.eye_callback, 10)
        self.timer = self.create_timer(1.3, self.timer_callback)
        self.lock = Lock()
        self.phone_detect_cnt = 0
        self.phone_detection = 0
        self.eye_cnt =0
        self.close_eye_cnt = 0
        self.eye_detection =0
        self.head_state = [0, 0, 0, 0, 0] # up, down, right, left, front
        self.before_state = 4
        self.same_state_cnt =0

    def headpose_callback(self, msg): # 30hz
        with self.lock:
            if(msg.data == 'Head down '):
                self.head_state[0] = self.head_state[0] + 1

            elif (msg.data == 'Head up '):
                self.head_state[1] = self.head_state[1] + 1

            elif (msg.data == 'Head left'):
                self.head_state[2] = self.head_state[2] + 1

            elif (msg.data == 'Head right'):
                self.head_state[3] = self.head_state[3] + 1

            else:
                self.head_state[4] = self.head_state[4] + 1
            
            if(self.same_state_cnt==3):
                print("고개가 치우침")
                self.headpose_report()
                self.same_state_cnt =0
    
    def headpose_report(self):
        # http 통신, json 데이터 형식 만들기
        # 고개가 한 쪽으로 3초 치우쳐짐
        self.send_json_data(1)

    def phone_callback(self, msg): # 3.2 hz
        with self.lock:
            if(msg.data):
                self.phone_detect_cnt = self.phone_detect_cnt +1 
            
            if(self.phone_detection == 3):
                print("핸드폰 그만보세요")
                self.phonedetection_report()
                self.phone_detect_cnt =0

    def phonedetection_report(self):
        # http 통신, json 데이터 형식 만들기
        # 핸드폰이 3초 동안 검출됨
        self.send_json_data(2)


    def eye_callback(self, msg): # 23 hz
        with self.lock: 
            self.eye_cnt = self.eye_cnt +1
            if(msg.data == 'True'):
                self.close_eye_cnt = self.close_eye_cnt +1  

            if(self.eye_detection == 3):
                print("눈을 뜨세요")
                self.eyedetection_report()
                self.eye_detection = 0

    def eyedetection_report(self):
        # http 통신, json 데이터 형식 만들기
        # 눈이 3초 동안 감김
        self.send_json_data(3)
        
    def sleepydriver_report(self):
        if(self.eye_detection >=2 and self.same_state_cnt>=2):
            self.same_state_cnt =0
            self.eye_detection = 0
            self.eyedetection_report()

    def timer_callback(self):
        with self.lock:
            # check phone detection 
            if(self.phone_detect_cnt >= 1):
                self.phone_detection = self.phone_detection + 1

            else:
                self.phone_detection = 0
            
            self.phone_detect_cnt = 0

            # check eyes' state
            if (self.eye_cnt !=0):
                close_eye_ratio = self.close_eye_cnt / self.eye_cnt
                self.close_eye_cnt = 0
                self.eye_cnt =0
                
                if(close_eye_ratio > 0.7):
                    self.eye_detection = self.eye_detection + 1
                    
                else:
                    self.eye_detection = 0

            # check head pose
            if(sum(self.head_state) !=0):
                curr_state = self.head_state.index(max(self.head_state))
                if(curr_state ==4):
                    pass
                elif(self.before_state == curr_state):
                    self.same_state_cnt = self.same_state_cnt +1
                else:
                    self.same_state_cnt = 0
                    self.before_state = curr_state

                self.head_state = [0, 0, 0, 0, 0]
            #self.sleepydriver_report()

    def send_json_data(self, value):
        korean_timezone = pytz.timezone('Asia/Seoul')
        current_time = datetime.now(korean_timezone).strftime("%Y-%m-%d %H:%M:%S")
        # JSON 데이터 생성
        data = {
            'type': value,
            'createdAt': current_time
        }
        # Spring 서버로 POST 요청 보내기
        print(" 살았음 ")
        try:
            response = requests.post(SPRING_SERVER_URL, json=data)
            if response.status_code == 200:
                print("JSON 데이터 전송 성공!")
            else:
                print("JSON 데이터 전송 실패, error: ", response.status_code)
        except Exception as e:
            print("An error occurred:", str(e))


def main(args=None):
    rclpy.init(args=args)
    driver_state_sub = driverStateSub()
    rclpy.spin(driver_state_sub)
    driver_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
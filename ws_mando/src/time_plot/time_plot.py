import matplotlib.pyplot as plt
from datetime import datetime
from time import time, sleep
import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Int32


class DataSubscriber(object):
    def __init__(self):
        rospy.init_node('data_subscriber')

        self.subscription = rospy.Subscriber(
            'data_time_plot', 
            Float32, 
            self.data_callback, 
            queue_size=10
        )
        self.r_subscription = rospy.Subscriber(
            '/r', 
            Float32, 
            self.r_callback, 
            queue_size=10
        )

        self.data = []  # (받아온 시간, 데이터)를 저장할 리스트
        self.r_values = []  # (받아온 시간, r 데이터)를 저장할 리스트
        self.state = 'first'
        
    def data_callback(self, msg):
        if self.state == 'first' and msg.data != 0:
            self.state = 'not first'
            self.init_time = time()  # 처음 시간 기록
        if self.state == 'not first':
            timestamp = time() - self.init_time  # 기록 시간 = 현재 시간 - 처음 시간
            data = msg.data  # 받아온 데이터
        
            print(data)

            self.data.append((timestamp, data))  # (받아온 시간, 데이터)를 리스트에 추가

    def r_callback(self, msg):
        if self.state == 'first':
            self.state = 'not first'
            self.init_time = time()  # 처음 시간 기록
        if self.state == 'not first':
            timestamp = time() - self.init_time  # 기록 시간 = 현재 시간 - 처음 시간
            r_value = msg.data  # 받아온 r 데이터
            self.r_values.append((timestamp, r_value))  # (받아온 시간, r 데이터)를 리스트에 추가


    def draw_graph(self):
        timestamps = [entry[0] for entry in self.data]  # 시간 데이터 추출
        data_values = [entry[1] for entry in self.data]  # 데이터 추출

        r_timestamps = [entry[0] for entry in self.r_values]  # r 시간 데이터 추출
        r_values = [entry[1] for entry in self.r_values]  # r 데이터 추출


        plt.plot(timestamps, data_values)  # 그래프 그리기
        plt.plot(r_timestamps, r_values, label='R')
        plt.xlabel('Time')
        plt.ylabel('y_m & r')
        plt.title('P:0.4 I: 0.21 D:1.0')
        plt.legend()

        # Save the graph as an image file (e.g., PNG format)
        file_name = 'P04I21D10R700.png'
        plt.savefig(file_name)

        print(f"Graph saved as '{file_name}'.")


        plt.show()

def main():
    data_subscriber = DataSubscriber()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        data_subscriber.draw_graph()

if __name__ == '__main__':
    main()
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
            Int32, 
            self.data_callback, 
            queue_size=10
        )

        self.data = []  # (받아온 시간, 데이터)를 저장할 리스트
        self.state = 'first'
        
    def data_callback(self, msg):
        if self.state == 'first':
            self.state = 'not first'
            self.init_time = time()  # 처음 시간 기록

        timestamp = time() - self.init_time  # 기록 시간 = 현재 시간 - 처음 시간
        data = msg.data  # 받아온 데이터
        print(data)

        self.data.append((timestamp, data))  # (받아온 시간, 데이터)를 리스트에 추가

    def draw_graph(self):
        timestamps = [entry[0] for entry in self.data]  # 시간 데이터 추출
        data_values = [entry[1] for entry in self.data]  # 데이터 추출

        plt.plot(timestamps, data_values)  # 그래프 그리기
        plt.xlabel('Time')
        plt.ylabel('Data')
        plt.title('Data over Time')
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
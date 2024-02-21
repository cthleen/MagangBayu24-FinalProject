import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import String

import cv2
import numpy as np

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.subscription_ = self.create_subscription(Int16, 'topic2', self.callback, 10)
        self.subscription2_ = self.create_subscription(String, 'topic3', self.string_callback, 10)
        self.publisher_ = self.create_publisher(Int16, 'topic1', 10)

        self.computer = np.zeros(10)
        self.player = np.zeros(10)

        self.end = 'hello world'
        self.endflag = 0

    def get_corner_points(self, frame_shape):
            height, width = frame_shape
            corners = []

            for i in range(4):
                for j in range(4):
                    corners.append((j * width // 3, i * height // 3))
            
            return corners
    

    # Menggambar langkah dari player
    def player_move(self, corners, cropped_frame):
        if self.player[1] == 1:
            a = corners[5][0] // 2
            b = corners[5][1] // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[2] == 1:
            a = corners[5][0] + (corners[6][0] - corners[5][0]) // 2
            b = corners[6][1] // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[3] == 1:
            a = corners[6][0] + (corners[7][0] - corners[6][0]) // 2
            b = corners[7][1] // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[4] == 1:
            a = corners[9][0] // 2
            b = corners[5][1] + (corners[9][1] - corners [5][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[5] == 1:
            a = corners[5][0] + (corners[6][0] - corners[5][0]) // 2
            b = corners[5][1] + (corners[9][1] - corners [5][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[6] == 1:
            a = corners[6][0] + (corners[7][0] - corners[6][0]) // 2
            b = corners[5][1] + (corners[9][1] - corners [5][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[7] == 1:
            a = corners[13][0] // 2
            b = corners[9][1] + (corners[13][1] - corners[9][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[8] == 1:
            a = corners[5][0] + (corners[6][0] - corners[5][0]) // 2
            b = corners[9][1] + (corners[13][1] - corners[9][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)

        if self.player[9] == 1:
            a = corners[6][0] + (corners[7][0] - corners[6][0]) // 2
            b = corners[9][1] + (corners[13][1] - corners[9][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (0, 0, 255), -1)


    # Menggambar langkah dari computers
    def computer_move(self, corners, cropped_frame):
        if self.computer[1] == 1:
            a = corners[5][0] // 2
            b = corners[5][1] // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[2] == 1:
            a = corners[5][0] + (corners[6][0] - corners[5][0]) // 2
            b = corners[6][1] // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[3] == 1:
            a = corners[6][0] + (corners[7][0] - corners[6][0]) // 2
            b = corners[7][1] // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[4] == 1:
            a = corners[9][0] // 2
            b = corners[5][1] + (corners[9][1] - corners [5][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[5] == 1:
            a = corners[5][0] + (corners[6][0] - corners[5][0]) // 2
            b = corners[5][1] + (corners[9][1] - corners [5][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[6] == 1:
            a = corners[6][0] + (corners[7][0] - corners[6][0]) // 2
            b = corners[5][1] + (corners[9][1] - corners [5][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[7] == 1:
            a = corners[13][0] // 2
            b = corners[9][1] + (corners[13][1] - corners[9][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[8] == 1:
            a = corners[5][0] + (corners[6][0] - corners[5][0]) // 2
            b = corners[9][1] + (corners[13][1] - corners[9][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)

        if self.computer[9] == 1:
            a = corners[6][0] + (corners[7][0] - corners[6][0]) // 2
            b = corners[9][1] + (corners[13][1] - corners[9][1]) // 2
            cv2.circle(cropped_frame, (a, b), 70, (225, 0, 0), -1)


    def string_callback(self, msg):
        self.get_logger().info('Received: %s' % msg.data)

        self.end = f'{msg.data}'
        self.endflag = 1

        self.camera_frame()


    # Menerima pesan dari control
    def callback(self, msg):
        self.get_logger().info('Received: %d' % msg.data)
        computer = msg.data

        self.computer[computer] = 1

        self.camera_frame()


    # Mendeteksi langkah player dengan menggunakan OpenCV
    def camera_frame(self):
        cap = cv2.VideoCapture(0)

        count = np.zeros(10)
        index = 0
               
        while True:
            ret, frame = cap.read()

            if not ret : 
                break

            flag = 0

            height, width = frame.shape[:2]

            size = min(height, width)

            flipped = cv2.flip(frame, 1)

            cropped_frame = flipped[0:size, 0:size]
            
            for i in range(1, 3):
                cv2.line(cropped_frame, (0, i * cropped_frame.shape[0] // 3), (cropped_frame.shape[1], i * cropped_frame.shape[0] // 3), (0, 255, 0), 2)
                cv2.line(cropped_frame, (i * cropped_frame.shape[1] // 3, 0), (i * cropped_frame.shape[1] // 3, cropped_frame.shape[0]), (0, 255, 0), 2)

            corners = self.get_corner_points(cropped_frame.shape[:2])

            gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
	        
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=0.1, minDist=30, param1=150, param2=30, minRadius=35, maxRadius=100)

            if circles is not None and self.endflag == 0:
                circles = np.round(circles[0, :]).astype("int") 
                
                for (x, y, r) in circles:
                    cv2.circle(cropped_frame, (x, y), r, (0, 255, 0), 4)

                    if corners[0][0] <= x <= corners[5][0] and corners[0][1] <= y <= corners[5][1]:
                        print("Detecting player's movement in Box 1")
                        count[1] += 1
                        if count[1] == 10.0:
                            flag = 1
                            self.player[1] = 1
                            index = 1
                            
                    if corners[1][0] <= x <= corners[6][0] and corners[1][1] <= y <= corners[6][1]:
                        print("Detecting player's movement in Box 1")
                        count[2] += 1
                        if count[2] == 10.0:
                            flag = 1
                            self.player[2] = 1
                            index = 2
                            
                    if corners[2][0] <= x <= corners[7][0] and corners[2][1] <= y <= corners[7][1]:
                        print("Detecting player's movement in Box 3")
                        count[3] += 1
                        if count[3] == 10.0:
                            flag = 1
                            self.player[3] = 1
                            index = 3
                            
                    if corners[4][0] <= x <= corners[9][0] and corners[4][1] <= y <= corners[9][1]:
                        print("Detecting player's movement in Box 4")
                        count[4] += 1
                        if count[4] == 10.0:
                            flag = 1
                            self.player[4] = 1
                            index = 4
                            
                    if corners[5][0] <= x <= corners[10][0] and corners[5][1] <= y <= corners[10][1]:
                        print("Detecting player's movement in Box 5")
                        count[5] += 1
                        if count[5] == 10.0:
                            flag = 1
                            self.player[5] = 1
                            index = 5
                            
                    if corners[6][0] <= x <= corners[11][0] and corners[6][1] <= y <= corners[11][1]:
                        print("Detecting player's movement in Box 6")
                        count[6] += 1
                        if count[6] == 10.0:
                            flag = 1
                            self.player[6] = 1
                            index = 6
                            
                    if corners[8][0] <= x <= corners[13][0] and corners[8][1] <= y <= corners[13][1]:
                        print("Detecting player's movement in Box 7")
                        count[7] += 1
                        if count[7] == 10.0:
                            flag = 1
                            self.player[7] = 1
                            index = 7
                            
                    if corners[9][0] <= x <= corners[14][0] and corners[9][1] <= y <= corners[14][1]:
                        print("Detecting player's movement in Box 8")
                        count[8] += 1
                        if count[8] == 10.0:
                            flag = 1
                            self.player[8] = 1
                            index = 8
                            
                    if corners[10][0] <= x <= corners[15][0] and corners[10][1] <= y <= corners[15][1]:
                        print("Detecting player's movement in Box 9")
                        count[9] += 1
                        if count[9] == 10.0:
                            flag = 1
                            self.player[9] = 1
                            index = 9
                                    
            self.player_move(corners, cropped_frame)
            self.computer_move(corners, cropped_frame)

            if self.endflag == 1:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                font_color = (0, 0, 0)
                thickness = 4

                frame_size = cropped_frame.shape

                text_size = cv2.getTextSize(self.end, font, font_scale, thickness)

                text_x = int((frame_size[0] - text_size[0][0]) // 2)
                text_y = int((frame_size[1] - text_size[0][1]) // 2)

                cv2.putText(frame, self.end, (text_x, text_y), font, font_scale, font_color, thickness)

            if flag == 1 and self.endflag == 0:
                player_msg = Int16()
                player_msg.data = index
            
                self.publisher_.publish(player_msg)

                break

            cv2.imshow('frame', cropped_frame)
            if cv2.waitKey(1) == 27:
                break

        cap.release()
        cv2.destroyAllWindows()
    

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    camera_publisher.camera_frame()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
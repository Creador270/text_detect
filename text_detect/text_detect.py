import cv2
import easyocr
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge

reader = easyocr.Reader(['en']) # this needs to run only once to load the model into memory

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('text_detector')
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
        self.publisher_img = self.create_publisher(Image, 'text_detect/image_out', 10)
        self.publisher_cord = self.create_publisher(Int16MultiArray, 'text_detect/coordinates', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0


    def filtro(self, mask):
        
        #fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        #particles
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        clean = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel2)
        
        inv = cv2.bitwise_not(clean)#invert image
        return inv

    def timer_callback(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error('Error reading camera')
            return

        #frame = cv2.flip(frame, 1)
        frame_g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        #Filters
        frame_clear = self.filtro(frame_g)
        centers= []
        
        #detect Text
        result = reader.readtext(frame_clear)
        for detection in result:
            box, text, conf = detection
            box = [[int(x) for x in subl] for subl in box] #convert to int

            #centers
            x_center = box[0][0] + round((box[2][0] - box[0][0])/2)
            y_center = box[0][1] + round((box[2][1] - box[0][1])/2)
            centers.append(x_center)
            centers.append(y_center)
            
            #drawing
            cv2.circle(frame, (x_center, y_center), 4, (0,0,255), 10)
            cv2.putText(frame, text, box[0],
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 0),
                        2,
                        cv2.LINE_AA
                        )
            cv2.putText(frame, str(round(conf, 3)), (box[3][0], box[3][1] + 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0), 
                        1,
                        cv2.LINE_AA
                        )
            cv2.rectangle(frame, 
                          (box[0]), #vertex 1
                          (box[2]), #vertex 2
                          (0, 255, 255), 2)

        image_msg = self.bridge.cv2_to_imgmsg(frame)
        self.publisher_img.publish(image_msg)

        cord_msg = Int16MultiArray()
        cord_msg.data = centers
        self.publisher_cord.publish(cord_msg)

        self.get_logger().info('Publishing image %d' % self.i)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

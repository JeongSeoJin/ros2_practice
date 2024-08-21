import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ultralytics import YOLO
import cv2
import math


classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]

class ObjectDetectionPublisher(Node):

    def __init__(self):
        super().__init__('object_detection_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'object_center', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # YOLO 모델 로드
        self.model = YOLO("yolo-Weights/yolov8n.pt")

        # 웹캠 설정
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

    def timer_callback(self):
        success, img = self.cap.read()
        if not success:
            return

        results = self.model(img, stream=True)
        for r in results:
            boxes = r.boxes

            for box in boxes:
                # 바운딩 박스 좌표
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # int 값으로 변환

                # 중심점 계산
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                # 중심점을 퍼블리시
                msg = Float64MultiArray()
                msg.data = [cx, cy]
                self.publisher_.publish(msg)

                # 디버그 정보 출력
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                print(f"Class: {classNames[cls]}, Confidence: {confidence}, Center: ({cx}, {cy})")

                # 바운딩 박스 그리기
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.putText(img, classNames[cls], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.circle(img, (int(cx), int(cy)), 5, (255, 255, 255), -1)

        cv2.imshow('Webcam', img)
        if cv2.waitKey(1) == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

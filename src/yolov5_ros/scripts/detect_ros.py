#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import torch
import numpy as np
import onnxruntime as ort
import warnings
from functools import partial
from cv_bridge import CvBridge, CvBridgeError


from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, RegionOfInterest
from yolov5_ros.msg import BoundingBox, BoundingBoxes
from deep_sort_realtime.deepsort_tracker import DeepSort
from threadpoolctl import threadpool_limits

class Yolov5Param:
    def __init__(self):
        # load local repository(YoloV5:v6.0)
        # 指定yolov5的源码路径，位于/home/rui27/yolov5/
        yolov5_path = rospy.get_param('/yolov5_path', '')
        # 指定yolov5的权重文件路径，位于/home/rui27/yolov5/yolov5s.pt
        weight_path = rospy.get_param('~weight_path', '')
        # yolov5的某个参数，这里不深究了
        conf = rospy.get_param('~conf', '0.5')
        # 使用pytorch加载yolov5模型，torch.hub.load会从/home/rui27/yolov5/中找名为hubconf.py的文件
        # hubconf.py文件包含了模型的加载代码，负责指定加载哪个模型
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')
        # 一个参数，用来决定使用cpu还是gpu，这里我们使用gpu
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()
        self.model.conf = conf

        # 初始化DeepSort
        self.tracker = DeepSort(
                 max_age=15,  # 最多允许丢失目标的帧数，超出就会删除该检测框以及相应的轨迹
                 max_iou_distance=1.0,  # 预测框和目标框的最大距离，超过该距离则判断丢失目标，该参数越高越不容易丢失目标，但是会容易导致误匹配
                 max_cosine_distance=0.2, # 行人重识别（ReID）的最大余弦距离，超过该阈值则判断丢失目标，该参数越高越不容易丢失目标，但是稳定性会降低
                 n_init=3,  # 连续检测到一定次数，才确认生成检测框和对应的轨迹
                 embedder="mobilenet",  # 使用的模型，mobilenet是适用于无人机等移动机器人的轻量化模型
                 half=True,  # 使用半精度浮点数（FP16）加速推理，需要使用GPU
                 bgr=True,  # 输入图像是否采用BGR格式，该格式是OpenCV的默认格式
                 embedder_gpu=True,  # 是否使用GPU
                 nn_budget=100  # 限制历史样本数量，加快匹配速度
        )

        # 打印GPU和CUDA的使用情况，检查yolov5和deepsort是否使用gpu加速
        print("Available providers:", ort.get_available_providers())
        print("CUDA Available:", torch.cuda.is_available())
        if torch.cuda.is_available():
           print("Using GPU:", torch.cuda.get_device_name(0))

        # target publishers
        # BoundingBoxes是本样例自定义的消息类型，用来记录识别到的目标
        # 使用/yolov5/targets topic发布出去
        self.target_pub = rospy.Publisher("/yolov5/targets",  BoundingBoxes, queue_size=1)

def image_cb(msg, cv_bridge, yolov5_param, color_classes, image_pub):
    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        # 将Opencv图像转换numpy数组形式，数据类型是uint8（0~255）
        # numpy提供了大量的操作数组的函数，可以方便高效地进行图像处理    
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = np.array(cv_image, dtype=np.uint8)
    except (CvBridgeError, e):
        print(e)
    # 实例化BoundingBoxes，存储本次识别到的所有目标信息
    bounding_boxes = BoundingBoxes()
    bounding_boxes.header = msg.header

    # 将BGR图像转换为RGB图像, 给yolov5，其返回识别到的目标信息
    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = yolov5_param.model(rgb_image)

    # 使用pandas读取yolov5返回的识别结果，简单易用，但是运算速度慢
    # boxs = results.pandas().xyxy[0].values
    # detections = []
    # boxs = [box for box in boxs if str(box[-1]) == 'person']
    # for box in boxs:
    #     x1, y1, x2, y2, conf, cls_name = box[0], box[1], box[2], box[3], box[4], box[-1]
    #     w, h = x2 - x1, y2 - y1
    #     detections.append(([x1, y1, w, h], conf, cls_name))  # DeepSORT 接收 xywh 格式

    # 直接用numpy进行数据的读取效率更高
    boxs = results.xyxy[0].cpu().numpy()  
    detections = []
    for box in boxs:
        x1, y1, x2, y2, conf, cls_id = box[:6]
        cls_name = yolov5_param.model.names[int(cls_id)]

        # 只识别处理person，剔除其他物体，提高检测和跟踪的运行速度
        if cls_name != 'person':
            continue

        w, h = x2 - x1, y2 - y1
        detections.append(([x1, y1, w, h], conf, cls_name))  # DeepSORT 接收 xywh 格式

    # 限制update时只使用一个线程，防止CPU占用率过高，影响GPU加速
    with threadpool_limits(limits=1, user_api='blas'):
        tracks = yolov5_param.tracker.update_tracks(detections, frame=frame)

    for track in tracks:
        if not track.is_confirmed():
            continue
        track_id = track.track_id
        ltrb = track.to_ltrb()  # (x1, y1, x2, y2)
        cls_name = track.get_det_class()  # 需要 deep_sort_realtime >= 1.3

        color = color_classes.get(cls_name, np.random.randint(0, 255, 3))
        color_classes[cls_name] = color

        # 画框 + ID
        cv2.rectangle(cv_image, (int(ltrb[0]), int(ltrb[1])), (int(ltrb[2]), int(ltrb[3])),
                      (int(color[0]), int(color[1]), int(color[2])), 2)
        cv2.putText(cv_image, f'{cls_name} ID:{track_id}', (int(ltrb[0]), int(ltrb[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 处理并存储识别 + 跟踪结果
        bounding_box = BoundingBox()
        bounding_box.probability = 1.0  # track没有置信度，设为1
        bounding_box.xmin = int(ltrb[0])
        bounding_box.ymin = int(ltrb[1])
        bounding_box.xmax = int(ltrb[2])
        bounding_box.ymax = int(ltrb[3])
        bounding_box.num = len(tracks)
        bounding_box.Class = cls_name
        bounding_boxes.bounding_boxes.append(bounding_box)   

    # 发布目标数据，topic为：/yolov5/targets
    # 可以使用命令查看：rotopic echo /yolov5/targets
    yolov5_param.target_pub.publish(bounding_boxes)
    # 将标识了识别目标的图像转换成ROS消息并发布
    image_pub.publish(cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main():
    warnings.simplefilter("ignore", category=FutureWarning)

    rospy.init_node("detect_ros")
    rospy.loginfo("starting detect_ros node")

    bridge = CvBridge()
    yolov5_param = Yolov5Param()
    color_classes = {}
    image_pub = rospy.Publisher("/yolov5/detection_image", Image, queue_size=1)
    bind_image_cb = partial(image_cb, cv_bridge=bridge, yolov5_param=yolov5_param, color_classes=color_classes, image_pub=image_pub)
    rospy.Subscriber("/camera/color/image_raw", Image, bind_image_cb)
   
    rospy.spin()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
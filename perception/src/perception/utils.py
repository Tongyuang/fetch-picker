'''
 # @ Author: Yuang Tong
 # @ Create Time: 2023-05-25 13:10:43
 # @ Modified by: Yuang Tong
 # @ Modified time: 2023-05-25 13:16:04
 # @ Description:
 '''
from ultralytics import YOLO
import cv2
import rosbag
from sensor_msgs.msg import PointCloud2
import os

class bbox:
    # from yolov8 output to bbox
    # [ 86.6558, 278.1412, 137.8118, 411.9097,   0.9700,   0.0000]
    def __init__(self,detect_result):
        self.x1 = detect_result[0]
        self.y1 = detect_result[1]
        self.x2 = detect_result[2]
        self.y2 = detect_result[3]
        self.confidence = detect_result[4]
        self.classidx = detect_result[5]

def drawbbox(frame,bbox,puttext=False):
    if bbox is not None:
        upperleftpos = (int(bbox.x1),int(bbox.y1))
        cv2.rectangle(frame,upperleftpos,(int(bbox.x2),int(bbox.y2)),(0, 255, 0), 2)
        if puttext:
            cv2.putText(frame,
                    'Hammer conf:{:.3f}'.format(bbox.confidence),
                    upperleftpos,
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale = 1.0,
                    color = (0,255,0),# green
                    )

def drawmask(frame,mask, mask_color_bgr = [0,0,255],mask_hue = 0.3):
    if mask is not None:
        for i in range(frame.shape[-1]):
            frame[:,:,i][mask>0] = frame[:,:,i][mask>0]*(1-mask_hue)+mask_color_bgr[i]*mask_hue
            
def detectObject(model,frame):
    """detect the objects in the frame

    Args:
        model: the yolov8 model
        frame: the frame to be detected
    
    Returns:
        bbox: the bounding box lists
        time: the detection time
        mask: the list of masks
    """
    result = model.predict(frame)[0]
    time = 0.0
    for time_key in result.speed.keys():
        time += result.speed[time_key]
    bbox_list, mask_list = [],[]
    detection_length = result.boxes.data.shape[0]
    
    if detection_length > 0 and detection_length == result.masks.data.shape[0]:
        for i in range(detection_length):
            box_npy = result.boxes.data[i].cpu().numpy()
            this_bbox = bbox(box_npy)
            bbox_list.append(this_bbox)
            mask_npy = result.masks.data[i].cpu().numpy()
            mask_list.append(mask_npy)
    
    return bbox_list,mask_list,time

def drawfps(frame,fps=120):
    cv2.putText(frame,
            'fps:{:.1f}'.format(fps),
            (frame.shape[1]-90,20),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 0.6,
            color = (0,255,0),# green
            )

def drawinffps(frame,inffps=120):
    cv2.putText(frame,
            'process fps:{:.1f}'.format(inffps),
            (frame.shape[1]-175,40),
            fontFace = cv2.FONT_HERSHEY_DUPLEX,
            fontScale = 0.6,
            color = (0,255,0),# green
            )

def savebag(message,save_dir = 'test.bag'):
    """Save a pointcloud2 bag

    Args:
        message (pointcloud2): 
        save_dir (str, optional):Save dir. Defaults to 'test.bag'.
    """
    bag = rosbag.Bag(save_dir,'w')
    try:
        bag.write('pointcloud2',message)
    finally:
        bag.close()

def readbag(save_dir='test.bag',topic='pointcloud2'):
    """Read message from bag

    Args:
        save_dir (str, optional): bag dir. Defaults to 'test.bag'.
    """
    bag = rosbag.Bag(save_dir)
    out_msg = None
    for tt,msg,t in bag.read_messages(topics=[topic]):
        out_msg = msg
    return out_msg        
#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
from torchvision.models.segmentation import deeplabv3_resnet50
from torchvision.models.segmentation.deeplabv3 import DeepLabHead
from torchvision.models.segmentation import deeplabv3_resnet101
from torchvision.models.segmentation import  deeplabv3_mobilenet_v3_large
import numpy as np
import cv2
from read_image_bag.srv import *

class ShapePredictNode :
    def __init__(self, model):
        # Params
        self.rgb = None
        self.depth = None
        self.smt = None
        self.rgb_b =None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        # Publishers
        self.pubimg = rospy.Publisher('/semantic/rgb', Image,queue_size=1)
        self.pubsmt = rospy.Publisher('/semantic/smt', Image,queue_size=1)
        self.pubdepth = rospy.Publisher('/semantic/depth', Image,queue_size=1)
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        ts = message_filters.TimeSynchronizer([rgb_sub,depth_sub], 1)
        ts.registerCallback(self.callback_getimage)
        ss = rospy.Service('rgbshape_model', Rgbimage, self.handle_rgbshape_service)
        self.model  = model

    def predict(self,image):
        transform = transforms.Compose([
              transforms.ToTensor(),
              transforms.Lambda(lambda x: x.float()),
              transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
          ])
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rgb = transform(image).unsqueeze(0)
        rgb = rgb.to(device)
        # print('size: ')
        # print(rgb.size())
        self.model.eval()
        outputs =  self.model(rgb)
        pred2 = torch.argmax(outputs['out'], 1)
        pred2 = pred2.squeeze(0)
        # print(pred2.size())
        pred2 = pred2.cpu().numpy()
        
        w, h = 640, 480
        data = np.zeros((h, w, 3), dtype=np.uint8)
        data = self.colorize_sweetpp(pred2)
        return data
    def remove_background(self):
        if self.rgb is not None:
            data =  self.rgb.copy()
           
            img_1 = data[:,:,0]
            img_1[self.depth > 1000] = 0
            img_1[self.depth < 300] = 0
            data[:,:,0] = img_1
            img_1 = data[:,:,1]
            img_1[self.depth > 1000] = 0
            img_1[self.depth < 300] = 0
            data[:,:,1] = img_1
            img_1 = data[:,:,2]
            img_1[self.depth > 1000] = 0
            img_1[self.depth < 300] = 0
            data[:,:,2] = img_1
            return data
        
    def handle_rgbshape_service(self,req):
        cv_image = self.br.imgmsg_to_cv2(req.s_input, "passthrough")
        if cv_image is not None:
                data =  cv_image.copy()
                data = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
                data = self.predict(data)
                imsmt = self.br.cv2_to_imgmsg(data)
                imsmt.header = req.s_input.header
                return RgbimageResponse(imsmt)
        
    def callback_getimage(self, rgb_sub,depth_sub):
        self.rgb = self.br.imgmsg_to_cv2(rgb_sub)
         
        self.depth = self.br.imgmsg_to_cv2(depth_sub)
        if self.rgb is not None:
                # self.smt = self.predict( self.rgb)
                self.rgb_b = self.remove_background()
                self.smt = self.predict( self.rgb_b)
                smtMsg = self.br.cv2_to_imgmsg(self.smt)
                smtMsg.header = rgb_sub.header
                self.pubimg.publish(rgb_sub)
                self.pubsmt.publish(smtMsg)
                self.pubdepth.publish(depth_sub)
    def colorize_sweetpp(self, img):
        h,w = img.shape
        l=np.zeros((h,w,3))
        l = l.astype(np.uint8)
        stemp = l[:,:,0]   #blue
        stemp[img == 1] = 128
        stemp[img == 2] = 0
        stemp[img == 3] = 160
        stemp[img == 4] = 232
        stemp[img == 5] = 60
        stemp[img == 6] = 0
        stemp[img == 7] = 0
        stemp[img == 9] = 255
        stemp[img == 10] = 0
        l[:,:,0] = stemp

        stemp = l[:,:,1]
        stemp[img == 1] = 64
        stemp[img == 2] = 255
        stemp[img == 3] = 170
        stemp[img == 4] = 30
        stemp[img == 5] = 20
        stemp[img == 6] = 16
        stemp[img == 7] = 255
        stemp[img == 9] = 0
        stemp[img == 10] = 0
        l[:,:,1] = stemp

        stemp = l[:,:,2]
        stemp[img == 1] = 128
        stemp[img == 2] = 0
        stemp[img == 3] = 250
        stemp[img == 4] = 244
        stemp[img == 5] = 220
        stemp[img == 6] = 172
        stemp[img == 7] = 255
        stemp[img == 9] = 0
        stemp[img == 10] = 255
        l[:,:,2] = stemp
        return l          
    def colorize_sweetpp2(self, img):
        h,w = img.shape
        l=np.zeros((h,w,3))
        l = l.astype(np.uint8)
        stemp = l[:,:,0]   #blue
        stemp[img == 1] = 128
        stemp[img == 2] = 0
        stemp[img == 3] = 160
        stemp[img == 4] = 232
        stemp[img == 5] = 60
        stemp[img == 6] = 0
        stemp[img == 7] = 0
        stemp[img == 9] = 255
        stemp[img == 10] = 0
        l[:,:,0] = stemp

        stemp = l[:,:,1]
        stemp[img == 1] = 64
        stemp[img == 2] = 255
        stemp[img == 3] = 170
        stemp[img == 4] = 30
        stemp[img == 5] = 20
        stemp[img == 6] = 16
        stemp[img == 7] = 255
        stemp[img == 9] = 0
        stemp[img == 10] = 0
        l[:,:,1] = stemp

        stemp = l[:,:,2]
        stemp[img == 1] = 128
        stemp[img == 2] = 0
        stemp[img == 3] = 250
        stemp[img == 4] = 244
        stemp[img == 5] = 220
        stemp[img == 6] = 172
        stemp[img == 7] = 255
        stemp[img == 9] = 0
        stemp[img == 10] = 255
        l[:,:,2] = stemp
        return l
    def start(self):
        print('--start-- service')
        rospy.spin()
    

def custom_DeepLabv3(in_channels, num_classes):
  model =  deeplabv3_mobilenet_v3_large(pretrained=True, progress=True)
  model.classifier = DeepLabHead(960, num_classes)
  #Set the model in training mode
  model.train()
  return model

if __name__ == '__main__':
    rospy.init_node("shape_node", anonymous=True)
    numclass = 7
    model = custom_DeepLabv3(3, numclass)
    model.load_state_dict(torch.load('/home/isl/catkin_ws/src/read_image_bag/weights/mobile_deeplap_shape.pth', 
                  map_location=lambda storage, loc: storage))    
    model.eval()
    my_node = ShapePredictNode(model)
  
    my_node.start()
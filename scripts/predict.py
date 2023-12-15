import torch
import torchvision.transforms as transforms
from torchvision.models.segmentation import deeplabv3_resnet50
from torchvision.models.segmentation.deeplabv3 import DeepLabHead
from torchvision.models.segmentation import deeplabv3_resnet101
from torchvision.models.segmentation import  deeplabv3_mobilenet_v3_large
import numpy as np
import cv2
numclass = 7
def colorize_sweetpp(img):
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

def custom_DeepLabv3(in_channels, num_classes):
  model =  deeplabv3_mobilenet_v3_large(pretrained=True, progress=True)
  model.classifier = DeepLabHead(960, num_classes)
  #Set the model in training mode
  model.train()
  return model
def predict(model,image,device):
        # image = cv2.imread(path +"/sample.png",1)
        transform = transforms.Compose([
              transforms.ToTensor(),
              transforms.Lambda(lambda x: x.float()),
              transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
          ])
        rgb = transform(image).unsqueeze(0)
        rgb = rgb.to(device)
        # print('size: ')
        # print(rgb.size())
        model.eval()
        outputs =  model(rgb)
        pred2 = torch.argmax(outputs['out'], 1)
        pred2 = pred2.squeeze(0)
        # print(pred2.size())
        pred2 = pred2.cpu().numpy()
       
        w, h = 640, 480
        data = np.zeros((h, w, 3), dtype=np.uint8)
        data = colorize_sweetpp(pred2)
        cv2.imshow("reslut", data)
        # cv2.imwrite(path + "/result.png",data)
        cv2.waitKey(0)
        return pred2


# Model class must be defined somewhere
model = custom_DeepLabv3(3, numclass)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.load_state_dict(torch.load('/home/isl/catkin_ws/src/read_image_bag/weights/mobile_deeplap_shape.pth', 
                  map_location=lambda storage, loc: storage))    
model.eval()
path = "/home/isl/catkin_ws/src/read_image_bag/images/rgb/1701666479916.png"
image = cv2.imread(path  ,1)
cv2.imshow('rgb',image)
mask = predict(model,image,device)

 
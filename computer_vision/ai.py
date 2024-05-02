import torch
import torch.nn as nn
from torchvision import transforms

class ai():
    def __init__(self, model):
        # Loading model
        self.model = torch.jit.load(model)
        self.model.eval()
        self.running_pred = torch.tensor([0,0,0])
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((120,160)),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def predict_image(self,img):
        img = self.transform(img)
        return torch.argmax(self.model(img),dim=1).item()
    
    def predict_running(self,img,*,alpha):
        pred = self.model(self.transform(img))
        self.running_pred = (pred*alpha+self.running_pred)/(1+alpha)
        return torch.argmax(self.running_pred,dim=1).item()


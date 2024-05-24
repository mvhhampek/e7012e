import torch
import torch.nn as nn
from torchvision import transforms

class ai():
    def __init__(self, model):
        # Loading model
        self.model = torch.jit.load(model)
        self.model.eval()
        self.running_pred = torch.tensor([0,1,0])
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((120,160)),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def predict_image(self,img):
        img = torch.unsqueeze(self.transform(img),dim=0)
        pred = nn.Softmax()(self.model(img)).detach().tolist()
        print(pred)
        if (pred[0][0]>0.99):
            return 0
        if (pred[0][1]>0.99):
            return 1
        return 2
        #return torch.argmax(pred,dim=1).item()
    
    def predict_running(self,img,*,alpha):
        pred = self.model(torch.unsqueeze(self.transform(img),dim=0))
        self.running_pred = (pred*alpha+self.running_pred)/(1+alpha)
        pred = nn.Softmax()(self.running_pred).detach().tolist()
        #print(pred)
        if (pred[0][0]>0.99):
            return 0
        if (pred[0][1]>0.95):
            return 1
        return 2


# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os
import pandas as pd
from IPython.display import HTML
import urllib
import math
# from sklearn import tree
# from sklearn.metrics import classification_report, confusion_matrix  
# from numpy.random import seed
# import pandas as pd
# from sklearn.model_selection import cross_val_score

class Robot:
    def __init__(self):

        self.url='http://192.168.0.35:8080/shot.jpg'
        self.acao = ['stop','frente','direita','esquerda']
        self.rho_resolution = 1
        self.theta_resolution = np.pi/180
        self.threshold = 155
        self.areaInteresse_X2 = 0.85
        self.areaInteresse_X1 = 0.40
        self.areaInteresse_Y = 0.15
        self.W = 0 
        self.H = 0
        self.cap = cv2.VideoCapture(0)
        self.pontosLimitesAng = [4,170]
        self.dataSet = []
        self.YTreinamento = ''

        
    def areaInteresse(self,img):        
        M = cv2.getRotationMatrix2D((int(img.shape[0])/2,int(img.shape[1])/2),-90,1)
        img = cv2.warpAffine(img,M,(int(img.shape[0]),int(img.shape[1])))

        self.W  = int(img.shape[1])
        self.H  = int(img.shape[0])


        shape = np.array([[int(self.areaInteresse_X1*self.W), int(self.H )], [int(self.areaInteresse_X2*self.W), int(self.H )], [int(self.areaInteresse_X2*self.W), int(self.areaInteresse_Y*self.H )], [int(self.areaInteresse_X1*self.W), int(self.areaInteresse_Y*self.H)]])
        mask = np.zeros_like(img)
        
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
    
        cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)

        masked_image = cv2.bitwise_and(img.copy(), mask)

        return masked_image

    def filtro(self,image):

            image = self.areaInteresse(image)
            gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
            ret3,th3 = cv2.threshold(blurred_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            edges_image = cv2.Canny(th3, 50, 120)

            return image , edges_image 
             

    def olhar(self, img):
        if(img == 'webcam'):
            _ , img = self.cap.read()
        else:
            if(img == 'android'):  
                imgResp=urllib.urlopen(self.url)
                imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
                img = cv2.imdecode(imgNp,-1)  
            else: 
              img = cv2.imread(img) 

        return img      
    def proxMovimentacao(self,curva):
        direita = 0 
        esqueda = 0 
        for angulo, pos in curva: 
            if(pos < self.areaInteresse_X1*self.W ):
                    esqueda = esqueda + 1
            if(pos > self.areaInteresse_X2*self.W ):
                    direita = direita + 1
            if((angulo > self.pontosLimitesAng[0]) and (angulo < 150 )):
                    direita = direita + 1
            if((angulo < self.pontosLimitesAng[1]) and (angulo > 150 )):
                    esqueda = esqueda + 1
            if(direita >  esqueda):
                print 'Siga para direita'
            else:
                print 'Siga para esquerda'

    def desenharLinhas(self,theta,rho,i,img, color=[0, 255, 0], thickness=7):
            a = np.cos(theta)
            b = np.sin(theta)
            angulo = int(math.degrees(theta))
            pos    = int(a*rho)

            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            if((angulo != 0) and (angulo != 90)): 
                cv2.putText(img, str(angulo) + "Graus", (int(pos),100 + i), cv2.FONT_HERSHEY_SIMPLEX,0.7, (255, 255, 255), 1)
                color=[0, 255, 0]
            else:
                color=[255 , 0, 0]
            cv2.line(img,(x1,y1),(x2,y2),color,thickness)

    def  analiseLinnhas(self,img, houghLines):
            i = 0
            for line in houghLines:
                for rho,theta in line:  
                    a = np.cos(theta)
                    pos    = a*rho
                    angulo = int(math.degrees(theta))
                    if((angulo != 0) and (angulo != 90)): 
                        if(self.YTreinamento != ''):
                            self.dataSet.append((a,pos,self.YTreinamento))
                    self.desenharLinhas(theta,rho,i,img)
                    i = i + 30

            #self.proxMovimentacao(curva)

    def weighted_img(self,img, initial_img, a=0.8, b=1., l=0.):
        return cv2.addWeighted(initial_img, a, img, b, l) 

    def detectarCaminho(self): 
            image = self.olhar('webcam')
            image, imageFiltrada = self.filtro(image)            
            linhasDetectadas = cv2.HoughLines(imageFiltrada, self.rho_resolution , self.theta_resolution , self.threshold)
            if(linhasDetectadas is not (None) ):
                hough_lines_image = np.zeros_like(image)
                self.analiseLinnhas(hough_lines_image, linhasDetectadas)
                original_image_with_hough_lines = self.weighted_img(hough_lines_image,image)
                return  original_image_with_hough_lines
            else:
                return None

    def imgTreinamento(self):
        return cv2.imread('imgTreinamento.png')

    def treinarRobot(self):
            image = self.imgTreinamento()
            cv2.namedWindow("Tela_Treinamento")
            cv2.setMouseCallback("Tela_Treinamento", self.clickComandoTreinamento)
            cv2.imshow("Tela_Treinamento", image)

            while 1:
                deteccao = self.detectarCaminho()
                if(deteccao is not (None)):
                    cv2.imshow('img',self.detectarCaminho())
                k = cv2.waitKey(30) & 0xff
                if k == 27:
                    break
            labels = ['inclinacaoCos', 'pos', 'comando']
            df = pd.DataFrame.from_records(self.dataSet, columns=labels)
            print df
            print 'Treinamento Finalizado'

    def mudarYTreinamento(self,cmd):
        if(self.YTreinamento == cmd):
            self.YTreinamento = ''
            return False
        else:
            self.YTreinamento = cmd
            return True

    def clickComandoTreinamento(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            image =  self.imgTreinamento()

            X = image.shape[1]
            Y = image.shape[0]

            if((y < Y/2) and (x < X/2)):
                cmd = self.acao[0]
            else:
                if((y < Y/2) and (x > X/2)):
                    cmd = self.acao[1]
                else:
                    if((y > Y/2) and (x > X/2)):
                        cmd = self.acao[2]  
                    else:
                        if((y > Y/2) and (x < X/2)):
                            cmd = self.acao[3]   

            if(self.mudarYTreinamento(cmd)):
                cv2.circle(image,(x, y), 13, (0,0,255), -1)
            cv2.imshow("Tela_Treinamento", image)

def main():                        
    v = Robot()
    v.treinarRobot()


    v.cap.release()
    cv2.destroyAllWindows()


main()
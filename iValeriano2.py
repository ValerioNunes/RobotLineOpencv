# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os
import pandas as pd
from IPython.display import HTML
import urllib
import math
import requests
import threading
import socket
from joblib import dump, load
from sklearn import tree
from sklearn.metrics import classification_report, confusion_matrix  
from numpy.random import seed
import pandas as pd
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import train_test_split  
import time
import thread

clear = lambda: os.system('cls')

class Memoria:
    def __init__(self):
        self.clf =  load('MemoriaValerianoRobot.joblib') 
        self.dataSet = []

    def persistenceModel(self,clf):
        dump(clf, 'MemoriaValerianoRobot.joblib') 

    def tratamendoDados(self):
            if(len(self.dataSet) > 0 ):
                labels = ['inclinacaoCos', 'pos', 'comando']
                df = pd.DataFrame.from_records(self.dataSet, columns=labels)
                self.iniciarTreinamento(df)
                print 'Treinamento Finalizado'

    def iniciarTreinamento(self,df):
            Y = df['comando']
            X = df.drop(columns = ['comando'])
            X_train, X_test, y_train, y_test = train_test_split(X, Y, test_size=0.30,random_state=1)  
            clf = tree.DecisionTreeClassifier()
            clf = clf.fit(X_train, y_train)
            y_pred = clf.predict(X_test)
            print(confusion_matrix(y_test, y_pred))  
            print(classification_report(y_test, y_pred))

            self.persistenceModel(clf)

    def mudarYTreinamento(self,cmd):
        if(self.YTreinamento == cmd):
            self.YTreinamento = ''
            return False
        else:
            self.YTreinamento = cmd
            return True

class Robot:
    def __init__(self ,  cam = 'webcam'):

        self.url='http://192.168.0.102:8080/shot.jpg'
        self.UDP_IP = "192.168.0.103"
        self.UDP_PORT = 1234


        self.acao = ['stop','frente','direita','esquerda','re']
        self.rho_resolution = 2
        self.theta_resolution = np.pi/180
        self.threshold = 155
        self.areaInteresse_X2 = 0.98
        self.areaInteresse_X1 = 0.01
        self.areaInteresse_Y = 0.07
        self.W = 0 
        self.H = 0
        self.cap = cv2.VideoCapture(0)
        self.pontosLimitesAng = [4,170]
        self.dataSet = []
        self.YTreinamento = ''
        self.Treinar = False
        self.UltimaAcao = 'stop'
        self.ContadorZeroLinhas = 0 
        self.MaxParaEmergencia = 20
        self.ThreadRequet = None
        self.inputCam = cam
        self.vel = 440
        self.velStop =0
        self.dadosAcao = {}
        self.dadosAcao['stop']     = [0,0]
        self.dadosAcao['frente']   = [0,0]
        self.dadosAcao['direita']  = [1,0]
        self.dadosAcao['esquerda'] = [0,1]
        self.dadosAcao['re']       = [1,1]
        self.filtroFlagView = False

        self.memoria = Memoria()
        self.clf = None

    def rotacionarImagem(self,img):   
        (h, w) = img.shape[:2]
        center = (w / 2, h / 2)
        angle90 = 90
        angle180 = 180
        angle270 = 270
        scale = 1.0     
        M = cv2.getRotationMatrix2D(center, angle180, scale)
        img = cv2.warpAffine(img, M, (w, h))

        self.W  = int(img.shape[1])
        self.H  = int(img.shape[0])


        # shape = np.array([[int(self.areaInteresse_X1*self.W), int(self.H )], [int(self.areaInteresse_X2*self.W), int(self.H )], [int(self.areaInteresse_X2*self.W), int(self.areaInteresse_Y*self.H )], [int(self.areaInteresse_X1*self.W), int(self.areaInteresse_Y*self.H)]])
        # mask = np.zeros_like(img)
        
        # if len(img.shape) > 2:
        #     channel_count = img.shape[2]
        #     ignore_mask_color = (255,) * channel_count
        # else:
        #     ignore_mask_color = 255
    
        # cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)

        # masked_image = cv2.bitwise_and(img.copy(), mask)

        return img

    def filtro(self,image):

            image = self.rotacionarImagem(image)
            gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
            ret3,th3 = cv2.threshold(blurred_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            edges_image = cv2.Canny(th3, 90, 120)
            if(self.filtroFlagView):
                cv2.imshow("test_Filtro", edges_image)
            return image , edges_image 
            


    def olhar(self):
        img =  self.inputCam
        if(img == 'webcam'):
            _ , img = self.cap.read()
        else:
            if(img == 'android'):  
                cap = cv2.VideoCapture(self.url)
                r , img =  cap.read()
            else: 
              img = cv2.imread(img) 

        return img      
    def enviarAcaoParaNode(self,cmd, force = False ):

        if((self.UltimaAcao != cmd) or force):
            try:
                if(self.ThreadRequet is not (None)):
                    if(self.ThreadRequet.isAlive()):
                        return False
                
                motor = self.dadosAcao[cmd]

                if(self.acao[0] != cmd):
                    velocidade  =  self.vel 
                else:
                    velocidade = 0 

                self.ThreadRequet = threading.Thread(target= realizarComado,args=(self.UDP_IP,self.UDP_PORT, motor[0],motor[1],  velocidade ,  velocidade,))
                self.ThreadRequet.start()
                self.UltimaAcao  = cmd
                return True

            except:
                print("Erro no enviarAcaoParaNode")
                return False
    def agrupamentoLinha(self, linhas):
            linhas = np.array(linhas)
            linhas = np.median(linhas, axis = 0)
            return linhas

    def proxMovimentacao(self,caminhos,img):
        comando = self.UltimaAcao

        if(len(caminhos) > 0):
            self.ContadorZeroLinhas = 0
            caminho = self.agrupamentoLinha(caminhos)
            comando = self.acao[int(self.memoria.clf.predict(caminho.reshape(1,-1))[0])]
          
            self.enviarAcaoParaNode(comando)
        else:
            
            self.ContadorZeroLinhas += 1
            if(self.ContadorZeroLinhas > self.MaxParaEmergencia ):
                    comando = 'stop'
                    self.stopRobot()  

        cv2.putText(img, comando.upper(), (50,100), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 3)
        if(comando == self.acao[0]):
            vel = self.velStop
        else:
            vel = self.vel
        cv2.putText(img,'Vel: ' + str(vel), (50,150), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 3)

    def desenharLinhas(self,theta,rho,img, color=[0, 255, 0], thickness=7):
            a = np.cos(theta)
            b = np.sin(theta)
            angulo = int(math.degrees(theta))
            x0 = a*rho
            y0 = b*rho
            TAM = 3000
            x1 = int(x0 + TAM*(-b))
            y1 = int(y0 + TAM*(a))
            x2 = int(x0 - TAM*(-b))
            y2 = int(y0 - TAM*(a))
            if((angulo != 0) and (angulo != 90)): 
                color=[0, 255, 0]
            else:
                color=[255 , 0, 0]
            cv2.line(img,(x1,y1),(x2,y2),color,thickness)

    def  stopRobot(self):
          self.enviarAcaoParaNode(self.acao[0])  

    def  analiseLinnhas(self,img, houghLines):
            caminhos = []
            if(len(houghLines) > 0):
                for line in houghLines:
                    for rho,theta in line:  
                        a = np.cos(theta)
                        b = np.sin(theta)
                        angulo = int(math.degrees(theta))
                        x0 = a*rho
                        y0 = b*rho
                        angulo = int(math.degrees(theta))

                        caminhos.append([a,b,x0,y0,angulo])
                        if(self.YTreinamento != ''):
                            self.dataSet.append((a,b,x0,y0,angulo,self.YTreinamento))

                        self.desenharLinhas(theta,rho,img)
            else:
                        self.stopRobot() 
            print  self.YTreinamento
            if(self.Treinar == False):
                self.proxMovimentacao(caminhos,img)
            else:
                if(self.YTreinamento != ''):
                    pass
                    #self.enviarAcaoParaNode(self.acao[self.YTreinamento])
                else: 
                    self.stopRobot() 

    def weighted_img(self,img, initial_img, a=0.8, b=1., l=0.):
        return cv2.addWeighted(initial_img, a, img, b, l) 

    def detectarLinhas(self,imageFiltrada,image):
        linhasDetectadas = cv2.HoughLines(imageFiltrada, self.rho_resolution , self.theta_resolution , self.threshold)
        
        if(linhasDetectadas is not (None)):
            hough_lines_image = np.zeros_like(image)
            self.analiseLinnhas(hough_lines_image, linhasDetectadas)
            
            #hough_lines_image = self.weighted_img(hough_lines_image,image)
            return  hough_lines_image

        return None

    def detectarCaminho(self): 
            image = None
            try:
                image = self.olhar()
            except:
                print "Erro ao Olhar"
            if(image is not (None) ):
                image, imageFiltrada = self.filtro(image)  
                #try:      
                imagemComAsLinhasEncontradas =  self.detectarLinhas(imageFiltrada,image)
                return imagemComAsLinhasEncontradas
                #except:
                #    print "Erro na deteccao de Linhas"
            
            return None

    def imgTreinamento(self):
        return cv2.imread('imgTreinamento.png')

    def persistenceModel(self,clf):
            dump(clf, 'MemoriaValerianoRobot.joblib') 

    def iniciarTreinamento(self,df):
            Y = df['comando']
            X = df.drop(columns = ['comando'])
            X_train, X_test, y_train, y_test = train_test_split(X, Y, test_size=0.30,random_state=1)  
            clf = tree.DecisionTreeClassifier()
            clf = clf.fit(X_train, y_train)
            y_pred = clf.predict(X_test)
            print(confusion_matrix(y_test, y_pred))  
            print(classification_report(y_test, y_pred))

            self.persistenceModel(clf)

    def treinarRobot(self):
            image = self.imgTreinamento()
            cv2.namedWindow("Tela_Treinamento")
            cv2.setMouseCallback("Tela_Treinamento", self.clickComandoTreinamento)
            cv2.imshow("Tela_Treinamento", image)
            self.Treinar = True
            self.Start()
            if(len(self.dataSet) > 0 ):
                labels = ['a','b','x0','y0','angulo','comando']# ['inclinacaoCos', 'pos', 'comando']
                df = pd.DataFrame.from_records(self.dataSet, columns=labels)
                self.iniciarTreinamento(df)
                print 'Treinamento Finalizado'

    def clickComandoTreinamento(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            image =  self.imgTreinamento()

            X = image.shape[1]
            Y = image.shape[0]

            if((y < Y/2) and (x < X/2)):
                cmd =  0 # self.acao[0]
            else:
                if((y < Y/2) and (x > X/2)):
                    cmd = 1 #self.acao[1]
                else:
                    if((y > Y/2) and (x > X/2)):
                        cmd = 2 #self.acao[2]  
                    else:
                        if((y > Y/2) and (x < X/2)):
                            cmd = 3 # self.acao[3]   

            if(self.mudarYTreinamento(cmd)):
                cv2.circle(image,(x, y), 13, (0,0,255), -1)
            cv2.imshow("Tela_Treinamento", image)

    def imgComandoVelocidade(self):
        return cv2.imread('imgVelocidade.png')

    def clickComandoVelocidade(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            image =  self.imgComandoVelocidade()

            X = image.shape[1]
            Y = image.shape[0]
           
            if((y > Y*0.5) and (x < X*0.4)):
                self.stopRobot() 
            else:
                if((y < Y/2) and (x > X*0.6)):
                    if(self.vel < 541 ):
                        self.vel += 10 
                else:
                    if((y > Y/2) and (x > X*0.6)):
                        if(self.vel > 10 ):
                            self.vel -= 10 

            self.enviarAcaoParaNode(self.UltimaAcao, force = True)
            cv2.circle(image,(x, y), 13, (0,0,255), -1)
            cv2.imshow("Tela_Velocidade", image)


    def mudarYTreinamento(self,cmd):
        if(self.YTreinamento == cmd):
            self.YTreinamento = ''
            return False
        else:
            self.YTreinamento = cmd
            return True

    def iniciarRobo(self):
            

            image = self.imgComandoVelocidade()
            cv2.namedWindow("Tela_Velocidade")
            cv2.setMouseCallback("Tela_Velocidade", self.clickComandoVelocidade)
            cv2.imshow("Tela_Velocidade", image)

    def Start(self):
            self.iniciarRobo()
            while 1:
                deteccao = self.detectarCaminho()
                if(deteccao is not (None)):
                    cv2.imshow('img',deteccao[::2,::2])
                k = cv2.waitKey(3) & 0xff
                if k == 27:
                    break  


sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
MESSAGE = ''
def realizarComado(UDP_IP, UDP_PORT, m1s , m2s  , m1v ,  m2v  ):
    global MESSAGE
    MESSAGE =  '{0}{1}{2}{3}'.format(m1s,m2s,m1v,m2v)   
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

def loopEnvioCMD( robot):
    global MESSAGE
    try :
        while True:
            if(MESSAGE != ''):
                sock.sendto(MESSAGE, (robot.UDP_IP, robot.UDP_PORT))   
            time.sleep(2)
    except:
        print "Erro no lood de envia de cmd"
		
v = Robot(cam = 'android')



def main():     
    clear()      
    print '<<<< VALERIANO ROBOT >>>>'
    print '1 - Treinar Novo Modelo'
    print '2 - Start Robot '
    x = input('Digite o item desejado:')      

    realizarComado(v.UDP_IP, v.UDP_PORT,0, 0,0, 0) # STOP
    try:
        thread.start_new_thread( loopEnvioCMD, (v, ))
    except:
        print "Error: unable to start thread"
    
    if(x == 1):
        v.treinarRobot()
    else:
        if(x == 2):
            v.Start()

    v.cap.release()
    cv2.destroyAllWindows()

try:
    main()
finally:
    realizarComado(v.UDP_IP, v.UDP_PORT,0, 0,0, 0)
    cv2.destroyAllWindows()
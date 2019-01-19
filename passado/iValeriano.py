import cv2
import numpy as np
import os
import pandas as pd
from IPython.display import HTML
import urllib



class Robot:
    def __init__(self):

        self.detector= cv2.xfeatures2d.SIFT_create()
        FLANN_INDEX_KDITREE=1
        flannParam= dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
        self.flann=cv2.FlannBasedMatcher(flannParam,{})
        self.cap = cv2.VideoCapture(0)
        
        self.url='http://192.168.0.35:8080/shot.jpg'
        self.DATASET  = '.\\dataset'
        self.conhecimento = []
        self.acoes =  ['stop','direita','esquerda']

    def buscarDataset(self,pasta):
        caminhos = [os.path.join(pasta, nome) for nome in os.listdir(pasta)]
        arquivos = [arq for arq in caminhos if os.path.isfile(arq)]
        imgs = [arq for arq in arquivos if (arq.lower().endswith(".jpg") or arq.lower().endswith(".png"))]
        return imgs

    def detectarPontos(self,fig):
        acao = fig.split('\\')[3].split('_')[0]
        if acao in self.acoes:
            img = cv2.imread(fig) 
            trainImg = self.filter(img)
            _ , trainDesc=self.detector.detectAndCompute(trainImg,None)
            return [trainDesc,acao,len(trainDesc)]

        return None

    def verbose(self, msg):
        print msg

    def treinarPlacas(self):
        self.verbose('Iniciando treinamento...')
        imgs = self.buscarDataset(self.DATASET+'\\placas')
        [self.conhecimento.append(self.detectarPontos(fig)) for fig in imgs]
        self.verbose('Fim de treinamento...')

    def filter(self,img):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            return img
             

    def olhar(self, img):
        if(img == ''):
            _ , img = self.cap.read()
        else:
            if(img == 'webcam'):  
                imgResp=urllib.urlopen(self.url)
                imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
                img = cv2.imdecode(imgNp,-1)  
            else: 
              img = cv2.imread(img) 
        img = self.filter(img)
        return img   

    def detectarLinha(self,img):

        # Apply edge detection method on the image 
        edges = cv2.Canny(img,50,150,apertureSize = 3) 
        
        # This returns an array of r and theta values 
        lines = cv2.HoughLines(edges,1,np.pi/180, 200) 
        print lines
        # The below for loop runs till r and theta values  
        # are in the range of the 2d array 
        for r,theta in lines[0]: 
            
            # Stores the value of cos(theta) in a 
            a = np.cos(theta) 
        
            # Stores the value of sin(theta) in b 
            b = np.sin(theta) 
            
            # x0 stores the value rcos(theta) 
            x0 = a*r 
            
            # y0 stores the value rsin(theta) 
            y0 = b*r 
            
            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta)) 
            x1 = int(x0 + 1000*(-b)) 
            
            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta)) 
            y1 = int(y0 + 1000*(a)) 
        
            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta)) 
            x2 = int(x0 - 1000*(-b)) 
            
            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta)) 
            y2 = int(y0 - 1000*(a)) 
            
            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2). 
            # (0,0,255) denotes the colour of the line to be  
            #drawn. In this case, it is red.  
            cv2.line(img,(x1,y1), (x2,y2), (0,0,255),2) 
            
        # All the changes made in the input image are finally 
        # written on a new image houghlines.jpg 
        cv2.imwrite('linesDetected.jpg', img) 


    def buscarAcao(self, QueryImg):
        pontosComum = [] 
        acao = [] 
        
        if(len(self.conhecimento)):
            
            queryKP,queryDesc=self.detector.detectAndCompute(QueryImg,None)

            for c in self.conhecimento:
                matches=self.flann.knnMatch(c[0],queryDesc,k=2)
                goodMatch=[]
                for m,n in matches:
                    if(m.distance < 0.7*n.distance):
                        goodMatch.append(m)
                pct = float(len(goodMatch))/c[2]
                if(pct > 0.5):
                    print c[1] , pct 
                    qp2=[]
                    for m in goodMatch:
                        if(len(queryKP) > m.queryIdx ):
                            qp2.append(queryKP[m.queryIdx])
                    QueryImg = cv2.drawKeypoints(QueryImg,qp2,None,(0,0,255),6)

                pontosComum.append(pct)
                acao.append(c[1])

            df = pd.DataFrame({'Acao' : acao ,
                               'PontosComum' : pontosComum })
            print df
        else:
            print  'vou ficar parado, nao sei nada !!!'

        return QueryImg
    
v = Robot()
v.treinarPlacas()

while 1:
    img = v.buscarAcao(v.olhar('webcam'))
    cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

v.cap.release()
cv2.destroyAllWindows()







# # Find edges in the image using canny edge detection method
# # Calculate lower threshold and upper threshold using sigma = 0.33
# sigma = 0.33
# v = np.median(grayScale)
# low = int(max(0, (1.0 - sigma) * v))
# high = int(min(255, (1.0 + sigma) * v))

# edged = cv2.Canny(grayScale, low, high)

# # After finding edges we have to find contours
# # Contour is a curve of points with no gaps in the curve
# # It will help us to find location of shapes

# # cv2.RETR_EXTERNAL is passed to find the outermost contours (because we want to outline the shapes)
# # cv2.CHAIN_APPROX_SIMPLE is removing redundant points along a line
# _, cnts, _ = cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


# def detectShape(cnt):
#     shape = 'unknown'
#     # calculate perimeter using
#     peri = cv2.arcLength(c, True)
#     # apply contour approximation and store the result in vertices
#     vertices = cv2.approxPolyDP(c, 0.07 * peri, True)

#     # If the shape it triangle, it will have 3 vertices
#     if len(vertices) == 3:
#         shape = 'triangle'

#     # if the shape has 4 vertices, it is either a square or
#     # a rectangle
#     elif len(vertices) == 4:
#         # using the boundingRect method calculate the width and height
#         # of enclosing rectange and then calculte aspect ratio

#         x, y, width, height = cv2.boundingRect(vertices)
#         aspectRatio = float(width) / height

#         # a square will have an aspect ratio that is approximately
#         # equal to one, otherwise, the shape is a rectangle
#         if aspectRatio >= 0.95 and aspectRatio <= 1.05:
#             shape = "square"
#         else:
#             shape = "rectangle"

#     # if the shape is a pentagon, it will have 5 vertices
#     elif len(vertices) == 5:
#         shape = "pentagon"

#     # otherwise, we assume the shape is a circle
#     else:
#         shape = "circle"

#     # return the name of the shape
#     return shape


# # Now we will loop over every contour
# # call detectShape() for it and
# # write the name of shape in the center of image

# # loop over the contours
# for c in cnts:
#     # compute the moment of contour
#     M = cv2.moments(c)
#     # From moment we can calculte area, centroid etc
#     # The center or centroid can be calculated as follows
#     if( M['m00'] > 0  ):
#         cX = int(M['m10'] / M['m00'])
#         cY = int(M['m01'] / M['m00'])

#         # call detectShape for contour c
#         shape = detectShape(c)
#         if(shape == "rectangle"):
#             # Outline the contours
#             cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

#             # Write the name of shape on the center of shapes
#             cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
#                         0.5, (255, 255, 255), 2)

#             # show the output image
#             cv2.imshow("Image", image)

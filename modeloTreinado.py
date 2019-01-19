from joblib import dump, load
from sklearn import tree
from sklearn.metrics import classification_report, confusion_matrix  
from numpy.random import seed
import pandas as pd
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import train_test_split  


self.clf = load('MemoriaValerianoRobot.joblib') 

print self.clf.predict([0.982,10])
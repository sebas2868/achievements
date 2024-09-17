# -*- coding: utf-8 -*-
"""
Created on Mon Jun 12 15:28:57 2023

@author: sebas
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Importar el data set
absolute_path = "E:\[03] Semillero Neuro Control\Codigo\RAN´S\MLP\MLP_Generalizado\Python\Churn_Modelling.csv"
dataset = pd.read_csv(absolute_path)

X = dataset.iloc[:, 3:13].values
y = dataset.iloc[:, 13].values

# Codificar datos categóricos
from sklearn.preprocessing import LabelEncoder
labelencoder_X_1 = LabelEncoder()
X[:, 1] = labelencoder_X_1.fit_transform(X[:, 1])
labelencoder_X_2 = LabelEncoder()
X[:, 2] = labelencoder_X_2.fit_transform(X[:, 2])

#El OneHotEncoder en las nuevas versiones está OBSOLETO
#onehotencoder = OneHotEncoder(categorical_features=[1])
#X = onehotencoder.fit_transform(X).toarray()

from sklearn.preprocessing import OneHotEncoder
from sklearn.compose import ColumnTransformer

transformer = ColumnTransformer(
    transformers=[
        ("Churn_Modelling",        # Un nombre de la transformación
         OneHotEncoder(categories='auto'), # La clase a la que transformar
         [1]            # Las columnas a transformar.
         )
    ], remainder='passthrough'
)

X = transformer.fit_transform(X)
X = X[:, 1:]#quitando colinealdiad

# Dividir el data set en conjunto de entrenamiento y conjunto de testing
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state = 0)

# Escalado de variables - Estandarización
from sklearn.preprocessing import StandardScaler
sc_X = StandardScaler()
X_train = sc_X.fit_transform(X_train)
X_test = sc_X.transform(X_test)

b=1 #bias value
#x = np.array([b*np.ones((np.size(X,0),1)),X])
x= np.concatenate((b*np.ones((np.size(X_train,0),1)), X_train), axis=1)
y = np.atleast_2d(y_train).transpose()
data_set = np.concatenate((x, y), axis=1)
#test set:
X2 = X_test
Y2 = y_test
x1= np.concatenate((b*np.ones((np.size(X2,0),1)), X2), axis=1)
y1 = np.atleast_2d(Y2).transpose()
#Step 1: Set the Network Parameters
def f_a(x):
    return np.maximum(0, x) #Relu
def dev_a(x):
    return np.where(x > 0, 1, 0) #dev_relu
def f_a_exit(x):
    return 1 / (1 + np.exp(-x))
def dev_a_exit(x):
  return f_a_exit(x) * (1 - f_a_exit(x))

hidden_layers = int(input("Número de capas ocultas: "))

#ciclo para inicializacion de pesos de la capa oculta
neuronas = np.zeros(hidden_layers)
neuronas[0] = int(input("Neuronas en la capa capa oculta 1: "))
#print(neuronas)
#inicializacion de los pesos_sinapticos y parametros de ADAM:
W = [0]*(hidden_layers+1)
momentum = [0]*(hidden_layers+1)
rms = [0]*(hidden_layers+1)
W[0]=np.random.uniform(-0.5, 0.5, (int(neuronas[0]), x.shape[1]))  #pesos de la capada de entrada-primera capa oculta
momentum[0]= np.zeros((int(neuronas[0]), x.shape[1]))
rms[0]= np.zeros((int(neuronas[0]), x.shape[1]))
epsilon = 1e-7
for i in range(1,hidden_layers):
    neuronas[i] = int(input("Neuronas en la capa capa oculta "+str(i+1)+": "))
    W[i]=(np.random.uniform(-0.5, 0.5, (int(neuronas[i]), int(neuronas[i-1]+1)))) #Pesos entre capas ocultas
    momentum[i] = np.zeros((int(neuronas[i]), int(neuronas[i-1]+1)))
    rms[i] = np.zeros((int(neuronas[i]), int(neuronas[i-1]+1)))
W[-1]=(np.random.uniform(-0.5, 0.5, (y.shape[1], int(neuronas[hidden_layers-1]+1)))) #pesos de la capa de la ultima capa oculta - capa de salida
momentum[-1] = np.zeros((y.shape[1], int(neuronas[hidden_layers-1]+1)))
rms[-1] = np.zeros((y.shape[1], int(neuronas[hidden_layers-1]+1)))
nEpocas = int(input("Número de épocas: "))
eta_i = 0.001#float(input("Tasa de aprendizaje inicial: "))
eta_f = 0.00001#float(input("Tasa de aprendizaje final: "))
alfa1 = 0.9 #momentum constant
alfa2 = 0.999 #rms constant
emedio = [0]*nEpocas
emedio_test = [0]*nEpocas
cte = 1
for epoca in range(nEpocas):
   sumt=0
   sumt_test =0
   #eta = eta_i*((eta_f/eta_i)**(epocas/nepocas)); #descenso expo
   eta = eta_i +(eta_f-eta_i)*(epoca/nEpocas)
   np.random.shuffle(data_set)
   x = data_set[:,0:12]
   y = data_set[:,12:]
   for j in range(x.shape[0]):      
        #Listas de valores de la red:
        I = [0]*(hidden_layers+1)#Potencial de activacion
        Y = [0]*(hidden_layers+2) #Salidas de las capas
        delta = [0]*(hidden_layers+1) #Grandiente local de las neuronas
        error = [1]
        #Propagación hacia adelante: 
        Y[0]=x[j,:].T
        I[0]=np.dot(W[0],Y[0])
        Y[1]=np.append([b], f_a(I[0]))
        for i in range(1,hidden_layers):
            I[i]=np.dot(W[i],Y[i].reshape(-1,1))
            Y[i+1]=np.append([b],f_a(I[i]))
        I[hidden_layers]=np.dot(W[hidden_layers],Y[hidden_layers].reshape(-1,1))
        Y[hidden_layers+1]=(f_a_exit(I[hidden_layers])) 
        #back propagation
        error[0] = -(y[j,:] * np.log(Y[hidden_layers+1].T+ epsilon) + (1 - y[j,:]) * np.log(1 - Y[hidden_layers+1].T+ epsilon))
        delta[0] = ((y[j,:] - Y[hidden_layers+1].T).T*dev_a_exit(I[hidden_layers])).T
        W_new = [0]*len(W)
        Gradient = np.dot(delta[0].T,Y[hidden_layers].reshape(1, -1))
        momentum[hidden_layers] = alfa1*momentum[hidden_layers]+(1+alfa1)*Gradient
        rms[hidden_layers] = alfa2*rms[hidden_layers]+(1+alfa2)*Gradient**2
        W_new[hidden_layers]= W[hidden_layers]+eta*(momentum[hidden_layers]/(np.sqrt(rms[hidden_layers])+10**-8))
        aux=0
        for k in range(1,hidden_layers+1):
            delta[k]= np.dot(delta[aux],W[hidden_layers-aux][:,1:])*dev_a(I[hidden_layers-aux-1]).T #revisar
            Gradient = np.dot(delta[k].T,Y[hidden_layers-1-aux].reshape(1, -1))
            momentum[hidden_layers-1-aux] = alfa1*momentum[hidden_layers-1-aux]+(1+alfa1)*Gradient
            rms[hidden_layers-1-aux] = alfa2*rms[hidden_layers-1-aux]+(1+alfa2)*Gradient**2
            W_new[hidden_layers-1-aux]=W[hidden_layers-1-aux]+eta*(momentum[hidden_layers-1-aux]/(np.sqrt(rms[hidden_layers-1-aux])+10**-8))
            aux = aux+1
        W=W_new
        sumt= sumt+sum(error[0])
   emedio[epoca] = sumt/len(x);
   for j1 in range(x1.shape[0]):      
         #Listas de valores de la red:
         I = [0]*(hidden_layers+1)#Potencial de activacion
         Y = [0]*(hidden_layers+2) #Salidas de las capas
         error = [1]
         #Propagación hacia adelante: 
         Y[0]=x1[j1,:].T
         I[0]=np.dot(W[0],Y[0])
         Y[1]=np.append([b], f_a(I[0]))
         for i in range(1,hidden_layers):
             I[i]=np.dot(W[i],Y[i].reshape(-1,1))
             Y[i+1]=np.append([b],f_a(I[i]))
         I[hidden_layers]=np.dot(W[hidden_layers],Y[hidden_layers].reshape(-1,1))
         Y[hidden_layers+1]=(f_a_exit(I[hidden_layers])) 
         error[0] = -(y1[j1,:] * np.log(Y[hidden_layers+1].T+ epsilon) + (1 - y1[j1,:]) * np.log(1 - Y[hidden_layers+1].T+ epsilon))
         sumt_test =sumt_test+sum(error[0])
   emedio_test[epoca] = sumt_test/len(x1)
   if cte==5:
       print(epoca+1)
       print("MSE_training:", emedio[epoca], "MSE_test:", emedio_test[epoca])
       cte=0
   cte = cte+1
plt.figure()
plt.plot(emedio, label='error_training',color='red')
plt.plot(emedio_test, label='error_test', color='blue')
plt.legend()
plt.xlabel("epoca")
plt.ylabel("BCE")
plt.show()

pred = []
for j1 in range(x1.shape[0]):      
      #Listas de valores de la red:
      I = [0]*(hidden_layers+1)#Potencial de activacion
      Y = [0]*(hidden_layers+2) #Salidas de las capas
      error = [1]
      #Propagación hacia adelante: 
      Y[0]=x1[j1,:].T
      I[0]=np.dot(W[0],Y[0])
      Y[1]=np.append([b], f_a(I[0]))
      for i in range(1,hidden_layers):
          I[i]=np.dot(W[i],Y[i].reshape(-1,1))
          Y[i+1]=np.append([b],f_a(I[i]))
      I[hidden_layers]=np.dot(W[hidden_layers],Y[hidden_layers].reshape(-1,1))
      Y[hidden_layers+1]=(f_a_exit(I[hidden_layers])) 
      pred.append(Y[hidden_layers+1])
 





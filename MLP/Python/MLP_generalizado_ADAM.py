# -*- coding: utf-8 -*-
"""
Created on Tue Jun  6 11:29:17 2023

@author: sebas
"""

import matplotlib.pyplot as plt
import numpy as np
X = np.array([[0,1],[0,0],[1,0],[1,1]])
Y = np.array([1,0,1,0])
b=-1 #bias value
#x = np.array([b*np.ones((np.size(X,0),1)),X])
x= np.concatenate((b*np.ones((np.size(X,0),1)), X), axis=1)
y = np.atleast_2d(Y).transpose()
data_set = np.concatenate((x, y), axis=1)

#Step 1: Set the Network Parameters
def f_a(x):
    return np.tanh(x)
def dev_a(x):
    return 1 - np.tanh(x)**2

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
for i in range(1,hidden_layers):
    neuronas[i] = 10#int(input("Neuronas en la capa capa oculta "+str(i+1)+": "))
    W[i]=(np.random.uniform(-0.5, 0.5, (int(neuronas[i]), int(neuronas[i-1]+1)))) #Pesos entre capas ocultas
    momentum[i] = np.zeros((int(neuronas[i]), int(neuronas[i-1]+1)))
    rms[i] = np.zeros((int(neuronas[i]), int(neuronas[i-1]+1)))
    
W[-1]=(np.random.uniform(-0.5, 0.5, (y.shape[1], int(neuronas[hidden_layers-1]+1)))) #pesos de la capa de la ultima capa oculta - capa de salida
momentum[-1] = np.zeros((y.shape[1], int(neuronas[hidden_layers-1]+1)))
rms[-1] = np.zeros((y.shape[1], int(neuronas[hidden_layers-1]+1)))

nEpocas = 2000# int(input("Número de épocas: "))
eta_i = 0.0125#float(input("Tasa de aprendizaje inicial: "))
eta_f = 0.00001#float(input("Tasa de aprendizaje final: "))
alfa1 = 0.9 #momentum constant
alfa2 = 0.999 #rms constant
emedio = [0]*nEpocas
cte = 1
for epoca in range(nEpocas):
   sumt=0
   #eta = eta_i*((eta_f/eta_i)**(epocas/nepocas)); #descenso expo
   eta = eta_i +(eta_f-eta_i)*(epoca/nEpocas)
   np.random.shuffle(data_set)
   x = data_set[:,0:3]
   y = data_set[:,3:]
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
        Y[hidden_layers+1]=(f_a(I[hidden_layers])) 
        #back propagation
        error[0] = (y[j,:] - Y[hidden_layers+1].T).T
        delta[0] = (error[0]*dev_a(I[hidden_layers])).T
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
        sumt= sumt+0.5*sum(error[0]**2)
   emedio[epoca] = sumt/len(x);
   if cte==25:
       print(epoca+1)
       cte=0
   cte = cte+1
plt.figure()
plt.plot(emedio)
plt.xlabel("epoca")
plt.ylabel("MSE")

X = np.array([[0,1],[0,0],[1,0],[1,1]])
Y = np.array([1,0,1,0])
b=-1 #bias value
#x = np.array([b*np.ones((np.size(X,0),1)),X])
x= np.concatenate((b*np.ones((np.size(X,0),1)), X), axis=1)
y = np.atleast_2d(Y).transpose()
data_set = np.concatenate((x, y), axis=1)
for j in range(x.shape[0]):      
      #Listas de valores de la red:
      I = [0]*(hidden_layers+1)#Potencial de activacion
      Y = [0]*(hidden_layers+2) #Salidas de las capas

      error = [1]
      #Propagación hacia adelante: 
      Y[0]=x[j,:].T
      I[0]=np.dot(W[0],Y[0])
      Y[1]=np.append([b], f_a(I[0]))
      for i in range(1,hidden_layers):
          I[i]=np.dot(W[i],Y[i].reshape(-1,1))
          Y[i+1]=np.append([b],f_a(I[i]))
      I[hidden_layers]=np.dot(W[hidden_layers],Y[hidden_layers].reshape(-1,1))
      Y[hidden_layers+1]=(f_a(I[hidden_layers])) 
      print(Y[hidden_layers+1])
#Instalación de Librerías
import numpy as np
import control
from scipy import signal
import matplotlib.pyplot as plt
import math
##############################
#Parámetros del Robot
m = 1 #masa del robot
l = 1 #Longitud del Robot
lc = 0.5 #Distancia al centro de masa
B = 0.01 #Coeficiente de fricción viscosa
I=(1/3)*m*pow(l, 2) #Momento de inercia
g=9.81 #Aceleración de la gravedad
##############################
K = math.pi/4
##############################
#Parámetros del Control PID
Kp = 8
Ti = 0.3
Td = 0.5
##############################
#Intervalo de tiempo de Simulación
start = 0
stop = 5
step = 0.01
tiempo = np.arange(start, stop, step)
##############################
#Parámetros de la Función de Transferencia
alfa1 = B*pow(lc, 2)/(m*pow(lc, 2)+I)
alfa0 = (m*g*lc)/(m*pow(lc, 2)+I)
gama = 1/(m*pow(lc, 2)+I)
b2 = gama*Kp*Ti*Td
b1 = gama*Kp*Ti
b0 = gama*Kp
a3 = Ti
a2 = Ti*alfa1+gama*Kp*Ti*Td
a1 = Ti*alfa0+gama*Kp*Ti
a0 = gama*Kp
##############################
Numerador = np.array([K*b2, K*b1, K*b0])
Denominador = np.array([a3, a2, a1, a0])
H = control.tf(Numerador, Denominador)
print('T(s)=',H)
##########################################
#Graficas
t, y = control.step_response(H, tiempo)
plt.plot(t, y)
plt.title('Control PID  del Robot')
plt.xlabel('t')
plt.ylabel('Angulo del Robot')
plt.grid()
plt.show()
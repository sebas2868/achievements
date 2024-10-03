#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile
import numpy as np
import matplotlib.pyplot as plt



class InversaPublisher(Node):
    def __init__(self):
        super().__init__('inversa_publisher')
        self.inversa_publisher_ = self.create_publisher(Float32MultiArray, 'salida_arduino', QoSProfile(depth=10))

        self.numero = None
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0

        self.a1 = 0.228
        self.a2 = 0.1365
        self.d1 = 0.045
        self.d2 = 0.0575
        self.d3 = 0.0811


        self.timer_period = 1/90
        self.timer = self.create_timer(self.timer_period, self.publicar_inversa)
        self.timer2 = self.create_timer(1/30, self.solicitar_numero)
        self.data = [0.0] * 5
        self.solicitar_numero()
        self.alpha = 1.3

    def dh_transform(self, theta, d, a, alpha):

        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, theta1, d1, theta2, theta3):
        # Parámetros D-H d
        d = [0.045, d1, -0.0575, 0.0811]
        a = [0, 0.228, 0.1365, 0]
        alpha = [0, 0, np.pi, 0]
        theta = [theta1, 0, theta2, theta3]
        T1 = self.dh_transform(theta[0], d[0], a[0], alpha[0])
        T2 = self.dh_transform(theta[1], d[1], a[1], alpha[1])
        T3 = self.dh_transform(theta[2], d[2], a[2], alpha[2])
        T4 = self.dh_transform(theta[3], d[3], a[3], alpha[3])
        T_final = T1 @ T2 @ T3 @ T4
        efector_final = T_final[0:3,3]
        return efector_final

    def fkine(self, q):
        x = -0.1365*np.sin(q[0])*np.sin(q[2]) + 0.1365*np.cos(q[0])*np.cos(q[2]) + 0.228*np.cos(q[0])
        y = 0.1365*np.sin(q[0])*np.cos(q[2]) + 0.228*np.sin(q[0]) + 0.1365*np.sin(q[2])*np.cos(q[0]) 
        z = q[1] - 0.0936
        return np.array([x,y,z])

    def newton(self, J,e):
        return np.dot(np.linalg.inv(J), e)
    def gradient(self, J,e):
        return self.alpha*np.dot(J.transpose(), e)

    def hallar_angulos_NUM(self, X, Y,Z,angulo_final,m):
        if m == 1:
            metodo = self.newton
        else:
            metodo = self.gradient

        delta = 0.001
        error= []
        xd = np.array([X, Y, Z])
        q = np.array([self.q1, self.q2, self.q3])
        epsilon = 1e-3
        max_iter = 2000
        i = 0
        while True:
            q1 = q[0]
            q2 = q[1]
            q3 = q[2]
            JT = 1/delta*np.array([
                self.fkine([q1+delta, q2, q3]) - self.fkine([q1,q2,q3]),
                self.fkine([q1, q2+delta, q3]) - self.fkine([q1,q2,q3]),
                self.fkine([q1, q2, q3+delta]) - self.fkine([q1,q2,q3])
                ])
            J = JT.transpose()
            Nx = np.cos(angulo_final)
            Ny = -np.sin(angulo_final)
            q4 = np.arctan2(Nx,Ny) - q1 - q3
            f=self.forward_kinematics(q1,q2,q3,q4)
            e = xd - f
            error.append(abs(e))
            # Actualizar las posiciones articulares
            q = q + metodo(J,e)
            # Condición de término
            if np.linalg.norm(e) < epsilon:
                break
            i+=1
            if i == max_iter:
                print("Límite de iteraciones alcanzado")
                break
            
        vueltas1 = np.floor(q[0]/(2*np.pi))
        q1 = q[0] - 2*vueltas1*np.pi
        vueltas2 = np.floor(q[2]/(2*np.pi))
        q3 = q[2] - 2*vueltas2*np.pi
        vueltas4 = np.floor(q4/(2*np.pi))
        q4 = q4 - 2*vueltas4*np.pi
        
        self.q1 = q1
        self.q2 = q[1]
        self.q3 = q3
        self.q4 = q4

        # Graficar el error acumulado 
        plt.plot(range(len(error)), error)
        plt.xlabel('Iteraciones')
        plt.ylabel('Norma del Error')
        plt.title('Convergencia del Error en el Método de Gradiente')
        plt.grid(True)

    def solicitar_numero(self):

        try:
            self.get_logger().info(f"Seleccione la Cinematica que va a usar.")
            self.get_logger().info(f"[1] Geometrico")
            self.get_logger().info(f"[2] MTH")
            self.get_logger().info(f"[3] Algebraico")
            self.get_logger().info(f"[4] Numerica: Newton")
            self.get_logger().info(f"[5] Numerica: Gradiente")

            numero = float(input("Ingrese un número: "))
            self.get_logger().info(f"Número ingresado: {numero}")

            if numero == 1:
                self.get_logger().info(f"Ha elegido la cinematica por el metodo geometrico")
                cinematica = "geometrica"
            elif numero == 2:
                self.get_logger().info(f"Ha elegido la cinematica por MTH")
                cinematica = "MTH"
            elif numero == 3:
                self.get_logger().info(f"Ha elegido la cinematica algebraica")
                cinematica = "Algebraico"
            elif numero == 4:
                self.get_logger().info(f"Ha elegido la cinematica numerica por Newton")
                cinematica = "Newton"
            elif numero ==5:
                self.get_logger().info(f"Ha elegido la cinematica numerica por Gradiente")
                cinematica = "Gradiente"
            else:
                raise ValueError

            Px = float(input("Ingrese su posicion en x: "))
            Py = float(input("Ingrese su posicion en y: "))
            Pz = float(input("Ingrese su posicion en z: "))   
            angulo_final = np.deg2rad(float(input("Ingrese su angulo final: ")))

            Nx = np.cos(angulo_final)
            Ny = -np.sin(angulo_final)
            Nz = 0
            Ox = -Ny
            Oy = Nx
            Oz = 0
            Ax = 0
            Ay = 0
            Az = 1
            

            if cinematica == "geometrica":
                try:
                    self.q2 = Pz - self.d1 + self.d2 + self.d3
                    D = Px**2 + Py**2
                    C2 = (D - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
                    C2 = np.clip(C2, -1, 1)
                    self.q3 = np.arctan2(np.sqrt(1 - C2**2), C2)
                    self.q1 = np.arctan2(Py, Px) - np.arctan2(self.a2 * np.sin(self.q3), self.a1 + self.a2 * np.cos(self.q3))
                    self.q4 = np.arctan2(Nx,Ny) - self.q1 - self.q3
                    # Conversion
                    self.q1 = np.rad2deg(self.q1)
                    self.q2 *= 100
                    self.q3 = np.rad2deg(self.q3)
                    self.q4 = np.rad2deg(self.q4)
                    self.get_logger().info(f"q1: {self.q1}, q2: {self.q2}, q3: {self.q3}, q4: {self.q4}")
                    self.publicar_inversa()

                except:
                    self.get_logger().info(f"Las coordenadas especificadas estan por fuera del rango del SCARA.")
            
            elif cinematica == "MTH":
                if Px == 0: Px = 0.00001
                self.q2 = Pz - self.d1 + self.d2 + self.d3
                D = Px**2 + Py**2
                C2 = (D - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
                C2 = np.clip(C2, -1, 1)
                self.q3 = np.arctan2(np.sqrt(1 - C2**2), C2)
                C1 = self.a2*np.sin(self.q3)*(Py/Px)+self.a2*np.cos(self.q3)+self.a1
                C1 = C1 / (Px + (Py**2/Px))
                C1 = np.clip(C1, -1, 1)
                self.q1 = np.arctan2(np.sqrt(1 - C1**2), C1)
                self.q4 = np.arctan2(Nx,Ny) - self.q1 - self.q3
                # Conversion
                self.q2 *= 100
                self.q3 = np.rad2deg(self.q3)
                self.q4 = np.rad2deg(self.q4)
                self.get_logger().info(f"q1: {self.q1}, q2: {self.q2}, q3: {self.q3}, q4: {self.q4}")
                self.publicar_inversa()
 
            elif cinematica == "Algebraico":

                self.q1 = np.arctan2(Py,Px) + np.arccos(np.clip((53.67*(Px**2 + Py**2)+1.79)/(24.46*np.sqrt(Px**2 + Py**2)),a_min=-1,a_max=1))
                self.q3 = np.arctan2(Py*np.cos(self.q1)-Px*np.sin(self.q1),Px*np.cos(self.q1)+Py*np.sin(self.q1)-0.228)
                self.q2 = Pz - self.d1 + self.d2 + self.d3
                self.q4 = np.arctan2(Nx,Ny) - self.q1 - self.q3
                # Conversion
                self.q1 = np.rad2deg(self.q1)
                self.q2 *= 100
                self.q3 = np.rad2deg(self.q3)
                self.q4 = np.rad2deg(self.q4)
                self.get_logger().info(f"q1: {self.q1}, q2: {self.q2}, q3: {self.q3}, q4: {self.q4}")
                self.publicar_inversa()
            
            elif cinematica == "Newton":
                self.hallar_angulos_NUM(Px,Py,Pz,angulo_final,1)
                # Conversion
                self.q1 = np.rad2deg(self.q1)
                self.q2 *= 100
                self.q3 = np.rad2deg(self.q3)
                self.q4 = np.rad2deg(self.q4)
                self.get_logger().info(f"q1: {self.q1}, q2: {self.q2}, q3: {self.q3}, q4: {self.q4}")
                self.publicar_inversa()
                plt.show()

            elif cinematica == "Gradiente":
                self.hallar_angulos_NUM(Px,Py,Pz,angulo_final,0)
                # Conversion
                self.q1 = np.rad2deg(self.q1)
                self.q2 *= 100
                self.q3 = np.rad2deg(self.q3)
                self.q4 = np.rad2deg(self.q4)
                self.get_logger().info(f"q1: {self.q1}, q2: {self.q2}, q3: {self.q3}, q4: {self.q4}")
                self.publicar_inversa()
                plt.show()

        except ValueError as e:
            print(e)
            self.get_logger().error("Por favor, ingrese un número válido.")
            self.solicitar_numero()  # Vuelve a solicitar en caso de error

    def publicar_inversa(self):
        msg = Float32MultiArray()
        self.data = [self.q1,self.q2,self.q3,self.q4,0]
        msg = Float32MultiArray(data=self.data)
        self.inversa_publisher_.publish(msg)
        #self.get_logger().info(f"Número publicado: {self.numero}")

def main(args=None):
    rclpy.init(args=args)
    nodo = InversaPublisher()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import numpy as np

#inisial titik awal
x0, y0 = 0, 0

#sudut rotasi pertama
theta_deg_1 = 40
theta_deg_2 = 30
theta_deg_3 = 20

#ubah ke radian
theta1 = np.radians(theta_deg_1)
theta2 = np.radians(theta_deg_2)
theta3 = np.radians(theta_deg_3)
#matriks rotasi

#using 2 last digits in NIU
R1 = np.array([[np.cos(theta1), -np.sin(theta1), 0],
             [np.sin(theta1), np.cos(theta1), 0],
             [0, 0, 1]])

T1 = np.array([[1, 0, 25],
              [0, 1, 0],
              [0, 0, 1]])

#using 2 last digits in NIF
R2 = np.array([[np.cos(theta2), -np.sin(theta2), 0],
             [np.sin(theta2), np.cos(theta2), 0],
             [0, 0, 1]])

T2 = np.array([[1, 0, 4],
               [0, 1, 0],
               [0, 0, 1]])

#using num of year that i have acc 
R3 = np.array([[np.cos(theta3), -np.sin(theta3), 0],
             [np.sin(theta3), np.cos(theta3), 0],
             [0, 0, 1]])

T3 = np.array([[1, 0, 24],
               [0, 1, 0],
               [0, 0, 1]])

#vektor posisi awal
P = np.array([x0, y0, 1])

#Hasil Transformasi
Product = T3 @ R3 @ T2 @ R2 @ T1 @ R1
P_transformated = Product @ P

print("Posisi setelah transformasi:", P_transformated)

#inverse kinematics
#fungsi linalg inv itu harus berbentuk matriks, sedangkan P_transformated itu vektor
M_inv = np.linalg.inv(Product)
P_initial = M_inv @ P_transformated

print("Inverse Kinematics Result:", P_initial)

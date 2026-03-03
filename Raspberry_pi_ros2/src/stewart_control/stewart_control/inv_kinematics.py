#!/home/rem/rtimulib-env/bin/python3


import numpy as np 

class StewartPlatform:
    def __init__(self, radious_base, radious_platform, gamma_base, gamma_platform) -> None:
        self.rb = radious_base  # rayon de la base
        self.rp = radious_platform  # rayon de la plateforme
        self.gamma_B = np.deg2rad(gamma_base)  # demi-angle base
        self.gamma_P = np.deg2rad(gamma_platform)  # demi-angle plateforme
        self.home_pos = np.array([0, 0, 0.185])  # position home de la plateforme
        self.B = None
        self.P = None
        self.L = None

    def frame(self):
        pi = np.pi
        phi_B = np.array([7*pi/6 + self.gamma_B, 7*pi/6 - self.gamma_B,
                          pi/2 + self.gamma_B, pi/2 - self.gamma_B,
                          11*pi/6 + self.gamma_B, 11*pi/6 - self.gamma_B])
        phi_P = np.array([3*pi/2 - self.gamma_P, 5*pi/6 + self.gamma_P,
                          5*pi/6 - self.gamma_P, pi/6 + self.gamma_P,
                          pi/6 - self.gamma_P, 3*pi/2 + self.gamma_P])

        B = self.rb * np.array([[np.cos(phi_B[i]), np.sin(phi_B[i]), 0] for i in range(6)]).T
        P = self.rp * np.array([[np.cos(phi_P[i]), np.sin(phi_P[i]), 0] for i in range(6)]).T
        return P, B

    def rotX(self, theta):
        theta = np.deg2rad(theta)
        return np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
    
    def rotY(self, theta):
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    
    def rotZ(self, theta):
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])

    def solve(self, trans, rotation):
        self.B, self.P = self.frame()
        R = self.rotX(rotation[0]) @ self.rotY(rotation[1]) @ self.rotZ(rotation[2])
        l = trans[:, None] + self.home_pos[:, None] + R @ self.P - self.B
        lll = np.linalg.norm(l, axis=0)
        return lll  # retourne le vecteur de 6 longueurs

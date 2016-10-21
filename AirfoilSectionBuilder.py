import numpy as np
import matplotlib.pyplot as plt
from SectionBuilderSubroutine import *

# this program is used to generate the script used in Abaqus;
# The aim is to generate the airfoil skin, C-beam and foam and mesh it.
# __author__ = 'Mr.Z'

# airfoil section control parameter
chord = 1000       # magnification factor for point geometry
thickness = .0003  # as fraction of chord
k_cut = 0.01      # airfoil tail cut ratio
# C beam parameter
X_A = 0.25
X_F = 0.25
X_D = 0.10
Y_D = 0.01
k_area = 0.20

# read the airfoil data
# input_file = 'OA209_200.dat'
input_file = 'opt_airfoil_3.dat'
#input_file = 'NACA8H12_200.dat'
#input_file = 'naca0012-102.dat'


mold_line_temp = np.loadtxt(input_file)
skin_outer = airfoilPreprocess(mold_line_temp, k_cut)

# scale to the target chord
skin_outer = skin_outer * chord
X_A = X_A * chord
X_F = X_F * chord
X_D = X_D * chord
Y_D = Y_D * chord

skin_outer1, skin_outer2, skin_outer3 = divideSkinbyLines(skin_outer, ((X_F, chord), (X_F, -chord)), ((X_A, chord), (X_A, -chord)))

distance = chord * thickness  # skin thickness
skin, skin_inner = generateSkin(skin_outer, thickness, chord)
skin_inner1, skin_inner2, skin_inner3 = divideSkinbyLines(skin_inner, ((X_F, chord), (X_F, -chord)), ((X_A, chord), (X_A, -chord)))

C_shape, C_inner = generateCtypeBeam(skin_inner, skin_inner2, X_D, Y_D, k_area, distance)

foam_right = np.array([skin_inner[-1], skin_inner[0]])   # right constraint line
foam, foam_up, foam_down = generateFoam(skin_inner,  C_inner, foam_right)

plt.plot(skin_inner1[:,0], skin_inner1[:,1], 'r', skin_inner2[:,0], skin_inner2[:,1], 'o',
         skin_inner3[:,0], skin_inner3[:,1], 'b')
plt.show()


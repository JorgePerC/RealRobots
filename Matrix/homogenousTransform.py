import sympy 
from sympy import Symbol, cos, sin 

c1, s1, d1 = sympy.symbols("c1 s1 d1")

transMatrix = sympy.Matrix([[ c1, -s1, 0, 0],
                            [ s1, c1, 0, 0],
                            [ 0, 0, 1, d1],
                            [ 0, 0, 0, 1]])
def rotZVector (dimention, degrees, vector):
    angle = math.radians(degrees)
    zMatrix = 0
    if (dimmention in ["x", "X"]):
        zMatrix = np.array(
            [1,0,0],
            [0,math.cos(angle), -math.sin(angle)],
            [0,math.sin(angle),math.cos(angle)])
    elif (dimmention in ["y", "Y"]):
        zMatrix = np.array(
            [math.cos(angle), 0, math.sin(angle)],
            [0, 1, 0],
            [-math.sin(angle), 0, math.cos(angle)])
    elif (dimmention in ["z", "Z"]):
        zMatrix = np.array(
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle),0],
            [0, 0, 1])
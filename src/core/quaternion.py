# coding: latin-1
#
#  quaternion.py
#
#
#  Created by Felix Rehrmann on 24.07.09.
#  Copyright (c) 2009 __MyCompanyName__. All rights reserved.
#


import numpy as np
import math

class quaternion(object):
    """ A quaternion class.
        q = [w, x·i, y·j, z·k]
    """

    def fromAngleAxis(phi,U):
        """ Constructs a quaternion by an angle and an axis.
            @param phi a scalar giving the angle in radians
            @param U vector with 3 components giving the rotation axis.
            Note: U will be normalized.
        """
        q = np.zeros(4)
        q[0] = np.cos(phi/2.0)
        q[1:4] = U/np.sqrt(np.dot(U,U)) * np.sin(phi/2.0)
        return quaternion(q)

    def shortRot(v1,v2):
        """ Finds the shortest rotation between two vectors.
            Better said the rotation from v1 to v2.
            From Game Programming Gems 1 after recipe of Stan Melax.
        """
        v1 /= np.sqrt(np.dot(v1,v1))
        v2 /= np.sqrt(np.dot(v2,v2))
        if np.sum(v1-v2) < 1.e-6: return quaternion()
        c = np.cross(v1,v2)
        f = dot(v1,v2)
        s = np.sqrt((1.0+d)*2.0)
        q = quaternion(c/s)
        q[0] = s / 2.0;
        return q


    fromAngleAxis = staticmethod(fromAngleAxis)
    shortRot = staticmethod(shortRot)


    def __init__(self,q=None):
        """ Constructs a Quaternion.
            Representation is [w, x i, y j, z k].
            @param q quaternion specification (Default:None).
            None: quaternion is [1 0 0 0]
            List or Array with four elements: taken in the row as shown above.
            List of Array with three elements: taken as x y z with w=0.
            Quaternione: makes a copy.
        """
        if q is None:
            self.__q = np.zeros(4)
            self.__q[0] = 1.0;
        elif type(q) is quaternion:
            self.__q = np.zeros(4)
            self.__q[0:4] = q.asArray()[0:4]
        elif len(q) == 4:
            self.__q = np.array(q)
        elif len(q) == 3:
            self.__q = np.hstack((np.zeros(1),np.array(q)))
        else:
            raise ValueError("Cannot construct quaternion from input.")

    def conj(self):
        return quaternion(self.__q * np.array([1., -1., -1., -1.]))

    def rot(self,other):
        """ Rotation of the 3d vector other.
            Assumed that self is a rotation of an csys c1 from a reference
            csys c0. Assumed farther that other is vector given in the csys c1
            than self * other * self.conj() gives the componentes of other in
            the csys c0. Or to say it in another way the vector other is
            rotated with the quaternions self. 
        """
        if type(other) is np.ndarray and len(other) == 3:
            q = self.__mul__(other)
            q = q * self.conj()
            return q.asArray()[1:4]
        elif type(other) is quaternion:
            return self.__q * other * self.conj()
        else:
            return NotImplemented

    def phiU(self):
        """ Retruns the angle and rotation axis.
            @return phi, U
            q = [cos(phi/2), U * sin(phi/2)]
        """
        phi = 2.0 * np.arccos(self.__q[0])
        sphi = np.sin(phi/2.)
        U = self.__q[1:4] / sphi
        return phi, U

    def asArray(self):
        """ Returns the qunaternion as numpy array.
        """
        return self.__q

    def toMatrix(self):
        """ The rotation matrix belonging to the quaternion.
            Copied from a Game Programming Gems 1 article by J. Shankel.
        """
        w,x,y,z = self.__q
        M = np.zeros((3,3))
        M[0,0] = 1.0 - 2.0 * ( y**2 + z**2 )
        M[1,0] = 2.0 * ( x*y + w*z )
        M[2,0] = 2.0 * ( x*z - w*y )

        M[0,1] = 2.0 * ( x*y - w*z )
        M[1,1] = 1.0 - 2.0 * ( x**2 + z**2 )
        M[2,1] = 2.0 * ( y*z + w*x )

        M[0,2] = 2.0 * ( w*y + x*z )
        M[1,2] = 2.0 * ( y*z - w*x )
        M[2,2] = 1.0 - 2.0 * ( x**2 + y**2 )
        return M

    def toEuler2(self):
        """ The Euler angles belonging to the quaternion.
        """
        w,x,y,z = self.__q
        sqw = w*w
        sqx = x*x
        sqy = y*y
        sqz = z*z
        unit = sqx + sqy + sqz + sqw # if normalised is one, otherwise is correction factor 
        test = x*y + z*w
        #Heading is yaw along Z , Attitude is pitch along Y and Bank is roll along X
        if test > 0.499*unit:  # singularity at north pole
	        heading  = 2 * math.atan2(x,w)
	        attitude = math.pi/2
	        bank     = 0

        elif test < -0.499*unit: # singularity at south pole
	        heading  = -2 * math.atan2(x,w)
	        attitude = -math.pi/2
	        bank     = 0

        else:
	        heading = math.atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw);
	        attitude = math.asin(2*test/unit);
	        bank = math.atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw)

        return np.array([bank, attitude, heading])

    def toEuler(self):
        """ The Eulet angles using the rotation matrix
            result = [heading, attitude, bank]
            ranges are [-pi, pi], [-pi/2, pi/2] and [-pi, pi]
        """
        M = np.zeros((3,3))
        M = self.toMatrix()
        x = np.linalg.norm([M[2,2] ,M[2,1]])
        result = np.array([0, math.atan2(-M[2,0],x), 0])
        if x > 0.0001:
            result[0] = math.atan2(M[1,0], M[0,0])
            result[2] = math.atan2(M[2,1], M[2,2])
        else:
            result[0] = 0.00
            if M[2,0] > 0.00:
                aux = 1.00
            else:
                aux=-1.00

            result[2] = aux * math.atan2(-M[0,1], M[1,1])

        return result

    def __add__(self,other):
        return quaternion(self.__q + other.__q)

    def __sub__(self,other):
        return quaternion(self.__q - other.__q)

    def __mul__(self,other):
        """ Quaternion rotation.
            quaternion * scalar
            quaternion * numpy.array with length 4
            quaternion * numpy.array with length 3
            quaternion * quaternion 
        """
        if type(other) is int or type(other) is float:
            return quaternion(self.__q * other)
        if type(other) is quaternion:
            q2 = other.asArray()
        else :
            try:
                v = np.array(other,dtype=float)
            except ValueError:
                return NotImplemented
            if len(v) == 3:
                q2 = np.hstack((np.zeros(1),v.flatten()))
            elif len(v) ==4:
                q2 = v.flatten()
            else:
                return NotImplemented
        # Computation
        a0,a1,a2,a3 = self.__q
        b0,b1,b2,b3 = q2
        q = np.zeros(4)
        q[0] = a0*b0 - a1*b1 - a2*b2 - a3*b3
        q[1] = a0*b1 + a1*b0 + a2*b3 - a3*b2
        q[2] = a0*b2 - a1*b3 + a2*b0 + a3*b1
        q[3] = a0*b3 + a1*b2 - a2*b1 + a3*b0
        return quaternion(q)

    def __rmul__(self,other):
        return self.__mul__(other)

    def __str__(self):
        return "%f + %f i + %f j + %f k"%tuple(self.__q)

    def __repr__(self):
        return "quaternion : "+self.__str__()

    def __len__(self):
        return 4

    def __abs__(self):
        return np.sqrt(np.dot(self.__q,self.__q))

    def __invert__(self):
        return quaternion(self.conj() * 1./np.dot(self.__q,self.__q))

    def __pos__(self):
        return quaternion(self.__q)

    def __neg__(self):
        return quaternion(self.__q * -1.0)

    def __setitem__(self,name,value):
        if name < 0 and name > 3:
            raise IndexError("Index out of bound.")
        try:
            self.__q[name] = float(value)
        except:
            raise ValueError("value is not valid.")

    def __getitem__(self,name):
        if name < 0 and name > 3:
            raise IndexError("Index out of bound.")
        return self.__q[name]



if __name__ == "__main__":
    a = quaternion()
    b = quaternion(np.array([1.0, 0.5, 1.3]))
    c = quaternion(np.array([0.2,1.0,0.3,-0.1]))
    d = quaternion(c)

    print ("a:",a)
    print ("b:",b)
    print ("c:",c)
    print ("d:",d)
    print ("c':",c.conj())
    print ("c*b:", c*b)
    print ("c*b*c':",c * b * c.conj())
    print ("c*v*c':",c*b.asArray()[1:4]*c.conj())
    print ("c.rot(v):",c.rot(b.asArray()[1:4]))

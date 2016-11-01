##########################################################################
# Quaternion helper class
# Update of the existing Quaternion helper in iMoby
# Author: Javier Hidalgo
##########################################################################

class Quaternion
    attr_accessor :w, :x, :y, :z

    def initialize( w, x, y, z )
	@w, @x, @y, @z = w, x, y, z
    end
    
    def q_to_dcm
	# here @w is meant to be the real part
	Matrix[[2*@w**2 + 2*@x**2 - 1, 2*@x*@y - 2*@w*@z, 2*@x*@z + 2*@w*@y],
	    [ 2*@x*@y + 2*@w*@z, 2*@w**2 + 2*@y**2 - 1, 2*@y*@z - 2*@w*@x],
	    [ 2*@x*@z - 2*@w*@y, 2*@y*@z + 2*@w*@x, 2*@w**2 + 2*@z**2 - 1]]
    end
    

    #gets the yaw angle in rad -pi/2 .. pi/2 
    def to_yaw
	Math.atan2(( 2*@x*@y+2*@w*@z),(@x**2 - @y**2 - @z**2 + @w**2))
    end
    #gets the roll angle in rad -pi/2 .. pi/2 
    def to_roll
	Math.atan2(( 2*@y*@z+2*@w*@x),(-(@x**2) - @y**2 + @z**2 + @w**2))
    end
    #gets the pitch angle in rad -pi .. pi
    def to_pitch
	Math.asin((-2*@x*@z+2*@w*@y)/(@x**2+@y**2+@z**2+@w**2))
    end
end


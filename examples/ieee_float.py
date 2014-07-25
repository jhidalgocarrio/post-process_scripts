import struct

getBin=lambda x: x > 0 and str(bin(x))[2:] or "-" + str(bin(x))[3:]  

def floatToBinary64(value):
val=struct.unpack('Q', struct.pack('d', value))[0]  
return getBin(val)  

def binaryToFloat(value):
hx=hex(int(value, 2))
return struct.unpack("d", struct.pack("q", int(hx, 16)))[0]  

# floats are represented by IEEE 754 floating-point format which are   
# 64 bits long (not 32 bits)  

# float to binary  
binstr = floatToBinary64(19.5)  
print('Binary equivalent of 19.5:')  
print(binstr + '\n')  

# binary to float  
fl=binaryToFloat(binstr)  
print('Decimal equivalent of ' + binstr)  
print(fl)


import cv2
from skimage import morphology
from PIL import Image as im
import matplotlib.pyplot as plt
from scipy import interpolate
import pandas as pd
import math
import numpy as np
from sklearn.linear_model import LinearRegression

def main_straight(df):
    circuit = pd.concat([df,df],axis = 0)
    circuit = np.array(circuit)
    print(circuit)
    size = len(circuit)
    for i in range(size):
        sub = []
        print(circuit[i])
        sub.append(circuit[i])
        score = 1
        for j in range(i+1,size):
            sub.append(circuit[j])
            print(sub)
            if (len(sub)>=3 and score>0.9):
                sub_plot = pd.DataFrame(sub, columns = ['x','y'])
                plt.plot(sub_plot['x'],sub_plot['y'])
                plt.grid()
        plt.show()
    

def distance_between_points(point1, point2):
    x1 = point1[0]
    x2 = point2[0]
    y1 = point1[1]
    y2 = point2[1]
    dx = x1-x2
    dy = y1-y2
    dx2 = np.power(dx,2)
    dy2 = np.power(dy,2)
    dist = np.sqrt(dx2+dy2)
    return dist

def closest_point_df(point,array):
    dist_min = 10000
    for idx in range(len(array)):
        dist = distance_between_points(point,array[idx])
        if(dist<dist_min and dist!=0):
            dist_min = dist
            min_idx = idx
            min = array[idx]
    return min,min_idx

def order_points(df):
    array = np.array(df)
    actual = array[0]
    i = 0
    size = len(array)
    sorted = []
    while(i<size):
        sorted.append(actual)
        min,idx = closest_point_df(actual,array)
        array = np.delete(array,idx,0)
        actual = min
        print('iteration:\t',i)
        i+=1
    return sorted
        


circuit_length_m = 5281

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def centerline_polar(df):
    rho_vect = []
    phi_vect = []
    for index, row in df.iterrows():
        rho,phi = cart2pol(row['x'],row['y'])
        rho_vect.append(rho)
        phi_vect.append(phi)
    return rho_vect, phi_vect

def centerline_scalar(df):
    x_vect = []
    y_vect = []
    for index, row in df.iterrows():
        x,y = pol2cart(row['rho'],row['phi'])
        x_vect.append(x)
        y_vect.append(y)
    return x_vect, y_vect

def circuit_length(df):
    df = np.array(df)
    size = len(df)
    length = 0
    for i in range(size):
        if(i<len(df)-1):
            x = df[i][0]
            y = df[i][1]
            x_next = df[i+1][0]
            y_next = df[i+1][1]
            dist = np.sqrt((x-x_next)**2 + (y-y_next)**2)
            length += dist
    return length

# Function to convert pixels to millimeters
def pixels_to_millimeters(pixels, dpi):
    inches = pixels / dpi
    millimeters = inches * 25.4
    return millimeters

def pixels_to_millimeters_list(vect,dpi):
    vect_mm = []
    for pixels in vect:
        mm = pixels_to_millimeters(pixels,dpi)
        vect_mm.append(mm)
    return vect_mm

# Load the image
image = cv2.imread('layout2.png', 0)  # Replace 'input_image.png' with your image file

# Threshold the image to create a binary image
ret, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)

# Perform skeletonization using scikit-image
skeleton = morphology.skeletonize(binary_image)

# Convert the skeletonized image to 8-bit format for visualization
skeleton = skeleton.astype(np.uint8) * 255

# Save the skeletonized image
cv2.imwrite('skeletonized_image.png', skeleton)  # Change the filename as needed
image_path = 'skeletonized_image.png'
image = im.open(image_path)


def centerline_coordinates(image_path):
    image = im.open(image_path)
    # Assicurati che l'immagine sia nel formato RGBA per gestire la trasparenza
    image = image.convert('RGBA')
    x_coord = []
    y_coord = []
    width, height = image.size
    for x in range(width):
        for y in range(height):
            pixel = image.getpixel((x, y))
            r, g, b, a = pixel
            if(pixel == (255,255,255,255)):
                x_coord.append(x)
                y_coord.append(y)
    return x_coord,y_coord



[x_coord,y_coord] = centerline_coordinates(image_path)
print(len(x_coord))
x = x_coord
y = y_coord
center = sum(x)/len(x), sum(y)/len(y)

x = x-np.ones(len(x))*center[0]
y = y-np.ones(len(y))*center[1]
center = sum(x)/len(x), sum(y)/len(y)
x = pd.DataFrame(x,columns = ['x'])
y = pd.DataFrame(y,columns = ['y'])
coord = pd.concat([x,y],axis = 1)
sorted = order_points(coord)
sorted = pd.DataFrame(sorted, columns = ['x','y'])

#main_straight(sorted)


# for index, row in sorted.head(500).iterrows():
#     plt.plot(row['x'],row['y'],'.')
# plt.show()
################################################
[rho,phi] = centerline_polar(sorted)
print(len(rho))
pi = 3.14
rho = pd.DataFrame(rho,columns = ['rho'])
phi = pd.DataFrame(phi,columns = ['phi'])
#phi = phi-pi/3.5
polars = pd.concat([rho,phi],axis = 1)


line_length = circuit_length(polars)
actual_ratio = circuit_length_m/line_length
gain = 0.0
i = 0
while(actual_ratio != 1):
    if(actual_ratio>1):
        gain+=0.001
    else:
        gain-=0.001
    rho_scaled = rho*gain
    polars_scaled = pd.concat([rho_scaled,phi],axis = 1)
    line_length_scaled = circuit_length(polars_scaled)
    actual_ratio = circuit_length_m/line_length_scaled
    print('iteration: ', i, '\tline length: ',line_length_scaled,'\tratio: ',actual_ratio)
    i +=1
    if(abs(actual_ratio-1)<0.001):
        actual_ratio = 1

[x_vect,y_vect] = centerline_scalar(polars_scaled)
x_vect = pd.DataFrame(x_vect,columns = ['x'])
y_vect = pd.DataFrame(y_vect,columns = ['y'])
coord_check = pd.concat([x_vect,y_vect],axis = 1)
plt.plot(x, y,'.')
plt.plot(x_vect, y_vect,'.')
plt.plot(center[0], center[1], marker='o')
plt.grid()
plt.show()
###### Visually checks if the conversion is correct ###########
# [x_vect,y_vect] = centerline_scalar(polars)
# x_vect = pd.DataFrame(x_vect,columns = ['x'])
# y_vect = pd.DataFrame(y_vect,columns = ['y'])
# coord_check = pd.concat([x_vect,y_vect],axis = 1)
# plt.plot(x_vect, y_vect,'.')
# plt.plot(center[0], center[1], marker='o')
# plt.grid()
# plt.show()



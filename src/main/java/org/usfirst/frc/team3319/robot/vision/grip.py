#!/usr/bin/env python3
#THE PRECEDING LINE IS EXTREMELY IMPORTANT: DO NOT REMOVE
import cv2
import numpy
import math
from enum import Enum
from networktables import NetworkTables, NetworkTablesInstance
import time

class TapeRecognitionPipeline:
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__resize_image_width = 320.0
        self.__resize_image_height = 240.0
        self.__resize_image_interpolation = cv2.INTER_LINEAR

        self.resize_image_output = None

        self.__blur_input = self.resize_image_output
        self.__blur_type = BlurType.Box_Blur
        self.__blur_radius = 5.660377358490567

        self.blur_output = None

        self.__hsv_threshold_input = self.blur_output
        self.__hsv_threshold_hue = [29.136690647482013, 132.38907849829351]
        self.__hsv_threshold_saturation = [29.811151079136703, 124.45392491467578]
        self.__hsv_threshold_value = [153.64208633093526, 244.12116040955632]

        self.hsv_threshold_output = None

        self.__mask_input = self.resize_image_output
        self.__mask_mask = self.hsv_threshold_output

        self.mask_output = None


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step Resize_Image0:
        self.__resize_image_input = source0
        (self.resize_image_output) = self.__resize_image(self.__resize_image_input, self.__resize_image_width, self.__resize_image_height, self.__resize_image_interpolation)
        

        # Step Blur0:
        self.__blur_input = self.resize_image_output
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type, self.__blur_radius)
       

        # Step HSV_Threshold0:
        self.__hsv_threshold_input = self.blur_output
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)
        

        # Step Mask0:
        self.__mask_input = self.resize_image_output
        self.__mask_mask = self.hsv_threshold_output
        (self.mask_output) = self.__mask(self.__mask_input, self.__mask_mask)
        

        image = cv2.cvtColor(self.mask_output, cv2.COLOR_BGR2GRAY)
        

        thresh = 127
        image = cv2.threshold(image, thresh, 255, cv2.THRESH_BINARY)[1]
        

        rows,cols = image.shape

        M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
        image = cv2.warpAffine(image,M,(cols,rows))        

        points = numpy.argwhere(image==255)
        

        if len(points)>0:
            [vx,vy,x,y] = cv2.fitLine(points, cv2.DIST_L2,0,0.01,0.01)
            
        else:
            [vx,vy,x,y] = [0,0,0,0]
        
        #publish results to network tables
        sd = tableInstance.getTable("LineData")
        print("Is table connected: "+str(tableInstance.isConnected())+"\n\n\n")
        sd.putNumber("vx",vx)
        sd.putNumber("vy",vy)
        sd.putNumber("x",x)
        sd.putNumber("y",y)
    
        NetworkTables.shutdown()


    @staticmethod
    def __resize_image(input, width, height, interpolation):
        """Scales and image to an exact size.
        Args:
            input: A numpy.ndarray.
            Width: The desired width in pixels.
            Height: The desired height in pixels.
            interpolation: Opencv enum for the type fo interpolation.
        Returns:
            A numpy.ndarray of the new size.
        """
        return cv2.resize(input, ((int)(width), (int)(height)), 0, 0, interpolation)


    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (numpy.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A numpy.ndarray that has been blurred.
        """
        if(type is BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif(type is BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif(type is BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))


    @staticmethod
    def __hsv_threshold(input, hue, sat, val):
        """Segment an image based on hue, saturation, and value ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max value.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

    @staticmethod
    def __mask(input, mask):
        """Filter out an area of an image using a binary mask.
        Args:
            input: A three channel numpy.ndarray.
            mask: A black and white numpy.ndarray.
        Returns:
            A three channel numpy.ndarray.
        """
        return cv2.bitwise_and(input, input, mask=mask)

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        contours =cv2.findContours(input, mode=mode, method=method)[0]
        return contours

    @staticmethod
    def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                        min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                        min_ratio, max_ratio):
        """Filters out contours that do not meet certain criteria.
        Args:
            input_contours: Contours as a list of numpy.ndarray.
            min_area: The minimum area of a contour that will be kept.
            min_perimeter: The minimum perimeter of a contour that will be kept.
            min_width: Minimum width of a contour.
            max_width: MaxWidth maximum width.
            min_height: Minimum height.
            max_height: Maximimum height.
            solidity: The minimum and maximum solidity of a contour.
            min_vertex_count: Minimum vertex Count of the contours.
            max_vertex_count: Maximum vertex Count.
            min_ratio: Minimum ratio of width to height.
            max_ratio: Maximum ratio of width to height.
        Returns:
            Contours as a list of numpy.ndarray.
        """
        output = []
        for contour in input_contours:
            x,y,w,h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)
        return output
    
    @staticmethod
    def grayscaleToBlackAndWhite(image, threshold):
        for row in image:
            for x in range(len(row)):
                if row[x]>threshold:
                    row[x] = 255
                else:
                    row[x] = 0
    
    @staticmethod
    def sum2DArray(arr):
        total =0
        for array in arr:
            for item in array:
                total+=item

        return total

    @staticmethod
    def binaryToPoints(twoDArray):
        points = []
        pointsIndex = 0
        for row in range(len(twoDArray)):
            for col in range(len(row)):
                if twoDArray[row][col]==255:
                    points[pointsIndex] = (row, col)
                    pointsIndex+=1
        return points


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

"""
pipeline = TapeRecognitionPipeline()
image = cv2.imread("C:/Users/Robotics/Documents/2019 Code/SwerveChassis/src/main/java/org/usfirst/frc/team3319/robot/vision/image.jpg")

cv2.imshow("image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
pipeline.process(image)
"""

def main():
    global tableInstance

    pipeline = TapeRecognitionPipeline()
    #cs = CameraServer.getInstance()

    # Capture from the first USB Camera on the system
    #camera = UsbCamera(name="Camera rPi Camera 0",path="/dev/video0")
    #server = cs.startAutomaticCapture(camera=camera,return_server=True)
    #camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    #camera.setResolution(320, 240)
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    tableInstance = NetworkTablesInstance()
    tableInstance.initialize(server="roboRIO-3319-FRC.local")
    print(tableInstance.isConnected())

    # Get a CvSink. This will capture images from the camera
    #cvSink = cs.getVideo()
    #cvSink.setEnabled(True)

    # Allocating new images is very expensive, always try to preallocate
    img = numpy.zeros(shape=(240, 320, 3), dtype=numpy.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        retval, img = capture.read(img)

        #
        # Insert your image processing logic here!
        #
        pipeline.process(img)
main()
#!/usr/bin/env python
"""Segmentation skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Grant Wang

This Python file is the skeleton code for Lab 3. You are expected to fill in
the body of the incomplete functions below to complete the lab. The 'test_..'
functions are already defined for you for allowing you to check your
implementations.

When you believe you have completed implementations of all the incompeleted
functions, you can test your code by running python segmentation.py at the
command line and step through test images
"""

import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

from scipy import ndimage
import scipy.misc as mc
from scipy import stats

# from scipy.misc import imresize
from skimage import filters
from sklearn.cluster import KMeans

from skimage.measure import block_reduce
import time
import pdb

this_file = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = '/'.join(this_file.split('/')[:-2]) + '/img'

def read_image(img_name, grayscale=False):
    """ reads an image

    Parameters
    ----------
    img_name : str
        name of image
    grayscale : boolean
        true if image is in grayscale, false o/w

    Returns
    -------
    ndarray
        an array representing the image read (w/ extension)
    """

    if not grayscale:
        img = cv2.imread(img_name)
    else:
        img = cv2.imread(img_name, 0)

    return img

def write_image(img, img_name):
    """writes the image as a file

    Parameters
    ----------
    img : ndarray
        an array representing an image
    img_name : str
        name of file to write as (make sure to put extension)
    """

    cv2.imwrite(img_name, img)

def show_image(img_name, title='Fig', grayscale=False):
    """show the  as a matplotlib figure

    Parameters
    ----------
    img_name : str
        name of image
    tile : str
        title to give the figure shown
    grayscale : boolean
        true if image is in grayscale, false o/w
    """

    if not grayscale:
        plt.imshow(img_name)
        plt.title(title)
        plt.show()
    else:
        plt.imshow(img_name, cmap='gray')
        plt.title(title)
        plt.show()


def threshold_segment_naive(gray_img, lower_thresh, upper_thresh):
    """perform grayscale thresholding using a lower and upper threshold by
    blacking the background lying between the threholds and whitening the
    foreground

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array
    lower_thresh : float or int
        lowerbound to threshold (an intensity value between 0-255)
    upper_thresh : float or int
        upperbound to threshold (an intensity value between 0-255)

    Returns
    -------
    ndarray
        thresholded version of gray_img
    """
    # TODO: Implement threshold segmentation by setting pixels of gray_img inside the
    # lower_thresh and upper_thresh parameters to 0
    # Then set any value that is outside the range to be 1
    # Hints: make a copy of gray_img so that we don't alter the original image
    # Boolean array indexing, or masking will come in handy.
    # See https://docs.scipy.org/doc/numpy-1.13.0/user/basics.indexing.html
    # thresh_img = gray_img < upper_thresh and gray_img > lower_thresh#upper_thresh
    thresh_high = gray_img < upper_thresh
    thresh_low = gray_img > lower_thresh
    thresh_img = np.logical_and(thresh_high, thresh_low)
    return thresh_img

def edge_detect_naive(gray_img):
    """perform edge detection using first two steps of Canny (Gaussian blurring and Sobel
    filtering)

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array

    Returns
    -------
    ndarray
        gray_img with edges outlined
    """

    gray_s = gray_img.astype('int16') # convert to int16 for better img quality
    # TODO: Blur gray_s using Gaussian blurring, convole the blurred image with
    # Sobel filters, and combine to compute the intensity gradient image (image with edges highlighted)
    # Hints: open-cv GaussianBlur will be helpful https://medium.com/analytics-vidhya/gaussian-blurring-with-python-and-opencv-ba8429eb879b
    # the scipy.ndimage.filters class (imported already) has a useful convolve function

    # Steps
    # 1. apply a gaussian blur with a 5x5 kernel.
    # 2. define the convolution kernel Kx and Ky as defined in the doc.
    # 3. compute Gx and Gy by convolving Kx and Ky respectively with the blurred image.
    # 4. compute G = sqrt(Gx ** 2 + Gy ** 2)
    # 5. Return G

    blurred = cv2.GaussianBlur(gray_s, (5,5), 0)
    Kx = np.array([
        [-1, 0, 1],
        [-2, 0, 2],
        [-1, 0, 1],
    ])
    Ky = np.array([
        [-1, -2, -1],
        [ 0,  0,  0],
        [ 1,  2,  1],
    ])
    Gx = cv2.filter2D(blurred, -1, Kx)
    Gy = cv2.filter2D(blurred, -1, Ky)
    G = np.sqrt(Gx ** 2 + Gy ** 2)
    return G

def edge_detect_canny(gray_img):
    """perform Canny edge detection

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array

    Returns
    -------
    ndarray
        gray_img with edges outlined
    """

    edges = cv2.Canny(gray_img, 100, 200)

    return edges

def cluster_segment(img, n_clusters, random_state=0):
    """segment image using k_means clustering

    Parameter
    ---------
    img : ndarray
        rgb image array
    n_clusters : int
        the number of clusters to form as well as the number of centroids to generate
    random_state : int
        determines random number generation for centroid initialization

    Returns
    -------
    ndarray
        clusters of gray_img represented with similar pixel values
    """
    # Remove this line when you implement this function.
    # raise NotImplementedError()

    # Do this only if clustering in HSV space isbetter
    # img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


    # Downsample img first using the mean to speed up K-means



    img_d = block_reduce(img, block_size=(2, 2, 1), func=np.mean)

    img_d = cv2.GaussianBlur(img_d,(5,5),0)






    # TODO: Generate a clustered image using K-means

    # first convert our 3-dimensional img_d array to a 2-dimensional array
    # whose shape will be (length * width, number of channels) hint: use img_d.shape
    img_r = img_d.reshape((img_d.shape[0]*img_d.shape[1], img_d.shape[2]))
    # print(img_r.shape)

    # fit the k-means algorithm on this reshaped array img_r using the
    # the scikit-learn k-means class and fit function
    # see https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html
    # the only parameters you have to worry about are n_clusters and random_state
    # print("channels",img_d.shape[2])
    kmeans = KMeans(n_clusters, random_state=random_state).fit(img_r)#[:,:3].reshape((img_d.shape[0]*img_d.shape[1], 2)))
    # print(kmeans)
    # get the labeled cluster image using kmeans.labels_
    clusters = kmeans.labels_  
    # print(len(set(clusters)))
    ### CHECK CLUSTER MEANS!! ###
    ### PUBLISH KMEANS IMAGE ###

    # reshape this clustered image to the original downsampled image (img_d) shape
    cluster_img = clusters.reshape((img_d.shape[0], img_d.shape[1]))
    # cluster_img = clusters.reshape(img_d.shape)
    # img_u = cluster_img



    # Upsample the image back to the original image (img) using nearest interpolation
    img_u = mc.imresize(cluster_img, (img.shape[0], img.shape[1]), interp='nearest')
    # print(len(set(img_u.ravel())))

    # Do this only if clustering in HSV space
    # img_u = cv2.cvtColor(img_u, cv2.COLOR_HSV2RGB)


    ## GET Cluster colors
    cluster_vals = list(set(img_u.ravel()))
    cluster_vals.sort()
    # print(cluster_vals)

    cluster_colors = {}
    for i in range(len(cluster_vals)):
        val = cluster_vals[i]
        color = kmeans.cluster_centers_[i]
        # if val == 0:
        #     continue
        cluster_colors[val] = color
    # print(cluster_colors)

    ## MAKE background color always 0
    background = stats.mode(img_u, axis = None)[0][0]
    if background != 0:
        img_u = np.where(img_u == background, 256, img_u)
        img_u = np.where(img_u == 0, background, img_u)
        img_u = np.where(img_u == 256, 0, img_u)
        temp = cluster_colors[0]
        cluster_colors[0] = cluster_colors[background]
        cluster_colors[background] = temp

    del cluster_colors[0]

    ## GET CONTOURS
    # # font = cv2.FONT_HERSHEY_COMPLEX
    # _, contours, _ = cv2.findContours(img_u, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # # _, contours, _ = cv2.findContours(img_u, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(len(contours))
    # centers = []
    # for cnt in contours:
    #     # approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
    #     # reduces edges. bigger multiplier yields fewer edges
    #     approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
    #     # print(approx)
    #     # print(len(approx))
    #     # cv2.drawContours(img_u, [approx], 0, (200), 5)
    #     # x = approx.ravel()[0]
    #     # y = approx.ravel()[1]

    #     if len(approx) == 4:
    #         print("rectangle")
    #         centers.append(np.mean(approx, axis = 0))
    #         cv2.drawContours(img_u, [approx], 0, (200), 5) #for debugging. remove later
    #         # cv2.putText(img_u, "Rectangle", (x, y), font, 1, (0))
    #     # else:
    #     #     cv2.drawContours(img_u, [approx], 0, (100), 5)

    #         # cv2.putText(img_u, "not a rectangle", (x, y), font, 1, (0))
    #     # break 

    ## GET PIXEL CENTERS of each cluster
    cluster_pixels = {}
    for y in range(len(img_u)):
        for x in range(len(img_u[0])):
            pixel = img_u[y][x]
            if pixel != 0:
                if not pixel in cluster_pixels.keys():
                    cluster_pixels[pixel] = []
                cluster_pixels[pixel].append([x, y])
    # print(cluster_pixels)

    centers = {}
    for key in cluster_pixels.keys():
        points = np.array(cluster_pixels[key])
        center = np.mean(points, axis = 0)
        centers[key] = center

    info = {"cluster_centers": centers, "cluster_colors": cluster_colors}

    return img_u.astype(np.uint8), info

def to_grayscale(rgb_img):
    return np.dot(rgb_img[... , :3] , [0.299 , 0.587, 0.114])

def segment_image(img):
    # ONLY USE ONE THRESHOLDING METHOD

    # perform thresholding segmentation
    upper_thresh = 100
    lower_thresh = 10
    binary = threshold_segment_naive(to_grayscale(img), lower_thresh, upper_thresh).astype(np.uint8)

    # binary = edge_detect_naive(to_grayscale(img)).astype(np.uint8)

    # binary = edge_detect_canny(to_grayscale(img)).astype(np.uint8)

    # perform clustering segmentation (make image binary)
    # binary = cluster_segment(img, 2).astype(np.uint8) / 255
    # if np.mean(binary) > 0.5:
    #     binary = 1 - binary #invert the pixels if K-Means assigned 1's to background, and 0's to foreground

    return binary

def segment_image2(img, num_clusters):
    # ONLY USE ONE THRESHOLDING METHOD


    # perform clustering segmentation (make image binary)
    # binary = cluster_segment(img, 3).astype(np.uint8) / 255
    binary, info = cluster_segment(img, num_clusters)
    binary = binary.astype(np.uint8)

    ## Instead of this, iterate over clusters and assign background to be 
    ## cluster with most pixels assigned, assign 1 to all others
    # if np.mean(binary) > 0.5:
    #     binary = 1 - binary #invert the pixels if K-Means assigned 1's to background, and 0's to foreground

    return binary, info

"""
below are tests used for sanity checking you image, edit as you see appropriate

"""

def test_thresh_naive(img, lower_thresh, upper_thresh):
    thresh = threshold_segment_naive(img, lower_thresh, upper_thresh)
    show_image(thresh, title='thresh naive', grayscale=True)
    cv2.imwrite(IMG_DIR + "/thresh.jpg", thresh.astype('uint8') * 255)

def test_edge_naive(img):
    edges = edge_detect_naive(img)
    show_image(edges, title='edge naive', grayscale=True)
    cv2.imwrite(IMG_DIR + "/edges.jpg", edges)

def test_edge_canny(img):
    edges = edge_detect_canny(img)
    show_image(edges, title='edge canny', grayscale=True)
    cv2.imwrite(IMG_DIR + "/edges_canny.jpg", edges)

def test_cluster(img, n_clusters):
    clusters, info = cluster_segment(img, n_clusters)
    clusters = clusters.astype(np.uint8)

    cv2.imwrite(IMG_DIR + "/cluster.jpg", clusters)
    clusters = cv2.imread(IMG_DIR + '/cluster.jpg')
    show_image(clusters, title='cluster')

def test_thresh_naive_helper(img_names):
    index = 0
    uppers = [105, 100, 220]
    lowers = [ 60,  10,  10]
    for img_name in img_names:
        test_img = read_image(img_name, grayscale=True)
        test_thresh_naive(test_img, lowers[index], uppers[index])
        index += 1

def test_edge_naive_helper(img_names):
    for img_name in img_names:
        test_img = read_image(img_name, grayscale=True)
        test_edge_naive(test_img)

def test_edge_canny_helper(img_names):
    for img_name in img_names:
        test_img = read_image(img_name, grayscale=True)
        test_edge_canny(test_img)

def test_cluster_helper(img_names):
    index = 0
    # n_clusters = [2, 3, 5]
    n_clusters = [
        # 2, 
        # 3, 
        5
    ]
    for img_name in img_names:
        test_img_color = read_image(img_name)
        test_cluster(test_img_color, n_clusters[index])
        index += 1

if __name__ == '__main__':
    # adjust the file names here
    img_names = [
        # IMG_DIR + '/lego.jpg',
        # IMG_DIR + '/staples.jpg',
        IMG_DIR + '/legos.jpg'
    ]
    # print(img_names)
    # uncomment the test you want to run
    # test_img_color = read_image(img_name)

    # it will plot the image and also save it
    # test_thresh_naive_helper(img_names)
    # test_edge_naive_helper(img_names)
    # test_edge_canny_helper(img_names)
    test_cluster_helper(img_names)

    # test_edge_naive(test_img)
    # test_edge_canny(test_img)
    # test_cluster(test_img_color, 2)

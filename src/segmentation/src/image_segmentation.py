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

from skimage import filters
from sklearn.cluster import KMeans

from skimage.measure import block_reduce
import time
import pdb

color_dict = {
    "white":  [ 223.34875017, 223.83849807, 225.89653249],
    "yellow": [ 247.74516109, 223.6023088 ,  29.2894295 ],
    "green":  [  56.2907913 , 148.41884933, 118.51122211],
    "red":    [ 238.99080433,  50.30322062,  72.59746392]
}

this_file = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = '../../img'

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
    # Downsample img first using the mean to speed up K-means
    img_d = block_reduce(img, block_size=(2, 2, 1), func=np.mean)
    img_d = cv2.GaussianBlur(img_d, (5, 5), 0)

    # first convert our 3-dimensional img_d array to a 2-dimensional array
    # whose shape will be (length * width, number of channels) hint: use img_d.shape
    img_r = img_d.reshape((img_d.shape[0]*img_d.shape[1], img_d.shape[2]))

    # fit the k-means algorithm on this reshaped array img_r using the
    # the scikit-learn k-means class and fit function
    # see https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html
    # the only parameters you have to worry about are n_clusters and random_state
    kmeans = KMeans(n_clusters, random_state=random_state).fit(img_r)#[:,:3].reshape((img_d.shape[0]*img_d.shape[1], 2)))
    # get the labeled cluster image using kmeans.labels_
    clusters = kmeans.labels_

    ### CHECK CLUSTER MEANS!! ###
    ### PUBLISH KMEANS IMAGE ###

    # reshape this clustered image to the original downsampled image (img_d) shape
    cluster_img = clusters.reshape((img_d.shape[0], img_d.shape[1]))

    # Upsample the image back to the original image (img) using nearest interpolation
    img_u = mc.imresize(cluster_img, (img.shape[0], img.shape[1]), interp='nearest')

    ## GET Cluster colors
    cluster_vals = list(set(img_u.ravel()))
    cluster_vals.sort()

    cluster_colors = {}
    for i in range(len(cluster_vals)):
        val = cluster_vals[i]
        color = kmeans.cluster_centers_[i]
        cluster_colors[val] = color

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

    ## GET PIXEL CENTERS of each cluster
    cluster_pixels = {}
    for y in range(len(img_u)):
        for x in range(len(img_u[0])):
            pixel = img_u[y][x]
            if pixel != 0:
                if not pixel in cluster_pixels.keys():
                    cluster_pixels[pixel] = []
                cluster_pixels[pixel].append([x, y])

    centers = {}
    for key in cluster_pixels.keys():
        points = np.array(cluster_pixels[key])
        center = np.mean(points, axis = 0)
        centers[key] = center
        cX, cY = (int(center[0]), int(center[1]))
        cv2.circle(img_u, (cX-5, cY-5), 7, (0, 0, 0), -1)

    # display color assignments:
    for name in color_dict.keys():
        col = color_dict[name]
        closest_pin_rgb = [0, 0, 0]
        closest_pin_distance = 1000000000
        k=0
        for i in range(len(cluster_colors.values())):
            c_col = list(cluster_colors.values())[i]
            pin_rgb_distance = np.linalg.norm(col - c_col)
            if pin_rgb_distance < closest_pin_distance:
                closest_pin_distance = pin_rgb_distance
                closest_pin_index = i
                closest_pin_rgb = c_col
                k = list(cluster_colors.keys())[i]

        cv2.putText(img_u, name, tuple([int(c) for c in centers[k]]), \
		      cv2.FONT_HERSHEY_SIMPLEX, 0.8, tuple(closest_pin_rgb), 3)


    info = {"cluster_centers": centers, "cluster_colors": cluster_colors}

    return img_u.astype(np.uint8), info

def segment_image(img, num_clusters):
    binary, info = cluster_segment(img, num_clusters)
    binary = binary.astype(np.uint8)

    return binary, info

def test_cluster(img, n_clusters):
    clusters, info = cluster_segment(img, n_clusters)
    clusters = clusters.astype(np.uint8)

    cv2.imwrite(IMG_DIR + "/cluster.jpg", clusters)
    clusters = cv2.imread(IMG_DIR + '/cluster.jpg')
    show_image(clusters, title='cluster')

def test_cluster_helper(img_names):
    index = 0
    n_clusters = [
        2
        3,
        5
    ]
    for img_name in img_names:
        test_img_color = read_image(img_name)
        test_cluster(test_img_color, n_clusters[index])
        index += 1

if __name__ == '__main__':
    # adjust the file names here
    img_names = [
        IMG_DIR + '/lego.jpg',
        IMG_DIR + '/staples.jpg',
        # IMG_DIR + '/legos.jpg',
        # IMG_DIR + '/shapes_and_colors.jpg',
        IMG_DIR + '/pins.png'
    ]

    test_cluster_helper(img_names)

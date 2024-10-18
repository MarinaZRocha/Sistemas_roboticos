#!/usr/bin/env python

import cv2
import numpy as np
import os
import glob

# Function to create directories if they don't exist
def create_directory(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

# Define checkerboard dimensions and square size
checkerboard = (6, 4)
square_size = 20  # Size of each square (e.g., in mm)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Create vectors to store 3D and 2D points
objpoints = []
imgpoints = []

# Define the world coordinates for 3D points (with physical size)
objp = np.zeros((1, checkerboard[0] * checkerboard[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:checkerboard[0], 0:checkerboard[1]].T.reshape(-1, 2) * square_size

# Extract paths of the images
images = glob.glob('/home/marina/sist_robotics/Sistemas_roboticos/perception/images2/*.jpg')

if len(images) == 0:
    print("No images found. Please check the image path.")
else:
    print(f"{len(images)} images found.")

# Initialize a counter for saving images
num = 0

# Directories to save the distorted and corrected images
distorted_dir = '/home/marina/sist_robotics/Sistemas_roboticos/perception/distorcida/'
corrected_dir = '/home/marina/sist_robotics/Sistemas_roboticos/perception/corrigida/'

# Create directories if they don't exist
create_directory(distorted_dir)
create_directory(corrected_dir)

for i, fname in enumerate(images):
    img = cv2.imread(fname)
    if img is None:
        print(f"Error loading image {fname}.")
        continue

    # Resize for display purposes only (reduce the size if images are large)
    display_img = cv2.resize(img, (800, 600)) if img.shape[1] > 800 else img

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        objpoints.append(objp)
        # Refine the 2D corner coordinates
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(display_img, checkerboard, corners2, ret)

        # Print progress information
        print(f"Image {i + 1}: {os.path.basename(fname)} processed successfully.")
        
        # Show the image briefly for verification
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        cv2.imshow('img', display_img)
        cv2.waitKey(100)  # Show each image for 100ms (adjust as needed)

cv2.destroyAllWindows()

# Check if valid images were found for calibration
if len(objpoints) == 0 or len(imgpoints) == 0:
    print("Error: No valid images found for calibration.")
else:
    # Perform camera calibration
    print("Starting calibration...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(f"Calibration completed successfully. Mean error: {ret}")

    print("Camera matrix: \n", mtx)
    print("Distortion coefficients: \n", dist)

    # Function to calculate and print reprojection errors
    def calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
        total_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
        mean_error = total_error / len(objpoints)
        return mean_error

    reprojection_error = calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist)
    print(f"Total reprojection error: {reprojection_error}")

    # Generate and save images showing distortion and correction
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            continue

        # Correct the distortion in the image
        undistorted_img = cv2.undistort(img, mtx, dist, None, mtx)

        # Save the original and corrected images
        distorted_path = os.path.join(distorted_dir, f'img{num}_distorted.png')
        corrected_path = os.path.join(corrected_dir, f'img{num}_corrected.png')

        cv2.imwrite(distorted_path, img)
        cv2.imwrite(corrected_path, undistorted_img)

        print(f"Distorted and corrected images saved as img{num}.png")
        num += 1

    # Save calibration data to a .YAML file
    file_name = "/home/marina/sist_robotics/Sistemas_roboticos/perception/img_calib.yaml"
    fs = cv2.FileStorage(file_name, cv2.FILE_STORAGE_WRITE)

    # Write the calibration data to the file
    fs.write("mtx", mtx)
    fs.write("dist", dist)
    fs.write("err", ret)

    fs.release()  # Close the file

    print(f"Calibration data saved to {file_name}")

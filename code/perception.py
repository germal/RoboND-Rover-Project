import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels below the threshold
# Threshold of RGB <= 160 does a nice job of identifying ground pixels only
def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be below all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] <= rgb_thresh[0]) \
                & (img[:,:,1] <= rgb_thresh[1]) \
                & (img[:,:,2] <= rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[below_thresh] = 1
    # Return the binary image
    return color_select

def rock_thresh(img, thresh_min=(0, 38, 145), thresh_max=(145, 148, 150)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in YUV
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = ((img[:,:,0] > thresh_min[0]) & (img[:,:,0] < thresh_max[0] )) \
                & ((img[:,:,1] > thresh_min[1]) & (img[:,:,1] < thresh_max[1])) \
                & ((img[:,:,2] > thresh_min[2]) & (img[:,:,1] < thresh_max[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    obstacle_threshed = obstacle_thresh(warped)
    warped_yuv=cv2.cvtColor(warped, cv2.COLOR_RGB2YUV)
    thresh_min=(0, 38, 153)
    thresh_max=(145, 148, 170)
    rock_threshed = rock_thresh(warped_yuv, thresh_min, thresh_max)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:,:,0] = np.multiply(obstacle_threshed, 255)
    Rover.vision_image[:,:,1] = np.multiply(rock_threshed, 255)
    Rover.vision_image[:,:,2] = np.multiply(threshed, 255)

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)
    xpix_obst, ypix_obst = rover_coords(obstacle_threshed)
    xpix_rock, ypix_rock = rover_coords(rock_threshed)

    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 12
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    #     print ("x:{0} y:{1} yaw:{2}".format(xpos, ypos, yaw))
    xpix_world, ypix_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    xpix_world_obst, ypix_world_obst = pix_to_world(xpix_obst, ypix_obst, xpos, ypos, yaw, world_size, scale)
    xpix_world_rock, ypix_world_rock = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # Rover.worldmap = np.zeros((200, 200, 3)).astype(np.float)
    # only update if pitch and roll near zero
    if not 1.0 <= Rover.pitch <= 359.0 and not 1.0 <= Rover.roll <= 359.0:
        for obstacle_x_world, obstacle_y_world in zip (xpix_world_obst, ypix_world_obst):
            Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        for rock_x_world, rock_y_world in zip (xpix_world_rock, ypix_world_rock):
            Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        for navigable_x_world, navigable_y_world, in zip(xpix_world, ypix_world):
            Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        Rover.worldmap = np.clip(Rover.worldmap, 0, 255)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    Rover.rock_dists, Rover.rock_angles = to_polar_coords(xpix_rock, ypix_rock)
    if len(Rover.rock_dists) == 0:
        Rover.rock_angles = None
        Rover.rock_dists = None
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)


    return Rover

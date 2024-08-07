import numpy as np

# returns approximate angle of yellow road line
# or UNKNOWN if no pixel of yellow line visible
def process_camera_image(camera_height, camera_width, camera_fov, image):
    num_pixels = camera_height * camera_width  # number of pixels in the image
    REF = np.array([95, 187, 203])  # road yellow (BGR format)
    # REF = np.array([255, 255, 255])  # road yellow (BGR format)
    sumx = 0  # summed x position of pixels
    pixel_count = 0  # yellow pixels count

    pixel = image
    for x in range(num_pixels):
        if color_diff(pixel, REF) < 30:
            sumx += x % camera_width
            pixel_count += 1  # count yellow pixels
        pixel = pixel[4:] # move the pixel to the next four memory space

    # if no pixels were detected...
    if pixel_count == 0:
        return 99999.99

    return ((sumx / pixel_count / camera_width) - 0.5) * camera_fov

def color_diff(a, b):
    return sum(abs(a[i] - b[i]) for i in range(3))
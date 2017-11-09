import cv2
import math
import numpy as np

def resizeAndPad(img, size, padColor=0):
  ''' 
  Takes an opencv input image and creates a opencv new image
  of the specified size with the original image scaled down to fit
  inside of it. 

  From https://stackoverflow.com/questions/44720580/resize-image-canvas-to-maintain-square-aspect-ratio-in-python-opencv
  '''

  h, w = img.shape[:2]
  sh, sw = size

  # interpolation method
  if h > sh or w > sw: # shrinking image
      interp = cv2.INTER_AREA
  else: # stretching image
      interp = cv2.INTER_CUBIC

  # aspect ratio of image
  aspect = w/h

  # compute scaling and pad sizing
  if aspect > 1: # horizontal image
      new_w = sw
      new_h = np.round(new_w/aspect).astype(int)
      pad_vert = (sh-new_h)/2
      pad_top, pad_bot = np.floor(pad_vert).astype(int), np.ceil(pad_vert).astype(int)
      pad_left, pad_right = 0, 0
  elif aspect < 1: # vertical image
      new_h = sh
      new_w = np.round(new_h*aspect).astype(int)
      pad_horz = (sw-new_w)/2
      pad_left, pad_right = np.floor(pad_horz).astype(int), np.ceil(pad_horz).astype(int)
      pad_top, pad_bot = 0, 0
  else: # square image
      new_h, new_w = sh, sw
      pad_left, pad_right, pad_top, pad_bot = 0, 0, 0, 0

  # set pad color
  if len(img.shape) is 3 and not isinstance(padColor, (list, tuple, np.ndarray)): # color image but only one color provided
      padColor = [padColor]*3

  # scale and pad
  scaled_img = cv2.resize(img, (new_w, new_h), interpolation=interp)
  scaled_img = cv2.copyMakeBorder(scaled_img, pad_top, pad_bot, pad_left, pad_right, borderType=cv2.BORDER_CONSTANT, value=padColor)

  return scaled_img

def generateColorMap(img, minVal=None, maxVal=None, colorMapType = cv2.COLORMAP_HOT):
  if len(img.shape) > 2 and img.shape[2] != 1:
    print("Not sure how to generate color maps for more than 1 channel images...")

  if minVal is None:
    minVal = np.nanmin(img)
  if maxVal is None:
    maxVal = np.nanmax(img)

  if (maxVal - minVal < 1E-10):
    print "Warning: generateColorMap rescaling broken due to bad image values."
    maxVal = minVal + 1.0

  img_rescaled = (255./(maxVal-minVal))*(img - minVal)

  return cv2.applyColorMap(img_rescaled.astype(np.uint8), colorMapType)

def generateGridOfImages(image_list, cols, min_gap_size, width=None, height=None):
  max_width = 0
  max_height = 0
  for image in image_list:
    max_height = max(max_height, image.shape[0])
    max_width = max(max_width, image.shape[1])

  rows = int(math.ceil(len(image_list) / cols))

  result = np.zeros((rows * max_height + (rows - 1) * min_gap_size,
                     cols * max_width + (cols - 1) * min_gap_size,
                     3), np.uint8)

  i = 0
  current_height = 0
  current_width = 0
  for y in range(rows):
    for x in range(cols):
      if len(image_list[i].shape) == 2 or image_list[i].shape[2] == 1:
        result[current_height : current_height + image_list[i].shape[0],
               current_width : current_width + image_list[i].shape[1]] = np.repeat(image_list[i][:, :, np.newaxis], 3, axis=2)
      else:  
        result[current_height : current_height + image_list[i].shape[0],
               current_width : current_width + image_list[i].shape[1]] = image_list[i]
      i += 1
      current_width += max_width + min_gap_size
    # next row
    current_height += max_height + min_gap_size
    current_width = 0

  if height is not None and width is not None:
    return resizeAndPad(result, (height, width))
  else:
    return result
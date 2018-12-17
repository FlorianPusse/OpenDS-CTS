import pygame
from Config import Config
import numpy as np
from collections import deque
import cv2

BLUE  = (128, 128, 255)
RED   = (255, 192, 192)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

depth_dict = {k:v for k,v in zip(range(Config.DEPTH_QUANTIZATION),
    [0.05,0.175,0.3,0.425,0.55,0.675,0.8,1])}  #bins

class MovieWriter(object):
  def __init__(self, file_name, frame_size, fps):
    self.vout = cv2.VideoWriter()
    if not self.vout.open(file_name,
            cv2.VideoWriter_fourcc('M','J','P','G'), fps, frame_size, True):
        print("Create movie failed: {0}".format(file_name))

  def add_frame(self, frame):
    self.vout.write(frame)

  def close(self):
    self.vout.release() 
    self.vout = None

  def isOpen(self):
      return self.vout != None and self.vout.isOpened()

class Display(object):
  def __init__(self):
    pygame.init()
    
    self.display_size = Config.DISPLAY_SIZE
    self.surface = pygame.display.set_mode(self.display_size, 0, 24)
    pygame.display.set_caption('NAV')
    self.font = pygame.font.SysFont(None, 20)
    self._values = deque(maxlen=100)
    if Config.RECORD:
        self.video_fps = 5
        self.frames = 0
        self.writer = MovieWriter('melonvideo.avi', self.display_size, self.video_fps) 

  def draw_center_text(self, str, center_x, top):
    text = self.font.render(str, True, WHITE, BLACK)
    text_rect = text.get_rect()
    text_rect.centerx = center_x
    text_rect.top = top
    self.surface.blit(text, text_rect)

  def show_image(self, im):
    data = im.astype(np.uint8)
    image = pygame.image.frombuffer(data, (84,84), 'RGB')
    image = pygame.transform.scale(image, (128, 128))
    self.surface.blit(image, (8, 8))
    self.draw_center_text("input", 50, 150)

  def show_depth(self, dm):
      dm = dm * 255.
      data = dm.astype(np.uint8)
      color_img = cv2.cvtColor(data, cv2.COLOR_GRAY2RGB) 

      image = pygame.image.frombuffer(color_img, (16,4), 'RGB')
      image = pygame.transform.scale(image, (128, 32))
      self.surface.blit(image, (200, 8))
      self.draw_center_text("depth", 250, 50)

  def show_policy(self, pi):
    start_x = 10

    y = 200
  
    for i in range(len(pi)):
      width = pi[i] * 100
      pygame.draw.rect(self.surface, WHITE, (start_x, y, width, 10))
      y += 20
    self.draw_center_text("Action Prob.", 50, y)
  
  def show_values(self):
    if  len(self._values) == 0:
      return

    min_v = float("inf")
    max_v = float("-inf")

    for v in self._values:
      min_v = min(min_v, v)
      max_v = max(max_v, v)

    top = 150
    left = 150
    width = 100
    height = 100
    bottom = top + width
    right = left + height

    d = max_v - min_v
    last_r = 0.0
    for i,v in enumerate(self._values):
      r = (v - min_v) / d
      if i > 0:
        x0 = i-1 + left
        x1 = i   + left
        y0 = bottom - last_r * height
        y1 = bottom - r * height
        pygame.draw.line(self.surface, BLUE, (x0, y0), (x1, y1), 1)
      last_r = r

    pygame.draw.line(self.surface, WHITE, (left,  top),    (left,  bottom), 1)
    pygame.draw.line(self.surface, WHITE, (right, top),    (right, bottom), 1)
    pygame.draw.line(self.surface, WHITE, (left,  top),    (right, top),    1)
    pygame.draw.line(self.surface, WHITE, (left,  bottom), (right, bottom), 1)

    self.draw_center_text("V", left + width/2, bottom+10)

  def update(self, state, prediction, value, depth):
      im_size = Config.IMAGE_HEIGHT*Config.IMAGE_WIDTH*Config.IMAGE_DEPTH
      im = state[:im_size] * 255.
      im = np.reshape(im, (Config.IMAGE_HEIGHT, Config.IMAGE_WIDTH, Config.IMAGE_DEPTH))
      self._values.append(value)

      # create depth_map (4,16) from depth (64, 8)
      depth_map = [depth_dict[np.argmax(depth[p])] for p in
              range(depth.shape[0])]
      depth_map = np.array(depth_map)

      self.surface.fill(BLACK)      
      self.show_image(im)
      self.show_policy(prediction) 
      self.show_values()
      self.show_depth(depth_map)
      pygame.display.update()

      if Config.RECORD and self.writer.isOpen(): 
          frame_str = self.surface.get_buffer().raw
          d = np.fromstring(frame_str, dtype=np.uint8)
          d = d.reshape((self.display_size[1], self.display_size[0], 3))
          self.writer.add_frame(d)
          self.frames += 1
          if self.frames == Config.VIDEO_DURATION*self.video_fps:
              print("Movie writing complete.")
              self.writer.close()

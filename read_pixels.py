#!/usr/bin/python3 -i

import serial # if you have not already done so
from time import sleep
import matplotlib.pyplot as plt
import re
import datetime
import numpy 
import pickle

class DataExtruder:

  def __init__(self,port='/dev/ttyACM0',baudrate=115200):
    self.pattern_pixels=re.compile(r'data=(?P<pixels>[\w ]*) \((?P<nerror>\d*) errors')
    self.port=port
    self.baudrate=baudrate
    self.ser = None
    self.data={
        'pixels':[],
        'nerror':[]
     }
    self.figure=plt.figure(figsize=[20,8])
    self.figure.show()
    self.figure_axe=self.figure.gca()


  def acquire(self,plot=True):
    if self.ser is None:
      self.ser=serial.Serial(self.port, self.baudrate)
    else:
      print('serial connection alredy opened')
    print('starting acquisition, press Ctrl+C to stop.')
    try:
      while True:
        data_serial=self.ser.readline().decode('utf-8')
        m=self.pattern_pixels.match(data_serial)
        if m:
          pixels_num=[];
          pixels_ascii=m.group('pixels');
          i=0
          npixel=0
          while i+1<len(pixels_ascii):
            if pixels_ascii[i]==' ':
              if pixels_ascii[i+1]==' ':
                pixels_num.append(-1)
                i=i+2
              else:
                print('ERROR reading pixel')
                break
            else:
              pixel=255-int(pixels_ascii[i:i+2],16)
              pixels_num.append(pixel)
              i=i+2
            npixel=npixel+1
          self.data['pixels'].append(pixels_num)
          self.data['nerror'].append(int(m.group('nerror')))
          if plot:
            self.plot_pixels()
        sleep(0.05)
    except KeyboardInterrupt:
      pass
    self.ser.close()
    self.ser=None

  def plot_pixels(self):
    plt.cla()
    self.figure_axe.set_position([0.05,0.1,0.94,0.8])
    if len(self.data['pixels'])==0:
      return
    last_reading=self.data['pixels'][len(self.data['pixels'])-1]
    if len(last_reading)!=3648:
      return
    x=range(1,3649)
    self.plt_pixels,=plt.plot(x,last_reading,'b-')
    self.figure_axe.set_ylim([-1,255])
    self.figure_axe.set_xlim([1,3648])
    self.figure_axe.set_ylabel('pixel value')
    self.figure_axe.set_xlabel('pixel')
    plt.draw()

if __name__ == '__main__':
  test=DataExtruder(port='/dev/ttyACM0',baudrate=115200)
  test.acquire()

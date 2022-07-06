# Geoscan FPV

  Videopresentation and video from drone flight on google drive
  https://drive.google.com/drive/u/0/folders/1seKN3CPWZcdEoEq8AW68mqNXvsFvLPHq

# arduino
  
  Arudino is a file that you need to upload in arduino(surprisingly) to make it convert serial signal from code to PPM signal for HF module

  To make it work you need to go to C:\Users\Username\Documents\Arduino\libraries\PPMEncoder\src\PPMEncoder.h and change "static const uint16_t MIN = 1000;" to "static const uint16_t MIN = 900;" and only then upload sketch to the arduino
  
# pioneerPPM
  
  PioneerPPM is a library for python, so you can send signal to arduino which sending signal to HF module which sends signal to drone's reciever which makes drone fly :)
  

# test
  
  Test is a temporary solution to solve tasks that we wanted to solve using pioneerPPM but using pioneer_sdk instead
  
  

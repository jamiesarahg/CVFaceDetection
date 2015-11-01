import csv
from os import listdir
import cv2
from os.path import isfile, join



def collectImages():
  """"""
  facialDataByPerson = {}
  width = 800 #width of screen of pygame ball
  frameRate = 50 # rate of pictures which are taken by number of pixels
  i = 0
  while i <= width:
    directory = '../images/400_' + str(i)
    onlyfiles = [ f for f in listdir(directory) if isfile(join(directory,f)) ]
    for item in onlyfiles:
      im = cv2.imread(directory+'/'+item, 0)
      if im.shape == (24, 24):
        cv2.imshow('who am i?', im)
        cv2.waitkey(0)
        name = raw_input()
        if name in facialDataByPerson.keys():
          facialDataByPerson[name].append(im)
        else:
          facialDataByPerson[name] = [im]
  print facialDataByPerson

collectImages()
#         reshaped = np.reshape(im, 576)
#         X_train.append(reshaped)
#         Y_train.append((400,i))

#     directory = '../images/' + str(i) + '_400'
#     onlyfiles = [ f for f in listdir(directory) if isfile(join(directory,f)) ]
#     for item in onlyfiles:
#       im = cv2.imread(directory+'/'+item, 0)
#       if im.shape == (24, 24):
#         reshaped = np.reshape(im, 576)
#         X_train.append(reshaped)
#         Y_train.append((i, 400))

#     i+= frameRate

#   return (X_train, Y_train)


# w = csv.writer(open("output.csv", "w"))
# for key, val in dict.items():
#     w.writerow([key, val])

# Then reading it would be:

# import csv
# dict = {}
# for key, val in csv.reader(open("input.csv")):
#     dict[key] = val
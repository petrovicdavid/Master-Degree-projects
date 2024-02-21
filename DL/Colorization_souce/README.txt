First of all:
Modify the subfolders (one for each specie) of the original grayscale images in order to substitute the space (" ") with a "-".
ex: from "G. BulloidesG" to "G-Bulloides".

In order to compile correctly:
- import cv2

In the first source code:
modify the line 7: insert the local path where there is the folder with the original gray scale images

In the second source code:
line 23: insert the global path where there are the models (folder with model already downloaded: MODEL)
line 41: insert the local path where there are the original gray scale images
line 42: insert the local path for the output

OUTPUT folder ForaminiferaColored where there are one subfolder for each specie
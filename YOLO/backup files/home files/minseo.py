#영상처리 

#1
# import numpy as np

# a=np.array([4,5,0,1,2,3,6,7,8,9,10,11])
# print(a)
# print(type(a))
# print(a.shape)
# a.sort()
# print(a)

# b=np.array([-4.3,-2.3,12.9,8.99,10.1,-1.2])
# b.sort()
# print(b)

# c=np.array(['one','two','three','four','five','six','seven'])
# c.sort()
# print(c)


# 2
# import cv2 as cv
# import sys

# img=cv.imread('/home/kminseo/Downloads/soccer.jpg')

# if img is None:
#     sys.exit('You can not find file ')

# cv.imshow('Image Display',img)

# cv.waitKey()
# cv.destroyAllWindows()


#3
# import cv2 as cv
# import sys

# img=cv.imread('/home/kminseo/Downloads/soccer.jpg')

# if img is None:
#     sys.exit('You can not find file')

# gray=cv.cvtColor(img,cv.COLOR_BGR2GRAY)
# gray_small=cv.resize(gray,dsize=(0,0),fx=0.5,fy=0.5)

# cv.imwrite('/home/kminseo/Downloads/soccer_gray.jpg',gray)
# cv.imwrite('/home/kminseo/Downloads/soccer_gray_small.jpg',gray_small)

# cv.imshow('Color image',img)
# cv.imshow('Gray image',gray)
# cv.imshow('Gray image small',gray_small)

# cv.waitKey()
# cv.destroyAllWindows()

#4
# import cv2 as cv
# import sys

# img=cv.imread('/home/kminseo/Downloads/girl_laughing.jpg')

# if img is None:
#     sys.exit('You can not find file')

# cv.rectangle(img,(103,70),(53,31),(0,0,255),2)
# cv.putText(img,'laugh',(53,24),cv.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
# cv.rectangle(img,(254,47),(205,18),(0,0,255),2)
# cv.putText(img,'laugh',(205,13),cv.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)

# cv.imshow('Draw',img)

# cv.waitKey()
# cv.destroyAllWindows()

#5
# import cv2 as cv
# import sys

# img=cv.imread('/home/kminseo/Downloads/girl_laughing.jpg')

# if img is None:
#     sys.exit('You can not find file')

# def draw(event,x,y,flags,param):
#     if event==cv.EVENT_LBUTTONDOWN:
#         cv.rectangle(img,x,y,(x+50,y+50),(0,0,255),2)
#     elif event==cv.EVENT_RBUTTONDOWN:
#         cv.rectangle(img,(x,y),(x+10,y+10),(255,0,0),2)
    
#     cv.imshow('Drawing',img)

# cv.namedWindow('Drawing')
# cv.imshow('Drawing',img)

# cv.setMouseCallback('Drawing',draw)

# while(True):
#     if cv.waitKey(1)==ord('q'):
#         cv.destroyAllWindows()
#         break


#6







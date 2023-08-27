import cv2

vid1 = cv2.VideoCapture(2)
vid1.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
vid1.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

count = 0

while True:

    _,fr = vid1.read()
    cv2.imshow("f",fr)

    k = cv2.waitKey(10)

    if k == 115:
        count += 1

        cv2.imwrite('callibration/'+str(count)+".png",fr)
        print("image saved",count)

    elif k==27:

        break
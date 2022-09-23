import cv2


vidcap = cv2.VideoCapture('/home/enbang/Downloads/2021-09-12 14-24-59.mkv')

count = 0

while (vidcap.isOpened()):

    ret, image = vidcap.read()

    if (int(vidcap.get(1)) % 5 == 0):
        print('Saved frame number : ' + str(int(vidcap.get(1))))
        cv2.imwrite("/home/enbang/A3/%d.jpg" % count, image)
        print('%d.jpg' % count)
        count += 1

vidcap.release()
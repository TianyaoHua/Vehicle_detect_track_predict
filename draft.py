import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle

def crop_minAreaRect(img, rect, box):
    W = rect[1][0]
    H = rect[1][1]

    Xs = [i[0] for i in box]
    Ys = [i[1] for i in box]
    x1 = min(Xs)
    x2 = max(Xs)
    y1 = min(Ys)
    y2 = max(Ys)

    rotated = False
    angle = rect[2]

    if angle < -45:
        angle += 90
        rotated = True

    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    size = (int((x2 - x1)), int((y2 - y1)))

    M = cv2.getRotationMatrix2D((size[0] / 2, size[1] / 2), angle, 1.0)

    cropped = cv2.getRectSubPix(img, size, center)
    cropped = cv2.warpAffine(cropped, M, size)

    croppedW = W if not rotated else H
    croppedH = H if not rotated else W

    croppedRotated = cv2.getRectSubPix(cropped, (int(croppedW), int(croppedH)),
                                       (size[0] / 2, size[1] / 2))
    return croppedRotated

with open('0001553.pkl', 'rb') as f:
    data = pickle.load(f)
mask = np.zeros((1080, 1920)).astype(np.uint8)
cnt = data["contours"][13]
print(len(cnt))
cv2.fillPoly(mask, cnt, (1))
cv2.drawContours(mask, cnt, -1, (1), 2)
rect = cv2.minAreaRect(np.concatenate(tuple(cnt)))
box = cv2.boxPoints(rect)
box = np.int0(box)
print(box)
mask_crop = crop_minAreaRect(mask, rect, box)

cv2.drawContours(mask,[box],0,(1),2)

plt.figure()
plt.imshow(mask_crop)
plt.figure()
plt.imshow(mask)
plt.show()
print(np.max(mask))
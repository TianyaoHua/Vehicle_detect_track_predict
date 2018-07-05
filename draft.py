import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle
import os
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
def draw():
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
def show(img, mode):
    plt.figure()
    plt.axis("off")
    if mode == 'gray':
        plt.imshow(img,cmap='gray')
    elif mode == 'rgb':
        plt.imshow(img[:, :, [2, 1, 0]])
def diff2mask(diff_4d):
    #cnt = cv2.findContours()
    th = np.median(diff_4d)
    mask = np.zeros_like(diff_4d)
    mask[diff_4d > th+3] = 1
    mask[diff_4d < th-3] = 1
    ##mask = cv2.medianBlur(mask, 5)
    return mask

def diff_4d(img1, img2):
    img_diff = (img1[:, :, 0].astype(np.double))-(img2[:, :, 0].astype(np.double))
    img_diff = (img_diff-np.min(img_diff))/(np.max(img_diff)-np.min(img_diff))*255
    img_diff = np.uint8(img_diff)
    return img_diff

def edge4d():
    cap = cv2.VideoCapture('C:\\My_Projects\\ProjectsWithZoran\\video_on_intersection\\Loc0_1.mp4')
    success, img1 = cap.read()
    output_dir = "C:\\My_Projects\\ProjectsWithZoran\\video_on_intersection\\tracked_video"
    count = 0
    #while success:
    success, img2 = cap.read()
    img_diff = diff_4d(img1, img2)
    plt.imshow(img_diff, cmap='gray')
    plt.show()
    mask = diff2mask(img_diff)
    plt.imshow(mask, cmap='gray')
    plt.show()
    alpha = 0.5
    color = (1,1,0)
    img1 = img1.astype(np.double)
    for c in range(3):
        img1[:, :, c] = np.where(mask == 1, img1[:, :, c] * (1 - alpha) + alpha * color[c] * 255, img1[:, :, c])
    img1 = img1.astype(np.uint8)
    #cv2.imwrite(os.path.join(output_dir, "%06d.jpg" % count), img1)
    img1 = img2.copy()
    count += 1
    if count%10 == 0:
        print(count)
edge4d()

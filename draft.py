import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle
with open('0001553.pkl', 'rb') as f:
    data = pickle.load(f)
mask = np.zeros((1080, 1920)).astype(np.uint8)
cv2.fillPoly(mask, data["contours"][5], (1))
print(mask)
plt.figure()
plt.imshow(mask)
plt.show()
print(np.max(mask))
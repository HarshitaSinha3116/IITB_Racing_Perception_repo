!pip install ultralytics opencv-python -q
from ultralytics import YOLO
import cv2
from google.colab import files
from matplotlib import pyplot as plt

uploaded = files.upload()

uploaded = files.upload()

image_path = "image.png"

image = cv2.imread(image_path)

if image is None:
  print("no image")
else:
  print("image loaded successfully")

img_with_boxes = image.copy()
f = 1000
H = 25

for box in results[0].boxes:
  x1, y1, x2, y2 = map(int, box.xyxy[0])

  h =y2-y1
  if(h==0):
    continue
    
  depth = (f*H)/h
  class_id = int(box.cls)

if(class_id == 1):
  c = (255, 0, 0)
elif(class_id == 2):
  c = (0, 140, 255)
else:
  c = (0, 255, 255)

cv2.rectangle(img_with_boxes, (x1, y1), (x2, y2), c, 2)

depth_text = f"{depth:.2f}"

cv2.putText(img_with_boxes, depth_text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)

img_rgb = cv2.cvtColor(image_with_boxes, cv2.COLOR_BGR2RGB)

plt.figure(figsize = (8, 6))
plt.imshow(img_rgb)
plt.axis("off")


  


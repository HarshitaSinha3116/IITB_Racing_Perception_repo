import cv2
image = cv2.imread(r"C:\Users\Harshita Sinha\Downloads")
boxes = [
 [2.0, [0.43476563692092896, 0.7194444537162781, 0.03984374925494194, 0.1111111119389534], 0.8438236117362976],
 [2.0, [0.47539061307907104, 0.737500011920929, 0.04296875, 0.11666666716337204], 0.8504939675331116],
 [0.0, [0.33671873807907104, 0.7111111283302307, 0.03593749925494194, 0.09444444626569748], 0.9057255387306213],
 [3.0, [0.2679687440395355, 0.7361111044883728, 0.04374999925494194, 0.125], 0.919158935546875],
 [0.0, [0.21054688096046448, 0.7479166388511658, 0.04921875149011612, 0.12638889253139496], 0.9196999073028564],
 [3.0, [0.38945311307907104, 0.7201389074325562, 0.03984374925494194, 0.10972221940755844], 0.9205850958824158],
 [3.0, [0.5414062738418579, 0.762499988079071, 0.05312500149011612, 0.14444445073604584], 0.9242506623268127],
 [0.0, [0.610156238079071, 0.7909722328186035, 0.06406249850988388, 0.1736111044883728], 0.9282561540603638]
]
width = 1280
height = 720
for class_id, bbox, confidence in boxes:
  x_centre, y_centre, w, h = bbox

  if(class_id == 0.0):
    c = (255, 0, 0)
  elif(class_id == 3.0):
    c = (0, 255, 255)
  elif(class_id == 2.0):
    c = (0, 140, 255)

  x_centre  _px = int(x_centre*width)
  y_centre_px = int(y_centre*height)

  w_px = int(w*width)
  h_px = int(h*height)

  x1 = int(x_centre_px - w_px/2)
  y1 = int(y_centre_px - h_px/2)

  x2 = int(x_centre_px + w_px/2)
  y2 = int(y_centre_px + h_px/2)

  cv2.rectangle(image, (x1, y1), (x2, y2), c, 1)

  ho = 25
  f = 1000
  hi = h*height
  d = (f*ho)/hi
  depth_text = f"{d:.2f} cm"

  cv2.putText(image, depth_text, (x1, y1-5), cv2.FONT_HERSHEY_COMPLEX, 0.3, c, 1) 

cv2.imshow("Loaded Image")
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("output.png", image)

 

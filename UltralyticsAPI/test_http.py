import cv2
import requests
import numpy as np

def parse_image(data):
    buff = np.fromstring(data, np.uint8).reshape(1, -1)
    img = cv2.imdecode(buff, cv2.IMREAD_COLOR)
    return img

cap = cv2.VideoCapture(1)
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    enc_img = cv2.imencode('.jpg', frame)[1].tobytes()
    ip = "172.29.19.148"
    ip = "localhost"
    ret = requests.get(f'http://{ip}:8000/process_image', data=enc_img)
    annotated_img = parse_image(ret.content)
    cv2.imshow("annotated", annotated_img)

    if cv2.waitKey(30) == 27:
        break

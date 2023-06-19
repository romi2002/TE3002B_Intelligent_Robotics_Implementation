from typing import Union
from fastapi import FastAPI, Request, Depends, Response
import cv2
import numpy as np
from contextlib import asynccontextmanager
import torch
import json
import base64

model = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global model
    #model = torch.hub.load("ultralytics/yolov5", "custom", path='best.pt')
    #model = torch.hub.load('./', 'custom', path='best.pt', source='local')
    model = torch.hub.load("ultralytics/yolov5", "custom", path='best.pt')
    yield

app = FastAPI(lifespan=lifespan)

@app.get("/")
def read_root():
    return {"Hello": "World"}

async def parse_image(request: Request):
    data: bytes = await request.body()
    buff = np.fromstring(data, np.uint8).reshape(1, -1)
    img = cv2.imdecode(buff, cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img

def serialize_image(img):
    return cv2.imencode('.jpg', img)[1].tobytes()

@app.get("/process_image", response_class=Response)
async def process_image(img = Depends(parse_image)):
    global model
    result = model(img)
    annotated_frame = result.render()[0]
    prediction = result.pred[0].cpu().numpy()
    data = {
        "results": str(result.pred),
        "prediction-dtype": str(prediction.dtype),
        "prediction": str(base64.b64encode(prediction), 'utf-8'),
        "prediction-shape": prediction.shape,
        "names": result.names}
    return Response(
        content=serialize_image(annotated_frame),
        headers={
            "data": json.dumps(data)
        },
        media_type="image/jpg"
    )


from typing import Union
from fastapi import FastAPI, Request, Depends, Response
import cv2
import numpy as np
from contextlib import asynccontextmanager
from ultralytics import YOLO
import json

model = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global model
    model = YOLO('best.pt')
    yield

app = FastAPI(lifespan=lifespan)

@app.get("/")
def read_root():
    return {"Hello": "World"}

async def parse_image(request: Request):
    data: bytes = await request.body()
    buff = np.fromstring(data, np.uint8).reshape(1, -1)
    img = cv2.imdecode(buff, cv2.IMREAD_COLOR)
    return img

def serialize_image(img):
    return cv2.imencode('.jpg', img)[1].tobytes()

@app.get("/process_image", response_class=Response)
async def process_image(img = Depends(parse_image)):
    global model
    result = model(img)[0]
    annotated_frame = result.plot()
    data = {
        "results": str(result.boxes.data.to('cpu').numpy().tostring()),
        "names": result.names,
        "speed": result.speed
    }
    return Response(
        content=serialize_image(annotated_frame),
        headers={
            "data": json.dumps(data)
        },
        media_type="image/jpg"
    )
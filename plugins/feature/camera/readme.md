<h1>Camera Plugin</h1>

<h2>Introduction</h2>

The Camera feature plugin allows SDRangel to capture images and video from cameras and telescopes.
This is to support multimode observations, such as combing radio and optical observations of meteors, but can also be used for observing remote radio equipment or conditions.

The Camera plugin supports cameras supported by the Qt6 Multimedia API as well as ASCOM Alpaca API. 

<h2>Interface</h2>

![Camera feature plugin GUI](../../../doc/img/Camera_plugin.png)

<h2>YOLO models</h2>

```
from ultralytics import YOLO
model = YOLO("yolov8n.pt")
model.export(format="onnx", opset=12)
```

COCO class list: https://raw.githubusercontent.com/amikelive/coco-labels/refs/heads/master/coco-labels-2014_2017.txt

<h2>API</h2>

Full details of the API can be found in the Swagger documentation. 

    curl -X PATCH "http://127.0.0.1:8091/sdrangel/featureset/feature/0/settings" -d '{"featureType": "Camera",  "CameraSettings": { "azimuth": 180, "elevation": 45 }}'

To camera an image/vidoe:

    curl -X POST "http://127.0.0.1:8091/sdrangel/featureset/feature/0/run"

<h2>Optional Prerequisites</h2>

ASCOM Platform:

https://ascom-standards.org/Downloads/Index.htm

Camera specific ASCOM drivers, such as for ZWO ASI cameras:

https://www.zwoastro.com/layouts/download-desktop-app/

Start ASCOM Remote Server.


ASCOM Alpaca Device API docs are at: https://ascom-standards.org/api/

If imageready is always false, try restarting the ASCOM Remote Server.



<h2>Attribution</h2>

- Object detection by Lars Meiertoberens from Noun Project (CC BY 3.0)
- html by wira wianda from Noun Project (CC BY 3.0)
- clock by Alv Jørgen Bovolden from Noun Project (CC BY 3.0)
- invert by Meko from Noun Project (CC BY 3.0)
- subtract-picture by Smashicons from Noun Project (CC BY 3.0)

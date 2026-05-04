<h1>Camera Plugin</h1>

<h2>Introduction</h2>

The Camera feature plugin allows SDRangel to capture images and video from cameras and telescopes.
This is to support multimode observations, such as combing radio and optical observations of meteors, but can also be used for observing remote radio equipment or conditions.

The Camera plugin supports cameras supported by the Qt6 Multimedia API as well as ASCOM Alpaca API. 

<h2>Interface</h2>

![Camera feature plugin GUI](../../../doc/img/Camera_plugin.png)

The main toolbar lets you start and stop capture, choose a camera, refresh the camera list, open the settings dialog, mute the associated audio input and control the image zoom.

<h3>Camera settings</h3>

Press the settings button to open the Camera Settings dialog.

<h3>Camera tab</h3>

The Camera tab contains the capture settings for the selected device.

For Qt cameras this includes resolution, capture mode, frame rate or capture interval, exposure, ISO, white balance, exposure compensation, focus mode, focus distance and zoom.

For Alpaca cameras this includes binning, gain, offset, readout mode and subframe settings. Setting subframe width or height to 0 requests the full available image in that dimension.

<h3>Alpaca tab</h3>

The Alpaca tab is used for ASCOM Alpaca cameras. It contains the remote server address and port, together with status fields showing the current camera state, capture time, sensor details, pixel size, sensor size and CCD temperature.

<h3>Colour tab</h3>

The Colour tab controls the post-processing applied to the image. It includes brightness, contrast, saturation and gamma controls, white balance mode and manual red, green and blue gains.

It also provides Gaussian blur, median blur, sharpen and Sobel edge filters, together with horizontal and vertical flip options and a reset button for the colour settings.

<h3>Overlay tab</h3>

The Overlay tab contains controls for the spectrum overlay, the date and time overlay and an HTML text overlay.

For each overlay you can choose the content, colour, font, font size and screen position. The spectrum overlay also lets you choose the device set used as the spectrum source and the display scale.

<h3>Recording tab</h3>

The Recording tab contains the base filenames used for saved images and video, lets you choose whether recording stores raw images or post-processed images and allows hardware-accelerated video encoding to be enabled when supported by the selected video backend.

<h3>Detection tab</h3>

The Detection tab contains the settings for object detection, motion detection and image differencing.

Object detection uses YOLO ONNX models. You can select the model and labels files, set the confidence and non-maximum suppression thresholds, choose the bounding box colour and select the inference target. If a model or labels entry is an HTTP or HTTPS URL, the file is downloaded when selected and the local downloaded file is then used.

Motion detection provides background history, sensitivity, shadow detection, morphological open and close sizes, persistence, minimum detected area and bounding box colour settings.

Difference detection provides the threshold, dilation size, mask history and close size settings. The last N masks can be combined before being applied to the image.

The Detection ROI settings define an x, y, width and height sub-region to use for object detection, motion detection and image differencing. A width or height of 0 uses the full image in that dimension.

<h3>Actions tab</h3>

The Actions tab defines what happens when an object class is detected. You can choose the object class, set a disappear debounce time, add device set actions and configure per-device actions such as preset recalls, commands and speech. Changes on this tab are applied immediately.

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
- media player icons by Ranah Pixel Studio from Noun Project (CC BY 3.0)

<h1>Camera Plugin</h1>

<h2>Introduction</h2>

The Camera feature plugin allows SDRangel to capture images and video from cameras and telescopes.
This is to support multimode observations, such as combing radio and optical observations of meteors, but can also be used for observing remote radio equipment or conditions.

The Camera plugin supports images and video from:

* Qt6 Multimedia API (FFmpeg backend)
* Qt5 Multimedia API (DirectShow on Windows, GStreamer/V4L2 on Linux)
* ASCOM Alpaca API (including support for Filter Wheels and Focusers)
* ASI cameras (ASICamera2 only currently included in Windows builds)
* Video files such as MP4 (Qt6 only)

The Camera plugin also supports a variety of post-processing, detection and overlay features, such as:

* Image stacking with dark, flat and bias calibration frames
* HDR stacking with multiple exposure buckets and merging algorithms
* Histogram stretching and colour adjustment
* YOLO object detection
* Motion detection
* Star detection and plate solving
* Difference detection between images
* ADS-B, satellite and star tracker object overlay
* Date/time and custom HTML overlay
* Spectrum overlay from SDRangel's SDR devices
* Azimuth/elevation and right ascension/declination sky grid overlays

Raw and post-processed images can be saved as JPEG files, and video can be recorded in H264-encoded MP4 files.

The Camera feature can send events to the Scheduler feature when motion or specific YOLO object classes are detected, allowing you to automate actions such as recording from SDR devices, sending notifications or running custom commands. 
Recoding images and video can also be triggered via the Scheduler feature, allowing triggering based on time or RF events.

<h2>Interface</h2>

![Camera feature plugin GUI](../../../doc/img/Camera_plugin.png)

The Camera feature window contains two toolbars above the image display.

<h3>1: Start/Stop capture</h3>

Starts or stops image capture from the selected camera.

<h3>2: Camera</h3>

Selects the camera source. This list contains Qt Multimedia cameras, ASCOM Alpaca cameras and the file camera source when available.

<h3>3: Refresh cameras</h3>

Refreshes the camera list.

<h3>4: Settings</h3>

Opens the Camera Settings dialog.

<h3>5: Select video file</h3>

Selects the video file used by the file camera source.

<h3>6: Restart video</h3>

Restarts playback of the selected video file from the beginning.

<h3>7: Play/Pause video</h3>

Plays or pauses the selected video file.

<h3>8: Loop video</h3>

When checked, video file playback loops when the end of the file is reached.

<h3>9: Playback rate</h3>

Sets the video file playback speed. 1.0 is normal speed.

<h3>10: Playback position</h3>

Seeks within the selected video file.

<h3>11: Audio mute</h3>

Left click mutes or unmutes audio from the camera or video source. Right click opens audio output device selection.

<h3>12: Zoom in</h3>

Zooms in on the image display.

<h3>13: Zoom out</h3>

Zooms out from the image display.

<h3>14: Fit image in view</h3>

Resizes the image view so the current image fits in the available display area.

<h3>15: Save current image</h3>

Saves the currently displayed image to a JPEG file.

<h3>16: Save images</h3>

When checked, saves captured images using the image filename and record mode configured in the Recording tab.

<h3>17: Record video</h3>

When checked, records video using the video filename and record mode configured in the Recording tab.

<h3>18: Image stacking</h3>

Enables image stacking using the calibration and stacking settings in the Cal / Stack tab.

<h3>19: Invert colours</h3>

Inverts the displayed image colours.

<h3>20: Histogram</h3>

Opens the histogram window for the current image.

<h3>21: Difference mask</h3>

Enables display of differences from previous images using the Difference Detection settings.

<h3>22: Motion detection</h3>

Enables motion detection using the Motion Detection settings.

<h3>23: Star detection and plate solving</h3>

Enables star detection and plate solving using the Star Detection settings.

<h3>24: Object detection</h3>

Enables YOLO object detection using the Object Detection settings.

<h3>25: Detection history</h3>

Opens the object detection history window.

<h3>26: Tracked object overlay</h3>

Overlays ADS-B, satellite and star tracked objects on the camera image using the Position and Sky Grid settings.

<h3>27: Date/time overlay</h3>

Overlays the configured date and time string on the image.

<h3>28: HTML text overlay</h3>

Overlays the configured HTML text on the image.

<h3>29: Spectrum overlay</h3>

Overlays a spectrum view from a selected SDRangel device set on the image.

<h3>30: Azimuthal grid overlay</h3>

Overlays the azimuth/elevation sky grid.

<h3>31: Equatorial grid overlay</h3>

Overlays the right ascension/declination sky grid.

<h3>32: Constellation overlay</h3>

Overlays selected constellation stars.

<h3>33: Image display</h3>

Displays the captured image, including enabled post-processing, detection results and overlays.

<h3>Camera settings</h3>

Press the settings button to open the Camera Settings dialog. The dialog contains the following tabs.

<h3>Camera tab</h3>

The Camera tab contains the capture settings for the selected device.

<ul>
<li>Resolution selects the camera capture size.</li>
<li>Frame Rate / Interval selects whether the camera captures continuous video frames or still images at a timed interval.</li>
<li>Frame rate selects a supported frame rate, or enters a frame rate directly when the camera exposes a numeric control.</li>
<li>Interval and units set the still-image capture interval in seconds or minutes.</li>
<li>Exposure, exposure slider and exposure units set the camera exposure time.</li>
<li>ISO sets the camera ISO value. -1 selects automatic ISO when supported.</li>
<li>White balance selects the camera white balance mode.</li>
<li>Exp. comp. sets exposure compensation in EV steps.</li>
<li>Focus mode selects the camera focus mode.</li>
<li>Focus distance sets the normalized manual focus distance, where 0.0 is near and 1.0 is far or infinity.</li>
<li>Zoom sets the hardware optical zoom factor.</li>
<li>Gain mode, gain slider and gain value set camera gain when supported by the selected device.</li>
<li>Offset mode, offset slider and offset value set camera offset when supported by the selected device.</li>
<li>Bin X and Bin Y set sensor binning for Alpaca cameras.</li>
<li>Subframe Width and Subframe Height set the captured subframe size after binning. A value of 0 uses the full available image in that dimension.</li>
<li>Subframe X and Subframe Y set the subframe offset after binning.</li>
<li>Readout mode selects the Alpaca camera sensor readout mode.</li>
<li>Colour image type selects RGB24 camera-side colour conversion or RAW16 Bayer data debayered in SDRangel for ASI cameras.</li>
<li>Cooler on enables or disables the ASI camera cooler.</li>
<li>Target temperature sets the ASI cooler target temperature.</li>
<li>USB bandwidth sets the ASI USB bandwidth limit.</li>
<li>High speed mode enables the ASI high speed transfer mode.</li>
<li>Auto exposure/gain enables automatic ASI exposure and gain when available.</li>
<li>Focus position sets the Alpaca focuser position.</li>
<li>Focus step size sets the increment used when changing Alpaca focuser position.</li>
<li>Filter selects the Alpaca filter wheel position by name.</li>
</ul>

<h3>Status tab</h3>

The Status tab displays read-only information about the current camera and processing pipeline.

<ul>
<li>Camera name and description identify the selected camera.</li>
<li>Sensor name and sensor type describe the camera sensor when reported by the device.</li>
<li>Pixel size and camera size show sensor pixel and image dimensions.</li>
<li>Camera state, capture time and receive image format show the current Alpaca camera capture status.</li>
<li>CCD temperature shows the reported camera sensor temperature.</li>
<li>Alpaca error code and error message show the last Alpaca API error.</li>
<li>Pipeline FPS shows the measured image-processing frame rate.</li>
<li>Plate solve status, matches, detected stars, RMS, pointing, catalog, candidates, outliers and solution show the latest plate-solving result.</li>
<li>Clear chart clears the status chart.</li>
</ul>

<h3>Alpaca tab</h3>

The Alpaca tab configures ASCOM Alpaca discovery, API access and auxiliary devices.

<ul>
<li>Host and port set the Alpaca camera server address.</li>
<li>Discovery enables Alpaca discovery instead of using only the configured host and port.</li>
<li>API log enables logging of Alpaca API requests.</li>
<li>Focuser host and port set the Alpaca focuser server address.</li>
<li>Filter wheel host and port set the Alpaca filter wheel server address.</li>
<li>Focuser enable and focuser selector enable and select an Alpaca focuser.</li>
<li>Filter wheel enable and filter wheel selector enable and select an Alpaca filter wheel.</li>
</ul>

<h3>Cal / Stack tab</h3>

The Cal / Stack tab configures calibration frames and image stacking.

<ul>
<li>Dark file, Flat file and Bias file set calibration images used before stacking.</li>
<li>The browse buttons next to those fields select the corresponding calibration image files.</li>
<li>Plate solve catalog, candidates and outliers display catalog information from the latest plate solve.</li>
<li>Frame count sets how many frames are combined for one stacked output image.</li>
<li>Current count displays how many frames have been accumulated in the current stack.</li>
<li>Method selects the stacking method.</li>
<li>HDR exposure count selects how many exposure buckets are used for HDR stacking.</li>
<li>HDR algorithm selects Debevec, Robertson or Mertens merging for HDR stacking.</li>
<li>Alignment selects how frames are aligned before stacking.</li>
<li>HDR Exposure 1 through HDR Exposure 4 sliders and spin boxes set the exposure values used by HDR stacking.</li>
</ul>

<h3>Colour tab</h3>

The Colour tab controls image post-processing.

<ul>
<li>White Balance selects Off, Auto or Manual post-processing white balance.</li>
<li>Red Gain, Green Gain and Blue Gain sliders and spin boxes set manual white-balance gains.</li>
<li>Highlight Protection reduces manual white-balance gain in saturated highlights to keep bright white areas neutral.</li>
<li>Greyscale converts the image to greyscale after white balance.</li>
<li>Brightness, Contrast, Saturation and Gamma sliders and spin boxes adjust image tone and colour.</li>
<li>Histogram Stretch Mode selects Off, Linear, Gamma, Asinh, Log or CLAHE stretching.</li>
<li>Black Point and White Point set the stretch input range.</li>
<li>Histogram Gamma, Asinh Strength and Log Strength set the parameters for their respective stretch modes.</li>
<li>Edge Display selects whether detected edges are overlaid on the image or shown as edges only.</li>
<li>Gaussian Blur and Median Blur set blur strength.</li>
<li>Sharpen sets sharpening amount.</li>
<li>Sobel Edge and Canny Edge set edge detection amounts.</li>
<li>Flip X and Flip Y mirror the image horizontally or vertically.</li>
<li>Unwarp Fisheye unwarps the image using the configured lens projection and FoV.</li>
<li>Reset color settings restores colour, histogram, filter and image post-processing controls to their defaults.</li>
</ul>

<h3>Overlay tab</h3>

The Overlay tab controls spectrum, sky grid, tracked-object, date/time and HTML text overlays.

<ul>
<li>Spectrum Device Set selects the device set whose spectrum view is overlaid.</li>
<li>Spectrum Scale sets the spectrum overlay scale factor.</li>
<li>Spectrum Position X and Y sliders set the spectrum overlay offset in pixels, with readouts showing the current values.</li>
<li>Azimuthal grid, Equatorial grid, Constellation and ADS-B / satellite objects settings configure sky overlays that are enabled from the main toolbar.</li>
<li>Grid and constellation colour buttons select overlay colours.</li>
<li>Minimum elevation sets the lowest elevation for tracked-object overlays.</li>
<li>Constellation selects which constellation stars are displayed.</li>
<li>Track object colour, font scale, grid label font and grid label font scale configure tracked-object and grid labels.</li>
<li>Date/time Format sets the `QDateTime` format string used by the date/time overlay.</li>
<li>Date/time Font, font scale and colour set the date/time text appearance.</li>
<li>Date/time Position X and Y sliders set the date/time overlay position, with readouts showing the current values.</li>
<li>Text sets the HTML text to overlay.</li>
<li>Text Font, font scale and colour set the HTML text appearance.</li>
<li>Text Position X and Y sliders set the HTML text overlay position, with readouts showing the current values.</li>
</ul>

<h3>Recording tab</h3>

The Recording tab configures file output.

<ul>
<li>Image filename and browse button set the base filename used when saving images.</li>
<li>Video filename and browse button set the base filename used when recording video.</li>
<li>Record mode selects whether raw, processed or both raw and processed images/video are recorded.</li>
<li>Hardware acceleration enables hardware-accelerated video encoding when supported by the selected video backend.</li>
</ul>

<h3>Detection tab</h3>

The Detection tab contains a common Detection ROI section and sub-tabs for object, motion, star and difference detection.

<ul>
<li>Detection ROI X, Y, Width and Height define the sub-region used for object detection, motion detection and image differencing. A width or height of 0 uses the full image in that dimension.</li>
<li>Show ROI overlays the detection ROI on the image.</li>
<li>Draw ROI lets you draw the ROI on the image.</li>
<li>Delete ROI clears the ROI.</li>
<li>Exclusion area add, remove and show controls manage motion-detection exclusion rectangles.</li>
<li>The exclusion area table lists the configured exclusion rectangles.</li>
</ul>

On the Object Detection sub-tab:

<ul>
<li>YOLO ONNX model selects the object detection model. The browse button selects a local model file.</li>
<li>Labels selects the class labels file. The browse button selects a local labels file.</li>
<li>Confidence sets the minimum detection confidence.</li>
<li>NMS sets the non-maximum suppression threshold.</li>
<li>BBox Colour selects the object detection bounding box colour.</li>
<li>Target selects the inference target, such as CPU, OpenCL or CUDA when available.</li>
</ul>

On the Motion Detection sub-tab:

<ul>
<li>Subtractor selects the background subtractor, MOG2 or KNN.</li>
<li>Mask view selects an intermediate motion mask stage to display instead of the normal image.</li>
<li>History sets the background history length.</li>
<li>Sensitivity sets the foreground classification threshold.</li>
<li>Learning rate sets the MOG2 learning rate, with Auto for automatic learning.</li>
<li>Confirm frames sets how many consecutive frames motion must persist before it is reported.</li>
<li>Downscale selects an optional reduced-resolution detection path.</li>
<li>Detect shadows enables MOG2 shadow detection.</li>
<li>Open size and Close size set morphological clean-up kernel radii.</li>
<li>Persistence keeps motion boxes visible for a number of frames after motion disappears.</li>
<li>Min area sets the minimum contour area in pixels.</li>
<li>BBox Colour selects the motion bounding box colour.</li>
<li>Reset Defaults restores motion, star, difference, plate solve and ROI detection settings to their defaults.</li>
</ul>

On the Star Detection sub-tab:

<ul>
<li>Enable is controlled by the Star detection and plate solving button in the main toolbar.</li>
<li>Threshold, Background blur, Min area, Max area and Max aspect ratio set star candidate extraction.</li>
<li>Debug view displays intermediate star-detection images.</li>
<li>Colour selects the star overlay colour.</li>
<li>Plate solve labels selects whether solved stars are labelled by name, magnitude and spectral class.</li>
<li>Max magnitude, Min matches, Acquisition radius, Final match radius and Search radius set plate-solving constraints.</li>
<li>Start mode selects the initial information used by the plate solver, from blind solving to current camera settings only.</li>
<li>Use current date/time uses the computer clock for solving; when unchecked, Solve date/time sets the date and time manually.</li>
<li>Use HYG catalog uses the downloaded HYG bright-star catalog when available.</li>
<li>Download HYG catalog downloads and imports the HYG bright-star catalog.</li>
<li>Apply mode selects which solved pointing values are copied back to camera settings.</li>
<li>Apply solved pointing applies the latest solution using the selected apply mode.</li>
<li>The solution label displays the latest plate-solving solution or reports that no solution is available.</li>
</ul>

On the Difference Detection sub-tab:

<ul>
<li>Threshold sets the pixel difference threshold for the diff mask.</li>
<li>Dilation sets the diff mask dilation kernel size.</li>
<li>Open size sets the morphological open radius before mask accumulation.</li>
<li>Mask history sets how many diff masks are retained and combined.</li>
<li>Close size sets the morphological close radius after mask accumulation.</li>
</ul>

If a YOLO model or labels entry is an HTTP or HTTPS URL, the file is downloaded when selected and the local downloaded file is then used.

<h3>Actions tab</h3>

The Actions tab defines what happens when an object class is detected.

<ul>
<li>Object class selects the YOLO class whose actions are being configured.</li>
<li>Disappear debounce sets how long a class must stay absent before disappearance actions run.</li>
<li>Add device set adds device set control settings for the selected object class.</li>
<li>The device set tabs configure per-device actions such as preset recalls, commands and speech. Tabs can be closed to remove a device set action.</li>
<li>The status label reports action configuration status. Changes on this tab are applied immediately.</li>
</ul>

<h3>Position tab</h3>

The Position tab configures camera location, pointing, lens model and weather lookup.

<ul>
<li>Latitude, Longitude and Altitude set the camera position.</li>
<li>The My Position import button sets the camera position from SDRangel's My Position preferences. Right click toggles continual synchronization from My Position.</li>
<li>Rotator selects a GS232Controller feature used to continually synchronize camera azimuth and elevation.</li>
<li>Azimuth, Elevation and Roll set the camera pointing direction.</li>
<li>FoV sets the camera field of view in degrees.</li>
<li>Projection selects Rectilinear, Equidistant fisheye or Equisolid fisheye lens projection.</li>
<li>Center offset X and Center offset Y set the lens centre offset in pixels.</li>
<li>Distortion K1 sets the first radial lens distortion coefficient.</li>
<li>OpenWeatherMap API Key is used to periodically fetch weather for the camera latitude and longitude.</li>
</ul>

<h3>Close</h3>

Closes the Camera Settings dialog.

<h2>YOLO models</h2>

```
from ultralytics import YOLO
model = YOLO("yolov8n.pt")
model.export(format="onnx", opset=12)
```

COCO class list: https://raw.githubusercontent.com/amikelive/coco-labels/refs/heads/master/coco-labels-2014_2017.txt

<h2>Optional Prerequisites</h2>

ASCOM Platform:

https://ascom-standards.org/Downloads/Index.htm

Camera specific ASCOM drivers, such as for ZWO ASI cameras:

https://www.zwoastro.com/layouts/download-desktop-app/

Start ASCOM Remote Server.


ASCOM Alpaca Device API docs are at: https://ascom-standards.org/api/

If imageready is always false, try restarting the ASCOM Remote Server.

<h2>API</h2>

Full details of the API can be found in the Swagger documentation. Here is a quick example of how to set the camera and capture interval from the command line:

    curl -X PATCH "http://127.0.0.1:8091/sdrangel/featureset/feature/0/settings" -d '{"featureType": "Camera",  "CameraSettings": { "cameraDescription": "c922 Pro Stream Webcam", "captureInterval": 1.0 }}'

To start capturing:

    curl -X POST "http://127.0.0.1:8091/sdrangel/featureset/feature/0/run"

<h2>Attribution</h2>

- Coded using AI. Thanks to all the open source code and documentation that made this possible!
- Hipparcos Catalog by ESA (CC BY-NC 3.0 IGO)
- HYG Catalog by astronexus (CC-BY-SA 4.0)
- Object detection by Lars Meiertoberens from Noun Project (CC BY 3.0)
- html by wira wianda from Noun Project (CC BY 3.0)
- clock by Alv Jørgen Bovolden from Noun Project (CC BY 3.0)
- invert by Meko from Noun Project (CC BY 3.0)
- subtract-picture by Smashicons from Noun Project (CC BY 3.0)
- media player icons by Ranah Pixel Studio from Noun Project (CC BY 3.0)
- meteor by Color Combo from Noun Project (CC BY 3.0)
- stack image by I Putu Dicky Adi Pranatha from Noun Project (CC BY 3.0)
- constellation by BomSymbols from Noun Project (CC BY 3.0)
- constellation by verry poernomo from from Noun Project (CC BY 3.0)
- order by kumakamu from Noun Project (CC BY 3.0)

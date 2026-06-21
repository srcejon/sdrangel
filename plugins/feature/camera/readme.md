<h1>Camera Plugin</h1>

<h2>Introduction</h2>

The Camera feature plugin allows SDRangel to capture images and video from cameras and telescopes.
This is to support multimode observations, such as combing radio and optical observations of meteors, but can also be used for observing remote radio equipment or conditions.

The Camera plugin supports images and video from:

* Qt6 Multimedia API (FFmpeg backend, which uses DirectShow on Windows, V4L2 on Linux and avfoundation on macOS)
* Qt5 Multimedia API (DirectShow on Windows, GStreamer/V4L2 on Linux)
* ASCOM Alpaca API supported by some telescopes (including support for Filter Wheels and Focusers)
* ASI cameras (ASICamera2 only currently included in Windows builds)
* Video files such as MP4, MOV, AVI (via FFmpeg)
* Streaming protocols such as http: rtsp: rtmp: (via FFmpeg)
* Image files such as PNG, JPEG and FITS.

The Camera plugin also supports a variety of post-processing, detection and overlay features, such as:

* Dark, flat and bias calibration frames
* Image stacking with alignment and quality rejection
* HDR stacking with multiple exposure brackets and merging algorithms
* Histogram stretching and colour adjustment
* YOLO AI object detection (CPU, OpenCV CUDA or TensorRT acceleration)
* Motion detection
* Star detection and plate solving
* Difference detection between images
* ADS-B, AIS, satellite and star tracker item overlay
* Date/time and custom HTML overlay
* Spectrum overlay from SDRangel's SDR devices
* Azimuth/elevation and right ascension/declination sky grid overlays
* Generation of 24-hour keograms
* CUDA acceleration is supported for most operations and the processing runs in multiple threads.

Raw, calibrated, filtered or post-processed images can be saved as JPEG, PNG or FITS files, and video can be recorded in H264/H265 encoded MP4 files. Video can also be streamed to YouTube Live via RTMP.

The Camera feature can send events to the Scheduler feature when motion or YOLO object classes are detected, allowing you to automate actions such as recording from SDR devices, sending notifications or running custom commands. 
Recoding images and video can also be triggered via the Scheduler feature, allowing triggering based on time or RF events.

<h2>Interface</h2>

![Camera feature plugin GUI](../../../doc/img/Camera_plugin.png)

The Camera feature window contains two toolbars above the image display. Each toolbar control is described below.

<h3>1: Start/Stop capture</h3>

Starts or stops image capture from the selected camera.

<h3>2: Camera</h3>

Selects the camera source. A prefix followed by a colon indicates the underlying API used to access the camera:

* qt: Qt Multimedia camera source for most webcams. For Qt6 this uses the FFmpeg backend (which itself uses DirectShow on Windows, V4L2 on Linux and avfoundation on macOS), while for Qt5 it directly uses GStreamer/V4L2 on Linux.
* alpaca: ASCOM Alpaca camera source, for smart telescopes such as Seestar or Dwarf and many others.
* asi: ASI camera source, which uses the ASICamera2 library for ZWO ASI cameras.
* video: Video file source, which can read from MP4, MOV or AVI files.
* image: Image file source, which can read from PNG, JPEG or FITS files.
* stream: Streaming camera source, which can read from RTSP, RTMP or HTTP streams.

<h3>3: Refresh cameras</h3>

Refreshes the camera list.

<h3>4: Settings</h3>

Opens the Camera Settings dialog, which contains the camera capture and post-processing settings described later in this section.

<h3>5: Select video file</h3>

Selects the video file used by the file (video:) camera source.

<h3>6: Restart video</h3>

Restarts playback of the selected video file from the beginning.

<h3>7: Step back one frame</h3>

Steps the video file playback back one frame.

<h3>8: Step forward one frame</h3>

Steps the video file playback forward one frame.

<h3>9: Play/Pause video</h3>

Plays or pauses playback of the selected video file.

<h3>10: Loop video</h3>

When checked, video file playback loops back to the start when the end of the file is reached.

<h3>11: Playback rate</h3>

Sets the video file playback speed. 1.0 is normal speed.

<h3>12: Playback position</h3>

Seeks within the selected video file. The label to the right of the slider shows the current playback position.

<h3>13: Audio mute</h3>

Left click mutes or unmutes audio from the camera or video source. Right click opens audio output device selection.

<h3>14: Zoom in</h3>

Zooms in on the image display.

<h3>15: Zoom out</h3>

Zooms out from the image display.

<h3>16: Fit image in view</h3>

Resizes the image view so the current image fits in the available display area.

<h3>17: Save current image</h3>

Saves the currently displayed image to a JPEG or PNG file.

<h3>18: Save images</h3>

When checked, saves captured images using the image basename and record mode configured in the Recording tab in the Camera Settings dialog.

<h3>19: Record video</h3>

When checked, records video using the video basename and record mode configured in the Recording tab in the Camera Settings dialog.

<h3>20: Generate keogram</h3>

When checked, accumulates a keogram (a long-duration strip image built from one column or row per sample) using the Keogram settings in the Recording tab.

<h3>21: Stream to YouTube Live</h3>

When checked, streams the captured video to YouTube Live. The YouTube Live URL and stream key need to be set in the Recording tab in the Camera Settings dialog.

<h3>22: Image stacking</h3>

Enables image stacking using the calibration and stacking settings in the Cal / Stack tab in the Camera Settings dialog.

<h3>23: Invert colours</h3>

Inverts the displayed image colours.

<h3>24: Histogram</h3>

Opens the histogram window for the current image.

<h3>25: Object detection</h3>

Enables YOLO object detection. A YOLO ONNX model to use must be set in the Object Detection sub-tab in the Camera Settings dialog.

<h3>26: Object detection history</h3>

Opens the YOLO object detection history dialog.

<h3>27: Motion detection</h3>

Enables motion detection using the Motion Detection settings in the Camera Settings dialog.

<h3>28: Difference mask</h3>

Enables display of differences from previous images using the Difference Detection settings.

<h3>29: Star detection and plate solving</h3>

Enables star detection and plate solving using the Star Detection settings.

<h3>30: Item overlay</h3>

Overlays ADS-B, AIS, satellite and star tracker tracked items on the camera image using the Position and ADS-B / Satellite / Star Tracker overlay settings.

<h3>31: Date/time overlay</h3>

Overlays the configured date and time string on the image.

<h3>32: HTML overlay</h3>

Overlays the configured HTML on the image.

<h3>33: Spectrum overlay</h3>

Overlays a spectrum view from a selected SDRangel device set on the image.

<h3>34: Azimuthal grid overlay</h3>

Overlays the azimuth/elevation sky grid.

<h3>35: Equatorial grid overlay</h3>

Overlays the right ascension/declination sky grid.

<h3>36: Constellation overlay</h3>

Overlays the selected constellation stars. This can be used to help determine the camera's pose.

<h3>37: Image display</h3>

Displays the captured image, including enabled post-processing, detection results and overlays.

<h2>Camera Settings dialog</h2>

Press the Settings button (4) to open the Camera Settings dialog. The dialog contains the tabs described below. Each setting is given a numbered heading. The Close button at the bottom of the dialog closes it.

<h3>Camera tab</h3>

The Camera tab contains the capture settings for the selected camera. The settings displayed will vary according to the camera and which capabilities it reports.

![Camera tab](../../../doc/img/Camera_plugin_camera_tab.png)

<h4>1. Resolution</h4>

Selects the camera capture size in pixels.

<h4>2. Frame rate / Interval mode</h4>

Selects whether the camera captures continuous video frames or still images at a timed interval.

<h4>3. Frame rate</h4>

Selects a supported frame rate from a list, or enters a frame rate in frames per second directly when the camera exposes a numeric control.

<h4>4. Interval</h4>

When in interval mode, sets the time between still-image captures. The adjacent units combo selects seconds or minutes.

<h4>5. Exposure</h4>

Sets the camera exposure time using the slider and spin box. The adjacent units combo selects the exposure time units.

<h4>6. ISO</h4>

Sets the camera ISO value. -1 selects automatic ISO when supported.

<h4>7. Exp. comp. (EV)</h4>

Sets exposure compensation in EV steps, from -2.0 to +2.0.

<h4>8. White balance</h4>

Selects the camera white balance mode.

<h4>9. Zoom</h4>

Sets the hardware optical zoom factor.

<h4>10. Focus mode</h4>

Selects the camera focus mode.

<h4>11. Focus distance</h4>

Sets the normalised manual focus distance, where 0.0 is near and 1.0 is far or infinity.

<h4>12. Gain</h4>

Sets the camera gain using the mode combo, slider and spin box, when supported by the selected device.

<h4>13. Offset</h4>

Sets the camera offset using the mode combo, slider and spin box, when supported by the selected device.

<h4>14. Bin X / Bin Y</h4>

Set the sensor binning in the X and Y directions for Alpaca cameras.

<h4>15. Subframe Width / Subframe Height</h4>

Set the captured subframe size after binning. A value of 0 uses the full available image in that dimension.

<h4>16. Subframe X / Subframe Y</h4>

Set the subframe offset after binning.

<h4>17. Readout mode</h4>

Selects the Alpaca camera sensor readout mode.

<h4>18. Colour image type</h4>

Selects RGB24 camera-side colour conversion, RAW16 16-bit Bayer data or RAW8 8-bit Bayer data debayered in SDRangel, for ASI cameras.

<h4>19. Cooler</h4>

Enables or disables the ASI camera cooler.

<h4>20. Target temp</h4>

Sets the ASI cooler target temperature.

<h4>21. USB bandwidth</h4>

Sets the ASI USB bandwidth limit as a percentage.

<h4>22. High speed mode</h4>

Enables the ASI high speed transfer mode.

<h4>23. Auto exp/gain</h4>

Selects no auto exposure/gain, software control, or camera hardware control where available. The adjacent S/W mode combo chooses whether software auto exposure adjusts exposure or gain first.

<h4>24. Target brightness / Percentile</h4>

Target brightness sets the brightness target for the measured percentile, and Percentile sets the brightness percentile used for the software auto-exposure measurement.

<h4>25. Min exposure / Max exposure</h4>

Set the minimum and maximum exposure time used by software auto-exposure.

<h4>26. Min gain / Max gain</h4>

Set the minimum and maximum gain used by software auto-exposure.

<h4>27. Max change</h4>

Sets the maximum exposure or gain change applied after each measured frame.

<h4>28. Focus position</h4>

Sets the Alpaca focuser position.

<h4>29. Focus step size</h4>

Sets the increment used when changing the Alpaca focuser position.

<h4>30. Auto focus</h4>

The Start button runs a software autofocus sweep using image sharpness. Capture must be running. The adjacent label shows the autofocus status.

<h4>31. Filter</h4>

Selects the Alpaca filter wheel position by name.

<h3>Status tab</h3>

The Status tab displays read-only information about the current camera and processing pipeline, and a chart of camera sensor temperature.

![Status tab](../../../doc/img/Camera_plugin_status_tab.png)

<h4>1. Name / Description</h4>

Identify the selected camera.

<h4>2. Sensor / Sensor type</h4>

Describe the camera sensor when reported by the device.

<h4>3. Pixel size / Sensor size</h4>

Show the sensor pixel size in microns and the image dimensions in pixels.

<h4>4. Camera state</h4>

Shows the current Alpaca camera capture state.

<h4>5. Capture time</h4>

Shows the last capture time in milliseconds.

<h4>6. Receive image format</h4>

Shows the format of the most recently received image.

<h4>7. CCD temp</h4>

Shows the reported camera sensor temperature in degrees Celsius.

<h4>8. Last error code / Last error message</h4>

Show the last Alpaca API error code and message.

<h4>9. Pipeline FPS</h4>

Shows the measured image-processing frame rate.

<h4>10. Clear chart</h4>

Clears the camera temperature chart.

<h3>Alpaca tab</h3>

The Alpaca tab configures ASCOM Alpaca discovery, API access and auxiliary devices.

![Alpaca tab](../../../doc/img/Camera_plugin_alpaca_tab.png)

<h4>1. Discover Alpaca servers</h4>

When checked, discovers Alpaca servers on the local network and populates the camera list from the responses, instead of using only the configured host and port.

<h4>2. Camera address</h4>

Sets the IP address / hostname and port of the ASCOM Remote Server for the camera.

<h4>3. Focuser address</h4>

Sets the IP address / hostname and port of the Alpaca focuser server.

<h4>4. Filter wheel address</h4>

Sets the IP address / hostname and port of the Alpaca filter wheel server.

<h4>5. Log Alpaca API</h4>

When checked, logs Alpaca API requests and responses using qDebug.

<h4>6. Use focuser</h4>

Enables an Alpaca focuser, and the adjacent combo selects one of the discovered focusers.

<h4>7. Use filter wheel</h4>

Enables an Alpaca filter wheel, and the adjacent combo selects one of the discovered filter wheels.

<h3>Cal / Stack tab</h3>

The Cal / Stack tab configures calibration frames and image stacking.

![Cal/Stack tab](../../../doc/img/Camera_plugin_cal_stack_tab.png)

<h4>1. Dark FITS / Flat FITS / Bias FITS</h4>

Set the optional FITS master dark, flat and bias frames used for pre-stacking calibration. The browse button next to each field selects the corresponding FITS file.

<h4>2. Method</h4>

Selects the stacking method used to combine frames: Average, Median, Sigma clipped average, Lucky sharp average or HDR.

<h4>3. HDR algorithm</h4>

Selects the algorithm used to merge bracketed HDR frames: Debevec, Robertson or Mertens.

<h4>4. Frames</h4>

Sets how many frames are combined into one stacked output image.

<h4>5. HDR frames</h4>

Selects how many exposure buckets (2 to 4) are used for HDR stacking.

<h4>6. Alignment</h4>

Selects optional frame alignment before stacking: None, Phase correlation (translation only) or Star centroid matching (which can also compensate for small rotation/scale drift).

<h4>7. Exposure 1 to Exposure 4</h4>

The HDR exposure sliders and spin boxes set the exposure values in milliseconds used by HDR stacking.

<h4>8. Display</h4>

Selects what is displayed: the stacked image output, one stored stack history frame, or a tiled overview of stored frames.

<h4>9. History frame</h4>

Selects the stack history frame to display or delete. The Delete button removes the selected stack history frame.

<h4>10. Quality reject</h4>

When Reject bad frames is checked, obvious bad stack frames are rejected using brightness, saturation, contrast and sharpness checks.

<h4>11. Stacked / queued / dropped / rejected</h4>

Displays counts of frames that have been stacked, queued, dropped and rejected in the current stack.

<h3>Colour tab</h3>

The Colour tab controls image colour adjustment and histogram stretching.

![Colour tab](../../../doc/img/Camera_plugin_colour_tab.png)

<h4>1. White Balance</h4>

Selects Off, Auto or Manual post-processing white balance.

<h4>2. Red Gain / Green Gain / Blue Gain</h4>

Sliders and spin boxes that set the manual white-balance gains.

<h4>3. Highlight Protection</h4>

Reduces manual white-balance gain in saturated highlights to keep bright white areas neutral.

<h4>4. Brightness / Contrast / Saturation / Gamma</h4>

Sliders and spin boxes that adjust image tone and colour.

<h4>5. Greyscale</h4>

Converts the image to greyscale after white balance.

<h4>6. Histogram Stretch Mode</h4>

Selects Off, Linear, Gamma, Asinh, Log or CLAHE stretching.

<h4>7. Black Point / White Point</h4>

Set the stretch input range.

<h4>8. Gamma / Asinh Strength / Log Strength</h4>

Set the parameters for the Gamma, Asinh and Log stretch modes respectively.

<h4>9. Reset color settings</h4>

Restores the colour and histogram stretch controls to their default values.

<h3>Filter tab</h3>

The Filter tab controls image filtering and geometric transformation.

![Filter tab](../../../doc/img/Camera_plugin_filter_tab.png)

<h4>1. Gaussian Blur / Median Blur</h4>

Sliders and spin boxes that set the blur strength. 0 disables the blur.

<h4>2. Sharpen</h4>

Sets the image sharpening amount.

<h4>3. Sobel Edge / Canny Edge</h4>

Set the Sobel and Canny edge detection blend amounts.

<h4>4. Edge Display</h4>

Selects whether detected edges are overlaid on the image or shown as edges only.

<h4>5. Flip X / Flip Y</h4>

Mirror the image horizontally or vertically.

<h4>6. Rotation</h4>

Rotates the image clockwise by 0, 90, 180 or 270 degrees.

<h4>7. Unwarp Fisheye</h4>

Unwarps the image from the configured fisheye lens projection using the camera FoV setting in the Position tab.

<h4>8. CUDA</h4>

When checked, uses OpenCV CUDA functions for supported camera processing steps.

<h3>Overlay tab</h3>

The Overlay tab controls the spectrum, sky grid, tracked-object, date/time and HTML text overlays.

![Overlay tab](../../../doc/img/Camera_plugin_overlay_tab.png)

<h4>1. Spectrum Device Set</h4>

Selects the device set whose spectrum view is overlaid.

<h4>2. Spectrum Position</h4>

The X and Y sliders set the spectrum overlay offset in pixels, with readouts showing the current values.

<h4>3. Spectrum Scale</h4>

Sets the spectrum overlay scale factor.

<h4>4. Equatorial grid / Azimuthal grid</h4>

The colour buttons select the colours used for the right ascension/declination and azimuth/elevation grids. The grids are enabled from the main toolbar.

<h4>5. Grid label font</h4>

The font combo and font scale spin box set the font and point size used for sky grid labels.

<h4>6. Constellation</h4>

Selects which constellation major stars to overlay (Ursa Major, Orion or Crux), and the colour button selects the constellation overlay colour.

<h4>7. Min elevation</h4>

Sets the lowest elevation in degrees for tracked-object overlays.

<h4>8. Track object font / colour</h4>

The font scale spin box and colour button set the tracked-object label point size and colour.

<h4>9. Show tracks / Heat map</h4>

Show tracks displays the recent track for each tracked overlay object, and Heat map shows a heat map built from recent tracked object positions. The adjacent button clears the heat map.

<h4>10. Date/time Format</h4>

Sets the `QDateTime` format string (e.g. `yyyy-MM-dd hh:mm:ss`) used by the date/time overlay.

<h4>11. Date/time Font / colour</h4>

The font combo, font scale spin box and colour button set the date/time text appearance.

<h4>12. Date/time Position</h4>

The X and Y sliders set the date/time overlay position in pixels, with readouts showing the current values. A Y value of 0 auto-positions the text at the bottom.

<h4>13. HTML</h4>

Sets the HTML text to render on the image. Use `src=file:///path/to/images/logo.png` for img tags; http is not supported.

<h4>14. Text Font / colour</h4>

The font combo, font scale spin box and colour button set the HTML text appearance.

<h4>15. Text Position</h4>

The X and Y sliders set the HTML text overlay position in pixels, with readouts showing the current values. A Y value of 0 auto-positions the text at the bottom.

<h3>Recording tab</h3>

The Recording tab configures image and video file output, video file playback, keograms and YouTube Live streaming.

![Recording tab](../../../doc/img/Camera_plugin_recording_tab.png)

<h4>1. Image basename</h4>

Sets the file basename used when saving images. The browse button selects the basename using a file dialog.

<h4>2. Video basename</h4>

Sets the file basename used when recording video. The browse button selects the basename using a file dialog.

<h4>3. Record</h4>

Selects which outputs are recorded: Raw FITS (uncalibrated Bayer still images), Calibrated (calibrated/debayered media without overlays), Filtered (filtered media after image processing, before overlays) and Post-processed (media with overlays).

<h4>4. Image limit</h4>

Number of image frames to save before image recording is automatically disabled; 0 (Unlimited) records until disabled manually.

<h4>5. Video limit</h4>

Seconds of video to record before video recording is automatically disabled; 0 (Unlimited) records until disabled manually.

<h4>6. Pre-record buffer</h4>

Seconds of live video frames to keep before recording starts, so that recordings include lead-in footage. The adjacent label shows the estimated memory used.

<h4>7. H/W acceleration</h4>

When checked, prefers hardware-accelerated video encoding when supported by the backend.

<h4>8. Codec</h4>

Selects the codec used for saved video recordings: H.264 or H.265.

<h4>9. Bitrate</h4>

Sets the saved video bitrate. Select Auto to choose from frame size and rate, select a preset, or enter a custom value such as 8000 Kbps or 8 Mbps.

<h4>10. Stream buffering</h4>

Sets the amount of decoded media buffered for stream: sources before playback starts or resumes after a stall.

<h4>11. Audio sync</h4>

Adjusts audio timing for video file playback, in milliseconds. A negative value delays the video, a positive value delays the audio.

<h4>12. Keogram File basename</h4>

Sets the file basename used for saved keograms (.jpg or .png). The browse button selects the basename using a file dialog.

<h4>13. Keogram Direction</h4>

Selects whether the keogram is built Horizontally or Vertically. The Preview check box shows a live keogram preview window.

<h4>14. Keogram 24h window</h4>

Selects the 24-hour window, Midnight to midnight or Midday to midday. The Sample period spin box sets the number of minutes between keogram samples.

<h4>15. YouTube URL / Key</h4>

Set the YouTube RTMP or RTMPS ingest URL and the stream key used for YouTube Live streaming.

<h4>16. YouTube Source</h4>

Selects whether YouTube receives Calibrated frames or Post-processed frames including overlays.

<h4>17. YouTube Bitrate / FPS / Size</h4>

Set the YouTube stream video bitrate (preset or custom value), the stream frame rate, and the output width and height in pixels (0 uses the source size).

<h3>Detection tab</h3>

The Detection tab contains a common ROI sub-tab and sub-tabs for object, motion, star and difference detection. The Reset Defaults button at the bottom of the tab restores all detection settings to their defaults.

On the ROI sub-tab:

![Detection ROI tab](../../../doc/img/Camera_plugin_detection_roi_tab.png)

<h4>1. ROI X / ROI Y / ROI Width / ROI Height</h4>

Define the sub-region used for object detection, motion detection and image differencing. A width or height of 0 uses the full image in that dimension.

<h4>2. Show</h4>

Overlays the detection ROI on the preview image.

<h4>3. Draw</h4>

Lets you draw the detection ROI on the preview image.

<h4>4. Delete</h4>

Deletes the detection ROI and resets its bounds to 0.

<h4>5. Exclusion areas</h4>

The exclusion area table lists the full-resolution exclusion rectangles used for detection. The Add and Remove buttons add and remove exclusion rectangles.

On the Object Detection sub-tab:

![Object Detection tab](../../../doc/img/Camera_plugin_object_detection_tab.png)

<h4>1. YOLO ONNX model</h4>

Selects the object detection model from the list of preset URLs or an entered path. The browse button selects a local model file.

<h4>2. Labels</h4>

Selects the class labels file (one name per line). The browse button selects a local labels file.

<h4>3. Confidence</h4>

Sets the minimum detection confidence threshold.

<h4>4. NMS</h4>

Sets the non-maximum suppression IoU threshold.

<h4>5. BBox Colour</h4>

Selects the bounding box colour for detections.

<h4>6. Target</h4>

Selects the inference target: OpenCV CPU, OpenCV CUDA, OpenCV CUDA FP16, TensorRT or TensorRT FP16, depending on what is available.

<h4>7. Tile large images</h4>

When checked, runs object detection on overlapping YOLO-sized tiles when the input image is larger than the model input.

<h4>8. Tile overlap</h4>

Sets the percentage overlap between adjacent YOLO tiles.

<h4>9. Ignored classes</h4>

Object class names to ignore, one per line. Matching detections are not drawn or recorded in the object history.

On the Motion Detection sub-tab:

![Motion Detection tab](../../../doc/img/Camera_plugin_motion_detection_tab.png)

<h4>1. Subtractor</h4>

Selects the background subtractor used for motion detection: MOG2 or KNN.

<h4>2. Learning rate</h4>

Sets the MOG2 learning rate, with Auto for automatic learning.

<h4>3. History</h4>

Sets the background history length.

<h4>4. Sensitivity</h4>

Sets the variance threshold for foreground classification.

<h4>5. Downscale</h4>

Selects an optional reduced-resolution detection path (100%, 50% or 25%).

<h4>6. Detect shadows</h4>

Enables MOG2 shadow detection.

<h4>7. Open size / Close size</h4>

Set the morphological clean-up kernel radii applied to the motion mask. 0 disables.

<h4>8. Min area</h4>

Sets the minimum contour area in square pixels.

<h4>9. Confirm frames</h4>

Sets how many consecutive frames motion must persist before it is reported.

<h4>10. Persistence</h4>

Keeps the last motion boxes visible for this many frames after motion disappears.

<h4>11. BBox Colour</h4>

Selects the motion bounding box colour.

<h4>12. Debug view</h4>

Selects an intermediate motion mask stage (Raw, Thresholded, Opened, Closed or Final) to display instead of the normal image.

On the Star Detection sub-tab:

![Star Detection tab](../../../doc/img/Camera_plugin_star_detection_tab.png)

<h4>1. Threshold / Background blur / Min area / Max area / Max aspect ratio</h4>

Set the star candidate extraction parameters.

<h4>2. Debug view</h4>

Displays an intermediate star-detection image (Background, Residual, Thresholded or Final).

<h4>3. Start mode</h4>

Selects the initial information used by the plate solver, from Blind solving through to using current camera settings only.

<h4>4. Az/El search radius</h4>

Sets the azimuth/elevation search radius in degrees.

<h4>5. FoV tolerance</h4>

Sets how well the field of view is known, as a percentage of the FoV. 0 pins to the entered FoV; larger values let the solver refine the FoV, for un-calibrated wide/fisheye lenses.

<h4>6. Min matches</h4>

Sets the minimum number of star matches required for a solution.

<h4>7. Max magnitude</h4>

Sets the faintest catalog star magnitude used for solving.

<h4>8. Acquisition radius / Final match radius</h4>

Set the star match radii in pixels used during acquisition and final matching.

<h4>9. Catalog</h4>

Selects the star catalog used for plate solving (Auto, HYG local/bundled, Siril Gaia DR3 SPCC online or Siril Gaia DR3 Astrometric local). The Download button downloads the selected catalog(s).

<h4>10. Date/time</h4>

Selects whether the solve date/time uses the capture time or a Custom value. When Custom, the date/time editor sets the value, the UTC button treats it as UTC, and the recycle button sets it to now.

<h4>11. Labels</h4>

Selects whether solved stars are labelled by Name, Name + magnitude or Name + magnitude + spectral class.

<h4>12. Colour</h4>

Selects the star overlay colour.

<h4>13. Cache size</h4>

Sets the maximum size of the star catalog region disk cache in GB; 0 disables pruning.

<h4>14. Solution</h4>

The Solution group displays the latest plate-solving result: detected stars, matched stars, RMS / max error and pointing. The Apply button applies the last solution to the camera settings, and the adjacent combo selects which solved pointing values (Az/El, Az/El/Roll, Az/El/Roll/FoV or Az/El/Roll/FoV/Lens) are applied.

On the Difference Detection sub-tab:

![Difference Detection tab](../../../doc/img/Camera_plugin_difference_detection_tab.png)

<h4>1. Dilation</h4>

Sets the diff mask dilation kernel size.

<h4>2. Open size</h4>

Sets the morphological open radius before mask accumulation. 0 disables.

<h4>3. Close size</h4>

Sets the morphological close radius after mask accumulation. 0 disables.

<h4>4. Threshold</h4>

Sets the pixel difference threshold for the diff mask.

<h4>5. Mask history</h4>

Sets how many diff masks are retained and combined with a bitwise OR.

If a YOLO model or labels entry is an HTTP or HTTPS URL, the file is downloaded when selected and the local downloaded file is then used.

<h3>Position tab</h3>

The Position tab configures the camera location, pointing, lens model and weather lookup.

![Position tab](../../../doc/img/Camera_plugin_position_tab.png)

<h4>1. Latitude / Longitude / Altitude</h4>

Set the camera position.

<h4>2. My Position import</h4>

Left click sets the camera position from SDRangel's My Position preferences. Right click toggles continual synchronization from My Position.

<h4>3. Rotator</h4>

Selects a GS232Controller feature used to continually synchronize the camera azimuth and elevation.

<h4>4. Azimuth / Elevation / Roll</h4>

Set the camera pointing direction.

<h4>5. FoV setting</h4>

Selects Manual FoV entry or calculation of the FoV from sensor width, sensor height and focal length.

<h4>6. FoV</h4>

Sets the field of view in degrees, when FoV setting is Manual.

<h4>7. Sensor width / Sensor height / Focal length</h4>

Set the sensor dimensions and focal length in millimetres used to calculate the FoV when FoV setting is Sensor/focal length.

<h4>8. Projection</h4>

Selects the lens projection: Rectilinear, Equidistant fisheye or Equisolid fisheye.

<h4>9. Center offset X / Center offset Y</h4>

Set the lens centre offset in pixels.

<h4>10. Distortion K1</h4>

Sets the first radial lens distortion coefficient.

<h4>11. OpenWeatherMap API Key</h4>

API key from openweathermap.org used to periodically fetch weather for the camera latitude and longitude.

<h2>YOLO models</h2>

```
from ultralytics import YOLO
model = YOLO("yolov8n.pt")
model.export(format="onnx", opset=12)
```

COCO class list: https://raw.githubusercontent.com/amikelive/coco-labels/refs/heads/master/coco-labels-2014_2017.txt

<h2>Optional Prerequisites</h2>

<h3>ASCOM Platform</h3>

The ASCOM Platform is required for Alpaca telescope and camera support. It can be downloaded from:

https://ascom-standards.org/Downloads/Index.htm

Camera specific ASCOM drivers, such as for ZWO ASI cameras:

https://www.zwoastro.com/layouts/download-desktop-app/

Start ASCOM Remote Server.

ASCOM Alpaca Device API docs are at: https://ascom-standards.org/api/

If imageready is always false, try restarting the ASCOM Remote Server.

If using a Seestar telescope, connect to the telescope using the Seestar app, before trying to control it with SDRangel.

<h2>API</h2>

Full details of the API can be found in the Swagger documentation. Here is a quick example of how to set the camera and capture interval from the command line:

    curl -X PATCH "http://127.0.0.1:8091/sdrangel/featureset/feature/0/settings" -d '{"featureType": "Camera",  "CameraSettings": { "cameraDescription": "c922 Pro Stream Webcam", "captureInterval": 1.0 }}'

To start capturing:

    curl -X POST "http://127.0.0.1:8091/sdrangel/featureset/feature/0/run"

<h2>Dev notes</h2>

To stream an mp4 via http:

    ffmpeg -re -i test.mp4 -c copy -f mpegts -listen 1 "http://127.0.0.1:8080"
    http://127.0.0.1:8080

TCP/MPEG-TS:

    ffmpeg -re -i test.mp4 -c copy -f mpegts -listen 1 tcp://127.0.0.1:8080
    tcp://127.0.0.1:8080

UDP/MPEG-TS:

    ffmpeg -re -i test.mp4 -c copy -f mpegts "udp://127.0.0.1:1234?pkt_size=1316"
    udp://@127.0.0.1:1234?buffer_size=67108864

RTP/MPEG-TS:

    ffmpeg -re -i test.mp4 -c copy -f rtp_mpegts "rtp://127.0.0.1:1234"
    rtp://127.0.0.1:1234?buffer_size=67108864&reorder_queue_size=5000&max_delay=500000

RTSP (struggles with very high bitrate):
    
    in mediamtx.yml set writeQueueSize: 16384
    mediamtx 
    ffmpeg -re -i test.mp4 -c copy -rtsp_transport tcp -f rtsp rtsp://127.0.0.1:8554/live
    rtsp://127.0.0.1:8554/live

RTMP:
    
    mediamtx 
    ffmpeg -re -i test.mp4 -c copy -f flv rtmp://127.0.0.1:1935/live
    rtmp://127.0.0.1:1935/live

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
- youtube by Kholila wale from Noun Project (CC BY 3.0)

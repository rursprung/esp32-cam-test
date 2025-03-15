# Example Application for ESP32-CAM

First of all: do *NOT* use this for anything productive, this is just a prototype! You're probably better off using the
[`CameraWebServer` Arduino example].

This repository needs a lot of cleanup and - hopefully - standardisation / moving things to other crates before this
can serve as a reasonable example for anything.

This is based on [Dominaezzz/esp32-mjpeg] which targets the ESP32S3. In an optimal world there would be some traits
implemented for both devices and an application can be agnostic of them.

[`CameraWebServer` Arduino example]: https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer
[Dominaezzz/esp32-mjpeg]: https://github.com/Dominaezzz/esp32-mjpeg

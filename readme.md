nrf52-ble-image-transfer-demo
=============================
This demo uses a camera sensor to capture a JPG image, and send it over BLE to a phone app. 
The image can be captured either in a single shot mode, or in a streaming mode where it will take pictures as fast as the BLE link can keep up. 

The resolution can be changed in 6 steps between 160x120 to 1600x1200, and an estimate of the transfer speed can be calculated based on the time it takes to transfer each image. 

The example is set up to request different BLE phy's, and can be used to demonstrate the difference between 1Mbps and 2Mbps BLE modes on phones that support it. 

The Android companion app can be found here:     
[https://github.com/NordicSemiconductor/Android-Image-Transfer-Demo](https://github.com/NordicSemiconductor/Android-Image-Transfer-Demo)

Requirements
------------
- nRF5 SDK version 13.0.0
- nRF52840 DK (PCA10056)
- Arducam Mini 2MP camera module (OV2640)
	- [https://www.amazon.com/Arducam-Module-Megapixels-Arduino-Mega2560/dp/B012UXNDOY](https://www.amazon.com/Arducam-Module-Megapixels-Arduino-Mega2560/dp/B012UXNDOY "Amazon link")

TODO
----
- Add support for the nRF52832 and future boards in the nRF52 family
- Update IAR and GCC project/make files
- Create a proper setup guide for configuring the hardware, on the 52840 and 52832 kits

Note
----

The project may need modifications to work with other versions or other boards. 

To compile it, clone the repository in the [SDK]/examples/ble_peripheral folder.

About this project
------------------
This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty. 

However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub. 

The application is built to be used with the official nRF5 SDK, that can be downloaded from developer.nordicsemi.com

Please post any questions about this project on [https://devzone.nordicsemi.com](https://devzone.nordicsemi.com)
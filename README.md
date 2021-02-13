# Body finder

A device prototype to find your buddies (or their bodies) when hiking or skiing.
In the most basic case you have 2 identical devices that track their GPS coordinates, then they transmit and recieve  over radio their positions.
On the screens devices show direction and distance to each other making finding your buddies easy and fun.

Powered by Arduino, GPS and Long Range radio transmission.

## Hardware

### Arduino board

The board is ASR6502 ([CubeCell GPS HTCC-AB02S](https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/dev-board/htcc-ab02s/hardware_update_log.html#v1)).

Amazon sells it as [MakerFocus LoRa GPS Module LoRaWAN 868 915mHz Development Board LoRa Kit Ultra Low Power Design CP2102 SX1262 ASR6502 Chip with 0.96 inch OLED Display](https://www.amazon.com/gp/product/B089YFZQQK/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1).

And it contains:
- AIR530 GPS module
- SX1262 radio transciever
- 0.96-inch 128*64 OLED display
- Power management system (micro JST battery plug, charge controller, solar panel interface)
- IO/PWM pins

![pinout](resources/pinout-htcab-02s.png?raw=true)

### Compass

Compass is [GY-271 QMC5883L](https://www.amazon.com/gp/product/B085W6YCM6/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1).
Attached SLC0/SDA0 and VDD/GND pins.

![compass](resources/compass.png?raw=true)


### Battery

[3.7V 1100mAh](https://www.amazon.com/gp/product/B0867KDMY7/ref=ppx_yo_dt_b_asin_title_o06_s00?ie=UTF8&psc=1) Lithium Rechargable Battery 1S 3C.
Attached to battery port on the board.

![battery](resources/battery.png?raw=true)

## Software setup

### Install Arduino IDE

Follow [heltec guide](https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/quick_start.html#install-cubecell-relevant-framework) to install CubeCell board libraries.

- Download and install latest [Arduino IDE](https://www.arduino.cc/en/software)
- Go to Preferences
![Preferences](resources/pref.png?raw=true)
- Add https://resource.heltec.cn/download/package_CubeCell_index.json to Additional Boards Manager Urls, press OK
![Preferences Boards](resources/boards.png?raw=true)
- Go Tools -> Boards Manager -> find CubeCell and install
![Install CubeCell library](resources/install.png?raw=true)
- Select board in Tools -> Board -> CubeCell -> CubeCell GPS (HTCC-AB02S)
![Select CubeCell GPS board](resources/htcc-ab02s.png?raw=true)
- Install QMC5883LCompass library: Tools -> Manage Libraries -> Search and install QMC5883LCompass
![Install compass library](resources/QMC5883LCompass.png?raw=true)

### Compiling code

- Clone this repo `git clone git@github.com:bestander/body-finder.git`
- Open body-finder.ino in Arduino IDE: File -> Open ...
- Connect the board to the computer and select USB port in Tools -> Port -> /dev/usbserial
![Select USB Port](resources/port.png?raw=true)
- Press upload button in IDE
![Upload](resources/upload.png?raw=true)

### VS Code support

You can also install Arduino extension in vscode for compiling and uploading code to the board.
Arduino IDE still needs to be installed.

![VS Code](resources/vscode.png?raw=true)

## 3d printing the case

2BD
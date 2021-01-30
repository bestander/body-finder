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
- Power management  system
- IO/PWM pins

### Compass

2BD

### Battery

2BD

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

### Compiling code

- Clone this repo `git clone git@github.com:bestander/body-finder.git`
- Open body-finder in Arduino IDE: File -> Open ...
- Connect the board to the computer and select USB port in Tools -> Port -> /dev/usbserial
![Select USB Port](resources/port.png?raw=true)
- Press upload button in IDE
![Upload](resources/upload.png?raw=true)

### VS Code support

You can also install Arduino extension in vscode for compiling and uploading code to the board.
Arduino IDE still needs to be installed.

![VS Code](resources/upload.png?raw=true)

## 3d printing


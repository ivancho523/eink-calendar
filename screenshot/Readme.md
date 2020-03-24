![udpx Logo](/screenshot/cale-logo.svg)

# CALE E-ink calendar

## A very easy and straight-forward E-Ink calendar

The original version and hackaday project is moved to the [legacy branch](https://github.com/martinberlin/eink-calendar/tree/legacy)

**Now what remains here in master is the version that will just do two things only:**

1. Will connect to [cale.es](http://cale.es) and grab a dynamic rendered BMP
2. Will go to sleep the amount of seconds defined in Config and return to point 1

### Our approach to make an easy E-Ink calendar

- A screenshot to BMP endpoint that prints a webpage with the contents you need displayed on Eink (This does for you CALE)
- The Firmware driving the Eink display will wake up every morning or every 2 hours and read this screenshot. Then it will stay in deep sleep mode, consuming 1 miliamper from the battery, until it wakes up again and repeats the loop. 

The goal is to build a dynamic calendar that is easy to install and has the lowest consumption as possible.

### Simple configuration

Just rename lib/Config/Config.h.dist to Config.h
and fill it with your WiFi name and password.

If you want to enable deepsleep to power your calendar with batteries, then uncomment the line:

    //#define DEEPSLEEP_ENABLED

Amount of seconds that the ESP32 is awake:

    #define SLEEP_AFTER_SECONDS 20 

Note that ESP8266 uses another function to deepsleep and has a maximun deepsleep time of about 3 hours:

    ESP.deepSleep(3600e6);  // 3600 = 1 hour in seconds

**Most important part of the configuration:**

    char screenUrl[] = "http://img.cale.es/bmp/USERNAME/SCREEN_ID";
    
    // Security setting, leave empty if your screen is publis
    String bearer = "";

Note that we don't recommend to use public screens since your calendar may contain private information like events, transfer or doctor appointments that you should not open to the world to see. So use always a security token.

This token is sent in the headers like:

Authorization: Bearer YOUR_TOKEN

And passed to cale.es that verifies that your user owns this screen and also that the token matches the one that is stored on our servers.

### Schematics

![ESP8266 and SPI eink](screenshot/preview/Schematic_CALE_ESP8266.png)

[Check more information and detailed schematics for the ESP32](https://cale.es/firmware)

### Hardware requirements

To build one of this you can start easy and get something that needs no soldiering at all and comes already wired in a single PCB with an ESP32 included. If you want to start easy like this our recommendation is to get a [Lilygo T5](https://cale.es/firmware-t5).
Now if you want to have a big Epaper like 800x480 then you need to wire it yourself. Please browser our supported [displays for CALE](https://cale.es/eink-displays)

The most important asset to achieve low consumption and long battery life is that the ESP32 you use consumes less than 1 mA/hour in deepsleep mode.

#### ESP32 wiring suggestion

Mapping suggestion for ESP32, e.g. LOLIN32:

    This pins defined in lib/Config/Config.h
    BUSY -> 4, RST -> 16, DC -> 17, CS -> SS(5)  

    This ones are fixed
    CLK -> SCK(18), DIN -> MOSI(23)

### Build logs and detailed instructions

[CALE in Hackaday](https://hackaday.io/project/169086-cale-low-energy-eink-wallpaper) Please follow the project there to get updates and more detailed build instructions.

### Support CALE

There are commercial solutions alike and they start up to 560€ for a Eink syncronized calendar (Check getjoan.com)
If you use this commercially in your office we want to ask you about a small donation and to send us a short history of how it's working so we can give support. Please also file a Github issue in case you find a bug with detailed instructions so we can reproduce it in our end. 

### Credits

ZinggJM - Our function to read binary BMP is based on the [gxEPD WiFi example](https://github.com/ZinggJM/GxEPD/tree/master/examples/GxEPD_WiFi_Example) (90% from the code is his)

The great scheme, host and path function parsePathInformation() was coded by [Samuel Archibald](https://github.com/iotpanic).
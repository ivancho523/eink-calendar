// Miniz zlib: Discard parts that we do not use:
#define MINIZ_NO_ARCHIVE_APIS
#define MINIZ_NO_STDIO
#include "miniz.c"
#include <Config.h>
#include <SPI.h>
#include <GxEPD.h>
#include <GxGDEW075T8/GxGDEW075T8.cpp>
#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>
#include <WiFiClient.h>

#define COMPRESSION_BUFFER 1923
#define DECOMPRESSION_BUFFER 15746

#ifdef ESP32
  #include <ESPmDNS.h>
  #include <WebServer.h>
    // Bluetooth configuration:
  #include <ArduinoJson.h>
  #include <Preferences.h>
  #include <nvs.h>
  #include <nvs_flash.h>
  #include <WiFi.h>
  Preferences preferences; 
  // TCP server at port 80 will respond to HTTP requests
  WebServer server(80);
  #elif ESP8266

  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
  #include <ESP8266WebServer.h>
  ESP8266WebServer server(80);
    
#endif

#ifdef WIFI_BLE
  #include "BluetoothSerial.h"
  // SerialBT class
  BluetoothSerial SerialBT;
  StaticJsonDocument<300> jsonBuffer;
#endif

// FONT used for title / message body
//Converting fonts with ümlauts: ./fontconvert *.ttf 18 32 252
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

bool debugMode = false;
unsigned int secondsToDeepsleep = 0;

String message;
// Makes a div id="m" containing response message to dissapear after 3 seconds
String javascriptFadeMessage = "<script>setTimeout(function(){document.getElementById('m').innerHTML='';},3000);</script>";

// USE GPIO numbers for ESP32
//CLK  = D8; D
//DIN  = D7; D
//BUSY = D6; D
//CS   = D1; IMPORTANT: Don't use D0 for Chip select
//DC   = D3;
//RST  = D4; Sinde D0 can be used connected to RST if you want to wake up from deepsleep!

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, 5, 17, 16);
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
GxEPD_Class display(io, 16, 4);

WiFiClient client; // wifi client object
// DEBUG_MODE is compiled now and cannot be changed on runtime (Check lib/Config)
char apName[] = "CALE-xxxxxxxxxxxx";
bool usePrimAP = true;
/** Flag if stored AP credentials are available */
bool hasCredentials = false;
/** Connection status */
volatile bool isConnected = false;
bool connStatusChanged = false;
uint8_t lostConnectionCount = 1;
/** SSIDs/Password of local WiFi networks */
String ssidPrim;
String pwPrim;


void deleteWifiCredentials() {
	Serial.println("Clearing saved WiFi credentials");
  server.send(200, "text/html", "Clearing saved WiFi credentials, after reset will need Bluetooth configuration");
	preferences.begin("WiFiCred", false);
	preferences.clear();
	preferences.end();
}

uint32_t readBuffer16(uint8_t * outBuffer, long *bytePointer)
{
  uint32_t result;
  long pointer = *bytePointer;
  ((uint8_t *)&result)[0] = outBuffer[pointer]; pointer++;
  ((uint8_t *)&result)[1] = outBuffer[pointer]; pointer++; // MSB
  *bytePointer = pointer;
  return result;
}


uint32_t readBuffer32(uint8_t * outBuffer, long *bytePointer)
{
  uint32_t result;
  long pointer = *bytePointer;
  ((uint8_t *)&result)[0] = outBuffer[pointer]; pointer++;
  ((uint8_t *)&result)[1] = outBuffer[pointer]; pointer++;
  ((uint8_t *)&result)[2] = outBuffer[pointer]; pointer++;
  ((uint8_t *)&result)[3] = outBuffer[pointer]; pointer++; // MSB
  *bytePointer = pointer;
  return result;
}

bool bmpBufferRead(uint8_t * outBuffer, long byteCount) {
  int displayWidth = display.width();
  int displayHeight= display.height();
  uint8_t buffer[displayWidth]; // pixel buffer, size for r,g,b
  long bytesRead = 32;  // summing the whole BMP info headers
  long bytePointer = 2; // Start reading after BMP signature 0x424D

    uint32_t fileSize = readBuffer32(outBuffer, &bytePointer);
    readBuffer32(outBuffer, &bytePointer);
    uint32_t imageOffset = readBuffer32(outBuffer, &bytePointer); // Start of image data
    uint32_t headerSize = readBuffer32(outBuffer, &bytePointer);
    uint32_t width  = readBuffer32(outBuffer, &bytePointer);
    uint32_t height = readBuffer32(outBuffer, &bytePointer);
    uint16_t planes = readBuffer16(outBuffer, &bytePointer);
    uint16_t depth = readBuffer16(outBuffer, &bytePointer); // bits per pixel
    uint32_t format = readBuffer32(outBuffer, &bytePointer);

    Serial.print("->BMP starts here. File size: "); Serial.println(fileSize);
    Serial.print("Image Offset: "); Serial.println(imageOffset);
    Serial.print("Header size: "); Serial.println(headerSize);
    Serial.print("Width * Height: "); Serial.print(String(width) + " x " + String(height));
    Serial.print(" / Bit Depth: "); Serial.println(depth);
    Serial.print("Planes: "); Serial.println(planes);Serial.print("Format: "); Serial.println(format);
    
    if ((planes == 1) && (format == 0 || format == 3)) { // uncompressed is handled
      // Attempt to move pointer where image starts
      bytePointer = imageOffset-bytesRead; 
      size_t buffidx = sizeof(buffer); // force buffer load
      
      for (uint16_t row = 0; row < height; row++) // for each line
      {
        //delay(1); // May help to avoid Wdt reset
        uint8_t bits;
        for (uint16_t col = 0; col < width; col++) // for each pixel
        {
          yield();
          // Time to read more pixel data?
          if (buffidx >= sizeof(buffer))
          {
            
            //TODO: Check if this assumption is correct, replaces old streaming read:
            //client.readBytes(buffer, sizeof(buffer));
             for(unsigned i = 0; i<sizeof(buffer); i++){
               buffer[i] = outBuffer[bytePointer];
               bytePointer++;
             }
            buffidx = 0; // Set index to beginning

            //Serial.printf("ReadBuffer Row: %d bytePointer: %d bytesRead: %d\n",row,bytePointer,bytesRead);
          }
          switch (depth)
          {
            case 1: // one bit per pixel b/w format
              {
                if (0 == col % 8)
                {
                  bits = buffer[buffidx++];
                  bytesRead++;
                }
                uint16_t bw_color = bits & 0x80 ? GxEPD_WHITE : GxEPD_BLACK;
                display.drawPixel(col, displayHeight-row, bw_color);
                bits <<= 1;
              }
              break;
              
            case 4: // was a hard work to get here
              {
                if (0 == col % 2) {
                  bits = buffer[buffidx++];
                  bytesRead++;
                }   
                bits <<= 1;           
                bits <<= 1;
                uint16_t bw_color = bits > 0x80 ? GxEPD_WHITE : GxEPD_BLACK;
                display.drawPixel(col, displayHeight-row, bw_color);
                bits <<= 1;
                bits <<= 1;
              }
              break;
              
             case 24: // standard BMP format
              {
                uint16_t b = buffer[buffidx++];
                uint16_t g = buffer[buffidx++];
                uint16_t r = buffer[buffidx++];
                uint16_t bw_color = ((r + g + b) / 3 > 0xFF  / 2) ? GxEPD_WHITE : GxEPD_BLACK;
                display.drawPixel(col, displayHeight-row, bw_color);
                bytesRead = bytesRead +3;
              }
          }
        } // end pixel
      } // end line
       // Uncomment to send answer, trying to make it faster not delivering this on calendar update
       //server.send(200, "text/html", "<div id='m'>Image sent to display</div>"+javascriptFadeMessage);
       display.update();
       Serial.printf("Read %lu bytes. Sending BMP pixels to display\n", bytesRead);
       return true;
       
    } else {
      server.send(200, "text/html", "<div id='m'>Unsupported image format (depth:"+String(depth)+")</div>"+javascriptFadeMessage);
      display.setCursor(10, 43);
      display.print("Compressed BMP files are not handled. Unsupported image format (depth:"+String(depth)+")");
      display.update();
      return false;
    }
}

void handleWebToDisplay() {
  int milliIni = millis();
  String url = calendarUrl;
  String zoom = ".8";
  String brightness = "100";
  if (server.args() > 0) {
    for (byte i = 0; i < server.args(); i++) {
      if (server.argName(i) == "url" && server.arg(i) != "") {
        url = server.arg(i);
      }
      if (server.argName(i) == "zoom" && server.arg(i) != "") {
        zoom = server.arg(i);
      }
      if (server.argName(i) == "brightness" && server.arg(i) != "") {
        brightness = server.arg(i);
      }
    }
  }
    if (url == "") {
      display.println("No Url received");
      display.update();
      return;
    }
  
  String image = screenshotPath+"?u=" + url + "&z=" + zoom + "&b=" + brightness +"&c=1&eink=GDEW042T2";
  String request;
  request  = "GET " + image + " HTTP/1.1\r\n";
  request += "Host: " + String(screenshotHost) + "\r\n";
  request += "Connection: close\r\n";
  request += "\r\n";
  Serial.println(screenshotHost+image);

  const int httpPort = 80;
  client.connect(screenshotHost, httpPort);
  client.print(request); //send the http request to the server
  client.flush();
  display.fillScreen(GxEPD_WHITE);
  
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  
 // NOTE: No need to discard headers anymore, unless they contain 0x4D42
long byteCount = 0; // Byte counter

// Start reading bits
uint8_t *inBuffer = new uint8_t[COMPRESSION_BUFFER];
inBuffer[byteCount] = 0x78; // zlib header[0]
byteCount++;
uint8_t lastByte = 0x00;
bool startFetch = false;
Serial.println("Zlib preview:");
Serial.print(inBuffer[0], HEX);Serial.println(" ");

  while (client.available()) {
    yield();
    uint8_t clientByte = client.read();
    if (clientByte == 0xDA && lastByte == 0x78) {
      startFetch = true;
    }
    if (startFetch) {
      inBuffer[byteCount] = clientByte;
      #ifdef ESP8266
      delay(1);
      #endif
      byteCount++;
    }
    lastByte = clientByte;
    }

    //Serial.printf("\nDone downloading compressed BMP. Length: %d\n", byteCount);
    
    int milliDecomp = millis();
  		uint8_t *outBuffer = new uint8_t[DECOMPRESSION_BUFFER];
			uLong uncomp_len;

    // status: { MZ_OK = 0, MZ_STREAM_END = 1, MZ_NEED_DICT = 2, MZ_ERRNO = -1, MZ_STREAM_ERROR = -2, 
    //           MZ_DATA_ERROR = -3, MZ_MEM_ERROR = -4, MZ_BUF_ERROR = -5, MZ_VERSION_ERROR = -6, MZ_PARAM_ERROR = -10000 };

    int cmp_status = uncompress(
      outBuffer, 
      &uncomp_len, 
      (const unsigned char*)inBuffer, 
      byteCount);

    int millisAfterDecomp = millis();
    delete(inBuffer);
    
    // Render BMP with outBuffer if this works
    Serial.printf("uncompress_status: %d Compressed length: %lu freeHeap: %d\n", 
                  cmp_status, byteCount, ESP.getFreeHeap());
    bool isRendered = 0;
    if (cmp_status == 0) {
      isRendered = bmpBufferRead(outBuffer,uncomp_len);
    } else {
      Serial.printf("uncompress status: %d Decompression error\n", cmp_status);
    }
    Serial.printf("Eink isRendered: %d BENCHMARKS:\n", isRendered);
    Serial.printf("Download: %d ms Decompress: %d ms Rendering: %lu seconds\n", milliDecomp-milliIni, millisAfterDecomp-milliDecomp, (millis()-millisAfterDecomp)/1000 );
    delete(outBuffer);
}

/** Callback for connection loss */
void lostCon(system_event_id_t event) {
	isConnected = false;
	connStatusChanged = true;

    Serial.printf("WiFi lost connection try %d to connect again\n", lostConnectionCount);
	// Avoid trying to connect forever if the user made a typo in password
	if (lostConnectionCount>4) {
		deleteWifiCredentials();
		ESP.restart();
	} else if (lostConnectionCount>1) {
		Serial.printf("Lost connection %d times", lostConnectionCount);
	}
	lostConnectionCount++;
	#ifdef WIFI_BLE
	  WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
	#else
      WiFi.begin(WIFI_SSID, WIFI_PASS);
	#endif
}

// Displays message doing a partial update
void displayMessage(String message, int height) {
  Serial.println("DISPLAY prints: "+message);
  display.setTextColor(GxEPD_WHITE);
  display.fillRect(0,0,display.width(),height,GxEPD_BLACK);
  display.setCursor(2, 25);
  display.print(message);
  display.updateWindow(0,0,display.width(),height, true); // Attempt partial update
  display.update(); // -> Since could not make partial updateWindow work
}

void handle_http_not_found() {
  server.send(404, "text/plain", "Not Found");
}

void handle_http_root() {

  String headers = "<head><link rel=\"stylesheet\" href=\"https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/css/bootstrap.min.css\">";
  headers += "<meta name='viewport' content='width=device-width,initial-scale=1'></head>";
  String html = "<body><main role='main'><div class='container-fluid'><div class='row'>";
  html += "<div class='col-md-12'><h4>" + String(apName) + ".local</h4>";
  html += "<form id='f' action='/web-image' target='frame' method='POST'>";
  html += "<div class='row'><div class='col-sm-12'>";

  html += "<select name='zoom' style='width:7em' class='form-control float-right'>";
  html += "<option value='2'>2</option>";
  html += "<option value='1.7'>1.7</option>";
  html += "<option value='1.5'>1.5</option>";
  html += "<option value='1.4'>1.4</option>";
  html += "<option value='1.3'>1.3</option>";
  html += "<option value='1.2'>1.2</option>";
  html += "<option value='1.1'>1.1 10% zoomed</option>";
  html += "<option value='' selected>Zoom</option>";
  html += "<option value='1'>1 no zoom</option>";
  html += "<option value='.9'>.9 10% smaller</option>";
  html += "<option value='.85'>.85</option>";
  html += "<option value='.8'>.8</option>";
  html += "<option value='.7'>.7</option>";
  html += "<option value='.6'>.6</option>";
  html += "<option value='.5'>.5 half size</option></select>&nbsp;";
  
  html += "<select name='brightness' style='width:8em' class='form-control float-right'>";
  html += "<option value='150'>+5</option>";
  html += "<option value='140'>+4</option>";
  html += "<option value='130'>+3</option>";
  html += "<option value='120'>+2</option>";
  html += "<option value='110'>+1</option>";
  html += "<option value='' selected>Brightness</option>";
  html += "<option value='90'>-1</option>";
  html += "</select>";

  html += "</div></div>";
  html += "<input placeholder='http://' id='url' name='url' type='url' class='form-control' value='"+calendarUrl+"'>";
  html += "<div class='row'><div class='col-sm-12 form-group'>";
  html += "<input type='submit' value='Website screenshot' class='btn btn-mini btn-dark'>&nbsp;";
  html += "<input type='button' onclick='document.getElementById(\"url\").value=\"\"' value='Clean url' class='btn btn-mini btn-default'></div>";
  html += "</div></form>";
  
  html += "<form id='f2' action='/display-write' target='frame' method='POST'>";
  html += "<label for='title'>Title:</label><input id='title' name='title' class='form-control'><br>";
  html += "<textarea placeholder='Content' name='text' rows=4 class='form-control'></textarea>";
  html += "<input type='submit' value='Send to display' class='btn btn-success'></form>";
  html += "<a class='btn btn-default' role='button' target='frame' href='/display-clean'>Clean screen</a> | ";
  html += "<a class='btn btn-danger' role='button' target='frame' href='/delete-wifi-credentials'>Reset WiFi credentials</a> | ";
  html += "<a class='btn btn-default' role='button' target='frame' href='/deep-sleep'>Deep sleep mode</a><br>";
  html += "<iframe name='frame'></iframe>";
  html += "<a href='/deep-sleep' target='frame'>Deep sleep</a><br>";
  html += "</div></div></div></main>";
  html += "</body>";

  server.send(200, "text/html", headers + html);
}

void handleDeepSleep() {
  server.send(200, "text/html", "Going to deep-sleep. Reset to wake up");
  delay(10);
  ESP.deepSleep(20e6);
}


void handleDisplayClean() {
  display.fillScreen(GxEPD_WHITE);
  display.update();
  server.send(200, "text/html", "Clearing display");
}

void handleDisplayWrite() {
  display.fillScreen(GxEPD_WHITE);

  // Analizo el POST iterando cada value
  if (server.args() > 0) {
    for (byte i = 0; i < server.args(); i++) {

      if (server.argName(i) == "title") {
        display.setFont(&FreeMonoBold24pt7b);
        display.setCursor(10, 43);
        display.print(server.arg(i));
      }
      if (server.argName(i) == "text") {
        display.setFont(&FreeMonoBold12pt7b);
        display.setCursor(10, 75);
        display.print(server.arg(i));
      }
    }
  }
  display.update();
  server.send(200, "text/html", "Text sent to display");
}

String ipAddress2String(const IPAddress& ipAddress){
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3]);
}

/** Callback for receiving IP address from AP */
void gotIP(system_event_id_t event) {
	#ifdef WIFI_BLE
    SerialBT.disconnect();
	  SerialBT.end();
    Serial.printf("SerialBT.end() freeHeap: %d\n", ESP.getFreeHeap());
	#endif
  
  if (isConnected) return;

  isConnected = true;
	connStatusChanged = true;

	if (!MDNS.begin(apName)) {
		Serial.println("Error setting up MDNS responder!");
    }
  MDNS.addService("http", "tcp", 80);

	Serial.printf("%s.local is online with IP:", apName);
  Serial.println(WiFi.localIP());  
  // Render display with default start page
  handleWebToDisplay();
  // Start HTTP server
  server.onNotFound(handle_http_not_found);
  // Routes:
  server.on("/", handle_http_root);
  server.on("/display-write", handleDisplayWrite);
  server.on("/web-image", handleWebToDisplay);
  server.on("/display-clean", handleDisplayClean);
  server.on("/deep-sleep", handleDeepSleep);
  server.on("/delete-wifi-credentials", deleteWifiCredentials);
  server.begin();
}

/**
 * Start connection to AP
 */
void connectWiFi() {
	// Setup callback function for successful connection
	WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
	// Setup callback function for lost connection
	WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);

	Serial.println();
	Serial.print("Start connection to ");
  
  Serial.println(ssidPrim);
  WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
}

void loop() {

 server.handleClient();

  // Note: Enable deepsleep only as last step when all the rest is working as you expect
#ifdef DEEPSLEEP_ENABLED
  if (secondsToDeepsleep>SLEEP_AFTER_SECONDS) {
      Serial.println("Going to sleep one hour. Waking up only if D0 is connected to RST");
      display.powerDown();
      delay(100);
      ESP.deepSleep(DEEPSLEEP_SECONDS*1000000ULL); // Expects microseconds
  }
  secondsToDeepsleep++;
  delay(1000);
#endif

}
#ifdef ESP32
/**
 * Create unique device name from MAC address
 **/
void createName() {
	uint8_t baseMac[6];
	// Get MAC address for WiFi station
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
	// Write unique name into apName
	sprintf(apName, "CALE-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

/**
 * initBTSerial
 * Initialize Bluetooth Serial
 * Start BLE server and service advertising
 * @return <code>bool</code>
 * 			true if success
 * 			false if error occured
 */
bool initBTSerial() {
		if (!SerialBT.begin(apName)) {
			Serial.println("Failed to start BTSerial");
			return false;
		}
		Serial.println("BTSerial active. Device name: " + String(apName));
		return true;
}

/**
 * readBTSerial
 * read all data from BTSerial receive buffer
 * parse data for valid WiFi credentials
 */
void readBTSerial() {
	if (!SerialBT.available()) return;
	uint64_t startTimeOut = millis();
	String receivedData;
	int msgSize = 0;
	// Read RX buffer into String
	
	while (SerialBT.available() != 0) {
		receivedData += (char)SerialBT.read();
		msgSize++;
		// Check for timeout condition
		if ((millis()-startTimeOut) >= 5000) break;
	}
	SerialBT.flush();
	Serial.println("Received message " + receivedData + " over Bluetooth");

	// Decode the message only if it comes encoded (Like ESP32 WIFI Ble does)
	if (receivedData[0] != '{') {
		int keyIndex = 0;
		for (int index = 0; index < receivedData.length(); index ++) {
			receivedData[index] = (char) receivedData[index] ^ (char) apName[keyIndex];
			keyIndex++;
			if (keyIndex >= strlen(apName)) keyIndex = 0;
		}
		Serial.println("Decoded message: " + receivedData); 
	}
	
	/** Json object for incoming data */
	auto error = deserializeJson(jsonBuffer, receivedData);
	if (!error)
	{
		if (jsonBuffer.containsKey("ssidPrim") &&
			jsonBuffer.containsKey("pwPrim") &&
			jsonBuffer.containsKey("ssidSec") &&
			jsonBuffer.containsKey("pwSec"))
		{
			ssidPrim = jsonBuffer["ssidPrim"].as<String>();
			pwPrim = jsonBuffer["pwPrim"].as<String>();

			Preferences preferences;
			preferences.begin("WiFiCred", false);
			preferences.putString("ssidPrim", ssidPrim);
			preferences.putString("pwPrim", pwPrim);
			preferences.putBool("valid", true);
			preferences.end();

			Serial.println("Received over bluetooth:");
			Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
			connStatusChanged = true;
			hasCredentials = true;
			delay(100);
			connectWiFi();
			
		}
		else if (jsonBuffer.containsKey("erase"))
		{ // {"erase":"true"}
			Serial.println("Received erase command");
			Preferences preferences;
			preferences.begin("WiFiCred", false);
			preferences.clear();
			preferences.end();
			connStatusChanged = true;
			hasCredentials = false;
			ssidPrim = "";
			pwPrim = "";

			int err;
			err=nvs_flash_init();
			Serial.println("nvs_flash_init: " + err);
			err=nvs_flash_erase();
			Serial.println("nvs_flash_erase: " + err);
		}
		 else if (jsonBuffer.containsKey("reset")) {
			WiFi.disconnect();
			esp_restart();
		}
	} else {
		Serial.println("Received invalid JSON");
	}
	jsonBuffer.clear();
}
#endif

void setup() {
  Serial.begin(115200);delay(10);
  Serial.printf("setup() freeHeap: %d\n", ESP.getFreeHeap());
  createName();

  display.init();
  display.setRotation(2); // Rotates display N times clockwise
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

#ifdef WIFI_BLE
	preferences.begin("WiFiCred", false);
    //preferences.clear(); // Uncomment to force delete preferences

	bool hasPref = preferences.getBool("valid", false);
	if (hasPref) {
		ssidPrim = preferences.getString("ssidPrim","");
		pwPrim = preferences.getString("pwPrim","");

		if (ssidPrim.equals("") 
				|| pwPrim.equals("")) {
			Serial.println("Found preferences but credentials are invalid");
		} else {
			Serial.println("Read from preferences:");
			Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
			hasCredentials = true;
		}
	}  else {
		Serial.println("Could not find preferences, need send data over BLE");
    // Start Bluetooth serial. This reduces Heap memory like crazy so start it only if preferences are not set!
    initBTSerial();
    Serial.printf("initBTSerial() freeHeap: %d\n", ESP.getFreeHeap());
	}
	preferences.end();

	if (hasCredentials) {
	    connectWiFi();
  }


	#else
	  WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);
    Serial.printf("Connecting to Wi-Fi using WIFI_AP %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  #endif

}

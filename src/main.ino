#include "esp_camera.h"
#include <Arduino.h>
#include <FS.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <SPIFFS.h>
#include <ArduinoJson.h>

#include <SocketIoClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <DHT.h>
#define TRIGGER_PIN 0
// wifimanager can run in a blocking mode or a non blocking mode
// Be sure to know how to process loops with no delay() if using non blocking
bool wm_nonblocking = false; // change to true to use non blocking

SocketIOclient socketIO;

char deviceid[10];
char server[50] = "http://localhost";
char port[6] = "8080";
char api_token[34] = "YOUR_API_TOKEN";
// flag for saving data
bool shouldSaveConfig = false;

WiFiManager wm; // global wm instance
// WiFiManagerParameter custom_deviceid; // global param ( for non blocking w params )
// WiFiManagerParameter custom_server;   // global param ( for non blocking w params )
// WiFiManagerParameter custom_port;     // global param ( for non blocking w params )

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 2;
const int PWMLightChannel = 3;

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
// Camera related constants
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define LED_FLASH_GPIO_NUM 4 // Example: GPIO 4

void startCameraServer();

#define DHT_SENSOR_PIN 2 // Change this to the pin you've connected to the DHT sensor
#define DHT_SENSOR_TYPE DHT11

#define sensorPin 4

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

int readSensor()
{
    int sensorValue = analogRead(sensorPin);             // Read the analog value from sensor
    int outputValue = map(sensorValue, 0, 1023, 255, 0); // map the 10-bit data to 8-bit data
                                                         //   analogWrite(ledPin, outputValue); // generate PWM signal
    return outputValue;                                  // Return analog moisture value
}

void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case sIOtype_DISCONNECT:
        Serial.printf("[IOc] Disconnected!\n");
        break;
    case sIOtype_CONNECT:
        Serial.printf("[IOc] Connected to url: %s\n", payload);

        // join default namespace (no auto join in Socket.IO V3)
        socketIO.send(sIOtype_CONNECT, "/");
        break;
    case sIOtype_EVENT:
    {
        char *sptr = NULL;
        int id = strtol((char *)payload, &sptr, 10);
        Serial.printf("[IOc] get event: %s id: %d\n", payload, id);
        if (id)
        {
            payload = (uint8_t *)sptr;
        }
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error)
        {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());
            return;
        }

        String eventName = doc[0];
        Serial.printf("[IOc] event name: %s\n", eventName.c_str());
        if (eventName == "datafromserver")
        {
            Serial.println("datafromserver");
            int data = doc[1];
            Serial.printf("data: %d\n", data);
        }
    }
    break;
    case sIOtype_ACK:
        Serial.printf("[IOc] get ack: %u\n", length);
        break;
    case sIOtype_ERROR:
        Serial.printf("[IOc] get error: %u\n", length);
        break;
    case sIOtype_BINARY_EVENT:
        Serial.printf("[IOc] get binary: %u\n", length);
        break;
    case sIOtype_BINARY_ACK:
        Serial.printf("[IOc] get binary ack: %u\n", length);
        break;
    }
}

void mountSPIFFS()
{
    if (SPIFFS.begin(true))
    {
        Serial.println("MOUNTED FILE SYSTEM");
        if (SPIFFS.exists("/config.json"))
        {
            // file exists, reading and loading
            Serial.println("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile)
            {
                Serial.println("opened config file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                DynamicJsonDocument json(1024);
                auto deserializeError = deserializeJson(json, buf.get());
                serializeJson(json, Serial);
                if (!deserializeError)
                {
                    Serial.println("\nparsed json");
                    strcpy(deviceid, json["deviceid"]);
                    strcpy(server, json["server"]);
                    strcpy(port, json["port"]);
                }
                else
                {
                    Serial.println("failed to load json config");
                }
                configFile.close();
            }
        }
    }
    else
    {
        Serial.println("FAILED TO MOUNT SPIFF");
    }
}

void setupCamera()
{

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    //                      for larger pre-allocated frame buffer.

    if (psramFound())
    {
        config.frame_size = FRAMESIZE_XGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
    }
    else
    {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

#if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
#endif

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the brightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    // s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif

    // Configure the LED flash pin as output
    pinMode(LED_FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(LED_FLASH_GPIO_NUM, LOW); // Ensure the LED is off initially
}

void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    pinMode(sensorPin, OUTPUT);
    dht_sensor.begin();
    // read configuration from FS json
    Serial.println("mounting FS...");

    mountSPIFFS();

    Serial.println("\n Starting");

    pinMode(TRIGGER_PIN, INPUT);

    // wm.resetSettings(); // wipe settings

    if (wm_nonblocking)
        wm.setConfigPortalBlocking(false);

    // add a custom input field
    // int customFieldLength = 100;

    WiFiManagerParameter custom_deviceid("deviceid", "Device ID", "", 10, "placeholder=\"0001\"");
    WiFiManagerParameter custom_server("server", "Server Addr", "", 50, "placeholder=\"192.168.1.1\"");
    WiFiManagerParameter custom_port("port", "Port", "", 6, "placeholder=\"3000\"");

    wm.addParameter(&custom_deviceid);
    wm.addParameter(&custom_server);
    wm.addParameter(&custom_port);
    wm.setSaveParamsCallback(saveParamCallback);

    // custom menu via array or vector
    //
    // menu tokens, "wifi","wifinoscan","info","param","close","sep","erase","restart","exit" (sep is seperator) (if param is in menu, params will not show up in wifi page!)
    // const char* menu[] = {"wifi","info","param","sep","restart","exit"};
    // wm.setMenu(menu,6);
    std::vector<const char *> menu = {"param", "wifi", "sep", "info", "restart", "exit"};
    wm.setMenu(menu);

    // set dark theme
    wm.setClass("invert");

    // set static ip
    //  wm.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0)); // set static ip,gw,sn
    //  wm.setShowStaticFields(true); // force show static ip fields
    //  wm.setShowDnsFields(true);    // force show dns field always

    // wm.setConnectTimeout(20); // how long to try to connect for before continuing
    wm.setConfigPortalTimeout(30); // auto close configportal after n seconds
    // wm.setCaptivePortalEnable(false); // disable captive portal redirection
    // wm.setAPClientCheck(true); // avoid timeout if client connected to softap

    // wifi scan settings
    // wm.setRemoveDuplicateAPs(false); // do not remove duplicate ap names (true)
    // wm.setMinimumSignalQuality(20);  // set min RSSI (percentage) to show in scans, null = 8%
    // wm.setShowInfoErase(false);      // do not show erase button on info page
    // wm.setScanDispPerc(true);       // show RSSI as percentage not graph icons

    // wm.setBreakAfterConfig(true);   // always exit configportal even if wifi save fails

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("ESP32DeviceConfigAP", "password"); // password protected ap

    if (!res)
    {
        Serial.println("Failed to connect or hit timeout");
        // ESP.restart();
    }
    else
    {
        // if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");

        // read updated parameters
        strcpy(deviceid, custom_deviceid.getValue());
        strcpy(server, custom_server.getValue());
        strcpy(port, custom_port.getValue());
        Serial.println("The values in the file are: ");
        Serial.println("deviceid : " + String(deviceid));
        Serial.println("server : " + String(server));
        Serial.println("port : " + String(port));

        const char *sockethost = server;
        int socketport = atoi(port);
        socketIO.begin(sockethost, socketport, "/socket.io/?EIO=4");
        socketIO.onEvent(socketIOEvent);

        // save the custom parameters to FS
        if (shouldSaveConfig)
        {
            Serial.println("saving config");
            DynamicJsonDocument json(1024);
            json["deviceid"] = deviceid;
            json["server"] = server;
            json["port"] = port;
            File configFile = SPIFFS.open("/config.json", "w");
            if (!configFile)
            {
                Serial.println("failed to open config file for writing");
            }

            serializeJson(json, Serial);
            serializeJson(json, configFile);
            configFile.close();
            // end save
        }

        setupCamera();

        startCameraServer();

        Serial.print("Camera Ready! Use 'http://");
        Serial.print(WiFi.localIP());
        Serial.println("' to connect");
    }
}

void checkButton()
{
    // check for button press
    if (digitalRead(TRIGGER_PIN) == LOW)
    {
        // poor mans debounce/press-hold, code not ideal for production
        delay(50);
        if (digitalRead(TRIGGER_PIN) == LOW)
        {
            Serial.println("Button Pressed");
            // still holding button for 3000 ms, reset settings, code not ideaa for production
            delay(3000); // reset delay hold
            if (digitalRead(TRIGGER_PIN) == LOW)
            {
                Serial.println("Button Held");
                Serial.println("Erasing Config, restarting");
                wm.resetSettings();
                ESP.restart();
            }

            // start portal w delay
            Serial.println("Starting config portal");
            wm.setConfigPortalTimeout(120);

            if (!wm.startConfigPortal("ESP32ConfigOnDemandAP", "password"))
            {
                Serial.println("failed to connect or hit timeout");
                delay(3000);
                // ESP.restart();
            }
            else
            {
                // if you get here you have connected to the WiFi
                // Serial.println("PARAM deviceid = " + getParam("deviceid"));
                // Serial.println("PARAM server = " + getParam("server"));
                Serial.println("connected...yeey :)");
            }
        }
    }
}

String getParam(String name)
{
    // read parameter from server, for customhmtl input
    String value;
    if (wm.server->hasArg(name))
    {
        value = wm.server->arg(name);
    }
    return value;
}

void saveParamCallback()
{
    Serial.println("[CALLBACK] saveParamCallback fired");
    shouldSaveConfig = true;
}

void updateSerial()
{
    delay(500);
    while (Serial.available())
    {
        Serial2.write(Serial.read()); // Forward what Serial received to Software Serial Port
    }
    while (Serial2.available())
    {
        Serial.write(Serial2.read()); // Forward what Software Serial received to Serial Port
    }
}

void loop()
{

    // // Example of taking a picture with flash
    // digitalWrite(LED_FLASH_GPIO_NUM, HIGH); // Turn on the LED
    // delay(100); // Wait for 100 milliseconds (adjust based on your flash needs)

    //  // Capture an image
    // camera_fb_t * fb = esp_camera_fb_get();
    // if (!fb) {
    //     Serial.println("Camera capture failed");
    // } else {
    //     // Use the image data from fb->buf with length fb->len
    //     esp_camera_fb_return(fb); // Return the frame buffer back to the driver for reuse
    // }

    // digitalWrite(LED_FLASH_GPIO_NUM, LOW); // Turn off the LED

    // // Add a delay or your own logic to determine when to take the next picture
    // delay(5000); // Example: 5-second delay between

    if (wm_nonblocking)
        wm.process(); // avoid delays() in loop when non-blocking and other long running code

    // checkButton();

    // updateSerial();
    // while (Serial2.available() > 0)
    //     if (gps.encode(Serial2.read()))
    //         displayInfo();

    // if (millis() > 5000 && gps.charsProcessed() < 10)
    // {
    //     Serial.println(F("No GPS detected: check wiring."));
    //     while (true)
    //         ;
    // }

    // HUMIDITY SENSOR SAMPLE
    // float humidity = dht_sensor.readHumidity();
    // float temperature = dht_sensor.readTemperature();
    // if (isnan(humidity) || isnan(temperature))
    // {
    //     Serial.println("Failed to read from DHT sensor!");
    //     return;
    // }

    // Serial.print("Humidity: ");
    // Serial.print(humidity);
    // Serial.print(" %\t");
    // Serial.print("Temperature: ");
    // Serial.print(temperature);
    // Serial.println(" *C");

    // Serial.print("Analog output: ");
    // Serial.println(readSensor());

    socketIO.loop();

    if (socketIO.isConnected())
    {
        // // Clears the trigPin
        // digitalWrite(trigPin, LOW);
        // delayMicroseconds(2);
        // // Sets the trigPin on HIGH state for 10 micro seconds
        // digitalWrite(trigPin, HIGH);
        // delayMicroseconds(10);
        // digitalWrite(trigPin, LOW);

        // // Reads the echoPin, returns the sound wave travel time in microseconds
        // duration = pulseIn(echoPin, HIGH);

        // // Calculate the distance
        // distanceCm = duration * SOUND_SPEED / 2;

        // // Convert to inches
        // distanceInch = distanceCm * CM_TO_INCH;

        // photoresistorvalue = analogRead(photoPin);
        // // Serial.print("Photoresistor value: ");
        // // Serial.println(photoresistorvalue);
        // ambianttemp = mlx.readAmbientTempC();
        // // Serial.print("Ambiant temperature: ");
        // // Serial.println(ambianttemp);
        // objecttemp = mlx.readObjectTempC();
        // // Serial.print("Object temperature: ");
        // // Serial.println(objecttemp);

        // bool led1PinStatus = digitalRead(led1Pin);
        // bool led2PinStatus = digitalRead(led2Pin);
        // bool led3PinStatus = digitalRead(led3Pin);
        // bool led4PinStatus = digitalRead(led4Pin);

        // // read humidity
        // float humi = dht_sensor.readHumidity();
        // // read temperature in Celsius
        // float tempC = dht_sensor.readTemperature();
        // // read temperature in Fahrenheit
        // float tempF = dht_sensor.readTemperature(true);

        // // check whether the reading is successful or not
        // if (isnan(tempC) || isnan(tempF) || isnan(humi))
        // {
        //   Serial.println("Failed to read from DHT sensor!");
        // }
        // else
        // {
        //   Serial.print("Humidity: ");
        //   Serial.print(humi);
        //   Serial.print("%");

        //   Serial.print("  |  ");

        //   Serial.print("Temperature: ");
        //   Serial.print(tempC);
        //   Serial.print("°C  ~  ");
        //   Serial.print(tempF);
        //   Serial.println("°F");
        // }

        // delay(1000);

        // // creat JSON message for Socket.IO (event)
        // DynamicJsonDocument doc(1024);
        // JsonArray array = doc.to<JsonArray>();

        // // add evnet name
        // // Hint: socket.on('event_name', ....
        // array.add("iotdata");

        // // add payload (parameters) for the event
        // JsonObject param1 = array.createNestedObject();
        // param1["pr"] = (uint32_t)photoresistorvalue;
        // param1["at"] = ambianttemp;
        // param1["ot"] = objecttemp;
        // param1["uss"] = distanceCm;
        // param1["humi"] = humi;
        // param1["tempC"] = tempC;
        // param1["tempF"] = tempF;
        // param1["led1"] = led1PinStatus;
        // param1["led2"] = led2PinStatus;
        // param1["led3"] = led3PinStatus;
        // param1["led4"] = led4PinStatus;

        // // JSON to String (serializion)
        // String output;
        // serializeJson(doc, output);

        // // Send event
        // socketIO.sendEVENT(output);

        // Print JSON for debugging
        // Serial.println(output);
    }

    
    // delay(2000);
}
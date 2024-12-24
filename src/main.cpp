#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "pass.h"

// function declarations
String sendPhoto();
void sendPhotoUART();

// global variables
const char* ssid = SSID;
const char* password = PASSWORD;

// String serverName = "192.168.220.230";
String serverName = "192.168.1.28";
String serverPath = "/face/recognize/";
const int serverPort = 5000;

WiFiClient client;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define PIXEL_FORMAT PIXFORMAT_RGB565
#define IMAGE_WIDTH 160
#define IMAGE_HEIGHT 120
#define UINT8_PER_CHANNEL 2
#define TESTING 0

const int timerInterval = 10000;    // time between each HTTP POST image (for testing)
unsigned long previousMillis = 0;   // last time image was sent

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXEL_FORMAT; // PIXFORMAT_JPEG

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QQVGA; // FRAMESIZE_SVGA
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA; // FRAMESIZE_CIF
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // sendPhoto();
  // sendPhotoUART();

  pinMode(33, OUTPUT);
  digitalWrite(33, 0);
}

void loop() {
  if (Serial.available() > 0) {
    // Read one byte from Serial
    int receivedByte = Serial.read();
    // Serial.write(receivedByte);

    if (receivedByte == 255) {
      sendPhotoUART();
    }
    else if (receivedByte == 0){
      sendPhoto();
    }
    else if (receivedByte == 1){
      const uint16_t size = 1024;
      uint8_t txBuffer[1024] = {255};
      uint8_t txSize[2] = {0};
      txSize[0] = size >> 8; // Assign high byte
      txSize[1] = size & 0xFF; // Assign low byte
      Serial.write(txSize, 2);
      delay(10);
      Serial.write(txBuffer, size);
      delay(10);
      Serial.write(txSize, 2);
      delay(10);
      txBuffer[0] = 244;
      Serial.write(txBuffer, size);
      delay(10);
      txSize[0] = 0;
      txSize[1] = 0;
      Serial.write(txSize, size);
    }
    //Serial.flush(); // Clear any remaining data in the buffer
  }
  if (TESTING){
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= timerInterval) {
      sendPhoto();
      // sendPhotoUART();
      previousMillis = currentMillis;
    }
  }
  else{
    delay(10);
  }
} 

String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    uint8_t txSize[2] = {0};
    Serial.write(txSize, 2);
    // Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  // Serial.println("Connecting to server: " + serverName);
  
  if (client.connect(serverName.c_str(), serverPort)) {
    // Serial.println("Connection successful!");

    String head, tail;
    // String head = "--ESP32CAM\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    // String tail = "\r\n--ESP32CAM--\r\n";
    if (PIXEL_FORMAT == PIXFORMAT_JPEG){
      head = "--ESP32CAM\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
      tail = "\r\n--ESP32CAM--\r\n";
    }

    uint32_t imageLen = fb->len;
    uint32_t extraLen = 0;
    if (PIXEL_FORMAT == PIXFORMAT_JPEG)
      extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    if (PIXEL_FORMAT == PIXFORMAT_JPEG){
      client.println("Content-Type: multipart/form-data; boundary=ESP32CAM");
    }
    else{
      client.println("Content-Type: application/octet-stream");
    }
    client.println();
    if (PIXEL_FORMAT == PIXFORMAT_JPEG)
      client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }

    if (PIXEL_FORMAT == PIXFORMAT_JPEG)
      client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      // Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    // Serial.println();
    client.stop();
    // Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    // Serial.println(getBody);
  }
  uint8_t txSize[2] = {0};
  Serial.write(txSize, 2);
  return getBody;
}

void sendPhotoUART(){
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    uint8_t txSize[2] = {0};
    Serial.write(txSize, 2);
    delay(1000);
    ESP.restart();
  }
  uint8_t *fbBuf = fb->buf;
  size_t fbLen = fb->len;
  uint8_t txSize[2] = {0};
  uint16_t size = 0;
  for (size_t n = 0; n < fbLen; n += 1024) {
    if (n + 1024 <= fbLen) {
        size = 1024;
    } else {
        size = fbLen % 1024;
    }
    txSize[0] = size >> 8; // Assign high byte
    txSize[1] = size & 0xFF; // Assign low byte

    Serial.write(txSize, 2); // Send length
    delay(10);
    Serial.write(fbBuf, size); // Send buffer data
    fbBuf += size; // Move pointer by size
  }
  delay(10);
  txSize[0] = 0;
  txSize[1] = 0;
  Serial.write(txSize, 2); // Send length

  uint8_t receivedSize = 0;
  unsigned long previousMillis = millis();
  while(1){
    if (Serial.available() >= 1){
      receivedSize = Serial.read();
      break;
    }
    if (millis() - previousMillis >= 20000){
      break;
    }
  }
  if (receivedSize >= 128){

  }
  else if (receivedSize > 0){
    uint8_t chunkSize = receivedSize * 4;
    previousMillis = millis();
    while (1) {
      if (Serial.available() >= chunkSize)
        break;
      if (millis() - previousMillis >= 5000){
        esp_camera_fb_return(fb);
        return;
      }
    }
    uint8_t chunk[chunkSize];
    for (uint8_t i = 0; i < chunkSize; i++) {
      chunk[i] = Serial.read();
    }
    for (uint8_t i = 0; i < receivedSize;++i){
      if (client.connect(serverName.c_str(), serverPort)){
        size_t start = chunk[i * 4] * UINT8_PER_CHANNEL + chunk[i * 4 + 1] * IMAGE_WIDTH * UINT8_PER_CHANNEL;
        size_t width = chunk[i * 4 + 2] - chunk[i * 4];
        size_t height = chunk[i * 4 + 3] - chunk[i * 4 + 1];
        String path = serverPath + "?aligned=True&width=" + String(width) + "&height=" + String(height);

        width *= UINT8_PER_CHANNEL;
        client.println("POST " + path + " HTTP/1.1");
        client.println("Host: " + serverName);
        client.println("Content-Length: " + String(width*height));
        client.println("Content-Type: application/octet-stream");
        client.println();
        fbBuf = fb->buf;
        for (size_t j = 0; j < height;++j){
          client.write(fbBuf + start + (j*IMAGE_WIDTH*UINT8_PER_CHANNEL), width);
        }
        
        String getAll;
        String getBody;
        int timoutTimer = 10000;
        long startTimer = millis();
        boolean state = false;
        while ((startTimer + timoutTimer) > millis()) {
          delay(100);      
          while (client.available()) {
            char c = client.read();
            if (c == '\n') {
              if (getAll.length()==0) { state=true; }
              getAll = "";
            }
            else if (c != '\r') { getAll += String(c); }
            if (state==true) { getBody += String(c); }
            startTimer = millis();
          }
          if (getBody.length()>0) { break; }
        }
        client.stop();
      }
    }
  }

  esp_camera_fb_return(fb);
}

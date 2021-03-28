#include <Arduino.h>

#include <esp_camera.h>
//#include "color_image_model.h"

// Pin definition for CAMERA_MODEL_AI_THINKER
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

#define FRAME_SIZE FRAMESIZE_QQVGA//FRAMESIZE_CIF//FRAMESIZE_QQVGA
// FRAMESIZE_UXGA(1600 x 1200)
// FRAMESIZE_QVGA(320 x 240)
// FRAMESIZE_QQVGA(160 x 120)
// FRAMESIZE_CIF(352 x 288)
// FRAMESIZE_VGA(640 x 480)
// FRAMESIZE_SVGA(800 x 600)
// FRAMESIZE_XGA(1024 x 768)
// FRAMESIZE_SXGA(1280 x 1024)
#define WIDTH 160
#define HEIGHT 120
#define W WIDTH
#define H HEIGHT

// #define W (WIDTH / BLOCK_SIZE)
// #define H (HEIGHT / BLOCK_SIZE)

// uint16_t prev_frame[H][W] = {0};
// uint16_t current_frame[H][W] = {0};
// uint16_t rgb_frame[H][W][3] = {0};
uint16_t image_buffer[H * W] = {0};
// double features[H * W * 3] = {0};

bool setup_camera(framesize_t);
bool capture_still();
void convert_to_rbg(uint8_t *, size_t);
void linearize_features();
void print_features();
void classify();

/**
 *
 */
void setup()
{
  // Serial.begin(115200);
  Serial.begin(2000000);
  Serial.println(setup_camera(FRAME_SIZE) ? "OK" : "ERR INIT");
}

/**
 *
 */
void loop()
{
  if (!capture_still())
  {
    Serial.println("Failed capture");
    delay(2000);

    return;
  }

  
  // linearize_features();
  // print_features();
  //classify();
  Serial.println("START\n\n\n\n\n\n\n");
  for(int i = 0; i < H*W; i++)
  {
    Serial.print(image_buffer[i]);
    Serial.print("\n");
  }
  Serial.println("\n\n\n\n\n\n\nDONE");
  delay(3000);
}

/**
 *
 */
bool setup_camera(framesize_t frameSize)
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
  config.pixel_format = PIXFORMAT_GRAYSCALE; //PIXFORMAT_RGB565;
  config.frame_size = frameSize;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  bool ok = esp_camera_init(&config) == ESP_OK;

  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_framesize(sensor, frameSize);

  return ok;
}

void get_image(uint8_t *buf, size_t len)
{
  // Reset RGB
  // for (int i = 0; i < H * W; i++)
  // {
  //   image_buffer[i] = 0;
  // }
  // For GRAYSCALE
  for (size_t i = 0; i < len; i++)
  {
    image_buffer[i] = buf[i];
  }
  // // For RGB565
  // for (size_t i = 0; i < len; i += 2)
  // {
  //   //        uint16_t pix = *(uint16_t*) buf[i];
  //   //        pix = __builtin_bswap16(pix);
  //   const uint8_t high = buf[i];
  //   const uint8_t low = buf[i + 1];
  //   const uint16_t pixel = (high << 8) | low;

  //   // Serial.print(pixel);
  //   image_buffer[i/2] = pixel;
  // }
}


/**
 * Convert RGB565 to RGB888
 */
// void convert_to_rgb(uint8_t *buf, size_t len)
// {
//   for (int y = 0; y < H; y++)
//   {
//     for (int x = 0; x < W; x++)
//     {
//       rgb_frame[y][x][0] = 0;
//       rgb_frame[y][x][1] = 0;
//       rgb_frame[y][x][2] = 0;
//     }
//   }

//   for (size_t i = 0; i < len; i += 2)
//   {
//     //        uint16_t pix = *(uint16_t*) buf[i];
//     //        pix = __builtin_bswap16(pix);
//     const uint8_t high = buf[i];
//     const uint8_t low = buf[i + 1];
//     const uint16_t pixel = (high << 8) | low;

//     const uint8_t r = (pixel & 0b1111100000000000) >> 11;
//     const uint8_t g = (pixel & 0b0000011111100000) >> 5;
//     const uint8_t b = (pixel & 0b0000000000011111);

//     const size_t j = i / 2;
//     const uint16_t x = j % WIDTH;
//     const uint16_t y = floor(j / WIDTH);
//     const uint8_t block_x = floor(x / BLOCK_SIZE);
//     const uint8_t block_y = floor(y / BLOCK_SIZE);

//     // average pixels in block (accumulate)
//     rgb_frame[block_y][block_x][0] += r;
//     rgb_frame[block_y][block_x][1] += g;
//     rgb_frame[block_y][block_x][2] += b;
//   }
// }

/**
 * Capture image and do down-sampling
 */
bool capture_still()
{
  camera_fb_t *frame = esp_camera_fb_get();

  if (!frame)
    return false;
  Serial.print("frame->len ::::   ");
  Serial.println(frame->len);
  // convert_to_rgb(frame->buf, frame->len);
  // Serial.println("START");
  
  get_image(frame->buf, frame->len);
  // Serial.println("DONE");
  return true;
}

/**
 * Convert image to features vector
 */
// void linearize_features()
// {
//   size_t i = 0;

//   for (int y = 0; y < H; y++)
//   {
//     for (int x = 0; x < W; x++)
//     {
//       features[i++] = rgb_frame[y][x][0];
//       features[i++] = rgb_frame[y][x][1];
//       features[i++] = rgb_frame[y][x][2];
//     }
//   }
// }

/**
 *
 */
// void print_features()
// {
//   for (size_t i = 0; i < H * W * 3; i++)
//   {
//     Serial.println(features[i]);
//     Serial.print(',');
//   }
// }

/**
 * Run the inference
 */
void classify()
{
  Serial.print("Object: ");
  //Serial.println(classIdxToName(predict(features)));
}





























// #include "esp_camera.h"
// #include "Arduino.h"
// #include "FS.h"               // SD Card ESP32
// #include "SD_MMC.h"           // SD Card ESP32
// #include "soc/soc.h"          // Disable brownour problems
// #include "soc/rtc_cntl_reg.h" // Disable brownour problems
// #include "driver/rtc_io.h"
// // #include <EEPROM.h> // read and write from flash memory

// // // define the number of bytes you want to access
// // #define EEPROM_SIZE 1

// // Pin definition for CAMERA_MODEL_AI_THINKER
// #define PWDN_GPIO_NUM 32
// #define RESET_GPIO_NUM -1
// #define XCLK_GPIO_NUM 0
// #define SIOD_GPIO_NUM 26
// #define SIOC_GPIO_NUM 27

// #define Y9_GPIO_NUM 35
// #define Y8_GPIO_NUM 34
// #define Y7_GPIO_NUM 39
// #define Y6_GPIO_NUM 36
// #define Y5_GPIO_NUM 21
// #define Y4_GPIO_NUM 19
// #define Y3_GPIO_NUM 18
// #define Y2_GPIO_NUM 5
// #define VSYNC_GPIO_NUM 25
// #define HREF_GPIO_NUM 23
// #define PCLK_GPIO_NUM 22

// int pictureNumber = 0;

// void setup()
// {
//   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

//   Serial.begin(115200);
//   //Serial.setDebugOutput(true);
//   //Serial.println();

//   camera_config_t config;
//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer = LEDC_TIMER_0;
//   config.pin_d0 = Y2_GPIO_NUM;
//   config.pin_d1 = Y3_GPIO_NUM;
//   config.pin_d2 = Y4_GPIO_NUM;
//   config.pin_d3 = Y5_GPIO_NUM;
//   config.pin_d4 = Y6_GPIO_NUM;
//   config.pin_d5 = Y7_GPIO_NUM;
//   config.pin_d6 = Y8_GPIO_NUM;
//   config.pin_d7 = Y9_GPIO_NUM;
//   config.pin_xclk = XCLK_GPIO_NUM;
//   config.pin_pclk = PCLK_GPIO_NUM;
//   config.pin_vsync = VSYNC_GPIO_NUM;
//   config.pin_href = HREF_GPIO_NUM;
//   config.pin_sscb_sda = SIOD_GPIO_NUM;
//   config.pin_sscb_scl = SIOC_GPIO_NUM;
//   config.pin_pwdn = PWDN_GPIO_NUM;
//   config.pin_reset = RESET_GPIO_NUM;
//   config.xclk_freq_hz = 20000000;
//   // config.pixel_format = PIXFORMAT_JPEG;
//   config.pixel_format = PIXFORMAT_RGB565;

//   if (psramFound())
//   {
//     // config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
//     config.frame_size = FRAMESIZE_CIF; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
//     config.jpeg_quality = 10;
//     config.fb_count = 2;
//   }
//   else
//   {
//     config.frame_size = FRAMESIZE_SVGA;
//     config.jpeg_quality = 12;
//     config.fb_count = 1;
//   }

//   // Init Camera
//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK)
//   {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     return;
//   }

//   // //Serial.println("Starting SD Card");
//   // if (!SD_MMC.begin())
//   // {
//   //   Serial.println("SD Card Mount Failed");
//   //   return;
//   // }

//   // uint8_t cardType = SD_MMC.cardType();
//   // if (cardType == CARD_NONE)
//   // {
//   //   Serial.println("No SD Card attached");
//   //   return;
//   // }

//   camera_fb_t *fb = NULL;

//   // Take Picture with Camera
//   fb = esp_camera_fb_get();
//   if (!fb)
//   {
//     Serial.println("Camera capture failed");
//     return;
//   }
//   // initialize EEPROM with predefined size
//   // EEPROM.begin(EEPROM_SIZE);
//   // pictureNumber = EEPROM.read(0) + 1;

//   // Path where new picture will be saved in SD Card
//   String path = "/picture" + String(pictureNumber) + ".jpg";

//   // fs::FS &fs = SD_MMC;
//   Serial.printf("Picture file name: %s\n", path.c_str());

//   // File file = fs.open(path.c_str(), FILE_WRITE);
//   // if (!file)
//   // {
//   //   Serial.println("Failed to open file in writing mode");
//   // }
//   // else
//   // {
//   //   file.write(fb->buf, fb->len); // payload (image), payload length
//   //   Serial.printf("Saved file to path: %s\n", path.c_str());
//   //   EEPROM.write(0, pictureNumber);
//   //   EEPROM.commit();
//   // }
//   // file.close();

//   const char *data = (const char *)fb->buf;
//   size_t size = fb->len;

//   Serial.println(data);
//   Serial.println("\n\n\n");
//   Serial.println(size);
//   esp_camera_fb_return(fb);

//   // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
//   pinMode(4, OUTPUT);
//   digitalWrite(4, LOW);
//   rtc_gpio_hold_en(GPIO_NUM_4);

//   delay(2000);
//   Serial.println("Going to sleep now");
//   delay(2000);
//   esp_deep_sleep_start();
//   Serial.println("This will never be printed");
// }

// void loop()
// {
// }





































































































// WEB SERVER //




// #include <Arduino.h>

// #include "esp_camera.h"
// #include <WiFi.h>
// #include "esp_timer.h"
// #include "img_converters.h"
// #include "Arduino.h"
// #include "fb_gfx.h"
// #include "soc/soc.h"          //disable brownout problems
// #include "soc/rtc_cntl_reg.h" //disable brownout problems
// #include "esp_http_server.h"

// //Replace with your network credentials
// const char *ssid = "AUTOBOT";
// const char *password = "robot109678";

// #define PART_BOUNDARY "123456789000000000000987654321"

// // This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
// #define CAMERA_MODEL_AI_THINKER
// //#define CAMERA_MODEL_M5STACK_PSRAM
// //#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// // Not tested with this model
// //#define CAMERA_MODEL_WROVER_KIT

// #if defined(CAMERA_MODEL_WROVER_KIT)
// #define PWDN_GPIO_NUM -1
// #define RESET_GPIO_NUM -1
// #define XCLK_GPIO_NUM 21
// #define SIOD_GPIO_NUM 26
// #define SIOC_GPIO_NUM 27

// #define Y9_GPIO_NUM 35
// #define Y8_GPIO_NUM 34
// #define Y7_GPIO_NUM 39
// #define Y6_GPIO_NUM 36
// #define Y5_GPIO_NUM 19
// #define Y4_GPIO_NUM 18
// #define Y3_GPIO_NUM 5
// #define Y2_GPIO_NUM 4
// #define VSYNC_GPIO_NUM 25
// #define HREF_GPIO_NUM 23
// #define PCLK_GPIO_NUM 22

// #elif defined(CAMERA_MODEL_M5STACK_PSRAM)
// #define PWDN_GPIO_NUM -1
// #define RESET_GPIO_NUM 15
// #define XCLK_GPIO_NUM 27
// #define SIOD_GPIO_NUM 25
// #define SIOC_GPIO_NUM 23

// #define Y9_GPIO_NUM 19
// #define Y8_GPIO_NUM 36
// #define Y7_GPIO_NUM 18
// #define Y6_GPIO_NUM 39
// #define Y5_GPIO_NUM 5
// #define Y4_GPIO_NUM 34
// #define Y3_GPIO_NUM 35
// #define Y2_GPIO_NUM 32
// #define VSYNC_GPIO_NUM 22
// #define HREF_GPIO_NUM 26
// #define PCLK_GPIO_NUM 21

// #elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
// #define PWDN_GPIO_NUM -1
// #define RESET_GPIO_NUM 15
// #define XCLK_GPIO_NUM 27
// #define SIOD_GPIO_NUM 25
// #define SIOC_GPIO_NUM 23

// #define Y9_GPIO_NUM 19
// #define Y8_GPIO_NUM 36
// #define Y7_GPIO_NUM 18
// #define Y6_GPIO_NUM 39
// #define Y5_GPIO_NUM 5
// #define Y4_GPIO_NUM 34
// #define Y3_GPIO_NUM 35
// #define Y2_GPIO_NUM 17
// #define VSYNC_GPIO_NUM 22
// #define HREF_GPIO_NUM 26
// #define PCLK_GPIO_NUM 21

// #elif defined(CAMERA_MODEL_AI_THINKER)
// #define PWDN_GPIO_NUM 32
// #define RESET_GPIO_NUM -1
// #define XCLK_GPIO_NUM 0
// #define SIOD_GPIO_NUM 26
// #define SIOC_GPIO_NUM 27

// #define Y9_GPIO_NUM 35
// #define Y8_GPIO_NUM 34
// #define Y7_GPIO_NUM 39
// #define Y6_GPIO_NUM 36
// #define Y5_GPIO_NUM 21
// #define Y4_GPIO_NUM 19
// #define Y3_GPIO_NUM 18
// #define Y2_GPIO_NUM 5
// #define VSYNC_GPIO_NUM 25
// #define HREF_GPIO_NUM 23
// #define PCLK_GPIO_NUM 22
// #else
// #error "Camera model not selected"
// #endif

// static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
// static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
// static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// httpd_handle_t stream_httpd = NULL;

// static esp_err_t stream_handler(httpd_req_t *req)
// {
//   camera_fb_t *fb = NULL;
//   esp_err_t res = ESP_OK;
//   size_t _jpg_buf_len = 0;
//   uint8_t *_jpg_buf = NULL;
//   char *part_buf[64];

//   res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//   if (res != ESP_OK)
//   {
//     return res;
//   }

//   while (true)
//   {
//     fb = esp_camera_fb_get();
//     if (!fb)
//     {
//       Serial.println("Camera capture failed");
//       res = ESP_FAIL;
//     }
//     else
//     {
//       if (fb->width > 400)
//       {
//         if (fb->format != PIXFORMAT_JPEG)
//         {
//           bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
//           esp_camera_fb_return(fb);
//           fb = NULL;
//           if (!jpeg_converted)
//           {
//             Serial.println("JPEG compression failed");
//             res = ESP_FAIL;
//           }
//         }
//         else
//         {
//           _jpg_buf_len = fb->len;
//           _jpg_buf = fb->buf;
//         }
//       }
//     }
//     if (res == ESP_OK)
//     {
//       size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
//       res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
//     }
//     if (res == ESP_OK)
//     {
//       res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
//     }
//     if (res == ESP_OK)
//     {
//       res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//     }
//     if (fb)
//     {
//       esp_camera_fb_return(fb);
//       fb = NULL;
//       _jpg_buf = NULL;
//     }
//     else if (_jpg_buf)
//     {
//       free(_jpg_buf);
//       _jpg_buf = NULL;
//     }
//     if (res != ESP_OK)
//     {
//       break;
//     }
//     //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
//   }
//   return res;
// }

// void startCameraServer()
// {
//   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//   config.server_port = 80;

//   httpd_uri_t index_uri = {
//       .uri = "/",
//       .method = HTTP_GET,
//       .handler = stream_handler,
//       .user_ctx = NULL};

//   //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
//   if (httpd_start(&stream_httpd, &config) == ESP_OK)
//   {
//     httpd_register_uri_handler(stream_httpd, &index_uri);
//   }
// }

// void setup()
// {
//   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

//   Serial.begin(115200);
//   Serial.setDebugOutput(false);

//   camera_config_t config;
//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer = LEDC_TIMER_0;
//   config.pin_d0 = Y2_GPIO_NUM;
//   config.pin_d1 = Y3_GPIO_NUM;
//   config.pin_d2 = Y4_GPIO_NUM;
//   config.pin_d3 = Y5_GPIO_NUM;
//   config.pin_d4 = Y6_GPIO_NUM;
//   config.pin_d5 = Y7_GPIO_NUM;
//   config.pin_d6 = Y8_GPIO_NUM;
//   config.pin_d7 = Y9_GPIO_NUM;
//   config.pin_xclk = XCLK_GPIO_NUM;
//   config.pin_pclk = PCLK_GPIO_NUM;
//   config.pin_vsync = VSYNC_GPIO_NUM;
//   config.pin_href = HREF_GPIO_NUM;
//   config.pin_sscb_sda = SIOD_GPIO_NUM;
//   config.pin_sscb_scl = SIOC_GPIO_NUM;
//   config.pin_pwdn = PWDN_GPIO_NUM;
//   config.pin_reset = RESET_GPIO_NUM;
//   config.xclk_freq_hz = 20000000;
//   config.pixel_format = PIXFORMAT_JPEG;

//   if (psramFound())
//   {
//     config.frame_size = FRAMESIZE_UXGA;
//     config.jpeg_quality = 10;
//     config.fb_count = 2;
//   }
//   else
//   {
//     config.frame_size = FRAMESIZE_SVGA;
//     config.jpeg_quality = 12;
//     config.fb_count = 1;
//   }

//   // Camera init
//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK)
//   {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     return;
//   }
//   // Wi-Fi connection
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("");
//   Serial.println("WiFi connected");

//   Serial.print("Camera Stream Ready! Go to: http://");
//   Serial.print(WiFi.localIP());

//   // Start streaming web server
//   startCameraServer();
// }

// void loop()
// {
//   delay(1);
// }
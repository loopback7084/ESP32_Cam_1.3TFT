#include <TFT_eSPI.h>
#include <esp_camera.h>
#include <JPEGDecoder.h>  // JPEG decoder library
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <base64.h>

#define wifi_width 18
#define wifi_height 16

static unsigned char wifi_bits[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0x00, 0xf0, 0x3f, 0x00,
   0xf8, 0xff, 0x00, 0x3c, 0xe0, 0x00, 0x08, 0x45, 0x00, 0xe0, 0x1f, 0x00,
   0xe0, 0x1f, 0x00, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  4
#define SIOD_GPIO_NUM  18
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    36
#define Y8_GPIO_NUM    37
#define Y7_GPIO_NUM    38
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    35
#define Y4_GPIO_NUM    26
#define Y3_GPIO_NUM    13
#define Y2_GPIO_NUM    34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM  27
#define PCLK_GPIO_NUM  25

static const char* ssid =     "ESP32Cam";
static const char* password = "ESP32Cam";

// this function determines the minimum of two numbers
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))


TFT_eSPI tft = TFT_eSPI();
DNSServer dnsServer;
WebServer webServer(80);
TaskHandle_t task1;

static uint64_t s = 0;
static uint64_t ts = 0;
static uint16_t i = 10;
static char buffer[64];
static char ipstr[16];
static camera_fb_t * fb1 = NULL;
static camera_fb_t * fb2 = NULL;

void jpegRender(int xpos, int ypos);

IPAddress ipaddr =  {192, 168, 000, 001};
IPAddress netmask = {255, 255, 255, 000};
IPAddress gateway = {192, 168, 000, 001};
IPAddress dns1 =    {192, 168, 000, 001};

void serveWeb ( void )
{
  String response = "";
  
  if ( fb2 = esp_camera_fb_get() )
  {

    response += "<img alt=\"Camera Frame\" title=\"Camera Frame\" style=\"width: 100%;\" ";

    response += "src=\"data:image/jpeg;charset=utf-8;base64, ";
      
    response += base64::encode ( fb2->buf, fb2->len );
    
    response += "\"";
    
    response += "/>";

    esp_camera_fb_return ( fb2 );
    
    fb2 = NULL; 
  }
  
  webServer.send ( 200, "text/html", response );
}

void handleWebClient ( void * v )
{
  while ( 1 )
  {
    webServer.handleClient();
    vTaskDelay ( 25 );
  }
}

void setup() {

  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig ( ipaddr, gateway, netmask );
  WiFi.softAP ( ssid, password );
  snprintf ( ipstr, 16, "%u.%u.%u.%u", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3] );
  dnsServer.start ( 53, "*", ipaddr );

  webServer.on ( "/", serveWeb );
  webServer.begin();
  
  // put your setup code here, to run once:
  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor ( TFT_WHITE );

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
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 8;
  config.fb_count = 1;
  
  esp_err_t err = esp_camera_init(&config);
  
  if (err != ESP_OK)
  {
    Serial.printf("Camera init Fail");
  }
  /*
  sensor_t * s = esp_camera_sensor_get();
  s->set_hmirror ( s, 1 );
  */
  tft.drawCentreString ( "Started", TFT_XC, TFT_YC, 1 );
  tft.drawXBitmap(220, 0, wifi_bits, wifi_width, wifi_height, TFT_WHITE);
  tft.drawString ( ipstr, 2, 2, 2 );

  xTaskCreatePinnedToCore ( handleWebClient, "webTask", 10000, NULL, 2, &task1, 1 );
}







void loop()
{
  s = millis() / 1000;
  
  if ( s != ts )
  {
    ts = s;

    tft.fillRect ( 210, 0, 10, 20, TFT_BLACK );
    snprintf ( buffer, 63, "%lu", WiFi.softAPgetStationNum() );
    tft.drawString ( buffer, 210, 2, 2 );
  }

  if ( fb1 = esp_camera_fb_get() )
  {
    if ( JpegDec.decodeArray ( fb1->buf, fb1->len ) )
    {
      jpegRender(-40,20);
    }

    esp_camera_fb_return ( fb1 );
    
    fb1 = NULL;
  }

  vTaskDelay ( 25 );
}

















void jpegRender(int xpos, int ypos)
{

  //jpegInfo(); // Print information from the JPEG file (could comment this line out)

  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = min(mcu_w, max_x % mcu_w);
  uint32_t min_h = min(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // Fetch data from the file, decode and display
  while (JpegDec.read()) {    // While there is more data in the file
    pImg = JpegDec.pImage ;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)

    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ( (mcu_y + win_h) >= tft.height())
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
  }

  tft.setSwapBytes(swapBytes);
  
}

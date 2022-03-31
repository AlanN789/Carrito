/************************ Adafruit IO Config *******************************/                  
#define IO_USERNAME "Your_Username"
#define IO_KEY "aio_BeGB50AA5jCfqXQLSKFqk5rdasdsa"
/******************************* WIFI **************************************/
#define WIFI_SSID "Your_Wifi"
#define WIFI_PASS "Yourwifipassword"
#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

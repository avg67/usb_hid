#include <FreeRTOS.h>
#include "usbh_core.h"
#include "usbh_hid.h"
#include "bflb_gpio.h"
//#include "bflb_uart.h"
#include "hidparser.h"
#include <semphr.h>

// Global HID LED request structure
struct hid_led_request {
    int hid_index;                      // Which HID device
    uint8_t led_state;                  // LED state bitmask
    SemaphoreHandle_t completion_sem;   // Signals when complete
    int result;                         // Return value
};
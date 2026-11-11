// usb_host.c

#include <FreeRTOS.h>
#include "usbh_core.h"
#include "usbh_hid.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "hidparser.h"
#include <semphr.h>
#include "hid_sync_led_control.h"
#include "ps2keyboard.h"

#define MAX_REPORT_SIZE 8

#define STATE_NONE      0 
#define STATE_DETECTED  1 
#define STATE_RUNNING   2
#define STATE_FAILED    3

struct hid_info_S {
  int state;
  int index;
  struct usbh_hid *class;
  struct usbh_urb intin_urb;
  volatile bool urb_busy;     // true while intin_urb is submitted and not yet completed
  uint8_t *buffer;
  hid_report_t report;
} hid_info[CONFIG_USBHOST_MAX_HID_CLASS];

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t hid_buffer[CONFIG_USBHOST_MAX_HID_CLASS][MAX_REPORT_SIZE];

// DMA buffer for SET_REPORT (LEDs). Must be in non-cacheable RAM like all other
// buffers handed to the EHCI controller, otherwise the controller reads stale data.
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t hid_led_buffer[CONFIG_USBHOST_MAX_HID_CLASS][USB_ALIGN_UP(2, CONFIG_USB_ALIGN_SIZE)];

// Reports received in the URB completion callback (interrupt context) are copied
// into this queue and processed in the usbh_hid thread.
struct hid_report_msg {
  uint8_t index;
  uint8_t len;
  uint8_t data[MAX_REPORT_SIZE];
};
#define HID_REPORT_QUEUE_LEN 32
static QueueHandle_t hid_report_queue = NULL;

extern struct bflb_device_s *gpio;
// Global queue for LED requests (length 1, always holds the latest LED state)
static QueueHandle_t hid_led_request_queue = NULL;
static uint8_t current_led_state = 0;     // last requested LED state (restored on new keyboards)
static TaskHandle_t usb_hid_task_handle = NULL;  // Store USB thread handle

int set_keyboard_leds_thread_safe(int hid_index, uint8_t led_state);

#define DEV_KBD   0
#define DEV_MOUSE 1

void ps2_tx_bit(int dev, char bit) {
  /*                             KBD         MOUSE    */
#ifdef M0S_DOCK
  // M0S Dock
  static const int clk[]  = { GPIO_PIN_10, GPIO_PIN_12 };
  static const int data[] = { GPIO_PIN_11, GPIO_PIN_13 };
#else
  #warning "Building for on-board BL616!"
  // on-board BL616
  static const int clk[]  = { GPIO_PIN_1, GPIO_PIN_2 };
  static const int data[] = { GPIO_PIN_0, GPIO_PIN_3 };
#endif
  
  // intended clock is ~14kHz
  
  // set data bit
  if(bit) bflb_gpio_set(gpio, data[dev]);
  else    bflb_gpio_reset(gpio, data[dev]);
  bflb_mtimer_delay_us(10);
    
  // clock low
  bflb_gpio_reset(gpio, clk[dev]);
  bflb_mtimer_delay_us(30);
  
  // clock high
  bflb_gpio_set(gpio, clk[dev]);
  bflb_mtimer_delay_us(30);
}

 
extern struct bflb_device_s *uart1;
// Define a handle for the queue
usb_osal_mq_t my_queue;
//#include "bflb_uart.h"

void ps2_tx_byte(const int dev, const unsigned char byte) {
  char parity = 1;

  //printf("%c-TX %02x\r\n", (dev==DEV_KBD)?'K':'M', byte); // AV removed
  
  ps2_tx_bit(dev, 0);     // start bit

  {
    unsigned char temp=byte;  // AV modified
    for(int i=0;i<8;i++) {
      ps2_tx_bit(dev, temp & 1);
      if(temp & 1) parity = !parity;
      temp >>= 1;
    }
  }
    
  ps2_tx_bit(dev, parity);
  ps2_tx_bit(dev, 1);     // stop bit
  if(dev==DEV_KBD) {
    //bflb_uart_putchar(uart1,byte);
    //xQueueSend()
    usb_osal_mq_send(my_queue, (uintptr_t)byte);
  }

  // stop bit leaves clk and data high
}

#include "keycodes.h"

void ps2_kbd_tx_ps2(char make, unsigned short ps2) {
  if(ps2 & EXT)  ps2_tx_byte(DEV_KBD, 0xe0);
  
  // send break code
  if(!make)  ps2_tx_byte(DEV_KBD, 0xf0);

  ps2_tx_byte(DEV_KBD, ps2 & 0xff);
}

void ps2_kbd_tx(char make, unsigned char code) {
  // ignore everything outside table and keys without PS/2 mapping
  if(code > KEYCODE_MAX || usb2ps2[code] == MISS) return;

  ps2_kbd_tx_ps2(make, usb2ps2[code]);
}

// ---------------------------------------------------------------------------
// Typematic (key repeat). USB keyboards do not repeat keys themselves, the
// host has to do it. Same timing as the former PS/2 setup: 500ms / 16 cps.
// All functions run in the usbh_hid thread.
// ---------------------------------------------------------------------------
#define KBD_REPEAT_DELAY_MS   500
#define KBD_REPEAT_RATE_MS    62      // ~16 characters per second

static unsigned char kbd_last_report[8] = { 0,0,0,0,0,0,0,0 };
static unsigned char kbd_repeat_key = 0;  // USB usage code of the repeating key, 0 = none
static TickType_t    kbd_repeat_next;     // tick of the next repeat

static bool kbd_key_repeatable(unsigned char code) {
  switch(code) {
    case 0x39:  // Caps Lock
    case 0x47:  // Scroll Lock
    case 0x48:  // Pause
    case 0x53:  // Num Lock
      return false;
    default:
      return (code <= KEYCODE_MAX) && (usb2ps2[code] != MISS);
  }
}

static bool kbd_key_in_report(const unsigned char *report, unsigned char code) {
  for(int i=0;i<6;i++)
    if(report[2+i] == code) return true;
  return false;
}

// forget all pressed keys (keyboard lost/failed)
static void kbd_reset_state(void) {
  memset(kbd_last_report, 0, sizeof(kbd_last_report));
  kbd_repeat_key = 0;
}

// called periodically from the usbh_hid thread
static void kbd_repeat_poll(void) {
  if(!kbd_repeat_key) return;

  const TickType_t now = xTaskGetTickCount();
  if((int32_t)(now - kbd_repeat_next) >= 0) {
    ps2_kbd_tx(1, kbd_repeat_key);         // PS/2 typematic: repeated make code only
    // schedule relative to the planned time to keep the average rate exact
    kbd_repeat_next += pdMS_TO_TICKS(KBD_REPEAT_RATE_MS);
    if((int32_t)(now - kbd_repeat_next) >= 0)   // far behind: don't send bursts
      kbd_repeat_next = now + pdMS_TO_TICKS(KBD_REPEAT_RATE_MS);
  }
}

void kbd_parse(unsigned char *buffer, int nbytes) {
  /* usb modifer bits:
        0     1     2    3    4     5     6    7
     LCTRL LSHIFT LALT LGUI RCTRL RSHIFT RALT RGUI
  */

  static const unsigned short ps2_modifier[] = 
    { 0x14, 0x12, 0x11, EXT|0x1f, EXT|0x14, 0x59, EXT|0x11, EXT|0x27 };

  // we expect boot mode packets which are exactly 8 bytes long
  if(nbytes != 8) return;

  // phantom state (too many keys pressed, all slots 0x01): ignore report,
  // keep the last valid state
  if(buffer[2] == 0x01) return;

  unsigned char *last_report = kbd_last_report;

  // check if modifier have changed
  if(buffer[0] != last_report[0]) {
    for(int i=0;i<8;i++) {
      // modifier released?
      if((last_report[0] & (1<<i)) && !(buffer[0] & (1<<i)))
	 ps2_kbd_tx_ps2(0, ps2_modifier[i]);
      // modifier pressed?
      if(!(last_report[0] & (1<<i)) && (buffer[0] & (1<<i)))
	 ps2_kbd_tx_ps2(1, ps2_modifier[i]);
    }
  }

  // Released keys: compare by key code, not by slot position. Keyboards may
  // shift the remaining keys to other slots when a key is released.
  for(int i=0;i<6;i++) {
    const unsigned char code = last_report[2+i];
    if(code && !kbd_key_in_report(buffer, code)) {
      ps2_kbd_tx(0, code);
      if(code == kbd_repeat_key) kbd_repeat_key = 0;
    }
  }

  // newly pressed keys
  for(int i=0;i<6;i++) {
    const unsigned char code = buffer[2+i];
    if(code && !kbd_key_in_report(last_report, code)) {
      ps2_kbd_tx(1, code);
      // the most recently pressed key repeats (like a PS/2 keyboard)
      if(kbd_key_repeatable(code)) {
        kbd_repeat_key = code;
        kbd_repeat_next = xTaskGetTickCount() + pdMS_TO_TICKS(KBD_REPEAT_DELAY_MS);
      } else {
        kbd_repeat_key = 0;
      }
    }
  }
  
  memcpy(last_report, buffer, 8);
}

void mouse_parse(signed char *buffer, int nbytes) {
  // we expect at least three bytes:
  if(nbytes < 3) return;

  // decellerate mouse somewhat and invert y-axis
  buffer[1] /= 2;
  buffer[2] = -buffer[2]/2;
  
  // first byte contains buttons and overflow and sign bits
  unsigned char b0 = buffer[0] & 3;  // buttons
  b0 |= 0x08;                        // always 1
  if(buffer[2] < 0) b0 |= 0x10;      // negative x
  if(buffer[1] < 0) b0 |= 0x20;      // negative y

  // always send imps four byte package
  ps2_tx_byte(DEV_MOUSE, b0);
  ps2_tx_byte(DEV_MOUSE, buffer[2]);
  ps2_tx_byte(DEV_MOUSE, buffer[1]);
  ps2_tx_byte(DEV_MOUSE, 0x00);      // z-axis is unused
}

// Runs in thread context (usbh_hid thread)
static void hid_process_report(struct hid_info_S *hid_info, unsigned char *buffer, int nbytes) {
  if (nbytes > 0) {
#ifdef USE_USB_LOG_RAW
    // just dump the report
    for (size_t i = 0; i < nbytes; i++) 
      USB_LOG_RAW("0x%02x ", buffer[i]);
    USB_LOG_RAW("\r\n");
#endif    
    // parse reply
    
    // check and skip report id if present
    if(hid_info->report.report_id_present) {
      if(!nbytes || (buffer[0] != hid_info->report.report_id))
	      return;

      // skip report id
      buffer++; nbytes--;
    }
  
    if(nbytes == hid_info->report.report_size) {
      if(hid_info->report.type == REPORT_TYPE_KEYBOARD)
	      kbd_parse(buffer, nbytes);
      
      if(hid_info->report.type == REPORT_TYPE_MOUSE)
      	mouse_parse((signed char*)buffer, nbytes);
    }
  }
  //  else
  //    USB_LOG_RAW("no data\r\n");
}  

// URB completion callback. Called from the EHCI interrupt (or from usbh_kill_urb
// in thread context). Must not block: only copy the report and hand it over to
// the usbh_hid thread.
void usbh_hid_callback(void *arg, int nbytes) {
  struct hid_info_S *hid_info = (struct hid_info_S *)arg;

  hid_info->urb_busy = false;

  // nbytes < 0 means something went wrong. E.g. a devices connected to a hub was
  // disconnected, but the hub wasn't monitored
  if(nbytes < 0) {
    hid_info->state = STATE_FAILED;
    return;
  }

  if(nbytes == 0 || !hid_report_queue)
    return;

  struct hid_report_msg msg;
  msg.index = (uint8_t)hid_info->index;
  msg.len = (uint8_t)((nbytes > MAX_REPORT_SIZE) ? MAX_REPORT_SIZE : nbytes);
  memcpy(msg.data, hid_info->buffer, msg.len);

  if (xPortIsInsideInterrupt()) {
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(hid_report_queue, &msg, &woken);
    portYIELD_FROM_ISR(woken);
  } else {
    xQueueSend(hid_report_queue, &msg, 0);
  }
}

static void usbh_hid_update(void) {
  // check for active devices
  for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
    char dev_str[] = "/dev/inputX";
    dev_str[10] = '0' + i;
    hid_info[i].class = (struct usbh_hid *)usbh_find_class_instance(dev_str);
    
    if(hid_info[i].class && hid_info[i].state == STATE_NONE) {
      printf("NEW %d\r\n", i);

      const uint8_t intf = hid_info[i].class->intf;
      printf("Interface %d\r\n", intf);
      if (intf < CONFIG_USBHOST_MAX_INTERFACES) {
        struct usbh_interface *itf = &hid_info[i].class->hport->config.intf[intf];
        printf("Interval: %d\r\n", itf->altsetting[0].ep[0].ep_desc.bInterval);
        printf("  class %d\r\n", itf->altsetting[0].intf_desc.bInterfaceClass);
        printf("  subclass %d\r\n", itf->altsetting[0].intf_desc.bInterfaceSubClass);
        printf("  protocol %d\r\n", itf->altsetting[0].intf_desc.bInterfaceProtocol);
      }
	
      // parse report descriptor ...
      printf("report descriptor: %p\r\n", hid_info[i].class->report_desc);
      
      if(!parse_report_descriptor(hid_info[i].class->report_desc, 128, &hid_info[i].report)) {
	      hid_info[i].state = STATE_FAILED;   // parsing failed, don't use
	      continue;
      }
      
      hid_info[i].state = STATE_DETECTED;
    }
    
    else if(!hid_info[i].class && hid_info[i].state != STATE_NONE) {
      printf("LOST %d\r\n", i);
      hid_info[i].state = STATE_NONE;
      hid_info[i].urb_busy = false;
    }
  }
}

int usbh_hid_set_report_patched(struct usbh_hid *hid_class, uint8_t report_type, uint8_t report_id, uint8_t *buffer, uint32_t buflen) 
{ 
    struct usb_setup_packet *setup; 
  
    if (!hid_class || !hid_class->hport) { 
        return -USB_ERR_INVAL; 
    } 
    setup = hid_class->hport->setup; 
  
    setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_INTERFACE; 
    setup->bRequest = HID_REQUEST_SET_REPORT; 
    setup->wValue = (uint16_t)(((uint32_t)report_type << 8U) | (uint32_t)report_id); 
    setup->wIndex = hid_class->intf; 
    setup->wLength = buflen; 
  
    return usbh_control_transfer(hid_class->hport, setup, buffer); 
} 

// Send the LED output report to keyboard i via SET_REPORT on EP0.
// The data stage is read by the EHCI controller via DMA -> non-cacheable buffer.
// In report protocol (boot protocol not set) keyboards with report IDs expect the
// report ID as first byte; the keyboard input report ID is used for this.
static int hid_send_led_report(int i, uint8_t leds_mask)
{
    uint8_t report_id = 0;
    uint32_t len = 0;

    if (hid_info[i].report.report_id_present) {
        report_id = hid_info[i].report.report_id;
        hid_led_buffer[i][len++] = report_id;
    }
    hid_led_buffer[i][len++] = leds_mask & 0x1f;  // Only bits 0-4 are valid LED bits

    return usbh_hid_set_report_patched(hid_info[i].class, HID_REPORT_OUTPUT, report_id,
                                       hid_led_buffer[i], len);
}

int usb_set_all_keyboard_leds(uint8_t leds_mask) 
{ 
    int result = 0;

    for (int i = 0; i < CONFIG_USBHOST_MAX_HID_CLASS; i++) { 
        if (hid_info[i].state != STATE_RUNNING || !hid_info[i].class) { 
            continue; 
        } 
        if (hid_info[i].report.type != REPORT_TYPE_KEYBOARD) { 
            continue; 
        } 
        printf("Setting LEDs for keyboard %d: mask=0x%02x\r\n", i, leds_mask & 0x1f);
        int ret = hid_send_led_report(i, leds_mask); 
        if (ret < 0) {
            printf("kbd%d SET_REPORT(LED) failed: %d\r\n", i, ret);
            result = ret;
        }
    } 
    return result;
} 

static int set_keyboard_leds_internal(struct hid_led_request *led_req)
{
    //struct usb_setup_packet *setup;
    //uint8_t led_buffer[1];
    //int ret;

    if (led_req->hid_index >= CONFIG_USBHOST_MAX_HID_CLASS) {
        printf("Invalid HID index: %d\r\n", led_req->hid_index);
        return -1;
    }
    
    #if 0
    struct hid_info_S *hid = &hid_info[led_req->hid_index];

    if (!hid->class || hid->state != STATE_RUNNING) {
        printf("HID device %d not ready (state=%d)\r\n", led_req->hid_index, hid->state);
        return -2;
    }

    if (hid->report.type != REPORT_TYPE_KEYBOARD) {
        printf("HID device %d is not a keyboard (type=%d)\r\n", 
               led_req->hid_index, hid->report.type);
        return -3;
    }

    if (!hid->class->hport) {
        printf("HID device %d has invalid hport\r\n", led_req->hid_index);
        return -4;
    }
#endif
    //led_buffer[0] = led_req->led_state;
    /*uint8_t state = led_req->led_state;
    led_buffer[0] = ((state & 0x01) << 2) | (state & 0x02) | ((state & 0x04) >> 2);
    printf("Using swapped bits: 0x%02x\r\n", led_buffer[0]);*/

    //printf("Setting LED: index=%d, state=0x%02x (Num=%d, Caps=%d, Scroll=%d), intf=%d\r\n",
    printf("Setting LED: index=%d, state=0x%02x (Num=%d, Caps=%d, Scroll=%d)\r\n",
           led_req->hid_index,
           led_req->led_state,
           (led_req->led_state >> 0) & 1,
           (led_req->led_state >> 1) & 1,
           (led_req->led_state >> 2) & 1);
          // hid->class->intf);
#if 0
    // Manual setup to workaround CherryUSB bug
    struct usb_setup_packet* setup = hid->class->hport->setup;
    
    setup->bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_INTERFACE;
    setup->bRequest = HID_REQUEST_SET_REPORT;
    setup->wValue = (uint16_t)((HID_REPORT_OUTPUT << 8) | 0);  // Report type (OUTPUT) in high byte
    setup->wIndex = hid->class->intf;  // FIX: Use interface number instead of 0!
    setup->wLength = 1;
    /*struct usb_setup_packet mysetup;
    
    mysetup.bmRequestType = USB_REQUEST_DIR_OUT | USB_REQUEST_CLASS | USB_REQUEST_RECIPIENT_INTERFACE;
    mysetup.bRequest = HID_REQUEST_SET_REPORT;
    mysetup.wValue = (HID_REPORT_OUTPUT << 8) | 0x00; // 0x0200 (Type: Output, ID: 0)
    mysetup.wIndex = hid->class->intf; // Interface-Nummer der Tastatur
    mysetup.wLength = 1;*/

    printf("USB Setup: bmRequestType=0x%02x, bRequest=0x%02x, wValue=0x%04x, wIndex=0x%04x, wLength=%d\r\n",
           setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);

    ret = usbh_control_transfer(hid->class->hport, setup, led_buffer);
    //usbh_hid_set_report(hid_class, 0, 0x02, &led_mask, 1);

    if (ret < 0) {
        printf("usbh_control_transfer failed: %d\r\n", ret);
        return ret;
    }
    #endif
    current_led_state = led_req->led_state;
    const int ret = usb_set_all_keyboard_leds(led_req->led_state);

    if (ret == 0)
        printf("LED request completed successfully\r\n");
    return ret;
}


/*static int set_keyboard_leds_internal(struct hid_led_request *led_req)
{
    uint8_t led_buffer[1];
    int ret;

    struct hid_info_S *hid = &hid_info[led_req->hid_index];

    if (!hid->class || hid->state != STATE_RUNNING) {
        printf("HID device not ready\r\n");
        return -2;
    }

    if (hid->report.type != REPORT_TYPE_KEYBOARD) {
        printf("Not a keyboard\r\n");
        return -3;
    }

    // CRITICAL CHECK: Does intout endpoint exist?
    if (!hid->class->intout) {
        printf("ERROR: No interrupt OUT endpoint! Keyboard may not support LED control.\r\n");
        printf("intout: %p\r\n", hid->class->intout);
        return -5;
    }

    if (!hid->class->hport) {
        printf("ERROR: No hport!\r\n");
        return -6;
    }

    memset(led_buffer, 0, sizeof(led_buffer));
    led_buffer[0] = led_req->led_state;

    printf("Sending LED via interrupt OUT: 0x%02x\r\n", led_req->led_state);

    // Create a synchronous URB transfer
    struct usbh_urb urb;
    memset(&urb, 0, sizeof(urb));

    // Correct signature: usbh_int_urb_fill(urb, hport, ep, buffer, length, timeout, callback, arg)
    usbh_int_urb_fill(&urb, 
                      hid->class->hport,      // hport (second param, not ep!)
                      hid->class->intout,     // ep (third param)
                      led_buffer, 
                      1, 
                      5000,                   // timeout in ms
                      NULL,                   // complete callback
                      NULL);                  // arg
    
    ret = usbh_submit_urb(&urb);
    printf("URB submit returned: %d\r\n", ret);
    
    if (ret < 0) {
        printf("Failed to submit URB\r\n");
        return ret;
    }

    printf("LED request sent via interrupt OUT\r\n");
    return 0;
}*/

static void hid_submit_intin(int i) {
  hid_info[i].urb_busy = true;
  int ret = usbh_submit_urb(&hid_info[i].intin_urb);
  if (ret < 0 && ret != -USB_ERR_BUSY) {   // BUSY: previous URB still in flight
    hid_info[i].urb_busy = false;
    // submit failed
    printf("%d: submit failed: %d\r\n", i, ret);
    hid_info[i].state = STATE_FAILED;
  }
}

static void usbh_hid_thread(void *argument)
{
  int ret;
  //usbh_hid_keyboard_init(hid->class);

  while (1) {

    usbh_hid_update();
    
    for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
      if(hid_info[i].state == STATE_DETECTED) {
	      printf("NEW device %d\r\n", i);
	      hid_info[i].state = STATE_RUNNING; 

#if 0
    	// set report protocol 1 if subclass != BOOT_INTF
    	// CherryUSB doesn't report the InterfaceSubClass (HID_BOOT_INTF_SUBCLASS)
    	// we thus set boot protocol on keyboards
    	if( hid_info[i].report.type == REPORT_TYPE_KEYBOARD ) {	
    	  // 0x0 = boot protocol, 0x1 = report protocol 
    	  printf("setting boot protocol\r\n");
    	  ret = usbh_hid_set_protocol(hid_info[i].class, HID_PROTOCOL_BOOT);
    	  if (ret < 0) {
    	    printf("failed\r\n");
    	    hid_info[i].state = STATE_FAILED;  // failed
    	    continue;
    	  }
    	}
#endif

    	if( hid_info[i].report.type == REPORT_TYPE_KEYBOARD ) {
    	  // restore current LED state on the newly connected keyboard
    	  ret = hid_send_led_report(i, current_led_state);
    	  if (ret < 0)
    	    printf("%d: initial SET_REPORT(LED) failed: %d\r\n", i, ret);
    	}
	
    	// request first urb
    	printf("%d: Submit urb size %d\r\n", i, hid_info[i].report.report_size + (hid_info[i].report.report_id_present ? 1:0));
        printf("%d: intin endpoint: %p\r\n", i, hid_info[i].class->intin);
        printf("%d: hport: %p\r\n", i, hid_info[i].class->hport);
        printf("%d: report type: %d\r\n", i, hid_info[i].report.type);
    	usbh_int_urb_fill(&hid_info[i].intin_urb,
                hid_info[i].class->hport,
                hid_info[i].class->intin, hid_info[i].buffer,
                hid_info[i].report.report_size + (hid_info[i].report.report_id_present ? 1:0),
                0, usbh_hid_callback, &hid_info[i]);
    	hid_submit_intin(i);
      } else if(hid_info[i].state == STATE_RUNNING && !hid_info[i].urb_busy) {
        // resubmit only after the previous URB has completed (callback was called)
        hid_submit_intin(i);
      }
    }

    // Process received reports in thread context (wait max. 10ms for the first one)
    {
      struct hid_report_msg msg;
      TickType_t wait = pdMS_TO_TICKS(10);
      while (xQueueReceive(hid_report_queue, &msg, wait) == pdTRUE) {
        wait = 0;
        if (msg.index < CONFIG_USBHOST_MAX_HID_CLASS && hid_info[msg.index].state == STATE_RUNNING)
          hid_process_report(&hid_info[msg.index], msg.data, msg.len);
      }
    }

    // Key repeat. Stop it (and forget pressed keys) if no keyboard is running
    // anymore, e.g. keyboard unplugged while a key was held down.
    {
      bool kbd_running = false;
      for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++)
        if(hid_info[i].state == STATE_RUNNING && hid_info[i].report.type == REPORT_TYPE_KEYBOARD)
          kbd_running = true;

      if(kbd_running)
        kbd_repeat_poll();
      else
        kbd_reset_state();
    }

    {
      struct hid_led_request led_req={0};
      // Process pending LED request (non-blocking)
      if (xQueueReceive(hid_led_request_queue, &led_req, 0) == pdTRUE) {
          printf("Processing queued LED request\r\n");
          
          // Process LED request in USB thread context
          led_req.result = set_keyboard_leds_internal(&led_req);
          
          // Signal completion back to requesting thread (if it waits)
          if (led_req.completion_sem)
            xSemaphoreGive(led_req.completion_sem);
      }
    }
  }
 
}

void usbh_hid_run(struct usbh_hid *hid_class) {
  printf("HID run\r\n");
  //usbh_hid_update();
  // LED 1 on
#ifdef M0S_DOCK
  bflb_gpio_reset(gpio, GPIO_PIN_27);
#endif
}

void usbh_hid_stop(struct usbh_hid *hid_class) {
  printf("HID stop\r\n");
  //usbh_hid_update();
  // LED 1 off
#ifdef M0S_DOCK
  bflb_gpio_set(gpio, GPIO_PIN_27);
#endif
}



static void ps2_kbd_thread(void *argument)
{
  uintptr_t received_val;
  printf("ps2_kbd_thread started\r\n");
   
   bflb_gpio_uart_init(gpio, GPIO_PIN_16, GPIO_UART_FUNC_UART1_TX);
   bflb_gpio_uart_init(gpio, GPIO_PIN_17, GPIO_UART_FUNC_UART1_RX);

   struct bflb_uart_config_s cfg = {0};
    //cfg.baudrate = 2000000;
    cfg.baudrate = 115200;
    cfg.data_bits = UART_DATA_BITS_8;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.parity = UART_PARITY_NONE;
    cfg.flow_ctrl = 0;
    cfg.tx_fifo_threshold = 7;
    cfg.rx_fifo_threshold = 7;
    cfg.bit_order = UART_LSB_FIRST;

    uart1 = bflb_device_get_by_name("uart1");   // AV added
    bflb_uart_init(uart1, &cfg);

  /* bflb_uart_putchar(uart1,' ');
   bflb_uart_putchar(uart1,'A');
   bflb_uart_putchar(uart1,'n');
   bflb_uart_putchar(uart1,'d');
   bflb_uart_putchar(uart1,'i');*/

/*   vTaskDelay(pdMS_TO_TICKS(5000));
   // From your menu or any thread
   printf("Testing LED control...\r\n");


// Test 1: Just Num Lock
set_keyboard_leds_thread_safe(0, 0x01);
vTaskDelay(pdMS_TO_TICKS(500));

// Test 2: Just Caps Lock
set_keyboard_leds_thread_safe(0, 0x02);
vTaskDelay(pdMS_TO_TICKS(500));

// Test 3: Num + Caps
set_keyboard_leds_thread_safe(0, 0x03);
vTaskDelay(pdMS_TO_TICKS(500));

// Test 4: All off
set_keyboard_leds_thread_safe(0, 0x00);

printf("LED tests complete\r\n");*/
  
  while (1) {
    //usb_osal_msleep(10);
    const int ret = usb_osal_mq_recv(my_queue, &received_val, USB_OSAL_WAITING_FOREVER);
    if (ret == 0) {
        // Data successfully received
        printf("Received value: %d\r\n", (BYTE)received_val);
        DecodeKey2NKC((BYTE)received_val);
    }
  }
}


void usbh_class_test(void) {
  printf("init usbh class\r\n");

  // Create queue for LED requests (length 1, used with xQueueOverwrite)
  hid_led_request_queue = xQueueCreate(1, sizeof(struct hid_led_request));
  if (!hid_led_request_queue) {
      printf("Failed to create LED request queue\r\n");
  }

  hid_report_queue = xQueueCreate(HID_REPORT_QUEUE_LEN, sizeof(struct hid_report_msg));
  if (!hid_report_queue) {
      printf("Failed to create HID report queue\r\n");
  }


  // mark all HID info entries as unused
  for(int i=0;i<CONFIG_USBHOST_MAX_HID_CLASS;i++) {
    hid_info[i].state = 0;
    hid_info[i].index = i;
    hid_info[i].urb_busy = false;
    hid_info[i].buffer = hid_buffer[i];      
  }
  my_queue = usb_osal_mq_create(64);

  usb_osal_thread_create("usbh_hid", 2048, CONFIG_USBHOST_PSC_PRIO + 1, usbh_hid_thread, NULL);
  usb_osal_thread_create("ps2_kbd", 2048, CONFIG_USBHOST_PSC_PRIO + 1, ps2_kbd_thread, NULL);
}
#if 0
/**
 * Set keyboard LEDs via USB HID Set Report
 * @param hid_class USB HID class instance
 * @param led_state LED state bitmask:
 *        Bit 0: Num Lock
 *        Bit 1: Caps Lock
 *        Bit 2: Scroll Lock
 * @return 0 on success, negative on error
 */
int set_keyboard_leds(struct usbh_hid *hid_class, uint8_t led_state)
{
    uint8_t led_buffer[1];
    int ret;

    if (!hid_class) {
        printf("Invalid HID class\r\n");
        return -1;
    }

    led_buffer[0] = led_state;

    // Send Set Report request (Output Report, Report ID = 0)
    ret = usbh_hid_set_report(hid_class, HID_REPORT_OUTPUT, 0, led_buffer, 1);

    if (ret < 0) {
        printf("Failed to set keyboard LEDs: %d\r\n", ret);
        return ret;
    }

    printf("Keyboard LEDs set to: 0x%02x (NumLock=%d, CapsLock=%d, ScrollLock=%d)\r\n",
           led_state,
           (led_state >> 0) & 1,
           (led_state >> 1) & 1,
           (led_state >> 2) & 1);

    return 0;
}
#endif



// ============================================================================
// PUBLIC FUNCTION - Thread-safe wrapper to set LEDs from any thread
// ============================================================================

int set_keyboard_leds_thread_safe(int hid_index, uint8_t led_state)
{
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    struct hid_led_request led_req={0};

    // If we're already in USB thread, call directly (avoid deadlock)
    if (current_task == usb_hid_task_handle) {
        printf("Setting LED from USB thread (direct call)\r\n");
        led_req.hid_index = hid_index;
        led_req.led_state = led_state;
        return set_keyboard_leds_internal(&led_req);
    }

    // From different thread: hand the request over to the USB thread without
    // waiting. The queue has length 1, so only the latest LED state is kept and
    // the caller (ps2_kbd thread) never blocks -> no deadlock with the USB thread,
    // which may itself block while feeding the ps2_kbd queue.
    led_req.hid_index = hid_index;
    led_req.led_state = led_state;
    led_req.completion_sem = NULL;
    led_req.result = -1;

    if (!hid_led_request_queue)
        return -1;

    xQueueOverwrite(hid_led_request_queue, &led_req);
    return 0;
}
// Mehari dashboard LCD controller

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "symbols.h"

#define LCD_HOST VSPI_HOST  // - SPI3_HOST.
#define DMA_CHAN 2

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_DC   17
#define PIN_NUM_RST  16
#define PIN_NUM_BCKL 22


// To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many.
// More means more memory use, but less overhead for setting up / finishing transfers.
// Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

typedef uint8_t  u8;
typedef uint16_t u16;

typedef struct {
  u8 cmd;
  u8 data[16];
  u8 databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {  // In DRAM for DMA access

  {0x36, {(1<<5)|(1<<6)}, 1},                 // Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0
  {0x3A, {0x55}, 1},                          // Interface Pixel Format, 16bits/pixel for RGB/MCU interface
  {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},  // Porch Setting
  {0xB7, {0x45}, 1},                          // Gate Control, Vgh=13.65V, Vgl=-10.43V
  {0xBB, {0x2B}, 1},                          // VCOM Setting, VCOM=1.175V
  {0xC0, {0x2C}, 1},                          // LCM Control, XOR: BGR, MX, MH
  {0xC2, {0x01, 0xff}, 2},                    // VDV and VRH Command Enable, enable=1
  {0xC3, {0x11}, 1},                          // VRH Set, Vap=4.4+...
  {0xC4, {0x20}, 1},                          // VDV Set, VDV=0
  {0xC6, {0x0f}, 1},                          // Frame Rate Control, 60Hz, inversion=0
  {0xD0, {0xA4, 0xA1}, 1},                    // Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V

  /* Positive Voltage Gamma Control */
  {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},

  /* Negative Voltage Gamma Control */
  {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},

  {0x11, {0}, 0x80},  // Sleep Out
  {0x29, {0}, 0x80},  // Display On
  {0, {0}, 0xff}
};

int CSPin = 5;  // Initial CS


// Send a command to the LCD. Uses spi_device_polling_transmit, which waits
// until the transfer is complete.
//
// Since command transactions are usually small, they are handled in polling
// mode for higher speed. The overhead of interrupt transactions is more than
// just waiting for the transaction to complete.

void lcd_cmd(spi_device_handle_t spi, const u8 cmd) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));  // Zero out the transaction
  t.length=8;                // Command is 8 bits
  t.tx_buffer=&cmd;          // The data is the cmd itself
  t.user=(void*)0;           // D/C needs to be set to 0
  ret=spi_device_polling_transmit(spi, &t);  // Transmit!
  assert(ret==ESP_OK);       // Should have had no issues.
}


// Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
// transfer is complete.
//
// Since data transactions are usually small, they are handled in polling
// mode for higher speed. The overhead of interrupt transactions is more than
// just waiting for the transaction to complete.

void lcd_data(spi_device_handle_t spi, const u8 *data, int len) {
  esp_err_t ret;
  spi_transaction_t t;
  if (len==0) return;             //no need to send anything
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=len*8;                 //Len is in bytes, transaction length is in bits.
  t.tx_buffer=data;               //Data
  t.user=(void*)1;                //D/C needs to be set to 1
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
}


// This function is called (in irq context!) just before a transmission starts. It will
// set the D/C line to the value indicated in the user field.

void lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  int dc=(int)t->user;
  gpio_set_level(PIN_NUM_DC, dc);
  gpio_set_level(CSPin, 0);  // Select CS of current target LCD
}

void lcd_spi_post_transfer_callback(spi_transaction_t *t) {
  gpio_set_level(CSPin, 1);  // Deselect CS of current target LCD
}


// Initialize the display
void lcd_init(spi_device_handle_t spi) {
  int cmd=0;
  const lcd_init_cmd_t* lcd_init_cmds;

  //Initialize non-SPI GPIOs
  gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
  gpio_set_direction(CSPin, GPIO_MODE_OUTPUT);
  gpio_set_level(CSPin, 1);  // Deselect CS of current target LCD

  //Reset the display
  gpio_set_level(PIN_NUM_RST, 0);  vTaskDelay(100/portTICK_RATE_MS);
  gpio_set_level(PIN_NUM_RST, 1);  vTaskDelay(100/portTICK_RATE_MS);

  //Send all the commands
  lcd_init_cmds = st_init_cmds;
  while (lcd_init_cmds[cmd].databytes!=0xff) {
    lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
    lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
    if (lcd_init_cmds[cmd].databytes&0x80) {
      vTaskDelay(100 / portTICK_RATE_MS);
    }
    cmd++;
  }

  printf("LCD initialization for CS pin %d complete.\n", CSPin);
}


///       Polygon rendering by edges
//
//        1) Determining the edge list sorted by y value.
//        2) Set up the active edge list.
//        3) For each scanline:
//           a) Determine the x positions for each edge, sorted by x value;
//           b) Fill the scanline between x value pairs;
//           c) Where this is the last scanline of an edge, advance to the ext edge.


struct edge {int x1; int y1; int x2; int y2;} edges[4];

int curedge;

void rotate(int x, int y, int xo, int yo, float sine, float cosine, int *xr, int *yr) {
  *xr = (int)(xo + (x-xo)*cosine - (y-yo)*sine);
  *yr = (int)(yo + (y-yo)*cosine + (x-xo)*sine);
}

//void order(int *a, int *b) {if (*a > *b) {int t = *a;  *a = *b;  *b = t;}}


void addedge(int x1, int y1, int x2, int y2, int xo, int yo, float sine, float cosine) {
  rotate(x1, y1, xo, yo, sine, cosine, &edges[curedge].x1, &edges[curedge].y1);
  rotate(x2, y2, xo, yo, sine, cosine, &edges[curedge].x2, &edges[curedge].y2);

  // Arrange the edge with y1 being <= y2
  if (edges[curedge].y1 > edges[curedge].y2) {
    int xt = edges[curedge].x1;             int yt = edges[curedge].y1;
    edges[curedge].x1 = edges[curedge].x2;  edges[curedge].y1 = edges[curedge].y2;
    edges[curedge].x2 = xt;                 edges[curedge].y2 = yt;
  }
  curedge++;
}


void initrectangle(int x1, int y1, int x2, int y2, int xo, int yo, float angle) {
  float sine   = sin(angle);
  float cosine = cos(angle);

  curedge = 0;
  addedge(x1, y1, x2, y1, xo, yo, sine, cosine);
  addedge(x2, y1, x2, y2, xo, yo, sine, cosine);
  addedge(x1, y1, x1, y2, xo, yo, sine, cosine);
  addedge(x1, y2, x2, y2, xo, yo, sine, cosine);

  // sort edges by initial y

  int i, j;

  struct edge tedge;

  // Sort edges such that
  //   The lowest y1's come first
  //   Where two edges have the same y1, then the lowest y2 comes first
  for (i=0; i<3; i++) {
    for (j=i+1; j<4; j++) {
      struct edge *ea = &edges[i];
      struct edge *eb = &edges[j];
      if ((ea->y1 > eb->y1) || ((ea->y1 == eb->y1) && (ea->y2 > eb->y2))) {
        tedge = *ea;  *ea = *eb;  *eb = tedge;
      }
    }
  }
  assert(edges[0].y1 == edges[1].y1);

  //printf("Sorted edges:\n");
  //for (i=0; i<4; i++) {
    //printf("  %d,%d .. %d,%d\n", edges[i].x1, edges[i].y1, edges[i].x2, edges[i].y2);
  //}

  curedge = 0;
}

int interpolate(int a, int a1, int a2, int b1, int b2) {
  // input: a is a position between a1 and a2
  // result is the corresponding position between b1 and b2
  if (a2 == a1) return b1;
  return b1 + ((a-a1)*(b2-b1))/(a2-a1);
}

void scanlinepolygon(int y, u16 *dest) {

  // First skip any and all exhausted edges
  while ((curedge < 4) && (edges[curedge].y2 <= y)) {curedge++;}

  if ((curedge < 4) && (y >= edges[curedge].y1)) {
    int i, j, t, edgecount, x[4];

    //printf("y = %d, consider edges:\n", y);

    edgecount = 0;
    while ((curedge+edgecount < 4) && (edges[curedge+edgecount].y1 <= y)) {
      struct edge *ep = &edges[curedge+edgecount];

      x[edgecount] = interpolate(y, ep->y1, ep->y2,  ep->x1, ep->x2);

      //printf("  %d,%d .. %d,%d -> %d\n", ep->x1, ep->y1, ep->x2, ep->y2, x[edgecount]);

      edgecount++;
    }

    // sort edge x's
    for (i=0; i<(edgecount-1); i++) {
      for (j=i+1; j<edgecount; j++) {
        if (x[i] > x[j]) {t = x[i]; x[i] = x[j]; x[j] = t;}
      }
    }

    // expect an even number of x's
    if ((edgecount%2) != 0) {
      //printf("unexpected odd edge count %d for y = %d. x's are: ", edgecount, y);
      //for (i=0; i<edgecount; i++) {printf(" %d", x[i]);}
      //printf(".\n");
      assert((edgecount%2) == 0);
    }

    // render extents in pairs

    //printf("fill y=%d, edgecount %d", y, edgecount);

    for (i = 0;  i+1 < edgecount;  i += 2) {
      //printf(", %d..%d", x[i], x[i+1]);
      for (j = x[i];  j < x[i+1]; j++) {dest[j] = 0xffe0;}
    }
    //printf(".\n");
  }
}




const int   centrex = 151;
const int   centrey = 166;
const float needlen = 124.0;

void calc_lines(u16 *dest, int line, int frame, int linect, int needlepos) { // needlepos 0..511
  int symindex = 9; // (frame/10) % numsyms;

  int x1 = -1, x2 = -1;
  int y1 = -1, y2 = -1;

  if (needlepos >= 0) {
    float angle = ((needlepos * 220.0 / 511.0) - 20.0)  * M_PI/180.0;
    x1 = centrex - (int)(cos(angle) * needlen);
    y1 = centrey - (int)(sin(angle) * needlen);

    if (y1 <= centrey) {x2 = centrex; y2 = centrey;}
    else               {x2 = x1; y2 = y1; x1 = centrex; y1 = centrey;}

    // Polygon version prototyping
    initrectangle(27,164, 153,169, 151,166, angle);
  }

  int width   = symbols[symindex].width;
  int height  = symbols[symindex].height;
  int xoffset = (320 - width)  / 2;
  int yoffset = (240 - height) / 2;

  for (int y = line; y < line + linect; y++) {
    u16 *linestart = dest;
    int x;
    if ((y < yoffset) || (y >= yoffset+height)) {
      for (x=0; x<320; x++) {*dest++ = 0xffff;}
    } else {
      x=0;
      while (x < xoffset) {*dest++ = 0xffff; x++;}
      u8 *p = bits + symbols[symindex].rows[y-yoffset];
      while ((x < 320) && (*p != 0x80)) {  // process pixels until eol
        int n      = *p & 0x7f;
        int repeat = *p & 0x80;
        p++;
        if (repeat) {
          u16 v = 0xffff - *((u16 *)p);  p += 2;
          for (int i=0; i<n; i++) {*dest++ = v;}
        } else {  // distinct values
          for (int i=0; i<n; i++) {
            *dest++ = 0xffff - *((u16 *)p);  p += 2;
          }
        }
        x += n;
      }
      while (x<320) {*dest++ = 0xffff; x++;}

      // Polygon version prototyping
      scanlinepolygon(y, linestart);
    }

    if ((y >= y1) && (y <= y2)) {
      int nx;
      if (y2 != y1) {
        nx = x1 + ((y-y1) * (x2-x1))/(y2-y1);
        //printf("y = %d, nx = %d.\n", y, nx);
      } else {
        nx = (x1 + x2) / 2;  // TEMP - needs to draw full width.
      }
      linestart[nx-1] = 0;
      linestart[nx  ] = 0;
      linestart[nx+1] = 0;
    }
  }
}



// To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
// before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
// because the D/C line needs to be toggled in the middle.)
// This routine queues these commands up as interrupt transactions so they get
// sent faster (compared to calling spi_device_transmit several times), and at
// the mean while the lines for next transactions can get calculated.

static void send_lines(spi_device_handle_t spi, int ypos, u16 *linedata) {
  int x;

  // Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
  // function is finished because the SPI driver needs access to it even while we're already calculating the next line.
  static spi_transaction_t trans[6];

  // In theory, it's better to initialize trans and data only once and hang on to the initialized
  // variables. We allocate them on the stack, so we need to re-init them each call.
  for (x=0; x<6; x++) {
    memset(&trans[x], 0, sizeof(spi_transaction_t));
    trans[x].length = (x&1) ? 8*4 : 8;  // 8 bytes for commands, 32 bytes for data
    trans[x].user   = (void*)(x&1);     // 0 for command, 1 for data
    trans[x].flags  = SPI_TRANS_USE_TXDATA;
  }
  trans[0].tx_data[0] = 0x2A;        // Column Address Set
  trans[1].tx_data[0] = 0;           // Start Col High
  trans[1].tx_data[1] = 0;           // Start Col Low
  trans[1].tx_data[2] = (320)>>8;    // End Col High
  trans[1].tx_data[3] = (320)&0xff;  // End Col Low

  trans[2].tx_data[0] = 0x2B;        // Page address set
  trans[3].tx_data[0] = ypos>>8;     // Start page high
  trans[3].tx_data[1] = ypos&0xff;   // start page low
  trans[3].tx_data[2] = (ypos+PARALLEL_LINES)>>8;    //end page high
  trans[3].tx_data[3] = (ypos+PARALLEL_LINES)&0xff;  //end page low

  trans[4].tx_data[0] = 0x2C;        // memory write
  trans[5].tx_buffer = linedata;     // finally send the line data
  trans[5].length    = 320*2*8*PARALLEL_LINES; // Data length, in bits
  trans[5].flags     = 0;            // undo SPI_TRANS_USE_TXDATA flag

  // Queue all transactions.
  for (x=0; x<6; x++) {
    assert(spi_device_queue_trans(spi, &trans[x], portMAX_DELAY) == ESP_OK);
  }

  // When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
  // mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
  // finish because we may as well spend the time calculating the next line. When that is done, we can call
  // send_line_finish, which will wait for the transfers to be done and check their status.
}


static void send_line_finish(spi_device_handle_t spi) {
  spi_transaction_t *rtrans;

  // Wait for all 6 transactions to be done and get back the results.
  for (int x=0; x<6; x++) {
    assert(spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY) == ESP_OK);
  }
}




// Run the four Mehari dashboard LCD displays.
//
// Because the SPI driver handles transactions in the background, we can calculate the next line
// while the previous one is being sent.

static void run_display(spi_device_handle_t spi) {

  u16 *linebuf[2];  // Each buffer holds PARRALEL_LINES uncompressed lines of the image

  // Allocate memory for the pixel buffers
  for (int i=0; i<2; i++) {
    linebuf[i] = heap_caps_malloc(320*PARALLEL_LINES*sizeof(u16), MALLOC_CAP_DMA);
    assert(linebuf[i] != NULL);
  }

  int frame = 0;

  //Indexes of the b and the line we're calculating.
  int sendingbuf     = -1;  // Buffer currently being sent to the LCD
  int calculatingbuf = 0;   // Buffer being prepared

  while(1) {

    frame++;

    for (int y=0; y<240; y+=PARALLEL_LINES) {

      // Generate some linebuf
      calc_lines(linebuf[calculatingbuf], y, frame, PARALLEL_LINES, frame%512);

      // Finish up the sending process of the previous line, if any
      if (sendingbuf != -1) send_line_finish(spi);

      // Swap sendingbuf and calculatingbuf
      sendingbuf     = calculatingbuf;
      calculatingbuf = (calculatingbuf+1) % 2;

      // Send the line we currently calculated.
      send_lines(spi, y, linebuf[sendingbuf]);

      // The line set is queued up for sending now; the actual sending happens in the
      // background. We can go on to calculate the next line set as long as we do not
      // touch line[sendingbuf]; the SPI sending process is still reading from that.
    }
  }
}




void app_main(void)
{
  spi_device_handle_t spi;

  spi_bus_config_t buscfg = {
    .miso_io_num     = -1, // MISO unused
    .mosi_io_num     = PIN_NUM_MOSI,
    .sclk_io_num     = PIN_NUM_CLK,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = PARALLEL_LINES*320*2+8
  };

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 10*1000*1000,  // Clock out at 10 MHz
    .mode           = 0,             // SPI mode 0
    .spics_io_num   = -1,            // Was: PIN_NUM_CS
    .queue_size     = 7,             // We want to be able to queue 7 transactions at a time
    .pre_cb         = lcd_spi_pre_transfer_callback,  // Selects target LCD CS and handles D/C line
    .post_cb        = lcd_spi_post_transfer_callback, // Deselects target LCD CS
  };

  gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_NUM_BCKL, 0);  // Disable backlight

  // Initialize the SPI bus
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN));

  // Attach the LCD to the SPI bus
  ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &spi));

  // Initialise each LCD
  CSPin = 19;
  lcd_init(spi);                    // Initialize the LCD
  gpio_set_level(PIN_NUM_BCKL, 1);  // Enable backlight
  run_display(spi);
}

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
  {0xB0, {0x00, 0xF8}, 2},                    // RAM control - set little endian
  {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},  // Porch Setting
  {0xB7, {0x45}, 1},                          // Gate Control, Vgh=13.65V, Vgl=-10.43V
  {0xBB, {0x2B}, 1},                          // VCOM Setting, VCOM=1.175V
  {0xC0, {0x3C}, 1},                          // LCM Control, XOR: BGR & INV, MX, MH
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

enum {maxedges = 20};  // Roughly 14 expected

struct vertex {int x; int y;};

struct edge {int x1; int y1; int x2; int y2;} edges[maxedges];  // x: 27.5, y: 30.2

int edgecount = 0;

void rotate(float x, float y, float xo, float yo, float sine, float cosine, float *xr, float *yr) {
  *xr = xo + (x-xo)*cosine - (y-yo)*sine;
  *yr = yo + (y-yo)*cosine + (x-xo)*sine;
}

void addedge(int x1, int y1, int x2, int y2, int xo, int yo, float sine, float cosine) {
  // x's in fixed point 27.5
  // y's in fixed point 30.2

  if ((x1 != y1) || (x2 != y2)) {  // Omit 0 length edges

    assert(edgecount < maxedges);

    float xr, yr;

    rotate(x1/32.0, y1/4.0, xo/32.0, yo/4.0, sine, cosine, &xr, &yr);
    edges[edgecount].x1 = (int)(xr*32.0);
    edges[edgecount].y1 = (int)(yr*4.0);

    rotate(x2/32.0, y2/4.0, xo/32.0, yo/4.0, sine, cosine, &xr, &yr);
    edges[edgecount].x2 = (int)(xr*32.0);
    edges[edgecount].y2 = (int)(yr*4.0);

    if (edges[edgecount].y1 != edges[edgecount].y2) {  // Omit perfectly horizontal lines

      // Arrange the edge with y1 being <= y2
      if (edges[edgecount].y1 > edges[edgecount].y2) {
        int xt = edges[edgecount].x1;               int yt = edges[edgecount].y1;
        edges[edgecount].x1 = edges[edgecount].x2;  edges[edgecount].y1 = edges[edgecount].y2;
        edges[edgecount].x2 = xt;                   edges[edgecount].y2 = yt;
      }

      edgecount++;
    }
  }
}




void initpolygon(int vertexcount, struct vertex vertices[], int originx, int originy, float angle) {

  // NOTE: integer inputs are in subpixel coordinates: x is 27.5, y is 30.2.

  float sine   = sin(angle);
  float cosine = cos(angle);

  edgecount = 0;

  for (int i = 0;  i < vertexcount;  i++) {
    addedge(
      vertices[i].x, vertices[i].y, vertices[(i+1)%vertexcount].x, vertices[(i+1)%vertexcount].y,
      originx, originy,
      sine, cosine
    );
  }

  //printf("initpolygon generated %d edges:\n", edgecount);
  //for (int i=0; i<edgecount; i++) {
  //  printf("  [%2d]  x1 %d.%d, y1 %d.%d,  x2 %d.%d, y2 %d.%d.\n", i,
  //    edges[i].x1>>5, edges[i].x1&31, edges[i].y1>>2, edges[i].y1&3,
  //    edges[i].x2>>5, edges[i].x2&31, edges[i].y2>>2, edges[i].y2&3
  //  );
  //}
}




int blendchannel(int paint, int intensity, int canvas) {
  if (intensity == 0)  return canvas;
  if (intensity > 127) return paint;

  //printf("Blend paint %d, intensity %d, canvas %d.", paint , intensity, canvas);

  // Calulate linear values of paint and canvas in the range 0 .. 63*63*128
  int paintpart  = paint  * paint  *      intensity;
  int canvaspart = canvas * canvas * (128-intensity);

  //printf(" paintpart %d, canvaspart %d.", paintpart, canvaspart);

  // Result is sum of paint and canvas converted back to log law in the range 0..63

  int result = (int)sqrt((paintpart + canvaspart) / 128.0);

  //printf(" result %d.\n", result);
  assert(result < 64);

  return result;
}


void blendsrgb(int r, int g, int b, int intensity, u16* dest) {
  // r, g, b in [0..63]
  // intensity in [0..128]

  int rp, gp, bp;  // New rgb values (r prime, g prime, b prime).

  if (intensity > 0) {
    if (intensity >= 127) {

      rp = r;  gp = g;  bp = b;

    } else {

      // Exiting red, green, blue values (srgb).

      int rd = ((*dest) >> 10) & 0x3E;
      int gd = ((*dest) >> 5)  & 0x3F;
      int bd = ((*dest) << 1)  & 0x3E;

      rp = blendchannel(r, intensity, rd);
      gp = blendchannel(g, intensity, gd);
      bp = blendchannel(b, intensity, bd);

      assert(rp < 64);
      assert(gp < 64);
      assert(bp < 64);
    }

    *dest = ((rp & 0x3E) << 10) | (gp << 5) | (bp >> 1);
  }
}




int interpolate(int a, int a1, int a2, int b1, int b2) {
  // input: a is a position between a1 and a2
  // result is the corresponding position between b1 and b2

  if (a2 == a1) {printf("interpolate called with a1 = a2.\n"); return b1;}

  int result = b1 + ((a-a1)*(b2-b1))/(a2-a1);

  if (!((result > 0)  &&  (result < 320*32))) {

    printf("interpolate: result out of range:\n");
    printf("  a: %d.%d, a1: %d.%d, a2: %d.%d, b1: %d.%d, b2: %d.%d -> result: %d.%d.\n",
      a>>2,  a&3,
      a1>>2, a1&3,  a2>>2, a2&3,
      b1>>5, b1&31, b2>>5, b2&31,
      result>>5, result&31
    );
  }


  return result;
}


void scanlinepolygon(int y, u16 *dest) {
  int i, j, t, xcount, x[4*edgecount];

  xcount = 0;

  for (int dy = 0;  dy < 4;  dy++) {  // For each subpixel scanline

    int sy = y*4 + dy;

    for (int e = 0;  e < edgecount;  e++) {
      struct edge *ep = &edges[e];

      if ((sy >= ep->y1) && (sy < ep->y2)) {

        int sx = interpolate(sy,  ep->y1, ep->y2,  ep->x1, ep->x2);

        x[xcount++] = sx*4 | dy;  // Note - include sub scanline as bottom bits of x positions

        //printf("Process edge from %d,%d to %d,%d for line %d giving %d.%d[%d].\n",
        //  ep->x1, ep->y1, ep->x2, ep->y2, y,
        //  sx>>5, sx&3, sy
        //);

      }
    }
  }

  if (xcount >= 2) {

    // sort edge x's
    for (i = 0;  i < (xcount-1);  i++) {
      for (j = i+1;  j<xcount;  j++) {
        if (x[i] > x[j]) {t = x[i]; x[i] = x[j]; x[j] = t;}
      }
    }

    //printf("Found %d x positions for y = %d. x's are: ", xcount, y);
    //for (i=0; i<xcount; i++) {
    //  printf(" %d.%d[%d]", x[i]>>7, (x[i]>>2)&31, x[i]&3);
    //}
    //printf(".\n");

    // expect an even number of x's
    if ((xcount%2) != 0) {
      printf("unexpected odd edge count %d for y = %d. x's are: ", xcount, y);
      for (i=0; i<xcount; i++) {printf(" %d[%d]", x[i]>>2, x[i]&3);}
      printf(".\n");
      assert((xcount%2) == 0);
    }

    // Run through edges detemining pixel intensities

    int x1[4] = {-1, -1, -1, -1};  // Start of subpixel active runs

    u8 intensity[320] = {0};  // Intensity accumulator

    for (i = 0;  i < xcount;  i++) {

      int sx = x[i]>>2;   // Subpixel x position (bottom 5 bits are subpixel offset)
      int sy = x[i] & 3;  // Subline - 0..3

      if (x1[sy] < 0) {  // This subline currently inactive

        x1[sy] = sx;  // Active from sx

      } else {  // End of active part of subline

        // Apply intensity from x1[sy] to sx

        int dx = sx - x1[sy];

        int x = x1[sy] >> 5;  // Initally touched pixel x

        //printf("y = %d: touching subline %d from %d.%d to %d.%d, dx = %d, initial x = %d.\n",
        //  y, sy, x1[sy]>>5, x1[sy]&31, sx>>5, sx&31, dx, x);

        if (x1[sy] & 31) {  // First pixel coverage starts mid-pixel
          int partcoverage = 32 - (x1[sy] & 31);
          if (partcoverage > dx) {partcoverage = dx;}
          intensity[x++] += partcoverage;
          dx -= partcoverage;
        }

        while (dx >= 32) {intensity[x++] += 32; dx -= 32;}  // Apply wholly covered subline pixels

        if (dx > 0) {intensity[x] = dx;}  // Apply final partially covered subline pixel

        x1[sy] = -1;  // subpixel line sy is off again
      }
    }

    // NOTE: all four x1's should now be -1.

    // We now have a buffer of intensity by pixel, each valued 0..128.

    for (j = 0;  j < 320;  j++) {
      if (intensity[j] > 0) {blendsrgb(63, 63, 63, intensity[j], &dest[j]);}  // white
    }
  }

  // render extents in pairs

  //  //printf("fill y=%d, xcount %d", y, xcount);
  //
  //  for (i = 0;  i+1 < xcount;  i += 2) {
  //    //printf(", %d..%d", x[i], x[i+1]);
  //    for (j = x[i];  j < x[i+1]; j++) {dest[j] = 0xffe0;}
  //  }
  //  //printf(".\n");
}




struct vertex needlevertices[] = {
  { 30*32, 168*4},
  { 47*32, 166*4},
  { 50*32, 165*4},
  {146*32, 165*4},
  {150*32, 161*4},
  {156*32, 161*4},
  {160*32, 165*4},
  {160*32, 171*4},
  {156*32, 175*4},
  {150*32, 175*4},
  {146*32, 171*4},
  { 50*32, 170*4},
  { 47*32, 169*4}
};


const int needlevertexcount = sizeof(needlevertices) / sizeof(struct vertex);

void calc_lines(u16 *dest, int line, int frame, int linect, int needlepos) { // needlepos 0..511
  int symindex = 9; // (frame/10) % numsyms;

  if (needlepos >= 0) {
    float angle = ((needlepos * 220.0 / 511.0) - 20.0)  * M_PI/180.0;
    //initneedle(angle);
    initpolygon(needlevertexcount, needlevertices, 153*32, 168*4, angle);
  }

  int width   = symbols[symindex].width;
  int height  = symbols[symindex].height;
  int xoffset = (320 - width)  / 2;
  int yoffset = (240 - height) / 2;

  for (int y = line; y < line + linect; y++) {
    u16 *linestart = dest;
    int x;
    if ((y < yoffset) || (y >= yoffset+height)) {
      for (x=0; x<320; x++) {*dest++ = 0;}
    } else {
      x=0;
      while (x < xoffset) {*dest++ = 0; x++;}
      u8 *p = bits + symbols[symindex].rows[y-yoffset];
      while ((x < 320) && (*p != 0x80)) {  // process pixels until eol
        int n      = *p & 0x7f;
        int repeat = *p & 0x80;
        p++;
        if (repeat) {
          u16 v = *((u16 *)p);  p += 2;
          for (int i=0; i<n; i++) {*dest++ = v;}
        } else {  // distinct values
          for (int i=0; i<n; i++) {
            *dest++ = *((u16 *)p);  p += 2;
          }
        }
        x += n;
      }
      while (x<320) {*dest++ = 0; x++;}

      // Overlay needle
      scanlinepolygon(y, linestart);
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

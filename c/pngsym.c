// pngsym - convert png to compressed symbol format

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <png.h>


void Fail(char *msg)  {printf("%s\n", msg); exit(99);}
void Check(int err, char *msg) {if (err) {printf("%s %d.\n", msg, err); exit(99);}}
void Assert(int truth, char *msg) {if (!truth) {printf("%s.\n", msg); exit(99);}}

void strlcat(char *dst, char *src, int dstlen) {
  int curlen = strlen(dst);
  if (curlen < dstlen-1) {
    strncpy(dst+curlen, src, dstlen-curlen);
  }
}

// -------------------------------------------------------------

// sRGB to/from linear

float logtolin(float x)
{
    if (x >= 0.0031308)
        return (1.055) * powf(x, 1.0/2.4) - 0.055;
    else
        return 12.92 * x;
}

float lintolog(float x)
{
    if (x >= 0.04045)
        return powf((x + 0.055)/(1 + 0.055), 2.4);
    else
        return x / 12.92;
}

// OKLAB colour support

struct Lab {float L; float a; float b;};
struct RGB {float r; float g; float b;};

struct Lab linear_srgb_to_oklab(struct RGB c)
{
  float l = 0.4122214708f * c.r + 0.5363325363f * c.g + 0.0514459929f * c.b;
  float m = 0.2119034982f * c.r + 0.6806995451f * c.g + 0.1073969566f * c.b;
  float s = 0.0883024619f * c.r + 0.2817188376f * c.g + 0.6299787005f * c.b;

    float l_ = cbrtf(l);
    float m_ = cbrtf(m);
    float s_ = cbrtf(s);

    return (struct Lab){
        0.2104542553f*l_ + 0.7936177850f*m_ - 0.0040720468f*s_,
        1.9779984951f*l_ - 2.4285922050f*m_ + 0.4505937099f*s_,
        0.0259040371f*l_ + 0.7827717662f*m_ - 0.8086757660f*s_,
    };
}

struct RGB oklab_to_linear_srgb(struct Lab c)
{
    float l_ = c.L + 0.3963377774f * c.a + 0.2158037573f * c.b;
    float m_ = c.L - 0.1055613458f * c.a - 0.0638541728f * c.b;
    float s_ = c.L - 0.0894841775f * c.a - 1.2914855480f * c.b;

    float l = l_*l_*l_;
    float m = m_*m_*m_;
    float s = s_*s_*s_;

    return (struct RGB){
    +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
    -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
    -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s,
    };
}

// ---

void srgbToLab8(int  sr, int  sg, int  sb, int sa,
                int *ll, int *la, int *lb) {

  float linred   = logtolin(((float)sr)/255.0) * ((float)sa)/255.0;
  float lingreen = logtolin(((float)sg)/255.0) * ((float)sa)/255.0;
  float linblue  = logtolin(((float)sb)/255.0) * ((float)sa)/255.0;

  struct Lab lab = linear_srgb_to_oklab((struct RGB){linred, lingreen, linblue});

  *ll = (int)(roundf(lab.L * 255.0));
  *la = (int)(roundf(lab.a * 255.0));
  *lb = (int)(roundf(lab.b * 255.0));
}

void srgbToLrgb(int  sr, int  sg, int  sb, int sa,
                int *ll, int *lr, int *lg, int *lb) {

  float linred   = logtolin(((float)sr)/255.0) * ((float)sa)/255.0;
  float lingreen = logtolin(((float)sg)/255.0) * ((float)sa)/255.0;
  float linblue  = logtolin(((float)sb)/255.0) * ((float)sa)/255.0;

  struct Lab lab = linear_srgb_to_oklab((struct RGB){linred, lingreen, linblue});

  *ll = (int)(roundf(lab.L * 255.0));

  lab.L = 0.5f;
  struct RGB rgb = oklab_to_linear_srgb(lab);
  if (rgb.r > 1.0001f) {printf("\nExpected r <= 1.0f, got %f.\n", rgb.r); Fail("");}
  if (rgb.g > 1.0001f) {printf("\nExpected g <= 1.0f, got %f.\n", rgb.g); Fail("");}
  if (rgb.b > 1.0001f) {printf("\nExpected b <= 1.0f, got %f.\n", rgb.b); Fail("");}

  *lr = (int)(roundf(rgb.r * 255.0));
  *lg = (int)(roundf(rgb.g * 255.0));
  *lb = (int)(roundf(rgb.b * 255.0));
}

// -------------------------------------------------------------


int col = 0;
int pos = 0;

void emitbyte(int v) {
  if (col == 0) {
    printf("  /*%5d*/ ", pos);
  } else if (col >= 16) {
    printf(",\n  /*%5d*/ ", pos);
    col = 0;
  } else {
    printf(", ");
  }
  printf("0x%02.2x",v);
  col++;
  pos++;
}

//--void emiteol() {
//--  emitbyte(0x80);
//--}

void emit(int n, int v) {
  Assert(v>=0,  "Emit expects input >= 0");
  Assert(v<128, "Emit expects input < 128");
  Assert(n>0, "Expected n>0 in emit");
  //if (n>1) {printf("(%dx%02.2x)", n, v);}
  while (n>127) {emitbyte(0xff); emitbyte(v); n-=127;}
  switch(n) {
    case 0:                                    break;
    case 1:  emitbyte(v);                      break;
    case 2:  emitbyte(v); emitbyte(v);         break;
    default: emitbyte(n | 0x80); emitbyte(v);  break;
  }
}



void extractCoLu(png_bytep pixel, int *colour, int *luminance) {

  int64_t a = pixel[3];
  int64_t r = pixel[0] * pixel[0] * a;  // Linear, in range 0..255^3
  int64_t g = pixel[1] * pixel[1] * a;  // Linear, in range 0..255^3
  int64_t b = pixel[2] * pixel[2] * a;  // Linear, in range 0..255^3

  int max = r;
  if (max < g) {max = g;}
  if (max < b) {max = b;}

  // max is linear luminance in range 0..255^3
  // scale r, g and b such that the brightest is 255^2 (or 0)

  if (max > 0) {
    r = (r * 65025) / max;
    g = (g * 65025) / max;
    b = (b * 65025) / max;
  }

  // Restore r, g, b, to log

  r = sqrt(r);  g = sqrt(g);  b = sqrt(b);
  *colour = (r<<16) | (g<<8) | b;

  // luminance is linear, return in range 0..255
  *luminance = max / 65025;
}

//float u8ToF(uint8_t v) {
//  return v/255.0f
//}
//
//uint8_t fToU8(float v) {
//  return (int)(v*255.99f);
//}


uint16_t rgb565(png_bytep pixel) {
  uint16_t r5 = pixel[0] >> 3;
  uint16_t g6 = pixel[1] >> 2;
  uint16_t b5 = pixel[2] >> 3;
  return (r5<<11) | (g6<<5) | b5;
}

uint16_t rgba565(png_bytep pixel) {
  float a = pixel[3]/255.0f;

  uint16_t r5 = ((int)(lintolog(logtolin(pixel[0]/255.0f) * a) * 255.99f)) >> 3;
  uint16_t g6 = ((int)(lintolog(logtolin(pixel[1]/255.0f) * a) * 255.99f)) >> 2;
  uint16_t b5 = ((int)(lintolog(logtolin(pixel[2]/255.0f) * a) * 255.99f)) >> 3;
  return (r5<<11) | (g6<<5) | b5;
}


void emitpixel(uint16_t pixel) {
//emitbyte(pixel >> 8);    emitbyte(pixel & 0xff);  // Big endian
  emitbyte(pixel & 0xff);  emitbyte(pixel >> 8);    // Little endian

}

void emitrow(uint16_t *row, int width){

  int x;
  int i=0;

  while (i < width) {

    int j = i;

    // Look for run of equal values
    while (    (j < width)
           && ((j+1 >= width) || (row[j+1] != row[j]))) {j++;}

    // Now j = first of a run of the same value, or end of row

    if (j>i) {
      // Emit run of nonequal values
      while (j-i > 127) {
        emitbyte(127);
        for (x=i; x<i+127; x++) {emitpixel(row[x]);}
        i += 127;
      }
      if (i<j) {
        emitbyte(j-i);
        for (x=i; x<j; x++) {emitpixel(row[x]);}
      }
      i=j;
    }

    if (i<width) {
      // i must be the start of a run of at least 2 equal values
      j = i+1;
      Assert(j < width, "Expected j < width");
      Assert(row[i] == row[j], "Expected row[i] == row[j]");

      j++;
      while ((j<width) && (row[j] == row[i])) {j++;}

      if ((j >= width) && (row[i] == 0)) {
        // Special case - omit trailing run of zero
        i = j;
      } else {
        // Emit run of equal values
        while (j-i > 127) {
          emitbyte(0x80 + 127);  emitpixel(row[i]);
          i += 127;
        }
        if (j>i) {
          emitbyte(0x80 + j-i);  emitpixel(row[i]);
          i = j;
        }
      }
    }
  }

  emitbyte(0x80);  // Signals end of row.
}


struct indexdata{
  char name[32];
  int  width;
  int  height;
  int  rows[240];
};


void processpng(char *ifn, struct indexdata *index) {
  FILE *ifp = fopen(ifn, "rb");
  Assert(ifp != NULL, "Couldn't open png");

  png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if(!png) abort();

  if(setjmp(png_jmpbuf(png))) Fail("Couldn't setjmp");

  png_init_io(png, ifp);

  png_infop info = png_create_info_struct(png);  if(!info) Fail("Couldn't create info struct.");
  png_read_info(png, info);

  index->width        = png_get_image_width(png, info);
  index->height       = png_get_image_height(png, info);
  png_byte color_type = png_get_color_type(png, info);
  png_byte bit_depth  = png_get_bit_depth(png, info);

  char* colourtype = 0;
  switch(color_type) {
    case PNG_COLOR_TYPE_RGB:        colourtype = "RGB"; break;
    case PNG_COLOR_TYPE_GRAY:       colourtype = "GRAY"; break;
    case PNG_COLOR_TYPE_PALETTE:    colourtype = "PALETTE"; break;
    case PNG_COLOR_TYPE_GRAY_ALPHA: colourtype = "GRAY_ALPHA"; break;
    case PNG_COLOR_TYPE_RGB_ALPHA:  colourtype = "RGB_ALPHA"; break;
    default:                        colourtype = "unrecognised"; break;
  }

  if (col > 0) {printf(",\n"); col = 0;}

  printf("  //\n");
  printf("  // symbol '%s' from '%s'.\n", index->name, ifn);
  printf("  // width: %d, height %d.\n", index->width, index->height);
  printf("  // original color_type %d (%s), bit_depth %d.\n", color_type, colourtype, bit_depth);
  printf("  //\n");


  if (   (color_type != PNG_COLOR_TYPE_RGB)
      && (color_type != PNG_COLOR_TYPE_RGB_ALPHA)) {
    printf("Expected colour type rgb-alpha, got %s.", colourtype); Fail("");
  }

  Assert(bit_depth  == 8, "Expected bit dpeth 8");

  if(png_get_valid(png, info, PNG_INFO_tRNS)) {png_set_tRNS_to_alpha(png);}

  png_read_update_info(png, info);

  int rowbytes = png_get_rowbytes(png,info);

  png_bytep *row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * index->height);
  for(int y = 0; y < index->height; y++) {
    row_pointers[y] = (png_byte*)malloc(rowbytes);
  }

  png_read_image(png, row_pointers);

  fclose(ifp);

  png_destroy_read_struct(&png, &info, NULL);

  int width = index->width;
  uint16_t row[width];

  for (int y=0; y<index->height; y++) {
    index->rows[y] = pos;
    png_bytep pngpixels = row_pointers[y];
    switch (color_type) {
      case PNG_COLOR_TYPE_RGB:       for (int x=0; x<width; x++) {
                                       row[x] = rgb565(pngpixels + 3*x);
                                     }
                                     break;
      case PNG_COLOR_TYPE_RGB_ALPHA: for (int x=0; x<width; x++) {
                                       row[x] = rgba565(pngpixels + 4*x);
                                     }
                                     break;
      default: Fail("Unsupported colour type");
    }
    emitrow(row, width);
  }
}


int main(int argc, char *argv[]) {
  int nfiles = argc-1;
  Assert(nfiles > 0, "Expected at least one filename parameter");

  struct indexdata *index = calloc(nfiles, sizeof(struct indexdata));

  printf("uint8_t bits[] = {\n");

  for (int i=0; i<nfiles; i++) {
    char *name = strrchr(argv[i+1], '/');
    if (name == NULL) {name = argv[i+1];} else {name = name+1;}
    int j=0;
    while ((j<31) && (name[j] != 0) && (name[j] != '.')) {
      index[i].name[j] = name[j];
      j++;
    }
    index[i].name[j] = 0;
    //printf("// argv[%d]: '%s'.\n", i+1, argv[i+1]);
    //printf("// name: '%s'.\n", index[i].name);

    processpng(argv[i+1], index+i);
  }

  if (col>0) {printf("\n");}
  printf("};\n\n");

  printf("struct symboldata {\n");
  printf("  int width;\n");
  printf("  int height;\n");
  printf("  const int *rows;\n");
  printf("} symbols[] = {\n");

  for (int i=0; i<nfiles; i++) {
    Assert(index[i].width  <= 320, "Expected width <= 320 pixels");
    Assert(index[i].height <= 320, "Expected height <= 320 pixels");
    printf("  {%d, %d, (const int[]){  // %s\n", index[i].width, index[i].height, index[i].name);
    col = 0;
    for (int j=0; j<index[i].height; j++) {
      if (col == 0)      {printf("    ");}
      else if (col < 16) {printf(", ");}
      else               {printf(",\n    "); col = 0;}
      printf("%5d", index[i].rows[j]);
      col++;
    }
    printf("\n  }},\n");
  }
  printf("};\n\n");

  printf("const int numsyms = %d;\n\n", nfiles);

  printf("enum symbols {\n");
  for (int i=0; i<nfiles; i++) {
    printf("  sym_%-10s= %d,\n", index[i].name, i);
  }
  printf("};\n");


  return 0;
}

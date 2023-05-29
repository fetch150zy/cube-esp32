/**
 * @file main.cpp
 * @author fetch150zy (mars_zhewei@outlook.com)
 * @brief A simple cube
 * @version 0.1
 * @date 2023-05-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define LCD_SPI_HW
#define WITH_LED

#include <Arduino.h>
#include <SPI.h>
#include <Ucglib.h>

// ESP-WROVER-KIT LCD (ILI9341)
#define LCD_RST   18   // RST  --> RESET
#define LCD_SCLK  19   // SCLK --> SCL
#define LCD_DC    21   // DC   --> D/C
#define LCD_CS    22   // CS   --> CS
#define LCD_MOSI  23   // MOSI --> SDA
#define LCD_MISO  25   // MISO <-- SDO
#define LCD_BL    5    // BL   --> LEDK

#if defined(WITH_LED)
#define LED_RED 0     // Onboard Red LED
#define LED_GREEN 2   // Onboard Green LED
#define LED_BLUE 4    // Onboard Blue LED
#endif

#if defined(LCD_SPI_HW)
static Ucglib_ILI9341_18x240x320_HWSPI ucg(LCD_DC, LCD_CS, LCD_RST);
#else
static Ucglib_ILI9341_18x240x320_SWSPI ucg(LCD_SCLK, LCD_MOSI, LCD_DC, LCD_CS, LCD_RST);
#endif

struct pt3d {
  ucg_int_t x, y, z;
};

struct surface {
  uint8_t p[4];
  int16_t z;
};

struct pt2d {
  ucg_int_t x, y;
  unsigned is_visible;
};

#define MX (ucg.getWidth()/2)
#define MY (ucg.getHeight()/2)
#define U 64
#define ZS U
// cube edge
struct pt3d cube[8] = {
  { -U, -U, U}, 
  { U, -U, U}, 
  { U, -U, -U}, 
  { -U, -U, -U}, 
  { -U, U, U}, 
  { U, U, U}, 
  { U, U, -U}, 
  { -U, U, -U}, 
};
// surfaces
struct surface cube_surface[6] = {
  { {0, 1, 2, 3}, 0 },	// bottom
  { {4, 5, 6, 7}, 0 },	// top
  { {0, 1, 5, 4}, 0 },	// back
  
  { {3, 7, 6, 2}, 0 },	// front
  { {1, 2, 6, 5}, 0 },	// right
  { {0, 3, 7, 4}, 0 },	// left
};

struct pt3d cube_copy[8];
struct pt2d cube_pt[8];
ucg_int_t x_min, x_max;
ucg_int_t y_min, y_max;

int16_t sin_tbl[65] = {
  0,1606,3196,4756,6270,7723,9102,10394,11585,12665,13623,14449,15137,15679,16069,16305,16384,16305,16069,15679,
  15137,14449,13623,12665,11585,10394,9102,7723,6270,4756,3196,1606,0,-1605,-3195,-4755,-6269,-7722,-9101,-10393,
  -11584,-12664,-13622,-14448,-15136,-15678,-16068,-16304,-16383,-16304,-16068,-15678,-15136,-14448,-13622,-12664,-11584,-10393,-9101,-7722,
  -6269,-4755,-3195,-1605,0
};

int16_t cos_tbl[65] = {
  16384,16305,16069,15679,15137,14449,13623,12665,11585,10394,9102,7723,6270,4756,3196,1606,0,-1605,-3195,-4755,
  -6269,-7722,-9101,-10393,-11584,-12664,-13622,-14448,-15136,-15678,-16068,-16304,-16383,-16304,-16068,-15678,-15136,-14448,-13622,-12664,
  -11584,-10393,-9101,-7722,-6269,-4755,-3195,-1605,0,1606,3196,4756,6270,7723,9102,10394,11585,12665,13623,14449,
  15137,15679,16069,16305,16384
};


void copy_cube(void)
{
  uint8_t i;
  for(i = 0; i < 8; i++)
    cube_copy[i] = cube[i];
}

void rotate_cube_y(uint16_t w)
{
  uint8_t i;
  int16_t x, z;
  /*
    x' = x * cos(w) + z * sin(w)
    z' = - x * sin(w) + z * cos(w)  
  */
  for(i = 0; i < 8; i++) {
    x = ((int32_t)cube_copy[i].x * (int32_t)cos_tbl[w] + (int32_t)cube_copy[i].z * (int32_t)sin_tbl[w])>>14;
    z = (- (int32_t)cube_copy[i].x * (int32_t)sin_tbl[w] + (int32_t)cube_copy[i].z * (int32_t)cos_tbl[w])>>14;
    cube_copy[i].x = x;
    cube_copy[i].z = z;
  }  
}

void rotate_cube_x(uint16_t w)
{
  uint8_t i;
  int16_t y, z;
  for(i = 0; i < 8; i++) {
    y = ((int32_t)cube_copy[i].y * (int32_t)cos_tbl[w] + (int32_t)cube_copy[i].z * (int32_t)sin_tbl[w])>>14;
    z = (- (int32_t)cube_copy[i].y * (int32_t)sin_tbl[w] + (int32_t)cube_copy[i].z * (int32_t)cos_tbl[w])>>14;
    cube_copy[i].y = y;
    cube_copy[i].z = z;
  }  
}

void trans_cube(uint16_t z)
{
  uint8_t i;
  for(i = 0; i < 8; i++)
    cube_copy[i].z += z;
}

void reset_min_max(void)
{
  x_min = 0x07fff;
  y_min = 0x07fff;
  x_max = -0x07fff;
  y_max = -0x07fff;
}

void convert_3d_to_2d(struct pt3d *p3, struct pt2d *p2)
{
  int32_t t;
  p2->is_visible = 1;
  if (p3->z >= ZS) {
    t = ZS;
    t *= p3->x;
    t <<=1;
    t /= p3->z;
    if (t >= -MX && t <= MX-1) {
      t += MX;
      p2->x = t;
      
      if (x_min > t)
	    x_min = t;
      if (x_max < t)
	    x_max = t;	
      
      t = ZS;
      t *= p3->y;
      t <<=1;
      t /= p3->z;
      if (t >= -MY && t <= MY-1) {
        t += MY;
        p2->y = t;
        
        if (y_min > t)
          y_min = t;
        if (y_max < t)
          y_max = t;	
      } else {
	    p2->is_visible = 0;
      }
    } else {
      p2->is_visible = 0;
    }
  } else {
    p2->is_visible = 0;
  }
}

void convert_cube(void)
{
  uint8_t i;
  reset_min_max();
  for(i = 0; i < 8; i++)
    convert_3d_to_2d(cube_copy+i, cube_pt+i);
}

void calculate_z(void)
{
  uint8_t i, j;
  uint16_t z;
  for(i = 0; i < 6; i++) {
    z = 0;
    for(j = 0; j < 4; j++)
      z+=cube_copy[cube_surface[i].p[j]].z;      
    z/=4;
    cube_surface[i].z = z;
  }
}

void draw_cube(void)
{
  uint8_t i, ii;
  uint8_t skip_cnt = 3;
  int16_t z, z_upper;
  
  z_upper = 32767;
  while(true)
  {
    ii = 6;
    z = -32767;
    for(i = 0; i < 6; i++) {
      if (cube_surface[i].z <= z_upper) {
        if (z < cube_surface[i].z) {
          z = cube_surface[i].z;
          ii = i;
        }
      }
    }
    
    if (ii >= 6)
      break;
    z_upper = cube_surface[ii].z;
    cube_surface[ii].z++;
    
    if (skip_cnt > 0) {
      skip_cnt--;
    } else {
      ucg.setColor(0, ((ii+1)&1)*255,(((ii+1)>>1)&1)*255,(((ii+1)>>2)&1)*255);
      ucg.drawTetragon( 
	  cube_pt[cube_surface[ii].p[0]].x, cube_pt[cube_surface[ii].p[0]].y,
	  cube_pt[cube_surface[ii].p[1]].x, cube_pt[cube_surface[ii].p[1]].y,
	  cube_pt[cube_surface[ii].p[2]].x, cube_pt[cube_surface[ii].p[2]].y,
	  cube_pt[cube_surface[ii].p[3]].x, cube_pt[cube_surface[ii].p[3]].y);
    }
  }
}

void calc_and_draw(uint16_t w, uint16_t v)
{
  copy_cube();
  rotate_cube_y(w);
  rotate_cube_x(v);
  trans_cube(U*8);
  convert_cube();
  calculate_z();
  draw_cube();
}

void setup(void) {
  delay(1000);

#if defined(WITH_LED)
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
#endif

  // Backlight
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, LOW);

#if defined(LCD_SPI_HW)
  SPI.begin(LCD_SCLK, LCD_MISO, LCD_MOSI, LCD_CS);
#endif

  // Initialize Ucglib
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.clearScreen();
  ucg.setRotate90();
  ucg.setFont(ucg_font_8x13_tf);
}

uint16_t w = 0;
uint16_t v = 0;

void loop(void)
{  
  calc_and_draw(w, v>>3);

  v+=3;
  v &= 511;

  w++;
  w &= 63;  
  delay(30);  
  
  ucg.setColor(0,0,0);
  ucg.drawBox(x_min, y_min, x_max-x_min+1, y_max-y_min+1);
}
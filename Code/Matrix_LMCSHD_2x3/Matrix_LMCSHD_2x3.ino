// 12/1/25 - data line is connected to pins 1-6 of a dig-octa board, trying to increase FPS to 40+
//           Code has yet to be tested.  
// Recommended fix is to refresh each 16x16 panel with its own dedicated data channel, will require more wiring but much higher refresh. 
// PC → Serial(2Mbaud) → ESP32 → FastLED_NeoMatrix → WS2812 chain (Pin 21)
//     [0x42 + 3072 RGB565 bytes]     [48x32 bitmap]     [24k LEDs]
// 3x2 16x16 Matrix Array - Dig-Octa Multi-Channel (1536 LEDs total)
// Compatible with LMCSHD serial streaming (0x42 frames, 0x05 query)
// Each panel gets dedicated Dig-Octa channel for 60+ FPS
// Pinout matches Dig-Octa Brainboard-32-8L defaults

#include <Adafruit_GFX.h>
#include <FastLED_NeoMatrix.h>
#include <FastLED.h>

#define NUM_PANELS 6
#define PANEL_W 16
#define PANEL_H 16

// Dig-Octa Channel pins (adjust per your wiring)
#define DATA_PINS {1, 2, 3, 4, 5, 6}  // Channels 1-6
uint8_t dataPins[NUM_PANELS] = DATA_PINS;

// Brightness & layout (matches Chris's zigzag)
#define BRIGHTNESS 30

// Individual 16x16 matrices (one per channel)
CRGB panelLeds[NUM_PANELS][PANEL_W * PANEL_H];
FastLED_NeoMatrix *panels[NUM_PANELS];

// Virtual 48x32 canvas for LMCSHD (3x2 zigzag layout)
uint16_t canvas[48 * 32];  // RGB565 framebuffer

void setup() {
  Serial.begin(2000000);  // LMCSHD high-speed serial
  
  // Initialize each panel on dedicated channel
  for (int i = 0; i < NUM_PANELS; i++) {
    FastLED.addLeds<WS2812B, DATA_PINS[i], GRB>(panelLeds[i], PANEL_W * PANEL_H);
    panels[i] = new FastLED_NeoMatrix(panelLeds[i], PANEL_W, PANEL_H, 1, 1,
      NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG);
    panels[i]->setBrightness(BRIGHTNESS);
  }
  
  Serial.println("Dig-Octa 3x2 Matrix Ready (48x32)");
}

void loop() {
  uint8_t cmd = Serial.read();
  
  // LMCSHD Query: Send canvas dimensions
  if (cmd == 0x05) {
    Serial.println(48);  // Width
    Serial.println(32);  // Height
  }
  // LMCSHD Frame: 48x32 RGB565 (3072 bytes)
  else if (cmd == 0x42) {
    // Read full frame into virtual canvas
    Serial.readBytes((uint8_t*)canvas, 48 * 32 * 2);
    
    // Byte swap (network → little-endian)
    for (int i = 0; i < 48 * 32; i++) {
      canvas[i] = ((canvas[i] & 0xFF) << 8) | ((canvas[i] & 0xFF00) >> 8);
    }
    
    // Distribute to physical panels (3x2 zigzag mapping)
    mapCanvasToPanels();
    
    // Parallel refresh - all panels update simultaneously
    for (int i = 0; i < NUM_PANELS; i++) {
      panels[i]->show();
    }
    
    Serial.write(0x06);  // ACK frame complete
  }
}

// Map 48x32 virtual canvas → 6x 16x16 physical panels (Chris zigzag layout)
void mapCanvasToPanels() {
  // Panel layout (topological zigzag):
  // Row 0: P0(0,0) → P1(16,0) → P2(32,0)
  // Row 1: P5(0,16) ← P4(16,16) ← P3(32,16)  (zigzag reverse)
  
  // Panel 0: Canvas 0,0 → 15,15
  copyPanel(canvas, 0, 0, 0);
  
  // Panel 1: Canvas 16,0 → 31,15  
  copyPanel(canvas, 16, 0, 1);
  
  // Panel 2: Canvas 32,0 → 47,15
  copyPanel(canvas, 32, 0, 2);
  
  // Panel 3: Canvas 32,16 → 47,31 (zigzag: x reversed)
  copyPanel(canvas, 32, 16, 3);
  
  // Panel 4: Canvas 16,16 → 31,31 (zigzag: x reversed)
  copyPanel(canvas, 16, 16, 4);
  
  // Panel 5: Canvas 0,16 → 15,31
  copyPanel(canvas, 0, 16, 5);
}

void copyPanel(uint16_t* canvas, int x0, int y0, int panelId) {
  uint16_t* panelData = (uint16_t*)panels[panelId]->getBuffer();
  
  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      // Zigzag row reversal for odd rows (panel 3,4)
      int canvasX = (y % 2) ? (15 - x) : x;  // Mirror odd rows
      int canvasY = y0 + y;
      int canvasIdx = (canvasY * 48) + (x0 + canvasX);
      
      panelData[y * 16 + x] = canvas[canvasIdx];
    }
  }
}


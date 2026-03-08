import processing.serial.*;

String myString = null;
Serial myPort; 

// The raw 8x8 grid from the sensor
float[][] grid = new float[8][8];
float centerTemp = 0;

void setup() {
  size(400, 450); // Extra 50px at bottom for text info
  noStroke();
  frameRate(60); 
  
  // List serial ports - check the console and change [0] if needed
  printArray(Serial.list());
  try {
    myPort = new Serial(this, Serial.list()[0], 115200);
    myPort.clear();
  } catch (Exception e) {
    println("Could not open serial port. Check your connection!");
  }
  
  colorMode(HSB, 360, 100, 100);
}

void draw() { 
  // 1. PROCESS SERIAL DATA
  while (myPort.available() > 0) {
    myString = myPort.readStringUntil('\n');
    if (myString != null) {
      String[] splitString = splitTokens(trim(myString), ",");
      
      // We expect 64 values
      if (splitString.length >= 64) {
        for (int i = 0; i < 64; i++) {
          int row = i / 8;
          int col = i % 8;
          
          float val = float(splitString[i]);
          
          // Capture center temperature for the readout
          if(i == 28) centerTemp = val; 

          // HUMAN SENSITIVITY CALIBRATION
          // Constrain to human range: 27C (room) to 34C (skin)
          float constrained = constrain(val, 20, 30);
          grid[row][col] = map(constrained, 20, 30, 240, 360); 
        }
      }
    }
  }

  background(0);

  // 2. RENDER INTERPOLATED HEATMAP
  renderInterpolated();

  // 3. UI OVERLAY
  drawUI();
}

void renderInterpolated() {
  int displaySize = 400; // The heatmap is 400x400
  float scale = 7.0 / displaySize; // Map 400 pixels to 7 intervals between 8 points

  loadPixels();
  for (int y = 0; y < displaySize; y++) {
    for (int x = 0; x < displaySize; x++) {
      
      float gx = x * scale;
      float gy = y * scale;
      
      int x1 = floor(gx);
      int y1 = floor(gy);
      int x2 = min(x1 + 1, 7);
      int y2 = min(y1 + 1, 7);
      
      float dx = gx - x1;
      float dy = gy - y1;
      
      // Bilinear Interpolation formula
      float interp = (1-dx)*(1-dy)*grid[y1][x1] + 
                     dx*(1-dy)*grid[y1][x2] + 
                     (1-dx)*dy*grid[y2][x1] + 
                     dx*dy*grid[y2][x2];
      
      pixels[y * width + x] = color(interp, 100, 100);
    }
  }
  updatePixels();
}

void drawUI() {
  // Crosshair in the center
  stroke(0, 0, 100);
  noFill();
  ellipse(width/2, 200, 10, 10);
  line(width/2 - 15, 200, width/2 + 15, 200);
  line(width/2, 200 - 15, width/2, 200 + 15);
  
  // Data Bar at bottom
  fill(0, 0, 10);
  noStroke();
  rect(0, 400, width, 50);
  
  fill(0, 0, 100);
  textAlign(CENTER, CENTER);
  textSize(18);
  text("Center Temp: " + nf(centerTemp, 0, 2) + "°C", width/2, 425);
  
  // Human detection range labels
  textSize(10);
  text("BLUE: <27°C", 50, 425);
  text("RED: >34°C", width-50, 425);
}

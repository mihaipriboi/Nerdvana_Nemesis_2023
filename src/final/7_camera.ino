  // Camera
Pixy2I2C pixy;

const double camera_fps = (1.0 / CAMERA_FPS) * 1000.0;
double camera_last_fps = 0;
int min_area_cube = 100;

#define LINE -1000
const int sig_to_col[] = { 0, 1, -1, LINE, LINE };  // 0 - none, -1 - green, 1 - red

bool sees_line;

int image_w, image_h;

void camera_setup(bool debug) {
  if (debug) Serial.println(F("Cameras starting..."));
  //display_print("Cam err!");
  pixy.init(0x54);
  if (debug) Serial.println(F("Cameras ok!"));

  pixy.getResolution();
  image_w = pixy.frameWidth;
  image_h = pixy.frameHeight;
  // Serial << image_w << " " << image_h << '\n';
}

void read_camera(bool debug) {
  if (millis() - last_camera_read >= camera_interval) {
    pixy.ccc.getBlocks();

    if(pixy.ccc.numBlocks < 1)
      cube_color = 0;
    else
      cube_color = sig_to_col[pixy.ccc.blocks[0].m_signature];

    last_camera_read = millis();
  }
}

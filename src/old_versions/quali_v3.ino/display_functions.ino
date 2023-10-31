/// Display functions

void display_print(const char *s) {
  #ifdef USE_DISPLAY
  u8g2.clearBuffer();
  u8g2.drawStr(30, 20, s);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}

void display_print(const char *s1, const char *s2) {
  #ifdef USE_DISPLAY
  u8g2.clearBuffer();
  u8g2.drawStr(30, 15, s1);
  u8g2.drawStr(30, 25, s2);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}

void display_print(const double n, const char *sufix = "") {
  #ifdef USE_DISPLAY
  char s[100];
  itoa(n, s, 10);
  strcat(s, sufix);

  u8g2.clearBuffer();
  u8g2.drawStr(30, 20, s);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}


void display_print(const double a, const double b) {
  #ifdef USE_DISPLAY
  char s1[100], s2[100];
  itoa(a, s1, 10);
  itoa(b, s2, 10);

  u8g2.clearBuffer();
  u8g2.drawStr(30, 15, s1);
  u8g2.drawStr(30, 25, s2);
  u8g2.sendBuffer();
  #endif // USE_DISPLAY
}
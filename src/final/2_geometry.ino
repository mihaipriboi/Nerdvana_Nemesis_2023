double myacos(double x) {
  double negate = double(x < 0);
  x = abs(x);
  double ret = -0.0187293;
  ret = ret * x;
  ret = ret + 0.0742610;
  ret = ret * x;
  ret = ret - 0.2121144;
  ret = ret * x;
  ret = ret + 1.5707288;
  ret = ret * sqrt(1.0 - x);
  ret = ret - 2 * negate * ret;
  return (negate * 3.14159265358979 + ret) * 57.2958;
}

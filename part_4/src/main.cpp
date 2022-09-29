#include "../include/slamBase.h"
#include <iostream>

void test01() {

  ParameterReader pd;

  cout << pd.getData("camera.cx") << endl;
  cout << pd.getData("detector") << endl;
}

int main(int argc, char **argv) {
  std::cout << "Hello SLAM!" << std::endl;

  // test01();
  return 0;
}

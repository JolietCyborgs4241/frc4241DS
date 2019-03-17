#include <iostream>

#include "simple_auto_arm_extension.c"



main() {

    int i;

    for (i = 0 ; i < 90 ; i++) {
        std::cout << "Angle: " << i << ", Total Arm Length: " << armLengthTable[i] << "\n";
    }
}

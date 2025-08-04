/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <iostream>
#include <chrono>
#include <thread>

#include "Utils/Functions.hpp"
#include "Utils/Classes.hpp"
#include "Utils/AHRS.hpp"
#include "Utils/Sensors.hpp"
#include "ahrs_loc_handler.hpp"
#include "Tests/test_setup.hpp"

int main() {
    AHRS_test();
    return 0;
}

/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <iostream>
#include "Tests/ControllerTests.hpp"
#include "Utils/Functions.hpp"
#include "Utils/Classes.hpp"
#include "Utils/AHRS.hpp"
#include "Tests/test_setup.hpp"

int main() {
    // TestReadCSV();
    // TestShortTermLocalization();
    // TestEigen();
    // TestNlohman();
    // TestPlotInputs();
    // TestSplines3();
    // ApiTest();
    // ApiTest2();
    // TestStanleyControllerSingleSample();
    // TestAffineTransformation();
    // TestProject2dPoints();
    // TestTimeTypes();
    // TestBuffer();
    // TestDelay();
    // TestBufferTemplate();
    // TestDelayTemplate();
    // TestRateLimiter();
    // ahrs_test();
    // TestLPF();
    // TestControlAPI_init();
    // TestLQRControllerSingleSample();
    // TestLQR_FromControlAPI();
    // TestControlAPI_init2();
    // TestLinInterp();
    // Test2ndOrderLPF();
    // TestPathInterp();
    Test_function_convert_path_control_points();
    return 0;
}

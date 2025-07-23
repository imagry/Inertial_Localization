/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once
// lengths
using IntPixels = int;
using Pixels = float;
using MinimapPreciseMeters = float;

using Meters = float;
using KiloPreciseMeters = float;
using Miles = float;
using PreciseMeters = double;

// Area
using Meters2 = float;

// speed
using PreciseMps = double;  // meter(s) per second
using _Mps = double;       // meter(s) per second
using Kph = float;         // kilometer(s) per hour
using Mph = float;         // mile(s) per hour

// angular speed
using DegreesPerSec = float;  // Degree(s) per sec
using RadiansPerSec = float;  // Radian(s) per sec

// acceleration
using Mps2 = float;          // meter per second^2
using Gauss = float;         // Acceleration of gravity
using Mps2Precise = double;  // meter per second^2

// jerk
using Mps3 = float;  // meter per second^3

// time
// probably should be used from chrono allowing math operations
using Seconds = float;
using PreciseSeconds = double;
// using Milliseconds = uint64_t;
// using Microseconds = uint64_t;

// angles
using Degrees = float;
using Radians = float;
using PreciseDegrees = double;
using PreciseRadians = double;

// temperature
using Celsius = float;
using Farenheit = float;
using Kelvin = float;

// Percentage unit of the full size
// values should be in range [0, 1]
using Percentage = double;

// other
using Unitless = float;

// using ClockUTC = std::chrono::system_clock;
// using TimePointUTC = std::chrono::time_point<ClockUTC>;

using Lane = float;
// using LaneIndex = uint8_t;

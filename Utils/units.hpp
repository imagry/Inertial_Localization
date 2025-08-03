/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

// ======================= Length Units =======================
using IntPixels = int;
using Pixels = float;
using MinimapPreciseMeters = float;

using Meters = float;
using KiloPreciseMeters = float;
using Miles = float;
using PreciseMeters = double;

// ======================= Area Units ========================
using Meters2 = float;

// ======================= Speed Units =======================
using PreciseMps = double;  // meter(s) per second
using Mps = double;        // meter(s) per second (renamed from _Mps for consistency)
using Kph = float;         // kilometer(s) per hour
using Mph = float;         // mile(s) per hour

// ======================= Angular Speed Units =======================
using DegreesPerSec = float;  // Degree(s) per sec
using RadiansPerSec = float;  // Radian(s) per sec

// ======================= Acceleration Units =======================
using Mps2 = float;          // meter per second^2
using Gauss = float;         // Acceleration of gravity
using Mps2Precise = double;  // meter per second^2

// ======================= Jerk Units =======================
using Mps3 = float;  // meter per second^3

// ======================= Time Units =======================
using Seconds = float;
using PreciseSeconds = double;
// Chrono time units should be used instead of these when possible
using Milliseconds = uint64_t;
using Microseconds = uint64_t;

// ======================= Angle Units =======================
using Degrees = float;
using Radians = float;
using PreciseDegrees = double;
using PreciseRadians = double;

// ======================= Temperature Units =======================
using Celsius = float;
using Fahrenheit = float;  // Fixed typo in Fahrenheit
using Kelvin = float;

// ======================= Other Units =======================
// Percentage unit of the full size, values should be in range [0, 1]
using Percentage = double;

// Generic unit-less value
using Unitless = float;

// Lane position
using Lane = float;

// Uncomment and use these when switching to std::chrono
// using ClockUTC = std::chrono::system_clock;
// using TimePointUTC = std::chrono::time_point<ClockUTC>;

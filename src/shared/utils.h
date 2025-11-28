#pragma once


// Clamp a value between min and max
template <typename T>
inline T clamp(T value, T minVal, T maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// Map a value from one range to another (linear interpolation)
// Maps 'value' from [inMin, inMax] to [outMin, outMax]
template <typename T>
inline T mapRange(T value, T inMin, T inMax, T outMin, T outMax) {
  if (inMax == inMin) return outMin;  // Avoid division by zero
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Absolute value
template <typename T>
inline T absValue(T value) {
  return (value < 0) ? -value : value;
}

// Calculate difference (always positive)
template <typename T>
inline T difference(T a, T b) {
  return (a > b) ? (a - b) : (b - a);
}

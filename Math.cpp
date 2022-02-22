#include "Math.hpp"

Vector2 operator*(float s, const Vector2& v) { return { v.x * s, v.y * s }; }
Vector3 operator*(float s, const Vector3& v) { return { v.x * s, v.y * s, v.z * s }; }
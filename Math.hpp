#ifndef MATH_HPP
#define MATH_HPP

#include <cmath>

#define PI               3.14159265358979f
#define DEGREE_TO_RADIAN 2.0f * PI / 360.0f
#define RADIAN_TO_DEGREE 1.0f / DEGREE_TO_RADIAN

struct Vector2
{
  float x, y;

  Vector2() = default;

  constexpr Vector2(float _x, float _y)
      : x(_x)
      , y(_y)
  {
  }

  Vector2 operator+(const Vector2& v) const { return { x + v.x, y + v.y }; }
  Vector2 operator-(const Vector2& v) const { return { x - v.x, y - v.y }; }
  Vector2 operator*(float s) const { return { x * s, y * s }; }
  Vector2 operator/(float s) const { return { x / s, y / s }; }
  Vector2 operator+=(const Vector2& v) { return *this = *this + v; }
  Vector2 operator-=(const Vector2& v) { return *this = *this - v; }
  Vector2 operator*=(float s) { return *this = *this * s; }
  Vector2 operator/=(float s) { return *this = *this / s; }
  bool    operator==(const Vector2& v) const { return x == v.x && y == v.y; }
  bool    operator!=(const Vector2& v) const { return !(*this == v); }

  Vector2 operator+() { return *this; }
  Vector2 operator-() { return { -x, -y }; }

  float lengthSq() const { return (x * x) + (y * y); }
  float length() const { return std::sqrt(lengthSq()); }

  Vector2 normalized() const { return *this / length(); }
  void    normalize() { *this = normalized(); }

  float distanceFrom(const Vector2& v) const { return (v - *this).length(); }

  float dot(const Vector2& v) const { return x * v.x + y * v.y; }
  float cross(const Vector2& v) const { return x * v.y - y * v.x; }

  Vector2 rotated(float angle) const { return Vector2 { x * std::cos(angle) - y * std::sin(angle), x * std::sin(angle) + y * std::cos(angle) }; }
  void    rotate(float angle) { *this = rotated(angle); }

  static constexpr Vector2 Zero() { return { 0.0f, 0.0f }; }
  static constexpr Vector2 Right() { return { 1.0f, 0.0f }; }
  static constexpr Vector2 Left() { return { -1.0f, 0.0f }; }
  static constexpr Vector2 Top() { return { 0.0f, -1.0f }; }
  static constexpr Vector2 Down() { return { 0.0f, 1.0f }; }
};

Vector2 operator*(float s, const Vector2& v);

struct Vector3
{
  float x, y, z;

  Vector3() = default;
  constexpr Vector3(float _x, float _y, float _z)
      : x(_x)
      , y(_y)
      , z(_z)
  {
  }

  Vector3 operator+(const Vector3& v) const { return { x + v.x, y + v.y, z + v.z }; }
  Vector3 operator-(const Vector3& v) const { return { x - v.x, y - v.y, z - v.z }; }
  Vector3 operator*(float s) const { return { x * s, y * s, z * s }; }
  Vector3 operator/(float s) const { return { x / s, y / s, z / s }; }
  Vector3 operator+=(const Vector3& v) { return *this = *this + v; }
  Vector3 operator-=(const Vector3& v) { return *this = *this - v; }
  Vector3 operator*=(float s) { return *this = *this * s; }
  Vector3 operator/=(float s) { return *this = *this / s; }
  bool    operator==(const Vector3& v) const { return x == v.x && y == v.y && z == v.z; }
  bool    operator!=(const Vector3& v) const { return !(*this == v); }

  Vector3 operator+() { return *this; }
  Vector3 operator-() { return { -x, -y, -z }; }

  float lengthSq() const { return (x * x) + (y * y) + (z * z); }
  float length() const { return std::sqrt(lengthSq()); }

  Vector3 normalized() const { return *this / length(); }
  void    normalize() { *this = normalized(); }

  float distanceFrom(const Vector3& v) const { return (v - *this).length(); }

  float   dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
  Vector3 cross(const Vector3& v) const { return { y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x }; }

  float angle(const Vector3& v) const { return std::acos(normalized().dot(v.normalized())); }

  static constexpr Vector3 Zero() { return { 0.0f, 0.0f, 0.0f }; }
  static constexpr Vector3 Right() { return { 1.0f, 0.0f, 0.0f }; }
  static constexpr Vector3 Left() { return { -1.0f, 0.0f, 0.0f }; }
  static constexpr Vector3 Top() { return { 0.0f, -1.0f, 0.0f }; }
  static constexpr Vector3 Down() { return { 0.0f, 1.0f, 0.0f }; }
  static constexpr Vector3 Forward() { return { 0.0f, 0.0f, 1.0f }; }
  static constexpr Vector3 Back() { return { 0.0f, 0.0f, -1.0f }; }
};

Vector3 operator*(float s, const Vector3& v);

struct Quaternion
{
private:
  Vector3 v() const { return { q1, q2, q3 }; }

public:
  float q0, q1, q2, q3;

  Quaternion() = default;
  constexpr Quaternion(float _q0, float _q1, float _q2, float _q3)
      : q0(_q0)
      , q1(_q1)
      , q2(_q2)
      , q3(_q3)
  {
  }

  Quaternion(const Vector3& v, float angle)
  {
    auto normalizedV = v.normalized();

    float c = std::cos(angle / 2.0f);
    float s = std::sin(angle / 2.0f);
    q0 = c;
    q1 = s * normalizedV.x;
    q2 = s * normalizedV.y;
    q3 = s * normalizedV.z;
  }

  static Quaternion Identity() { return { 1.0f, 0.0f, 0.0f, 0.0f }; }

  Quaternion conjugate() const { return { q0, -q1, -q2, -q3 }; }

  float magnitude() const { return std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); }

  Quaternion operator*(const Quaternion& p) const
  {
    float nq0 = (q0 * p.q0) - (q1 * p.q1) - (q2 * p.q2) - (q3 * p.q3);
    float nq1 = (q0 * p.q1) + (q1 * p.q0) + (q2 * p.q3) - (q3 * p.q2);
    float nq2 = (q0 * p.q2) + (q2 * p.q0) - (q1 * p.q3) + (q3 * p.q1);
    float nq3 = (q0 * p.q3) + (q3 * p.q0) + (q1 * p.q2) - (q2 * p.q1);
    return { nq0, nq1, nq2, nq3 };
  }

  void normalize() { *this = normalized(); }

  Quaternion normalized() const
  {
    float m = magnitude();
    return { q0 / m, q1 / m, q2 / m, q3 / m };
  }

  Vector3 operator*(const Vector3& v) const { return (*this * Quaternion(0.0f, v.x, v.y, v.z) * this->conjugate()).v(); }

  float roll() const { return std::atan2(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3); }
  float pitch() const { return std::asin(2.0f * (q0 * q2 - q1 * q3)); }
  float yaw() const { return std::atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3); }
};

#endif /* MATH_HPP */
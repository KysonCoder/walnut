#ifndef ARRAY_OPERATIONS_H_
#define ARRAY_OPERATIONS_H_

#include <array>
#include <stdexcept>
#include <algorithm>

template <typename T, std::size_t N>
class ArrayOperations {
 public:
  // 数组加法
  static std::array<T, N> Add(const std::array<T, N>& a,
                               const std::array<T, N>& b) {
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = a[i] + b[i];
    }
    return result;
  }

  // 数组减法
  static std::array<T, N> Subtract(const std::array<T, N>& a,
                                    const std::array<T, N>& b) {
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = a[i] - b[i];
    }
    return result;
  }

  // 数组乘法（逐元素）
  static std::array<T, N> Multiply(const std::array<T, N>& a,
                                    const std::array<T, N>& b) {
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = a[i] * b[i];
    }
    return result;
  }

  // 数组除法（逐元素）
  static std::array<T, N> Divide(const std::array<T, N>& a,
                                  const std::array<T, N>& b) {
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
      if (b[i] == 0) {
        throw std::runtime_error("Division by zero");
      }
      result[i] = a[i] / b[i];
    }
    return result;
  }

  // 求最大值
  static T Max(const std::array<T, N>& arr) {
    if (N == 0) {
      throw std::runtime_error("Cannot find max of empty array");
    }
    return *std::max_element(arr.begin(), arr.end());
  }

  // 求最小值
  static T Min(const std::array<T, N>& arr) {
    if (N == 0) {
      throw std::runtime_error("Cannot find min of empty array");
    }
    return *std::min_element(arr.begin(), arr.end());
  }
};

// ==================== 运算符重载 ====================

// 数组加法: a + b
template <typename T, std::size_t N>
std::array<T, N> operator+(const std::array<T, N>& a,
                           const std::array<T, N>& b) {
  return ArrayOperations<T, N>::Add(a, b);
}

// 数组减法: a - b
template <typename T, std::size_t N>
std::array<T, N> operator-(const std::array<T, N>& a,
                           const std::array<T, N>& b) {
  return ArrayOperations<T, N>::Subtract(a, b);
}

// 数组乘法（逐元素）: a * b
template <typename T, std::size_t N>
std::array<T, N> operator*(const std::array<T, N>& a,
                           const std::array<T, N>& b) {
  return ArrayOperations<T, N>::Multiply(a, b);
}

// 数组除法（逐元素）: a / b
template <typename T, std::size_t N>
std::array<T, N> operator/(const std::array<T, N>& a,
                           const std::array<T, N>& b) {
  return ArrayOperations<T, N>::Divide(a, b);
}

// 数组与标量乘法: arr * scalar
template <typename T, std::size_t N>
std::array<T, N> operator*(const std::array<T, N>& arr, T scalar) {
  std::array<T, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = arr[i] * scalar;
  }
  return result;
}

// 标量与数组乘法: scalar * arr
template <typename T, std::size_t N>
std::array<T, N> operator*(T scalar, const std::array<T, N>& arr) {
  return arr * scalar;
}

// 数组与标量除法: arr / scalar
template <typename T, std::size_t N>
std::array<T, N> operator/(const std::array<T, N>& arr, T scalar) {
  if (scalar == 0) {
    throw std::runtime_error("Division by zero");
  }
  std::array<T, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = arr[i] / scalar;
  }
  return result;
}

// 数组 += 运算
template <typename T, std::size_t N>
std::array<T, N>& operator+=(std::array<T, N>& a,
                             const std::array<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    a[i] += b[i];
  }
  return a;
}

// 数组 -= 运算
template <typename T, std::size_t N>
std::array<T, N>& operator-=(std::array<T, N>& a,
                             const std::array<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    a[i] -= b[i];
  }
  return a;
}

// 数组 *= 运算（逐元素）
template <typename T, std::size_t N>
std::array<T, N>& operator*=(std::array<T, N>& a,
                             const std::array<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    a[i] *= b[i];
  }
  return a;
}

// 数组 /= 运算（逐元素）
template <typename T, std::size_t N>
std::array<T, N>& operator/=(std::array<T, N>& a,
                             const std::array<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    if (b[i] == 0) {
      throw std::runtime_error("Division by zero");
    }
    a[i] /= b[i];
  }
  return a;
}

// 数组 *= 标量
template <typename T, std::size_t N>
std::array<T, N>& operator*=(std::array<T, N>& arr, T scalar) {
  for (std::size_t i = 0; i < N; ++i) {
    arr[i] *= scalar;
  }
  return arr;
}

// 数组 /= 标量
template <typename T, std::size_t N>
std::array<T, N>& operator/=(std::array<T, N>& arr, T scalar) {
  if (scalar == 0) {
    throw std::runtime_error("Division by zero");
  }
  for (std::size_t i = 0; i < N; ++i) {
    arr[i] /= scalar;
  }
  return arr;
}

#endif  // ARRAY_OPERATIONS_H_

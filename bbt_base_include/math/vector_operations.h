#ifndef VECTOR_OPERATIONS_H_
#define VECTOR_OPERATIONS_H_

#include <vector>
#include <stdexcept>
#include <algorithm>

template <typename T>
class VectorOperations {
 public:
  // 向量加法
  static std::vector<T> Add(const std::vector<T>& a, const std::vector<T>& b) {
    CheckSize(a, b);
    std::vector<T> result;
    result.reserve(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
      result.push_back(a[i] + b[i]);
    }
    return result;
  }

  // 向量减法
  static std::vector<T> Subtract(const std::vector<T>& a, const std::vector<T>& b) {
    CheckSize(a, b);
    std::vector<T> result;
    result.reserve(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
      result.push_back(a[i] - b[i]);
    }
    return result;
  }

  // 向量乘法（逐元素）
  static std::vector<T> Multiply(const std::vector<T>& a, const std::vector<T>& b) {
    CheckSize(a, b);
    std::vector<T> result;
    result.reserve(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
      result.push_back(a[i] * b[i]);
    }
    return result;
  }

  // 向量除法（逐元素）
  static std::vector<T> Divide(const std::vector<T>& a, const std::vector<T>& b) {
    CheckSize(a, b);
    std::vector<T> result;
    result.reserve(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
      if (b[i] == 0) {
        throw std::runtime_error("Division by zero");
      }
      result.push_back(a[i] / b[i]);
    }
    return result;
  }

  // 求最大值
  static T Max(const std::vector<T>& vec) {
    if (vec.empty()) {
      throw std::runtime_error("Cannot find max of empty vector");
    }
    return *std::max_element(vec.begin(), vec.end());
  }

  // 求最小值
  static T Min(const std::vector<T>& vec) {
    if (vec.empty()) {
      throw std::runtime_error("Cannot find min of empty vector");
    }
    return *std::min_element(vec.begin(), vec.end());
  }

 private:
  static void CheckSize(const std::vector<T>& a, const std::vector<T>& b) {
    if (a.size() != b.size()) {
      throw std::invalid_argument("Vector sizes do not match");
    }
  }
};

// ==================== 运算符重载 ====================

// 向量加法: a + b
template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b) {
  return VectorOperations<T>::Add(a, b);
}

// 向量减法: a - b
template <typename T>
std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b) {
  return VectorOperations<T>::Subtract(a, b);
}

// 向量乘法（逐元素）: a * b
template <typename T>
std::vector<T> operator*(const std::vector<T>& a, const std::vector<T>& b) {
  return VectorOperations<T>::Multiply(a, b);
}

// 向量除法（逐元素）: a / b
template <typename T>
std::vector<T> operator/(const std::vector<T>& a, const std::vector<T>& b) {
  return VectorOperations<T>::Divide(a, b);
}

// 向量与标量乘法: vec * scalar
template <typename T>
std::vector<T> operator*(const std::vector<T>& vec, T scalar) {
  std::vector<T> result;
  result.reserve(vec.size());
  for (const auto& elem : vec) {
    result.push_back(elem * scalar);
  }
  return result;
}

// 标量与向量乘法: scalar * vec
template <typename T>
std::vector<T> operator*(T scalar, const std::vector<T>& vec) {
  return vec * scalar;
}

// 向量与标量除法: vec / scalar
template <typename T>
std::vector<T> operator/(const std::vector<T>& vec, T scalar) {
  if (scalar == 0) {
    throw std::runtime_error("Division by zero");
  }
  std::vector<T> result;
  result.reserve(vec.size());
  for (const auto& elem : vec) {
    result.push_back(elem / scalar);
  }
  return result;
}

// 向量 += 运算
template <typename T>
std::vector<T>& operator+=(std::vector<T>& a, const std::vector<T>& b) {
  if (a.size() != b.size()) {
    throw std::invalid_argument("Vector sizes do not match");
  }
  for (size_t i = 0; i < a.size(); ++i) {
    a[i] += b[i];
  }
  return a;
}

// 向量 -= 运算
template <typename T>
std::vector<T>& operator-=(std::vector<T>& a, const std::vector<T>& b) {
  if (a.size() != b.size()) {
    throw std::invalid_argument("Vector sizes do not match");
  }
  for (size_t i = 0; i < a.size(); ++i) {
    a[i] -= b[i];
  }
  return a;
}

// 向量 *= 运算（逐元素）
template <typename T>
std::vector<T>& operator*=(std::vector<T>& a, const std::vector<T>& b) {
  if (a.size() != b.size()) {
    throw std::invalid_argument("Vector sizes do not match");
  }
  for (size_t i = 0; i < a.size(); ++i) {
    a[i] *= b[i];
  }
  return a;
}

// 向量 /= 运算（逐元素）
template <typename T>
std::vector<T>& operator/=(std::vector<T>& a, const std::vector<T>& b) {
  if (a.size() != b.size()) {
    throw std::invalid_argument("Vector sizes do not match");
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (b[i] == 0) {
      throw std::runtime_error("Division by zero");
    }
    a[i] /= b[i];
  }
  return a;
}

// 向量 *= 标量
template <typename T>
std::vector<T>& operator*=(std::vector<T>& vec, T scalar) {
  for (auto& elem : vec) {
    elem *= scalar;
  }
  return vec;
}

// 向量 /= 标量
template <typename T>
std::vector<T>& operator/=(std::vector<T>& vec, T scalar) {
  if (scalar == 0) {
    throw std::runtime_error("Division by zero");
  }
  for (auto& elem : vec) {
    elem /= scalar;
  }
  return vec;
}

#endif  // VECTOR_OPERATIONS_H_

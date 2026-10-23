#ifndef MASKED_ARRAY_OPERATIONS_H_
#define MASKED_ARRAY_OPERATIONS_H_

#include <array>
#include <optional>
#include <algorithm>
#include <cstddef>
#include <cstdio>

template <typename T, std::size_t N>
class MaskedArray {
 public:
  MaskedArray(const std::array<T, N>& data,
              const std::array<bool, N>& mask,
              T default_value = T{})
      : data_(data), mask_(mask), default_value_(default_value) {}

  T operator[](std::size_t i) const {
    if (i >= N) {
      std::fprintf(stderr, "[ERROR] Index out of bounds: %zu (size=%zu)\n", i, N);
      return default_value_;
    }
    return mask_[i] ? data_[i] : default_value_;
  }

  T& operator[](std::size_t i) {
    if (i >= N) {
      std::fprintf(stderr, "[ERROR] Index out of bounds: %zu (size=%zu)\n", i, N);
      static T dummy{};
      return dummy;
    }
    if (!mask_[i]) {
      static T dummy{};
      return dummy;
    }
    return data_[i];
  }

  bool is_valid(std::size_t i) const {
    if (i >= N) {
      std::fprintf(stderr, "[ERROR] Index out of bounds: %zu (size=%zu)\n", i, N);
      return false;
    }
    return mask_[i];
  }

  void update_mask(const std::array<bool, N>& new_mask) {
    mask_ = new_mask;
  }

  void set_valid(std::size_t i, bool valid) {
    if (i >= N) {
      std::fprintf(stderr, "[ERROR] set_valid: index out of bounds: %zu (size=%zu)\n", i, N);
      return;
    }
    mask_[i] = valid;
  }

  void set_valid_range(std::size_t start, std::size_t end, bool valid) {
    if (start > end || start >= N) {
      std::fprintf(stderr, "[ERROR] set_valid_range: invalid range [%zu, %zu] (size=%zu)\n", start, end, N);
      return;
    }
    for (std::size_t i = start; i <= end && i < N; ++i) {
      mask_[i] = valid;
    }
  }

  void set_all_valid() {
    mask_.fill(true);
  }

  void set_all_invalid() {
    mask_.fill(false);
  }

  MaskedArray& operator=(const std::array<T, N>& arr) {
    for (std::size_t i = 0; i < N; ++i) {
      if (mask_[i]) {
        data_[i] = arr[i];
      }
    }
    return *this;
  }

  void assign_from_array(const std::array<T, N>& arr) {
    *this = arr;
  }

  void assign_from_array(const std::array<T, N>& arr,
                         const std::array<bool, N>& new_mask) {
    mask_ = new_mask;
    *this = arr;
  }

  const std::array<T, N>& data() const { return data_; }
  const std::array<bool, N>& mask() const { return mask_; }
  T default_value() const { return default_value_; }
  void set_default(T value) { default_value_ = value; }

  std::size_t valid_count() const {
    return std::count(mask_.begin(), mask_.end(), true);
  }

  bool any_valid() const {
    return std::any_of(mask_.begin(), mask_.end(), [](bool b) { return b; });
  }

  bool all_valid() const {
    return std::all_of(mask_.begin(), mask_.end(), [](bool b) { return b; });
  }

  std::array<T, N> to_array() const {
    std::array<T, N> result{};
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = mask_[i] ? data_[i] : default_value_;
    }
    return result;
  }

 private:
  std::array<T, N> data_;
  std::array<bool, N> mask_;
  T default_value_;
};

// ==================== Arithmetic Operators ====================

template <typename T, std::size_t N>
MaskedArray<T, N> operator+(const MaskedArray<T, N>& a,
                            const MaskedArray<T, N>& b) {
  std::array<T, N> result_data{};
  std::array<bool, N> result_mask{};
  for (std::size_t i = 0; i < N; ++i) {
    result_mask[i] = a.mask()[i] && b.mask()[i];
    if (result_mask[i]) {
      result_data[i] = a.data()[i] + b.data()[i];
    }
  }
  return MaskedArray<T, N>(result_data, result_mask, a.default_value());
}

template <typename T, std::size_t N>
MaskedArray<T, N> operator-(const MaskedArray<T, N>& a,
                            const MaskedArray<T, N>& b) {
  std::array<T, N> result_data{};
  std::array<bool, N> result_mask{};
  for (std::size_t i = 0; i < N; ++i) {
    result_mask[i] = a.mask()[i] && b.mask()[i];
    if (result_mask[i]) {
      result_data[i] = a.data()[i] - b.data()[i];
    }
  }
  return MaskedArray<T, N>(result_data, result_mask, a.default_value());
}

template <typename T, std::size_t N>
MaskedArray<T, N> operator*(const MaskedArray<T, N>& a,
                            const MaskedArray<T, N>& b) {
  std::array<T, N> result_data{};
  std::array<bool, N> result_mask{};
  for (std::size_t i = 0; i < N; ++i) {
    result_mask[i] = a.mask()[i] && b.mask()[i];
    if (result_mask[i]) {
      result_data[i] = a.data()[i] * b.data()[i];
    }
  }
  return MaskedArray<T, N>(result_data, result_mask, a.default_value());
}

template <typename T, std::size_t N>
MaskedArray<T, N> operator/(const MaskedArray<T, N>& a,
                            const MaskedArray<T, N>& b) {
  std::array<T, N> result_data{};
  std::array<bool, N> result_mask{};
  for (std::size_t i = 0; i < N; ++i) {
    result_mask[i] = a.mask()[i] && b.mask()[i];
    if (result_mask[i]) {
      if (b.data()[i] == 0) {
        std::fprintf(stderr, "[ERROR] Division by zero at index %zu\n", i);
        result_mask[i] = false;
      } else {
        result_data[i] = a.data()[i] / b.data()[i];
      }
    }
  }
  return MaskedArray<T, N>(result_data, result_mask, a.default_value());
}

// ==================== Scalar Operations ====================

template <typename T, std::size_t N>
MaskedArray<T, N> operator*(const MaskedArray<T, N>& arr, T scalar) {
  std::array<T, N> result_data = arr.data();
  for (std::size_t i = 0; i < N; ++i) {
    if (arr.mask()[i]) {
      result_data[i] = arr.data()[i] * scalar;
    }
  }
  return MaskedArray<T, N>(result_data, arr.mask(), arr.default_value());
}

template <typename T, std::size_t N>
MaskedArray<T, N> operator*(T scalar, const MaskedArray<T, N>& arr) {
  return arr * scalar;
}

template <typename T, std::size_t N>
MaskedArray<T, N> operator/(const MaskedArray<T, N>& arr, T scalar) {
  std::array<T, N> result_data = arr.data();
  std::array<bool, N> result_mask = arr.mask();
  if (scalar == 0) {
    std::fprintf(stderr, "[ERROR] Scalar division by zero\n");
    return MaskedArray<T, N>(result_data, std::array<bool, N>{}, arr.default_value());
  }
  for (std::size_t i = 0; i < N; ++i) {
    if (arr.mask()[i]) {
      result_data[i] = arr.data()[i] / scalar;
    }
  }
  return MaskedArray<T, N>(result_data, result_mask, arr.default_value());
}

// ==================== Compound Assignment Operators ====================

template <typename T, std::size_t N>
MaskedArray<T, N>& operator+=(MaskedArray<T, N>& a,
                              const MaskedArray<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    if (a.mask()[i] && b.mask()[i]) {
      a[i] = a.data()[i] + b.data()[i];
    }
  }
  return a;
}

template <typename T, std::size_t N>
MaskedArray<T, N>& operator-=(MaskedArray<T, N>& a,
                              const MaskedArray<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    if (a.mask()[i] && b.mask()[i]) {
      a[i] = a.data()[i] - b.data()[i];
    }
  }
  return a;
}

template <typename T, std::size_t N>
MaskedArray<T, N>& operator*=(MaskedArray<T, N>& a,
                              const MaskedArray<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    if (a.mask()[i] && b.mask()[i]) {
      a[i] = a.data()[i] * b.data()[i];
    }
  }
  return a;
}

template <typename T, std::size_t N>
MaskedArray<T, N>& operator/=(MaskedArray<T, N>& a,
                              const MaskedArray<T, N>& b) {
  for (std::size_t i = 0; i < N; ++i) {
    if (a.mask()[i] && b.mask()[i]) {
      if (b.data()[i] == 0) {
        std::fprintf(stderr, "[ERROR] Division by zero in operator/= at index %zu\n", i);
      } else {
        a[i] = a.data()[i] / b.data()[i];
      }
    }
  }
  return a;
}

// ==================== Aggregate Functions ====================

template <typename T, std::size_t N>
std::optional<T> masked_max(const MaskedArray<T, N>& arr) {
  bool found = false;
  T max_val{};
  for (std::size_t i = 0; i < N; ++i) {
    if (arr.mask()[i]) {
      if (!found || arr.data()[i] > max_val) {
        max_val = arr.data()[i];
        found = true;
      }
    }
  }
  return found ? std::optional<T>(max_val) : std::nullopt;
}

template <typename T, std::size_t N>
std::optional<T> masked_min(const MaskedArray<T, N>& arr) {
  bool found = false;
  T min_val{};
  for (std::size_t i = 0; i < N; ++i) {
    if (arr.mask()[i]) {
      if (!found || arr.data()[i] < min_val) {
        min_val = arr.data()[i];
        found = true;
      }
    }
  }
  return found ? std::optional<T>(min_val) : std::nullopt;
}

template <typename T, std::size_t N>
std::optional<T> masked_sum(const MaskedArray<T, N>& arr) {
  if (!arr.any_valid()) {
    return std::nullopt;
  }
  T sum{};
  for (std::size_t i = 0; i < N; ++i) {
    if (arr.mask()[i]) {
      sum += arr.data()[i];
    }
  }
  return sum;
}

template <typename T, std::size_t N>
std::optional<T> masked_mean(const MaskedArray<T, N>& arr) {
  auto sum = masked_sum(arr);
  if (!sum.has_value()) {
    return std::nullopt;
  }
  return sum.value() / static_cast<T>(arr.valid_count());
}

// ==================== Factory Functions ====================

template <typename T, std::size_t N>
MaskedArray<T, N> apply_mask(const std::array<T, N>& data,
                             const std::array<bool, N>& mask,
                             T default_val = T{}) {
  return MaskedArray<T, N>(data, mask, default_val);
}

template <typename T, std::size_t N>
MaskedArray<T, N> all_valid(const std::array<T, N>& data,
                            T default_val = T{}) {
  std::array<bool, N> mask;
  mask.fill(true);
  return MaskedArray<T, N>(data, mask, default_val);
}

template <typename T, std::size_t N>
MaskedArray<T, N> all_invalid(const std::array<T, N>& data,
                              T default_val = T{}) {
  std::array<bool, N> mask;
  mask.fill(false);
  return MaskedArray<T, N>(data, mask, default_val);
}

#endif  // MASKED_ARRAY_OPERATIONS_H_

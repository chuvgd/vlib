/**
 * @file matrix-arm.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-10-06
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef MATRIX_ARM_HPP
#define MATRIX_ARM_HPP

#include "arm_math.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <initializer_list>
#include <type_traits>

// Set to 1 to enable run-time bounds checking in operator() and operator[].
// Leave at 0 in release builds on STM32.
#ifndef MATRIX_ARM_BOUNDS_CHECK
    #define MATRIX_ARM_BOUNDS_CHECK 0
#endif

#if MATRIX_ARM_BOUNDS_CHECK
    #include <cassert>
    #define MATRIX_ARM_CHECK(cond) assert(cond)
#else
    #define MATRIX_ARM_CHECK(cond) ((void)0)
#endif

namespace vlib {
namespace algo {

/**
 * @brief Fixed-size matrix backed by CMSIS-DSP.
 *
 * Requirements:
 *  - C++14 or later (constexpr constructors with loops).
 *
 * Design notes (STM32-friendly):
 *  - Only the raw data is stored. The arm_matrix_instance_f32 descriptor is
 *    built on demand via cmsis() using the official arm_mat_init_f32().
 *    Each instance occupies exactly R*C*4 bytes of RAM, same as a plain
 *    array.
 *  - All static factories are constexpr, so constant matrices (e.g. eye())
 *    live in flash (.rodata) instead of being constructed at startup.
 *  - Copy/move are compiler-generated and correct, because the object does
 *    not hold a self-referential pointer.
 *  - reshape<R2,C2>() reinterprets the flat buffer with a new shape. Data
 *    order (row-major) is preserved. Compile-time size check.
 *  - Access is by either m[i][j] (array-style) or m(i, j) (Eigen-style).
 *  - No heap, no dynamic allocation, no expression templates.
 */
template<std::size_t R, std::size_t C>
class Matrixf_ARM {
    static_assert(R > 0 && C > 0, "Matrixf_ARM: empty matrix");
    static_assert(R <= 65535 && C <= 65535, "Matrixf_ARM: CMSIS uses uint16_t for dimensions");

public:
    // ======================================================================
    // Construction
    // ======================================================================

    /// Default: zero-initialised. constexpr.
    constexpr Matrixf_ARM() noexcept:
        data_ {} {}

    /// From initializer list. Missing elements are zero, extra are ignored.
    ///   Matrixf_ARM<3,3> m = {1,2,3, 4,5,6, 7,8,9};
    ///   Matrixf_ARM<6,1> v = {0, 0, 0, 0, 0, 0};
    constexpr Matrixf_ARM(std::initializer_list<float> il) noexcept:
        data_ {} {
        const std::size_t n = (il.size() < R * C) ? il.size() : (R * C);
        const float* p      = il.begin();
        for (std::size_t i = 0; i < n; ++i)
            data_[i] = p[i];
    }

    /// Copy from a fixed-size C array. Compile-time size check.
    explicit Matrixf_ARM(const float (&arr)[R * C]) noexcept {
        for (std::size_t i = 0; i < R * C; ++i)
            data_[i] = arr[i];
    }

    /// Copy from a raw pointer. Caller must guarantee at least R*C floats.
    explicit Matrixf_ARM(const float* p) noexcept {
        for (std::size_t i = 0; i < R * C; ++i)
            data_[i] = p[i];
    }

    // Copy/move: compiler-generated ones are correct here.
    Matrixf_ARM(const Matrixf_ARM&)                = default;
    Matrixf_ARM(Matrixf_ARM&&) noexcept            = default;
    Matrixf_ARM& operator=(const Matrixf_ARM&)     = default;
    Matrixf_ARM& operator=(Matrixf_ARM&&) noexcept = default;
    ~Matrixf_ARM()                                 = default;

    // ======================================================================
    // Static factories (all constexpr -> usable as flash constants)
    // ======================================================================

    static constexpr Matrixf_ARM zeros() noexcept {
        return Matrixf_ARM {};
    }

    static constexpr Matrixf_ARM ones() noexcept {
        Matrixf_ARM m;
        for (std::size_t i = 0; i < R * C; ++i)
            m.data_[i] = 1.f;
        return m;
    }

    static constexpr Matrixf_ARM eye() noexcept {
        Matrixf_ARM m;
        constexpr std::size_t N = (R < C) ? R : C;
        for (std::size_t i = 0; i < N; ++i)
            m.data_[i * C + i] = 1.f;
        return m;
    }

    /// Diagonal matrix from a column vector of length R.
    static constexpr Matrixf_ARM diag(const Matrixf_ARM<R, 1>& vec) noexcept {
        Matrixf_ARM m;
        constexpr std::size_t N = (R < C) ? R : C;
        for (std::size_t i = 0; i < N; ++i)
            m.data_[i * C + i] = vec.data_[i];
        return m;
    }

    // ======================================================================
    // Element access
    //
    // Two styles, both read/write:
    //   m[i][j]   array-style
    //   m(i, j)   Eigen/MATLAB-style, row-first
    //
    // For vectors only, the second index may be omitted:
    //   Vec3f v;   v(i) == v(i, 0)
    //   Matrixf_ARM<1,4> r;  r(i) == r(0, i)
    //
    // Bounds checking is enabled by defining MATRIX_ARM_BOUNDS_CHECK=1.
    // ======================================================================

    // --- m[i][j] -----------------------------------------------------------
    float* operator[](std::size_t row) noexcept {
        MATRIX_ARM_CHECK(row < R);
        return data_ + row * C;
    }
    const float* operator[](std::size_t row) const noexcept {
        MATRIX_ARM_CHECK(row < R);
        return data_ + row * C;
    }

    // --- m(i, j) -----------------------------------------------------------
    float& operator()(std::size_t row, std::size_t col) noexcept {
        MATRIX_ARM_CHECK(row < R && col < C);
        return data_[row * C + col];
    }
    const float& operator()(std::size_t row, std::size_t col) const noexcept {
        MATRIX_ARM_CHECK(row < R && col < C);
        return data_[row * C + col];
    }

    // --- m(i) : vector-only, single index ----------------------------------
    // Enabled only when the matrix is a row vector or a column vector.
    // For 1x1 both conditions hold, but there is still only one overload,
    // so no ambiguity.
    template<std::size_t CC = C, std::size_t RR = R>
    typename std::enable_if<CC == 1 || RR == 1, float&>::type operator()(std::size_t i) noexcept {
        MATRIX_ARM_CHECK(i < R * C);
        return data_[i];
    }
    template<std::size_t CC = C, std::size_t RR = R>
    typename std::enable_if<CC == 1 || RR == 1, const float&>::type
    operator()(std::size_t i) const noexcept {
        MATRIX_ARM_CHECK(i < R * C);
        return data_[i];
    }

    // --- data / dimensions -------------------------------------------------
    float* data() noexcept {
        return data_;
    }
    const float* data() const noexcept {
        return data_;
    }

    static constexpr std::size_t rows() noexcept {
        return R;
    }
    static constexpr std::size_t cols() noexcept {
        return C;
    }
    static constexpr std::size_t size() noexcept {
        return R * C;
    }

    // ======================================================================
    // Shape change (reshape)
    //
    // Reinterprets the flat row-major buffer with a new shape. The total
    // number of elements must match, enforced at compile time.
    //
    // Example:
    //     Matrixf_ARM<2,6> a = { ... 12 values ... };
    //     auto b = a.reshape<6,2>();   // b(0,0)=a(0,0), b(0,1)=a(0,1), ...
    //     auto c = a.flatten();        // Matrixf_ARM<12,1>
    //
    // This is NOT a transpose. Use trans() for that.
    // ======================================================================

    template<std::size_t R2, std::size_t C2>
    Matrixf_ARM<R2, C2> reshape() const noexcept {
        static_assert(R2 > 0 && C2 > 0, "reshape: empty matrix");
        static_assert(R2 <= 65535 && C2 <= 65535, "reshape: CMSIS uses uint16_t");
        static_assert(R2 * C2 == R * C, "reshape: total size must match");

        Matrixf_ARM<R2, C2> res;
        std::memcpy(res.data(), data_, R * C * sizeof(float));
        return res;
    }

    /// In-place target version: avoids constructing a temporary in hot paths.
    template<std::size_t R2, std::size_t C2>
    void reshapeTo(Matrixf_ARM<R2, C2>& out) const noexcept {
        static_assert(R2 > 0 && C2 > 0, "reshape: empty matrix");
        static_assert(R2 <= 65535 && C2 <= 65535, "reshape: CMSIS uses uint16_t");
        static_assert(R2 * C2 == R * C, "reshape: total size must match");

        std::memcpy(out.data(), data_, R * C * sizeof(float));
    }

    /// Flatten to a column vector of size R*C.
    Matrixf_ARM<R * C, 1> flatten() const noexcept {
        return reshape<R * C, 1>();
    }

    /// Flatten to a row vector of size R*C.
    Matrixf_ARM<1, R * C> flattenRow() const noexcept {
        return reshape<1, R * C>();
    }

    // ======================================================================
    // CMSIS bridge
    //
    // Builds a descriptor on the stack using the official CMSIS API. The
    // 8-byte descriptor only lives for the duration of the call.
    //
    // DO NOT take the address of the returned value directly:
    //     arm_mat_add_f32(&a.cmsis(), ...);   // WRONG
    // Instead:
    //     auto ma = a.cmsis();
    //     arm_mat_add_f32(&ma, ...);          // correct
    // ======================================================================

    arm_matrix_instance_f32 cmsis() noexcept {
        arm_matrix_instance_f32 m;
        arm_mat_init_f32(&m, static_cast<uint16_t>(R), static_cast<uint16_t>(C), data_);
        return m;
    }

    arm_matrix_instance_f32 cmsis() const noexcept {
        arm_matrix_instance_f32 m;
        // CMSIS declares pData as non-const float* but never writes through
        // it for input arguments. Safe cast here.
        arm_mat_init_f32(
            &m,
            static_cast<uint16_t>(R),
            static_cast<uint16_t>(C),
            const_cast<float*>(data_)
        );
        return m;
    }

    // ======================================================================
    // Compound assignment
    // ======================================================================

    Matrixf_ARM& operator+=(const Matrixf_ARM& rhs) noexcept {
        auto a = cmsis();
        auto b = rhs.cmsis();
        arm_mat_add_f32(&a, &b, &a);
        return *this;
    }

    Matrixf_ARM& operator-=(const Matrixf_ARM& rhs) noexcept {
        auto a = cmsis();
        auto b = rhs.cmsis();
        arm_mat_sub_f32(&a, &b, &a);
        return *this;
    }

    Matrixf_ARM& operator*=(float s) noexcept {
        auto a = cmsis();
        arm_mat_scale_f32(&a, s, &a);
        return *this;
    }

    Matrixf_ARM& operator/=(float s) noexcept {
        auto a = cmsis();
        arm_mat_scale_f32(&a, 1.f / s, &a);
        return *this;
    }

    // ======================================================================
    // Binary operators (return new matrices)
    // ======================================================================

    Matrixf_ARM operator+(const Matrixf_ARM& rhs) const noexcept {
        Matrixf_ARM res;
        auto a = cmsis();
        auto b = rhs.cmsis();
        auto r = res.cmsis();
        arm_mat_add_f32(&a, &b, &r);
        return res;
    }

    Matrixf_ARM operator-(const Matrixf_ARM& rhs) const noexcept {
        Matrixf_ARM res;
        auto a = cmsis();
        auto b = rhs.cmsis();
        auto r = res.cmsis();
        arm_mat_sub_f32(&a, &b, &r);
        return res;
    }

    Matrixf_ARM operator-() const noexcept {
        Matrixf_ARM res;
        auto a = cmsis();
        auto r = res.cmsis();
        arm_mat_scale_f32(&a, -1.f, &r);
        return res;
    }

    Matrixf_ARM operator*(float s) const noexcept {
        Matrixf_ARM res;
        auto a = cmsis();
        auto r = res.cmsis();
        arm_mat_scale_f32(&a, s, &r);
        return res;
    }

    friend Matrixf_ARM operator*(float s, const Matrixf_ARM& m) noexcept {
        return m * s;
    }

    Matrixf_ARM operator/(float s) const noexcept {
        Matrixf_ARM res;
        auto a = cmsis();
        auto r = res.cmsis();
        arm_mat_scale_f32(&a, 1.f / s, &r);
        return res;
    }

    /// Matrix multiplication: (R x C) * (C x C2) -> (R x C2)
    template<std::size_t C2>
    Matrixf_ARM<R, C2> operator*(const Matrixf_ARM<C, C2>& rhs) const noexcept {
        Matrixf_ARM<R, C2> res;
        auto a = cmsis();
        auto b = rhs.cmsis();
        auto r = res.cmsis();
        arm_mat_mult_f32(&a, &b, &r);
        return res;
    }

    // ======================================================================
    // Comparison
    // ======================================================================

    /// Bit-exact comparison. For float tolerance use isApprox().
    bool operator==(const Matrixf_ARM& rhs) const noexcept {
        for (std::size_t i = 0; i < R * C; ++i)
            if (data_[i] != rhs.data_[i])
                return false;
        return true;
    }
    bool operator!=(const Matrixf_ARM& rhs) const noexcept {
        return !(*this == rhs);
    }

    /// Element-wise approximate equality.
    bool isApprox(const Matrixf_ARM& rhs, float eps = 1e-6f) const noexcept {
        for (std::size_t i = 0; i < R * C; ++i) {
            float d = data_[i] - rhs.data_[i];
            if (d < 0.f)
                d = -d;
            if (d > eps)
                return false;
        }
        return true;
    }

    // ======================================================================
    // Sub-matrix / row / column
    // ======================================================================

    /// Compile-time size check only; caller guarantees start_row/start_col
    /// are in range.
    template<std::size_t BR, std::size_t BC>
    Matrixf_ARM<BR, BC> block(std::size_t start_row, std::size_t start_col) const noexcept {
        static_assert(BR > 0 && BC > 0, "empty block");
        static_assert(BR <= R && BC <= C, "block out of range");
        Matrixf_ARM<BR, BC> res;
        for (std::size_t r = 0; r < BR; ++r) {
            std::memcpy(
                res.data() + r * BC,
                data_ + (start_row + r) * C + start_col,
                BC * sizeof(float)
            );
        }
        return res;
    }

    Matrixf_ARM<1, C> row(std::size_t r) const noexcept {
        return block<1, C>(r, 0);
    }

    Matrixf_ARM<R, 1> col(std::size_t c) const noexcept {
        return block<R, 1>(0, c);
    }

    // ======================================================================
    // Transpose / trace / norm / inverse
    // ======================================================================

    Matrixf_ARM<C, R> trans() const noexcept {
        Matrixf_ARM<C, R> res;
        auto a = cmsis();
        auto r = res.cmsis();
        arm_mat_trans_f32(&a, &r);
        return res;
    }

    float trace() const noexcept {
        const std::size_t n = (R < C) ? R : C;
        float s             = 0.f;
        for (std::size_t i = 0; i < n; ++i)
            s += data_[i * C + i];
        return s;
    }

    /// Frobenius norm. Computed directly, no temporaries.
    float norm() const noexcept {
        float s = 0.f;
        for (std::size_t i = 0; i < R * C; ++i)
            s += data_[i] * data_[i];
        return sqrtf(s);
    }

    /// Inverse. Only available for square matrices.
    /// Returns zeros() if CMSIS reports a singular matrix.
    template<std::size_t RR = R, std::size_t CC = C>
    typename std::enable_if<RR == CC, Matrixf_ARM>::type inv() const noexcept {
        Matrixf_ARM res;
        auto a        = cmsis();
        auto r        = res.cmsis();
        arm_status st = arm_mat_inverse_f32(&a, &r);
        if (st != ARM_MATH_SUCCESS)
            return Matrixf_ARM::zeros();
        return res;
    }

private:
    float data_[R * C];
};

// Convenient aliases ------------------------------------------------------

template<std::size_t R, std::size_t C>
using Matf = Matrixf_ARM<R, C>;

template<std::size_t N>
using Vecf = Matrixf_ARM<N, 1>;

using Vec2f = Matrixf_ARM<2, 1>;
using Vec3f = Matrixf_ARM<3, 1>;
using Vec4f = Matrixf_ARM<4, 1>;

using Mat2f = Matrixf_ARM<2, 2>;
using Mat3f = Matrixf_ARM<3, 3>;
using Mat4f = Matrixf_ARM<4, 4>;

} // namespace algo
} // namespace vlib

#endif // MATRIX_ARM_HPP

/**
 * @file knn.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-10-23
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef VLIB_ALGO_KNN_HPP
#define VLIB_ALGO_KNN_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "vlib/algorithm/arm-matrix.hpp"

namespace vlib {

/**
 * @brief 1-NN 分类器：定长样本表，无堆分配，零初始化。
 *
 * Simple::k 是样本的类别标签（沿用 k-NN 的叫法，不是邻居个数），
 * Simple::vec 是 DIM 维特征向量。
 *
 * 距离用平方欧氏距离比较，省掉每个邻居一次 sqrtf；只有
 * nearest_distance() 才开方。
 *
 * RAM 占用 = SIZE * (DIM * 4 + sizeof(int16_t)) 字节。
 */
template<std::size_t DIM, std::size_t SIZE>
class KNN {
    static_assert(DIM > 0, "KNN: DIM must be positive");
    static_assert(SIZE > 0, "KNN: need at least one sample");

public:
    /// 一个训练样本：k 为标签，vec 为特征。
    struct Simple {
        int16_t          k;
        ARM::Vector<DIM> vec;
    };
    using Simples = std::array<Simple, SIZE>;

    constexpr KNN(const std::array<Simple, SIZE>& simples) noexcept:
        simples(simples) {}

    /// 最近邻样本的类别标签。
    int16_t find(const ARM::Vector<DIM>& vec) const noexcept {
        std::size_t nearest = 0;
        float       min_sq  = sq_distance(vec, simples[0].vec);

        for (std::size_t i = 1; i < SIZE; ++i) {
            const float sq = sq_distance(vec, simples[i].vec);
            if (sq < min_sq) {
                min_sq  = sq;
                nearest = i;
            }
        }

        return simples[nearest].k;
    }

    /// 到最近邻的欧氏距离（已开方），用于调试或在线调阈值。
    float nearest_distance(const ARM::Vector<DIM>& vec) const noexcept {
        float min_sq = sq_distance(vec, simples[0].vec);

        for (std::size_t i = 1; i < SIZE; ++i) {
            const float sq = sq_distance(vec, simples[i].vec);
            if (sq < min_sq)
                min_sq = sq;
        }

        return sqrtf(min_sq);
    }

private:
    Simples simples;

    static float sq_distance(const ARM::Vector<DIM>& a, const ARM::Vector<DIM>& b) noexcept {
        const float* pa = a.data();
        const float* pb = b.data();
        float        s  = 0.f;

        for (std::size_t i = 0; i < DIM; ++i) {
            const float d  = pa[i] - pb[i];
            s             += d * d;
        }

        return s;
    }
};

} // namespace vlib

#endif // VLIB_ALGO_KNN_HPP

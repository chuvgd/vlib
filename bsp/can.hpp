#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace vlib {

// ---------------- 通用 CAN 帧 ----------------
template<std::size_t LEN = 8>
struct CAN_Frame {
    std::uint32_t                 id = 0;   // 标准/扩展 ID
    std::array<std::uint8_t, LEN> bytes {}; // 数据（LEN 字节）
};

using CanFrame   = CAN_Frame<8>;  // 经典 CAN
using CanFdFrame = CAN_Frame<64>; // CAN FD

// ---------------- 描述符契约检测（C++11 可用的 void_t）----------------
namespace detail {
template<class...>
struct make_void {
    using type = void;
};
template<class... Ts>
using void_t = typename make_void<Ts...>::type;

template<class T, class = void>
struct can_descriptor: std::false_type {};

template<class T>
struct can_descriptor<
    T,
    void_t<typename T::data, typename T::backend, decltype(T::send(std::declval<const typename T::data&>(), std::declval<typename T::backend>()))>>:
    std::true_type {};
} // namespace detail

// ---------------- 通用句柄 ----------------
template<class Ex>
struct CAN_Handle {
    static_assert(detail::can_descriptor<Ex>::value, "Ex must provide data/backend and static send()");

    using data    = typename Ex::data;
    using backend = typename Ex::backend;

    explicit CAN_Handle(backend h = backend {}):
        backend_(h) {}

    CAN_Handle& operator<<(const data& frame) {
        Ex::send(frame, backend_);
        return *this;
    }

private:
    backend backend_ {};
};

} // namespace vlib

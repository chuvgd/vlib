#include <array>
#include <cstddef>
#include <cstdint>

template<typename ITEM, std::size_t SIZE>
class Callback_List {
    std::array<ITEM, SIZE> data {};

public:
    bool reg(uint32_t id, uint32_t mask, bool extended, void (*cb)(uint32_t, const uint8_t*, uint8_t, void*), void* ctx = nullptr) {
        for (auto& it: data) {
            if (it.callback == nullptr) {
                it = ITEM {id, mask, extended, cb, ctx};
                return true;
            }
        }
        return false;
    }

    void dispatch(uint32_t id, bool extended, const uint8_t* buf, uint8_t len) const {
        for (const auto& it: data) {
            if (it.callback && it.extended == extended && (id & it.mask) == (it.id & it.mask)) {
                it.callback(id, buf, len, it.ctx);
            }
        }
    }
};

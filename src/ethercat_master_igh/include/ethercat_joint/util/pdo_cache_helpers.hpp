/**
 * @file pdo_cache_helpers.hpp
 * @brief PDO 状态缓存的 seqlock 读写辅助（双后端共用）。
 */

#ifndef ETHERCAT_JOINT_UTIL_PDO_CACHE_HELPERS_HPP_
#define ETHERCAT_JOINT_UTIL_PDO_CACHE_HELPERS_HPP_

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "ethercat_joint/util/seqlock.hpp"

namespace ethercat_joint {

/** 在 seqlock 保护下读取单个缓存字段。 */
template <typename T>
inline T seqlockReadField(const SeqLock& lock, const T& field)
{
    for (;;) {
        const uint64_t s = lock.readBegin();
        const T value = field;
        if (!lock.readRetry(s)) {
            return value;
        }
    }
}

/** 在 seqlock 保护下按索引读取 vector 元素；越界返回 fallback。 */
template <typename T>
inline T seqlockReadIndex(const SeqLock& lock, const std::vector<T>& vec, size_t index, T fallback = T{})
{
    for (;;) {
        const uint64_t s = lock.readBegin();
        const T value = (index < vec.size()) ? vec[index] : fallback;
        if (!lock.readRetry(s)) {
            return value;
        }
    }
}

/** RAII：writeBegin / writeEnd。 */
class SeqLockWriter {
public:
    explicit SeqLockWriter(SeqLock& lock) noexcept : lock_(lock) { lock_.writeBegin(); }
    ~SeqLockWriter() { lock_.writeEnd(); }
    SeqLockWriter(const SeqLockWriter&) = delete;
    SeqLockWriter& operator=(const SeqLockWriter&) = delete;

private:
    SeqLock& lock_;
};

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_UTIL_PDO_CACHE_HELPERS_HPP_

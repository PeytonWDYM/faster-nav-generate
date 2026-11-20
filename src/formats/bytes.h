#pragma once

#include <bit>
#include <cstddef>
#include <cstring>
#include <span>
#include <stdexcept>
#include <type_traits>

namespace navfast::formats {

static_assert(std::endian::native == std::endian::little);

class Bytes {
public:
    using Span = std::span<const std::byte>;

    explicit Bytes(Span data, const char* error) : data_(data), error_(error) {}

    std::size_t size() const { return data_.size(); }
    Span span() const { return data_; }

    // Checked slices and unaligned reads of little-endian disk records.
    Bytes slice(std::size_t start, std::size_t length) const {
        if (start > size() || length > size() - start) fail();
        return Bytes(data_.subspan(start, length), error_);
    }

    Bytes records(std::size_t start, std::size_t count, std::size_t stride) const {
        if (start > size() || count > (size() - start) / stride) fail();
        return slice(start, count * stride);
    }

    template<class T>
    T read(std::size_t offset) const {
        static_assert(std::is_trivially_copyable_v<T>);
        T result;
        const auto record = slice(offset, sizeof(T));
        std::memcpy(&result, record.span().data(), sizeof(T));
        return result;
    }

    [[noreturn]] void fail() const { throw std::runtime_error(error_); }

private:
    Span data_;
    const char* error_;
};

} // namespace navfast::formats

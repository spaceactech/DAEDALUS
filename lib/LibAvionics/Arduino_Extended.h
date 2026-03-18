#ifndef ARDUINO_EXTENDED_H
#define ARDUINO_EXTENDED_H

// ---------------- C++20 feature detection ----------------
#if defined(_MSVC_LANG)
#  define XCORE_CPP_LANG _MSVC_LANG
#elif defined(__cplusplus)
#  define XCORE_CPP_LANG __cplusplus
#else
#  define XCORE_CPP_LANG 0L
#endif

#if (XCORE_CPP_LANG >= 202002L)
#  define XCORE_HAS_CPP20 1
#else
#  define XCORE_HAS_CPP20 0
#endif
// ---------------------------------------------------------

#include "lib_xcore"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <cmath>
#include <type_traits>
#include <utility>

#if XCORE_HAS_CPP20
#  include <concepts>
#endif

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                  \
  ((byte) & 0x80 ? '1' : '0'), ((byte) & 0x40 ? '1' : '0'),   \
    ((byte) & 0x20 ? '1' : '0'), ((byte) & 0x10 ? '1' : '0'), \
    ((byte) & 0x08 ? '1' : '0'), ((byte) & 0x04 ? '1' : '0'), \
    ((byte) & 0x02 ? '1' : '0'), ((byte) & 0x01 ? '1' : '0')

namespace traits {

#if XCORE_HAS_CPP20
  using std::derived_from;
  using std::same_as;

  template<typename S>
  concept has_ostream = requires(S s, const char *str) {
    { s << str } -> std::same_as<S &>;
  };

  template<typename S>
  concept arduino_stream_derived = std::derived_from<S, Stream>;

  template<typename S>
  concept has_flush = requires(S s) {
    { s.flush() } -> std::same_as<void>;
  };

  template<typename S, typename... Ts>
  concept has_print = requires(S s, Ts... vs) {
    { s.print(vs...) };
  };

  template<typename S>
  concept ostream_flush = has_ostream<S> && has_flush<S>;

#else
  // ------- C++17 fallback: trait detectors (no <concepts>) -------
  template<typename S, typename = void>
  struct has_ostream_impl : std::false_type {};
  template<typename S>
  struct has_ostream_impl<S,
                          std::void_t<decltype(std::declval<S &>() << std::declval<const char *>())>> : std::true_type {};
  template<typename S>
  constexpr bool has_ostream = has_ostream_impl<S>::value;

  template<typename S>
  constexpr bool arduino_stream_derived =
    std::is_base_of<Stream, std::remove_reference_t<S>>::value;

  template<typename S, typename = void>
  struct has_flush_impl : std::false_type {};
  template<typename S>
  struct has_flush_impl<S, std::void_t<decltype(std::declval<S &>().flush())>>
      : std::true_type {};
  template<typename S>
  constexpr bool has_flush = has_flush_impl<S>::value;

  // Detect s.print(arg) for a single argument (sufficient for operator<< below)
  template<typename S, typename T, typename = void>
  struct has_print_one_impl : std::false_type {};
  template<typename S, typename T>
  struct has_print_one_impl<S, T,
                            std::void_t<decltype(std::declval<S &>().print(std::declval<T>()))>> : std::true_type {};
  template<typename S, typename T>
  constexpr bool has_print_one = has_print_one_impl<S, T>::value;

  // Flush-capable ostream?
  template<typename S>
  constexpr bool ostream_flush = has_ostream<S> && has_flush<S>;
#endif
}  // namespace traits

namespace stream {
  inline const char *crlf = "\r\n";
  inline const char *lf   = "\n";

  namespace detail {
    class flush_type {};
  }  // namespace detail

  constexpr detail::flush_type flush = detail::flush_type();
}  // namespace stream

// ----- operator<< for Arduino-like streams -----
#if XCORE_HAS_CPP20
template<typename S, typename T>
  requires(traits::arduino_stream_derived<S> || traits::has_print<S, T>)
S &operator<<(S &stream, T &&v) {
  stream.print(xcore::forward<T>(v));
  return stream;
}
#else
template<typename S, typename T,
         typename std::enable_if<
           (traits::arduino_stream_derived<S> || traits::has_print_one<S, T>), int>::type = 0>
S &operator<<(S &stream, T &&v) {
  stream.print(xcore::forward<T>(v));
  return stream;
}
#endif

// Append to Arduino String
template<typename T>
String &operator<<(String &string, T &&v) {
  string += xcore::forward<T>(v);
  return string;
}

template<typename... IoTypes>
struct IoHook {
private:
  xcore::tuple<IoTypes &...> serials_;  // Store references to Serial objects

public:
  explicit IoHook(IoTypes &...serials) : serials_(serials...) {}

  template<typename Arg>
  IoHook &operator<<(Arg &&arg) {
    forEach([&](auto &serial) { serial << xcore::forward<Arg>(arg); });
    return *this;
  }

private:
  template<typename Func, size_t Index = 0>
  void forEach(Func &&func) {
    if constexpr (Index < sizeof...(IoTypes)) {
      func(xcore::get<Index>(serials_));
      forEach<Func, Index + 1>(xcore::forward<Func>(func));
    }
  }
};

struct FakeOStream {
  template<typename T>
  constexpr FakeOStream &operator<<(T &&) {
    return *this;
  }
};

namespace detail {

  // Whether an ostream also supports .flush()
  template<typename T>
  struct flush_ostream {
#if XCORE_HAS_CPP20
    static constexpr bool value = false;
    // specialize below via concept
    template<typename U>
      requires traits::ostream_flush<U>
    struct specialize;
#else
    static constexpr bool value = traits::ostream_flush<T>;
#endif
  };

#if XCORE_HAS_CPP20
  template<traits::ostream_flush T>
  struct flush_ostream<T> {
    static constexpr bool value = true;
  };
#endif

  // CSV stream
#if XCORE_HAS_CPP20
  template<traits::has_ostream OStream, size_t ReserveSize = 0,
           bool NewLine = false, bool AutoFlush = true>
  class csv_stream {
#else
  template<typename OStream, size_t ReserveSize = 0,
           bool NewLine = false, bool AutoFlush = true>
  class csv_stream {
    static_assert(traits::has_ostream<OStream>,
                  "csv_stream requires an ostream supporting operator<<(const char*)");
#endif
  private:
    OStream *m_stream = {};
    String   m_string = {};

  public:
    explicit csv_stream(OStream &stream) : m_stream(&stream) {
      m_string.reserve(ReserveSize);
    }

    csv_stream(const csv_stream &other)     = delete;
    csv_stream(csv_stream &&other) noexcept = delete;

    template<typename T>
    csv_stream &operator<<(T &&value) {
      m_string += xcore::forward<T>(value);
      m_string += ",";
      return *this;
    }

    ~csv_stream() {
      // End of a message

      // Remove trailing comma (only if something was added)
      if (m_string.length() && m_string[m_string.length() - 1] == ',')
        m_string.remove(m_string.length() - 1);

      if constexpr (NewLine) {
        m_string += stream::lf;
      }

      // Flush to stream
      *m_stream << m_string;

      if constexpr (AutoFlush) {
#if XCORE_HAS_CPP20
        if constexpr (flush_ostream<OStream>::value) {
          m_stream->flush();
        }
#else
        if (flush_ostream<OStream>::value) {
          m_stream->flush();
        }
#endif
      }
    }
  };
}  // namespace detail

/**
 * @tparam OStream Output Stream Type with "<<" stream operator
 * @tparam ReserveSize Internal string reserve size
 * @tparam NewLine Whether to insert LF at the end or not
 * @param stream Output stream OStream object
 * @return Csv stream object
 */
#if XCORE_HAS_CPP20
template<traits::has_ostream OStream, size_t ReserveSize = 0,
         bool NewLine = false, bool AutoFlush = true>
detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush>
csv_stream(OStream &stream) {
  return detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush>(stream);
}
#else
template<typename OStream, size_t ReserveSize = 0,
         bool NewLine = false, bool AutoFlush = true,
         typename std::enable_if<traits::has_ostream<OStream>, int>::type = 0>
detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush>
csv_stream(OStream &stream) {
  return detail::csv_stream<OStream, ReserveSize, NewLine, AutoFlush>(stream);
}
#endif

#if XCORE_HAS_CPP20
template<traits::has_ostream OStream, size_t ReserveSize = 0,
         bool AutoFlush = true>
detail::csv_stream<OStream, ReserveSize, true, AutoFlush>
csv_stream_lf(OStream &stream) {
  return csv_stream<OStream, ReserveSize, true, AutoFlush>(stream);
}
#else
template<typename OStream, size_t ReserveSize = 0,
         bool AutoFlush                                                   = true,
         typename std::enable_if<traits::has_ostream<OStream>, int>::type = 0>
detail::csv_stream<OStream, ReserveSize, true, AutoFlush>
csv_stream_lf(OStream &stream) {
  return csv_stream<OStream, ReserveSize, true, AutoFlush>(stream);
}
#endif

// I2C Scanner
void i2c_detect(Stream &output_stream, TwoWire &i2c_wire, uint8_t addr_from,
                uint8_t addr_to);

constexpr uint8_t *byte_cast(void *ptr) { return static_cast<uint8_t *>(ptr); }

inline __attribute__((__always_inline__)) void do_nothing() {}

#endif  // ARDUINO_EXTENDED_H

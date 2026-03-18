#ifndef FILE_UTILITY_H
#define FILE_UTILITY_H

#include <Arduino.h>
#include <Arduino_Extended.h>
#include <STM32SD.h>

enum class FsMode : uint8_t {
  READ = 0,
  WRITE,
  APPEND
};

class FsUtil {
protected:
  uint32_t m_sector_count = {};
  uint8_t  m_buf[512]     = {};

  File   m_file     = {};
  String m_filename = {};

public:
  FsUtil() {
    m_filename.reserve(64);
  }

  constexpr File &file() {
    return m_file;
  }

  template<FsMode Mode>
  void open_one() {
    m_file = open<Mode>(m_filename);
  }

  void close_one() {
    m_file.close();
  }

  void flush_one() {
    m_file.flush();
  }

  template<FsMode Mode>
  [[nodiscard]] File open(const String &filename) {
    return open<Mode>(filename.c_str());
  }

  template<FsMode Mode>
  [[nodiscard]] File open(const char *filename) {
    File file;

    if constexpr (Mode == FsMode::READ) {
      file = SD.open(filename, FILE_READ);
    } else if constexpr (Mode == FsMode::WRITE) {
      file = SD.open(filename, FILE_WRITE);
    } else if constexpr (Mode == FsMode::APPEND) {
      file = SD.open(filename, FILE_WRITE);
      file.seek(file.size());
    }

    return file;
  }

  void find_file_name(const char *prefix, const char *extension = "csv") {
    uint32_t file_idx = 1;
    do {
      m_filename = "";
      m_filename << prefix << file_idx++ << "." << extension;
    } while (SD.exists(m_filename.c_str()));
  }
};

#endif  //FILE_UTILITY_H

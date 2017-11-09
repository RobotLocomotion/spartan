#pragma once

#include <ctime>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

static double getUnixTime(void) {
  struct timespec tv;

  if (clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

  return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

static std::string getTimestampString(void) {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream out;
  out << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S") << std::endl;
  return out.str();
}

// Update the input string.
// https://stackoverflow.com/questions/1902681/expand-file-names-that-have-environment-variables-in-their-path
static void autoExpandEnvironmentVariables(std::string& text) {
  static std::regex env("\\$\\{([^}]+)\\}");
  std::smatch match;
  while (std::regex_search(text, match, env)) {
    const char* s = getenv(match[1].str().c_str());
    const std::string var(s == NULL ? "" : s);
    text.replace(match[0].first, match[0].second, var);
  }
}

// Leave input alone and return new string.
// https://stackoverflow.com/questions/1902681/expand-file-names-that-have-environment-variables-in-their-path
static std::string expandEnvironmentVariables(const std::string& input) {
  std::string text = input;
  autoExpandEnvironmentVariables(text);
  return text;
}

// Compresses a std::vector into a unsigned char buffer.
template <typename Type>
static std::vector<unsigned char> zipStdVector(
    const typename std::vector<Type>& vec) {
  // TODO: actually use ZLIB.
  return std::vector<unsigned char>(
      (unsigned char*)const_cast<std::vector<Type>&>(vec).data(),
      (unsigned char*)(const_cast<std::vector<Type>&>(vec).data() +
                       vec.size()));
}

// Decompresses an unsigned char buffer (in std::vector<unsigned char> form)
// into a std::vector.
template <typename Type>
static std::vector<Type> unzipStdVector(
    const typename std::vector<unsigned char>& vec) {
  // TODO: actually use ZLIB.
  return std::vector<Type>(
      (Type*)const_cast<std::vector<unsigned char>&>(vec).data(),
      (Type*)(const_cast<std::vector<unsigned char>&>(vec).data() +
              vec.size()));
}

// Decompresses an unsigned char buffer into a std::vector.
template <typename Type>
static std::vector<Type> unzipStdVector(const unsigned char* vec, int size) {
  // TODO: actually use ZLIB.
  return std::vector<Type>((const Type*)vec, (const Type*)(vec + size));
}

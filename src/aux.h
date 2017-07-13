#ifndef AUX_H
#define AUX_H

#include <iostream>

#ifndef NDEBUG
#define AUX_DEBUG(...) aux::log_start(__FILE__, __LINE__, __VA_ARGS__)
#else
#define AUX_DEBUG(...)
#endif


namespace aux {

template <typename Arg> void log_continue(const Arg &arg) {
  std::cout << arg << std::endl;
}

template <typename Arg, typename... Args>
void log_continue(const Arg &arg, const Args &... args) {
  std::cout << arg << " ";
  log_continue(args...);
}

template <typename... Args>
void log_start(const char *file, size_t line, const Args &... args) {
  std::cout << "[" << file << ":" << line << "] ";
  log_continue(args...);
}

}

#endif

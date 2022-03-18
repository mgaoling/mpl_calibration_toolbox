#ifndef CALIB_TOOLBOX_UTILITY_HPP_
#define CALIB_TOOLBOX_UTILITY_HPP_

#include <string>

namespace colorful_char {

std::string info(std::string input_str) {
  return "\033[1;32m>> " + input_str + " \033[0m";
}

std::string warning(std::string input_str) {
  return "\033[1;35m>> " + input_str + " \033[0m";
}

std::string error(std::string input_str) {
  return "\033[1;31m>> " + input_str + " \033[0m";
}

}  // namespace colorful_char

#endif  // CALIB_TOOLBOX_UTILITY_HPP_
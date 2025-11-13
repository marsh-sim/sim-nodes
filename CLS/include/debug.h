
#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>
#include <ctime>
#include <iomanip>

#ifdef DEBUG

#define DEBUG_COUT(msg) \
  do {
      std::cout << "[DEBUG] " << msg << std::endl; \
  } while(0)

#define DEBUG_CERR(msg) \
  do {
      std::cerr << "[DEBUG] " << msg << std::endl; \
  } while(0)

#define DEBUG_VAR(var) \
  do { \
      std::cout << "[DEBUG] " << #var << " = " << std::endl; \
  } while(0)
 
#define DEBUG_HEX(var) \
  do { \
      std::cout << "[DEBUG] " << #var << " = 0x" \
      << std::hex << var << std::dec << std::endl; \
  } while(0)
 
#else
  #define DEBUG_COUT(msg) do {} while(0);
  #define DEBUG_CERR(msg) do {} while(0);
  #define DEBUG_VAR(msg) do {} while(0);
  #define DEBUG_HEX(msg) do {} while(0);
#endif // def DEBUG
#endif // ndef DEBUG_H

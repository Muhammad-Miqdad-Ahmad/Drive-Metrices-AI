#pragma once
#include <stdio.h>

#define DEBUG_PRINTF(...)                                                      \
  do {                                                                         \
    printf("[i] ");                                                            \
    printf(__VA_ARGS__);                                                       \
  } while (0)

#define SUCCESS_PRINTF(...)                                                    \
  do {                                                                         \
    printf("[*] ");                                                            \
    printf(__VA_ARGS__);                                                       \
  } while (0)

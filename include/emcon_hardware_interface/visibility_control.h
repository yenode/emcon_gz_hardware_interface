// Copyright 2026 Silent Sentry Project
// SPDX-License-Identifier: Apache-2.0

#ifndef EMCON_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define EMCON_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then adapted) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EMCON_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
    #define EMCON_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
  #else
    #define EMCON_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define EMCON_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef EMCON_HARDWARE_INTERFACE_BUILDING_DLL
    #define EMCON_HARDWARE_INTERFACE_PUBLIC EMCON_HARDWARE_INTERFACE_EXPORT
  #else
    #define EMCON_HARDWARE_INTERFACE_PUBLIC EMCON_HARDWARE_INTERFACE_IMPORT
  #endif
  #define EMCON_HARDWARE_INTERFACE_PUBLIC_TYPE EMCON_HARDWARE_INTERFACE_PUBLIC
  #define EMCON_HARDWARE_INTERFACE_LOCAL
#else
  #define EMCON_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
  #define EMCON_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define EMCON_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
    #define EMCON_HARDWARE_INTERFACE_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define EMCON_HARDWARE_INTERFACE_PUBLIC
    #define EMCON_HARDWARE_INTERFACE_LOCAL
  #endif
  #define EMCON_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // EMCON_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

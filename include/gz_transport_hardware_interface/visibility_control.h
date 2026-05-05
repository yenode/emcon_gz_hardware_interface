// Copyright 2026 Aditya Pachauri
// SPDX-License-Identifier: Apache-2.0

#ifndef GZ_TRANSPORT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define GZ_TRANSPORT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then adapted) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
  #else
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef GZ_TRANSPORT_HARDWARE_INTERFACE_BUILDING_DLL
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_PUBLIC GZ_TRANSPORT_HARDWARE_INTERFACE_EXPORT
  #else
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_PUBLIC GZ_TRANSPORT_HARDWARE_INTERFACE_IMPORT
  #endif
  #define GZ_TRANSPORT_HARDWARE_INTERFACE_PUBLIC_TYPE GZ_TRANSPORT_HARDWARE_INTERFACE_PUBLIC
  #define GZ_TRANSPORT_HARDWARE_INTERFACE_LOCAL
#else
  #define GZ_TRANSPORT_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
  #define GZ_TRANSPORT_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_PUBLIC
    #define GZ_TRANSPORT_HARDWARE_INTERFACE_LOCAL
  #endif
  #define GZ_TRANSPORT_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // GZ_TRANSPORT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

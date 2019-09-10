// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef HUNGARIAN_ASSIGNER__VISIBILITY_CONTROL_HPP_
#define HUNGARIAN_ASSIGNER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(HUNGARIAN_ASSIGNER_BUILDING_DLL) || defined(HUNGARIAN_ASSIGNER_EXPORTS)
    #define HUNGARIAN_ASSIGNER_PUBLIC __declspec(dllexport)
    #define HUNGARIAN_ASSIGNER_LOCAL
  #else  // defined(HUNGARIAN_ASSIGNER_BUILDING_DLL) || defined(HUNGARIAN_ASSIGNER_EXPORTS)
    #define HUNGARIAN_ASSIGNER_PUBLIC __declspec(dllimport)
    #define HUNGARIAN_ASSIGNER_LOCAL
  #endif  // defined(HUNGARIAN_ASSIGNER_BUILDING_DLL) || defined(HUNGARIAN_ASSIGNER_EXPORTS)
#elif defined(__linux__)
  #define HUNGARIAN_ASSIGNER_PUBLIC __attribute__((visibility("default")))
  #define HUNGARIAN_ASSIGNER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define HUNGARIAN_ASSIGNER_PUBLIC __attribute__((visibility("default")))
  #define HUNGARIAN_ASSIGNER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // HUNGARIAN_ASSIGNER__VISIBILITY_CONTROL_HPP_

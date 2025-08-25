# - Try to find SCIP
# See http://scip.zib.de/ for more information on SCIP
#
# Once done, this will define
#
#  SCIP_INCLUDE_DIRS   - where to find scip/scip.h, etc.
#  SCIP_LIBRARIES      - List of libraries when using scip.
#  SCIP_FOUND          - True if scip found.
#
#  SCIP_VERSION        - The version of scip found (x.y.z)
#  SCIP_VERSION_MAJOR  - The major version of scip
#  SCIP_VERSION_MINOR  - The minor version of scip
#  SCIP_VERSION_PATCH  - The patch version of scip
#
# Variables used by this module, they can change the default behaviour and
# need to be set before calling find_package:
#
# SCIP_ROOT            - The preferred installation prefix for searching for
#                        Scip.  Set this if the module has problems finding
#                        the proper SCIP installation. SCIP_ROOT is also
#                        available as an environment variable.
#
# Author:
# Wolfgang A. Welz <welz@math.tu-berlin.de>
#
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

# If SCIP_ROOT is not set, look for the environment variable
if(NOT SCIP_ROOT AND NOT "$ENV{SCIP_ROOT}" STREQUAL "")
  set(SCIP_ROOT $ENV{SCIP_ROOT})
endif()

find_path(SCIP_INCLUDE_DIR
  NAMES scip/scip.h
  HINTS ${SCIP_ROOT}
  PATH_SUFFIXES src include scip/src)

find_library(SCIP_LIBRARY
  NAMES scip
  HINTS ${SCIP_ROOT}
  PATH_SUFFIXES lib lib/shared lib/static)

#find_package_handle_standard_args(SCIP
#  REQUIRED_VARS SCIP_INCLUDE_DIR SCIP_LIBRARY
#  VERSION_VAR SCIP_VERSION)

set(SCIP_LIBRARIES ${SCIP_LIBRARY})

set(SCIP_INCLUDE_DIRS ${SCIP_INCLUDE_DIR})

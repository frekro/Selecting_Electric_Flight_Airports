find_path(SOPLEX_INCLUDE_DIRS
    NAMES gurobi_c.h
    HINTS ${SOPLEX_DIR} $ENV{SOPLEX_DIR}
    PATH_SUFFIXES include)

find_library(SOPLEX_LIBRARY
    NAMES soplex
    HINTS ${SOPLEX_DIR} $ENV{SOPLEX_DIR}
    PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SOPLEX DEFAULT_MSG SOPLEX_LIBRARY)

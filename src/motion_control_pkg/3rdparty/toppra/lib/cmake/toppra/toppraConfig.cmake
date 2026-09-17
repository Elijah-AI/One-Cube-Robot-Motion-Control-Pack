set(TOPPRA_WITH_PINOCCHIO OFF)
set(TOPPRA_WITH_qpOASES   OFF)
set(TOPPRA_WITH_GLPK      OFF)

find_package(Eigen3 REQUIRED)

if(TOPPRA_WITH_PINOCCHIO)
  find_package(pinocchio REQUIRED)
endif()
# qpOASES and GLPK do not need to be found again, because
# they are not used as targets.

include("${CMAKE_CURRENT_LIST_DIR}/toppraTargets.cmake")

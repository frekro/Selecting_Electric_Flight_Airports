set(COMMON_SRC
  main.cpp
  global.cpp
  seaport_program/seaport_class.cpp
  seaport_program/graph/airport_graph.cpp
  seaport_program/scip_instance/model/model_init.cpp
  seaport_program/scip_instance/model/variables.cpp
  seaport_program/scip_instance/model/pathvar_prob.cpp
  seaport_program/scip_instance/model/constraints.cpp
  seaport_program/scip_instance/pricer/path_pricer.cpp
  seaport_program/scip_instance/pricer/path_pricer_rc.cpp
  seaport_program/scip_instance/constraint_handlers/costoptimalpath_cons.cpp
  )

add_library(common ${COMMON_SRC})

set(libs
  ${libs}
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${SCIP_LIBRARIES}
  ${GMP_LIBRARY}
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY})

if(SOPLEX_LIBRARY)
  set(libs
    ${libs}
    ${SOPLEX_LIBRARY})
endif()

if(GRAPHVIZ_FOUND)
  set(libs
    ${libs}
    ${GRAPHVIZ_GVC_LIBRARY}
    ${GRAPHVIZ_CDT_LIBRARY}
    ${GRAPHVIZ_CGRAPH_LIBRARY}
    ${GRAPHVIZ_PATHPLAN_LIBRARY})
endif()

target_link_libraries(common ${libs})

add_executable(facilitylocation main.cpp)

target_link_libraries(facilitylocation common)


set(headers
  PyUtil.h
  PySignal.h
  exportdecl.h
  )

add_cnoid_library(CnoidPyUtil SHARED PyUtil.cpp ${headers})
if (APPLE)
  target_link_libraries(CnoidPyUtil ${PYTHON_LIBRARIES} ${Boost_PYTHON27_LIBRARY})
else()
  target_link_libraries(CnoidPyUtil ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY})
endif()
apply_common_setting_for_library(CnoidPyUtil "${headers}")

add_cnoid_python_module(PyUtil
  PyUtilModule.cpp
  PySignal.cpp
  PyValueTree.cpp
  PyEigenTypes.cpp
  PyEigenArchive.cpp
  PySeqTypes.cpp
  PySceneGraph.cpp
  PyGeometryTypes.cpp
  PyTask.cpp
  )

target_link_libraries(PyUtil CnoidUtil CnoidPyUtil)

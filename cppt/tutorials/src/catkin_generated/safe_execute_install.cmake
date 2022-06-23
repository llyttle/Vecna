execute_process(COMMAND "/other/dev/vros/src/original/tutorials/src/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/other/dev/vros/src/original/tutorials/src/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

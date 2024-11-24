file(GLOB HOUDINI_ROOTS "C:/Program Files/Side Effects Software/Houdini ${HOUDINI_VERSION}.[0-9.]*")
foreach(HOUDINI_ROOT ${HOUDINI_ROOTS})
	message("Find Houdini at ${HOUDINI_ROOT}")
	list(APPEND CMAKE_PREFIX_PATH "${HOUDINI_ROOT}/toolkit/cmake")
endforeach()
find_package(Houdini REQUIRED)

add_library(${PROJECT_NAME}HoudiniNodes SHARED
	SOP_NemoPlay.cpp
	SOP_Nemo.cpp
	initialize.cpp)
target_link_libraries(${PROJECT_NAME}HoudiniNodes PRIVATE Houdini)
target_link_libraries(${PROJECT_NAME}HoudiniNodes PRIVATE
	${PROJECT_NAME}Evaluate
	CUDA::cudart)
set_target_properties(${PROJECT_NAME}HoudiniNodes PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

houdini_configure_target(${PROJECT_NAME}HoudiniNodes INSTDIR ${CMAKE_CURRENT_BINARY_DIR})

if(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME}HoudiniNodes
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.dll" "${CMAKE_CURRENT_SOURCE_DIR}/client/dso/windows-${HOUDINI_VERSION}/"
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.dll" "${CMAKE_CURRENT_SOURCE_DIR}/client/dso/")
endif()
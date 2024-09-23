add_library(${PROJECT_NAME}Evaluate Evaluate.cpp Runtime.cpp Context.cpp AnimSequence.cpp) 
target_include_directories(${PROJECT_NAME}Evaluate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}Evaluate PUBLIC
	CONAN_PKG::boost
	CONAN_PKG::glm
	CONAN_PKG::nlohmann_json
	CONAN_PKG::fmt)

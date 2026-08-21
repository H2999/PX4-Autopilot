px4_add_library(RateLADRC
	LADRC.hpp
	RateLADRC.cpp
	RateLADRC.hpp
)
target_compile_options(RateLADRC PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(RateLADRC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(RateLADRC PRIVATE mathlib)


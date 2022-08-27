cmake_minimum_required(VERSION 3.20)

# ---- Options ----

option(COPY_BUILD "Copy the build output to the Fallout 4 directory." OFF)

# ---- Cache build vars ----

macro(set_from_environment VARIABLE)
	if (NOT DEFINED ${VARIABLE} AND DEFINED ENV{${VARIABLE}})
		set(${VARIABLE} $ENV{${VARIABLE}})
	endif ()
endmacro()

set_from_environment(Fallout4Path)
set_from_environment(VCPKG_ROOT)

if (DEFINED VCPKG_ROOT)
	set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
	set(VCPKG_TARGET_TRIPLET "x64-windows-static-md" CACHE STRING "")
else ()
	message(
		WARNING
		"Variable VCPKG_ROOT is not set. Continuing without vcpkg."
	)
endif ()

set(Boost_USE_STATIC_RUNTIME OFF CACHE BOOL "")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL" CACHE STRING "")

# ---- Project ----

project(
	UneducatedShooter
	VERSION 1.0.0
	LANGUAGES CXX
)

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/cmake/Version.h.in
	${CMAKE_CURRENT_BINARY_DIR}/include/Version.h
	@ONLY
)

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.rc.in
	${CMAKE_CURRENT_BINARY_DIR}/version.rc
	@ONLY
)

# ---- Include guards ----

if (PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
	message(
		FATAL_ERROR
		"In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
	)
endif ()

# ---- Globals ----

if (MSVC)
	if (NOT ${CMAKE_GENERATOR} STREQUAL "Ninja")
		add_compile_options(
			/MP	# Build with Multiple Processes
		)
	endif ()
endif ()

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)

set(Boost_USE_STATIC_LIBS ON)

# ---- Dependencies ----

if (NOT TARGET CommonLibF4)
	add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../CommonLibF4" CommonLibF4)
endif ()

find_package(spdlog REQUIRED CONFIG)

#find_library(HAVOK_BASE NAME hkBase HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
#find_library(HAVOK_COMPAT NAME hkCompat HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
#find_library(HAVOK_PHYSICS NAME hknpPhysics HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
#find_library(HAVOK_SERIALIZE NAME hkSerialize HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# ---- Add source files ----

include(cmake/sourcelist.cmake)

source_group(
	TREE ${CMAKE_CURRENT_SOURCE_DIR}
	FILES ${SOURCES}
)

source_group(
	TREE ${CMAKE_CURRENT_BINARY_DIR}
	FILES ${CMAKE_CURRENT_BINARY_DIR}/include/Version.h
)

set(
	HAVOK_PATH
	"D:/Games/Tullius Mod Pack  V 3.4.6.4/Workspace/F4SE Stuff/Havok Reference/From Nebula/Source/"
)

#file(GLOB_RECURSE HAVOK
#         ${HAVOK_PATH}/*
#)

#foreach(source IN LISTS HAVOK)
#    get_filename_component(source_path "${source}" PATH)
#    string(REPLACE ${HAVOK_PATH} "" source_path_rel "${source_path}")
#    string(REPLACE "/" "\\" source_path_msvc "${source_path_rel}")
#    source_group("${source_path_msvc}" FILES "${source}")
#	message("${source_path_msvc}")
#	message("${source_path_rel}")
#endforeach()

#file(GLOB_RECURSE HAVOK
#         ${CMAKE_CURRENT_SOURCE_DIR}/src/Havok/*
#)

#foreach(source IN LISTS HAVOK)
#    get_filename_component(source_path "${source}" PATH)
#    string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR}/src/Havok/ "" source_path_rel "${source_path}")
#    string(REPLACE "/" "\\" source_path_msvc "${source_path_rel}")
#    source_group("${source_path_msvc}" FILES "${source}")
#endforeach()

# ---- Create DLL ----

add_library(
	${PROJECT_NAME}
	SHARED
#	${HAVOK}
	${SOURCES}
	${CMAKE_CURRENT_BINARY_DIR}/version.rc
	.clang-format
)

target_compile_definitions(
	${PROJECT_NAME}
	PRIVATE
		_UNICODE
)

target_compile_features(
	${PROJECT_NAME}
	PRIVATE
		cxx_std_20
)

#include_directories(${HAVOK_PATH})

include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}/../Utilities
)

target_include_directories(
	${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_BINARY_DIR}/include
		${CMAKE_CURRENT_SOURCE_DIR}/src
#		${CMAKE_CURRENT_SOURCE_DIR}/src/Havok
)

target_link_libraries(
	${PROJECT_NAME}
	PRIVATE
		CommonLibF4::CommonLibF4
		spdlog::spdlog
#		${HAVOK_BASE}
#		${HAVOK_COMPAT}
#		${HAVOK_PHYSICS}
#		${HAVOK_SERIALIZE}
)

target_precompile_headers(
	${PROJECT_NAME}
	PRIVATE
		src/PCH.h
)

if (MSVC)
	target_compile_options(
		${PROJECT_NAME}
		PRIVATE
			/sdl	# Enable Additional Security Checks
			/utf-8	# Set Source and Executable character sets to UTF-8
			/Zi	# Debug Information Format

			/permissive-	# Standards conformance

			/Zc:alignedNew	# C++17 over-aligned allocation
			/Zc:auto	# Deduce Variable Type
			/Zc:char8_t
			/Zc:__cplusplus	# Enable updated __cplusplus macro
			/Zc:externC
			/Zc:externConstexpr	# Enable extern constexpr variables
			/Zc:forScope	# Force Conformance in for Loop Scope
			/Zc:hiddenFriend
			/Zc:implicitNoexcept	# Implicit Exception Specifiers
			/Zc:lambda
			/Zc:noexceptTypes	# C++17 noexcept rules
			/Zc:preprocessor	# Enable preprocessor conformance mode
			/Zc:referenceBinding	# Enforce reference binding rules
			/Zc:rvalueCast	# Enforce type conversion rules
			/Zc:sizedDealloc	# Enable Global Sized Deallocation Functions
			/Zc:strictStrings	# Disable string literal type conversion
			/Zc:ternary	# Enforce conditional operator rules
			/Zc:threadSafeInit	# Thread-safe Local Static Initialization
			/Zc:tlsGuards
			/Zc:trigraphs	# Trigraphs Substitution
			/Zc:wchar_t	# wchar_t Is Native Type

			/external:anglebrackets
			/external:W0

			/W4	# Warning level
			/WX	# Warning level (warnings are errors)

			"$<$<CONFIG:DEBUG>:>"
			"$<$<CONFIG:RELEASE>:/Zc:inline;/JMC-;/Ob3>"
	)

	target_link_options(
		${PROJECT_NAME}
		PRIVATE
			/WX	# Treat Linker Warnings as Errors

			"$<$<CONFIG:DEBUG>:/INCREMENTAL;/OPT:NOREF;/OPT:NOICF>"
			"$<$<CONFIG:RELEASE>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>"
	)
endif ()

# ---- File copying ----

if (COPY_BUILD)
	if (DEFINED Fallout4Path)
		add_custom_command(
			TARGET ${PROJECT_NAME}
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${Fallout4Path}/Data/F4SE/Plugins/
			COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_PDB_FILE:${PROJECT_NAME}> ${Fallout4Path}/Data/F4SE/Plugins/
		)
	else ()
		message(
			WARNING
			"Variable Fallout4Path is not defined. Skipping post-build copy command."
		)
	endif ()
endif ()
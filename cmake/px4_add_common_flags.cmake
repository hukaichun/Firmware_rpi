############################################################################
#
# Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include(px4_base)

#=============================================================================
#
#	px4_add_common_flags
#
#	Set the default build flags.
#
#	Usage:
#		px4_add_common_flags()
#
function(px4_add_common_flags)

	add_compile_options(
		-g # always build debug symbols

		# optimization options
		-fdata-sections
		-ffunction-sections
		-fomit-frame-pointer
		-funsafe-math-optimizations

		-fno-strict-aliasing
		-fno-math-errno

		# visibility
		-fvisibility=hidden
		-include visibility.h

		# Warnings
		-Wall
		-Wextra
		# -Werror

		-Warray-bounds
		-Wcast-align
		-Wdisabled-optimization
		-Wdouble-promotion
		-Wfatal-errors
		-Wfloat-equal
		-Wformat-security
		-Winit-self
		-Wlogical-op
		-Wpointer-arith
		-Wshadow
		-Wuninitialized
		-Wunknown-pragmas
		-Wunused-variable

		# disabled warnings
		-Wno-implicit-fallthrough # set appropriate level and update
		-Wno-missing-field-initializers
		-Wno-missing-include-dirs # TODO: fix and enable
		-Wno-unused-parameter
		)

	# compiler specific flags
	if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")

		# force color for clang (needed for clang + ccache)
		add_compile_options(-fcolor-diagnostics)

		# QuRT 6.4.X compiler identifies as Clang but does not support this option
		if (NOT "${PX4_PLATFORM}" STREQUAL "qurt")
			add_compile_options(
				-Qunused-arguments

				-Wno-unknown-warning-option
				-Wno-unused-const-variable
				-Wno-varargs
			)
		endif()

	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

		if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
			# force color for gcc > 4.9
			add_compile_options(-fdiagnostics-color=always)
		endif()

		add_compile_options(
			-fno-builtin-printf
			-fno-strength-reduce

			-Wformat=1
			-Wunused-but-set-variable

			-Wno-format-truncation # TODO: fix
		)

		# -fcheck-new is a no-op for Clang in general
		# and has no effect, but can generate a compile
		# error for some OS
		add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fcheck-new>)

	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	  message(FATAL_ERROR "Intel compiler not yet supported")
	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	  message(FATAL_ERROR "MS compiler not yet supported")
	endif()

	# C only flags
	set(c_flags)
	list(APPEND c_flags
		-fno-common

		-Wbad-function-cast
		-Wnested-externs
		-Wstrict-prototypes
	)
	foreach(flag ${c_flags})
		add_compile_options($<$<COMPILE_LANGUAGE:C>:${flag}>)
	endforeach()


	# CXX only flags
	set(cxx_flags)
	list(APPEND cxx_flags
		-fno-exceptions
		-fno-rtti
		-fno-threadsafe-statics

		-Wreorder

		# disabled warnings
		-Wno-overloaded-virtual # TODO: fix and remove
	)
	foreach(flag ${cxx_flags})
		add_compile_options($<$<COMPILE_LANGUAGE:CXX>:${flag}>)
	endforeach()


	include_directories(
		${PX4_BINARY_DIR}
		${PX4_BINARY_DIR}/src
		${PX4_BINARY_DIR}/src/lib
		${PX4_BINARY_DIR}/src/modules

		${PX4_SOURCE_DIR}/src
		${PX4_SOURCE_DIR}/src/include
		${PX4_SOURCE_DIR}/src/lib
		${PX4_SOURCE_DIR}/src/lib/DriverFramework/framework/include
		${PX4_SOURCE_DIR}/src/lib/matrix
		${PX4_SOURCE_DIR}/src/modules
		${PX4_SOURCE_DIR}/src/platforms
		${PX4_SOURCE_DIR}/src/platforms/common
		)

	add_definitions(
		-DCONFIG_ARCH_BOARD_${PX4_BOARD_NAME}
		-D__CUSTOM_FILE_IO__
		-D__STDC_FORMAT_MACROS
		)

endfunction()

#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

export IDF_PATH=./esp-idf/

PROJECT_NAME := remote-cube

include $(IDF_PATH)/make/project.mk

format:
	find components -iname '*.h' -o -iname '*.c' | xargs clang-format -i
	find main -iname '*.h' -o -iname '*.c' | xargs clang-format -i

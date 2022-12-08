#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := 58_v5_app

EXTRA_COMPONENT_DIRS = $(IDF_PATH)/examples/common_components/protocol_examples_common
EXTRA_COMPONENT_DIRS = $(IDF_PATH)/components

EXTRA_COMPONENT_DIRS = $(PROJECT_PATH)/components
EXTRA_COMPONENT_DIRS = $(PROJECT_PATH)/


COMPONENT_DIRS = $(PROJECT_PATH)/components
COMPONENT_DIRS = $(PROJECT_PATH)/main


include $(IDF_PATH)/make/project.mk

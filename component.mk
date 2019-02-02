#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
#

COMPONENT_SRCDIRS := . babel_gen u8g2/csrc nkolban sensors bt
COMPONENT_ADD_INCLUDEDIRS := . babel_gen u8g2/csrc nkolban sensors bt
CXXFLAGS += -std=c++14

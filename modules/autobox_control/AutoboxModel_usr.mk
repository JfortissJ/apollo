# CHANGE INCLUDES HERE!!! 
# WE DO NOT AUTOMATICALLY GENERATE THIS

# =============================================================================
#  Make include file AutoboxModel_usr.mk:
#
#   RTI1401 7.9 (02-Nov-2017)
#   Thu Aug 16 11:37:18 2018
#
#   Copyright 2017, dSPACE GmbH. All rights reserved. Brand names
#   or product names are trademarks or registered trademarks of their
#   respective companies or organizations.
# =============================================================================

# =============================================================================
# ===== Define file version macro. Never change this value. ===================
# =============================================================================
USER_MAKEFILE_VERSION = 2
# =============================================================================

# -----------------------------------------------------------------------------
# Macros for user-specific settings.
#
# All macros below may list several items (files or directories). You can
# separate multiple items with blanks or list them on several lines using
# the \ (backslash) as line continuation character.
#
# The directory names may include absolute or partial path
# descriptions, e.g., ".\project1\sources"
#
# If path names contain white space characters they need to be set
# in double quotes (").
# White space characters and double quotes are not supported
# with file names like "my source.c".
#
# Note that white space characters and double quotes with path
# names are available since RTI/RTI-MP of dSPACE Release 6.0.
# Such paths do not work with previous versions of RTI/RTI-MP.
#
# Examples:
#
# USER_SRCS = file1.c file2.c file3.c
#
# USER_SRCS = \
#   file1.c \
#   file2.c \
#   file3.c
#
# SFCN_DIR = \
#   "\project one\sfcns" \
#   "\project two\sfcns"
#
# -----------------------------------------------------------------------------

# Directories where S-Function C source files are stored.
SFCN_DIR =

# Additional C source files to be compiled (file name extension .c).
USER_SRCS = \
header.pb-c.c \
error_code.pb-c.c \
autobox_chassis.pb-c.c \
autobox_control.pb-c.c \
autobox_localization.pb-c.c \
bridge_receiver_autobox_chassis.c \
bridge_receiver_autobox_control.c \
bridge_receiver_autobox_localization.c \
bridge_receiver_autobox_trajectory.c \
bridge_buffer.c \
bridge_header_item.c \
bridge_header.c \
bridge_proto_diserialized_buf.c \
autobox_trajectory.pb-c.c \
pnc_point.pb-c.c \
pose.pb-c.c \
localization.pb-c.c \
geometry.pb-c.c \
localization_status.pb-c.c \
protobuf-c.c \
bridge_receiver_preprocessing.c \

# Additional assembler source files to be compiled (file name extension .asm).
USER_ASM_SRCS =

# Directories where additional C and assembler source files are stored.
USER_SRCS_DIR = \
"sfunctions" \
"..\..\protoc_genfiles\modules\canbus\proto" \
"..\..\protoc_genfiles\modules\common\proto" \
"..\..\protoc_genfiles\modules\common\configs\proto" \
"..\..\protoc_genfiles\modules\autobox_bridge\proto" \
"..\..\protoc_genfiles\modules\localization\proto" \
"..\bridge_c\client" \
"..\bridge_c\common" \
"..\..\third_party\protobuf-c" \
"..\..\third_party"

# Path names for user include files.
USER_INCLUDES_PATH = \
"sfunctions" \
"..\..\protoc_genfiles\modules\canbus\proto" \
"..\..\protoc_genfiles\modules\common\proto" \
"..\bridge_c\client" \
"..\bridge_c\common" \
"..\..\protoc_genfiles\modules\autobox_bridge\proto" \
"..\..\third_party\protobuf-c" \
"..\..\third_party"

# Additional user object files to be linked.
USER_OBJS =

# Additional user libraries to be linked.
USER_LIBS =

# EOF -------------------------------------------------------------------------

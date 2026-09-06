# Builds a logue-sdk unit project into a host dylib the bench can dlopen.
#
#   make -f unit.mk UNIT_DIR=<path to project> PLATFORM=<drumlogue|nts-3_kaoss>
#
# Reads the project's own config.mk for its source list, so it tracks the
# project rather than duplicating it. The unit's sources are unmodified; the
# only accommodations are the Mach-O attribute override and the portable
# CMSIS shim, both host-only.

HOST_DIR      ?= $(CURDIR)
PLATFORM_ROOT ?= $(HOST_DIR)/../../../platform
OUT_DIR       ?= $(HOST_DIR)/build/units

ifndef UNIT_DIR
$(error UNIT_DIR is required)
endif
ifndef PLATFORM
$(error PLATFORM is required (drumlogue | nts-3_kaoss))
endif

include $(UNIT_DIR)/config.mk

# The stock config.mk lists header.c/unit.cc for most templates; fall back to
# those when a project leaves the lists empty.
CSRC   ?=
CXXSRC ?=
ifeq ($(strip $(CSRC)$(CXXSRC)),)
  CSRC   := header.c
  CXXSRC := unit.cc
endif

PLAT_INC := $(PLATFORM_ROOT)/$(PLATFORM)/common
ATTRS    := -DATTRIBUTES_H_=1 -include $(HOST_DIR)/harness_attrs.h
# Enables a unit's optional host-bench diagnostics. Never set for the device
# build, so the shipped unit carries none of this.
ATTRS    += -DCD_HOST_DIAG=1
INCS     := -I$(UNIT_DIR) -I$(PLAT_INC) -I$(HOST_DIR)/compat $(UINCDIR:%=-I%)
OPT      := -O2 -ffast-math
WARN     := -Wall -Wextra -Wno-unused-parameter -Wno-unknown-attributes

OBJDIR := $(OUT_DIR)/obj/$(PROJECT)
OBJS   := $(CSRC:%.c=$(OBJDIR)/%.o) $(CXXSRC:%.cc=$(OBJDIR)/%.oo)
TARGET := $(OUT_DIR)/$(PROJECT)_$(PLATFORM).dylib

all: $(TARGET)

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: $(UNIT_DIR)/%.c | $(OBJDIR)
	clang -std=c11 $(OPT) $(WARN) $(INCS) $(ATTRS) $(UDEFS) -fPIC -c $< -o $@

$(OBJDIR)/%.oo: $(UNIT_DIR)/%.cc | $(OBJDIR)
	clang++ -std=gnu++14 $(OPT) $(WARN) $(INCS) $(ATTRS) $(UDEFS) -fPIC -c $< -o $@

# Unit entry points must stay visible for dlsym.
$(TARGET): $(OBJS)
	@mkdir -p $(OUT_DIR)
	clang++ $(OBJS) -dynamiclib -o $@
	@echo "Built $@"
	@echo "HOSTUNIT=$@"

# Print where the artifact would land without building it.
target-path:
	@echo "HOSTUNIT=$(TARGET)"

clean:
	rm -rf $(OBJDIR) $(TARGET)

.PHONY: all clean target-path

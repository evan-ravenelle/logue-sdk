# Host engine + per-platform shims.
#
# The shims cannot share an include path: drumlogue and nts-3 both define
# unit_header_t / unit_param_t / unit_runtime_desc_t with different layouts.
# Each is a separate translation unit compiled against its own SDK headers,
# and they meet only at the cdh_vtable boundary in cdhost.h.

PLATFORM_ROOT := ../../../..
DL_INC        := $(PLATFORM_ROOT)/drumlogue/common
N3_INC        := $(PLATFORM_ROOT)/nts-3_kaoss/common

BUILD := build
LIB   := $(BUILD)/libcdhost.dylib

CC   := clang
CXX  := clang++
WARN := -Wall -Wextra -Wno-unused-parameter
OPT  := -O2

# Mach-O cannot take the SDK's section(".unit_header"); see harness_attrs.h.
ATTRS := -DATTRIBUTES_H_=1 -include $(CURDIR)/harness_attrs.h

OBJS := $(BUILD)/engine.o $(BUILD)/plat_drumlogue.o $(BUILD)/plat_nts3.o

all: $(LIB)

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/engine.o: engine.cc cdhost.h | $(BUILD)
	$(CXX) -std=gnu++14 $(OPT) $(WARN) -I. -fPIC -c $< -o $@

$(BUILD)/plat_drumlogue.o: plat_drumlogue.c cdhost.h | $(BUILD)
	$(CC) -std=c11 $(OPT) $(WARN) -I. -Icompat -I$(DL_INC) $(ATTRS) -fPIC -c $< -o $@

$(BUILD)/plat_nts3.o: plat_nts3.c cdhost.h | $(BUILD)
	$(CC) -std=c11 $(OPT) $(WARN) -I. -Icompat -I$(N3_INC) $(ATTRS) -fPIC -c $< -o $@

# @rpath install name so the tools and the bench resolve it from any cwd.
$(LIB): $(OBJS)
	$(CXX) $(OBJS) -dynamiclib -install_name @rpath/libcdhost.dylib \
		-framework AudioToolbox -framework CoreFoundation -o $@
	@echo "Built $@"

# Self-test / probe tools.
tools: $(BUILD)/selftest $(BUILD)/probe

$(BUILD)/selftest: selftest.c $(LIB)
	$(CC) -std=c11 -I. $< -L$(BUILD) -lcdhost -Wl,-rpath,@executable_path -o $@

$(BUILD)/probe: probe.c $(LIB)
	$(CC) -std=c11 -I. $< -L$(BUILD) -lcdhost -Wl,-rpath,@executable_path -o $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean tools

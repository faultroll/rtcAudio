
prfx   ?= 
cc     := $(prfx)gcc
cxx    := $(prfx)g++
ar     := $(prfx)ar
ranlib := $(prfx)ranlib
strip  := $(prfx)strip

name    := rtc_audio
srcs    := 
include rtc_base/SOURCE.mk
include system_wrappers/SOURCE.mk
include third_party/SOURCE.mk
include common_audio/SOURCE.mk
include modules/SOURCE.mk
include adapter_c/SOURCE.mk
# include demos/SOURCE.mk
# srcs_c doesn't need libstdc++
# srcs_c + srcs is what adapter_c uses
# srcs_cpl_only currently not in use
srcs    := $(srcs_c) $(srcs) # $(srcs_cpl_only) # $(srcs_main)
objs    := $(patsubst %.cc,%.o,$(filter %.cc, $(srcs))) \
           $(patsubst %.c,%.o,$(filter %.c, $(srcs))) \
           $(patsubst %.S,%.o,$(filter %.S, $(srcs)))
deps    := $(patsubst %.o,%.d,$(objs))
libs    := -lpthread
cflags   = -I. -DWEBRTC_POSIX # -DWEBRTC_WIN
cflags  += -Wno-unused-parameter -Wno-sign-compare -g # -DNDEBUG
# can be found in SDK Makefile or on board /lib/ dir
# cflags  += -mcpu=cortex-a9 -mfloat-abi=softfp -mfpu=neon # a9_vfpv3_neon, arm-hisiv600-linux-
# cflags  += -mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4 # a7_softfp_neon-vfpv4, arm-himix200-linux-
# Define |PFFFT_SIMD_DISABLE| for no neon flag
# cflags  += -mcpu=cortex-a53 -DPFFFT_SIMD_DISABLE # cannot find neon flag, aarch64-himix100-linux-
ldflags := -lstdc++
# for reproducible build
objs    := $(sort $(objs))
# cflags  += -Wno-builtin-macro-redefined -U__FILE__ -D__FILE__=\"$(notdir $<)\"

targets := lib$(name).so lib$(name).a
all : $(targets)

clean : 
	rm -f $(targets)
	rm -f $(objs) $(deps)

lib$(name).so : $(objs)
	@$(cc) -shared -Wl,--gc-sections -Wl,--as-needed -Wl,--export-dynamic $(ldflags) $^ -o $@ $(libs)
	@$(strip) --strip-all $@
	$(info $(cc) -shared $(notdir $^) -o $(notdir $@))

lib$(name).a : $(objs)
	@$(ar) -crD $@ $^
	@$(ranlib) -D $@
	@$(strip) --strip-unneeded $@
	$(info $(ar) -crD $(notdir $@) $(notdir $^))

%.o : %.cc
	@$(cxx) -Os -Wall -Wextra -std=c++11 -fPIC -fpermissive -fcompare-debug-second $(cflags) -c $< -o $@ -MMD -MF $*.d -MP
	$(info $(cxx) -c $(notdir $<) -o $(notdir $@))
%.o : %.c
	@$(cc) -Os -Wall -Wextra -std=c11 -fPIC -D_POSIX_C_SOURCE=200809L $(cflags) -c $< -o $@ -MMD -MF $*.d -MP
	$(info $(cc) -c $(notdir $<) -o $(notdir $@))
%.o : %.S
	@$(cc) -Os -Wall -Wextra -std=c11 -fPIC -D_POSIX_C_SOURCE=200809L $(cflags) -c $< -o $@ -MMD -MF $*.d -MP
	$(info $(cc) -c $(notdir $<) -o $(notdir $@))

-include $(deps)

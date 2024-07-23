APP?=SimpleConnection
FUNC?=_sinkreaction_function_0
SRC_DIR=$(CURDIR)/src/static/patmos
DEST_DIR=$(CURDIR)/src-gen/static/$(APP)
INCD_DIR=$(CURDIR)/include
export LF_PROJECT_ROOT:=$(DEST_DIR)
export LF_MAIN_TARGET:=$(APP)
export LF_WCET_FUNC:=$(FUNC)

.PHONY: gen copy comp all clean wcet sim lin

all: del gen copy comp lin sim wcet
gen: 
	lfc-dev src/static/$(APP).lf 
copy:
	cp $(SRC_DIR)/lf_patmos_support.h 	  $(DEST_DIR)/include/core/platform/
	cp $(SRC_DIR)/platform.h              $(DEST_DIR)/include/core/
	cp $(SRC_DIR)/lf_patmos_support.c     $(DEST_DIR)/core/platform/
	cp $(SRC_DIR)/lf_atomic_patmos.c      $(DEST_DIR)/core/platform/
	cp $(SRC_DIR)/lf_patmos_support.h     $(INCD_DIR)/core/platform/
	cp $(SRC_DIR)/platform.h              $(INCD_DIR)/core
	cp $(SRC_DIR)/Makefile                $(DEST_DIR)
comp: 
	make -C $(DEST_DIR) 
lin: 
	$(CURDIR)/bin/$(APP)
sim: 
	pasim $(DEST_DIR)/$(APP).elf
clean:
	make clean -C $(DEST_DIR)
del:
	rm -rf bin include src-gen
wcet:
	make wcet -C $(DEST_DIR) 
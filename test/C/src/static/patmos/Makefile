# LF_PROJECT_ROOT ?= "$(CURDIR)/../../.."
# LF_MAIN_TARGET ?= ScheduleTest
LF_MAIN_TARGET_LC := $(shell echo $(LF_MAIN_TARGET) | tr '[:upper:]' '[:lower:]')

SERIAL?=/dev/ttyUSB0

INCS  := -I"$(LF_PROJECT_ROOT)/include" \
	-I"$(LF_PROJECT_ROOT)/include/api" \
	-I"$(LF_PROJECT_ROOT)/include/core" \
	-I"$(LF_PROJECT_ROOT)/include/core/platform" \
	-I"$(LF_PROJECT_ROOT)/include/core/modal_models" \
	-I"$(LF_PROJECT_ROOT)/include/core/utils" \
	-I"$(LF_PROJECT_ROOT)/include/core/threaded" \
	-I"$(LF_PROJECT_ROOT)/include/federated" \
	-I"$(LF_PROJECT_ROOT)/include/federated/network" \
#	-I"/home/ehsan/lingua-franca/test/C/src-gen/static/ScheduleTest/core/federated/RTI" 
	
CC := patmos-clang 

CFLAGS := $(INCS) -DINITIAL_EVENT_QUEUE_SIZE=10 -DINITIAL_REACT_QUEUE_SIZE=10 -DLF_REACTION_GRAPH_BREADTH=1 
CFLAGS += -DLF_SINGLE_THREADED=0 -DSCHEDULER=SCHED_STATIC
SRC_FILES := _$(LF_MAIN_TARGET_LC)_main.c $(LF_MAIN_TARGET).c lib/schedule.c _sink.c _source.c static_schedule.c
SRC_FILES += core/reactor_common.c core/lf_token.c core/reactor.c core/tag.c core/environment.c 
SRC_FILES += core/utils/util.c core/utils/vector.c  core/utils/circular_buffer.c
SRC_FILES += core/utils/pqueue.c core/utils/pqueue_tag.c core/utils/pqueue_base.c
SRC_FILES += core/utils/hashset/hashset_itr.c core/utils/hashset/hashset.c 
SRC_FILES += core/clock.c core/platform/lf_atomic_irq.c 
SRC_FILES += core/platform/lf_patmos_support.c core/platform/lf_atomic_patmos.c 
# SRC_FILES += core/threaded/scheduler_adaptive.c core/threaded/scheduler_GEDF_NP.c core/threaded/reactor_threaded.c core/threaded/watchdog.c
SRC_FILES += core/threaded/scheduler_instance.c core/threaded/scheduler_static.c
#  core/threaded/scheduler_sync_tag_advance.c core/threaded/scheduler_NP.c
# SRC_FILES += core/federated/RTI/rti_local.c 
OBJ_FILES := $(patsubst %.c,%.o,$(SRC_FILES))
EXE_NAME := $(LF_MAIN_TARGET).elf

.PHONY: all clean wcet

all: $(EXE_NAME)
$(EXE_NAME): $(OBJ_FILES)
	$(CC) $(CFLAGS) -o $@ $^ -O2 -mserialize=lf.pml
$(OBJ_FILES): %.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ_FILES) $(EXE_NAME)
wcet: $(EXE_NAME)
	platin wcet --disable-ait -i lf.pml -b $(EXE_NAME) -e _sinkreaction_function_0 --report report.txt
	# platin visualize -i lf.pml -f main --show-timings=platin
	
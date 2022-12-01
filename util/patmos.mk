LF_SRC ?= .
INCS=-I"$(LF_SRC)/include/" \
	-I"$(LF_SRC)/include/api" \
	-I"$(LF_SRC)/include/core" \
	-I"$(LF_SRC)/include/core/modal_models" \
	-I"$(LF_SRC)/include/core/utils" \
	-I"$(LF_SRC)/include/core/platform"

LFLIB="$(LF_SRC)/core/platform/lf_patmos_support.c" \
	"$(LF_SRC)/core/platform/lf_unix_clock_support.c" \
	"$(LF_SRC)/core/port.c" \
	"$(LF_SRC)/core/tag.c" \
	"$(LF_SRC)/core/utils/pqueue.c" \
	"$(LF_SRC)/core/utils/util.c" \
	"$(LF_SRC)/core/utils/vector.c" \
	"$(LF_SRC)/core/reactor_common.c" \
	"$(LF_SRC)/core/reactor.c"

DEFS?=-DINITIAL_EVENT_QUEUE_SIZE=10 -DINITIAL_REACT_QUEUE_SIZE=10

all:
	patmos-clang -O2 $(INCS) $(DEFS) *.c $(LFLIB)

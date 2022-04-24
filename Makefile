LIB_SRCS := \
		src/nav_alg.cpp \
		src/nav_solution.cpp \

API_SRCS := \
		src/analysis_api.cpp
TEST_SRC := \
		test/test.cpp

LIB_OBJS := ${LIB_SRCS:cpp=o}
API_OBJS := ${API_SRCS:cpp=o}

HDRS := include
HDRS += modules/PX4-Matrix/matrix
LIB = lib/libnav.so
API = lib/libnavapi.so

TEST_BIN = test.out

CC = g++

CPPFLAGS=-g -ggdb -O0 -fPIC -Wall -Werror -Wpedantic -DPYTHON
LINKAGE=-shared

LDFLAGS=navapi
LDPATH=lib/

.PHONY: all api lib clean
all: api
lib: ${LIB}
api: ${API}

%.o:%.cpp
	${CC}  $(addprefix -I,$(HDRS)) ${CPPFLAGS} -c $< -o $@

$(LIB): $(LIB_OBJS)
	${CC} ${LINKAGE} $< -o $@

$(API): ${LIB_OBJS} ${API_OBJS}
	${CC} ${LINKAGE} $^ -o $@

test_dynamic: ${TEST_SRC} ${API}
	${CC} -L${LDPATH} ${CPPFLAGS} $(addprefix -I,$(HDRS)) -l${LDFLAGS} ${TEST_SRC} -o ${TEST_BIN}

test_static: ${TEST_SRC} ${API_SRCS} ${LIB_SRCS}
	${CC} ${CPPFLAGS} $(addprefix -I,$(HDRS)) $^ -o ${TEST_BIN}

clean_lib:
	rm -f ${LIB} ${LIB_OBJS}
clean_api:
	rm -f ${API} ${API_OBJS}
clean_test:
	rm -f ${TEST_BIN}
clean: clean_lib clean_api clean_test
	@echo "all clean"
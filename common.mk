BUILD_DIR ?= ./build
SRC_DIR ?= ./src
INC_DIR ?= ./include

SRCS += $(shell find -L $(SRC_DIR) -name "*.cpp" -or -name "*.c")
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find -L $(INC_DIR) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

COMMONFLAGS = -Wall -Wextra

ifeq ($(mode), debug)
    COMMONFLAGS += -Og -g -DDEBUG
else
    COMMONFLAGS += -Ofast -DNDEBUG
endif

CXXFLAGS ?= -std=c++11 $(COMMONFLAGS)
CXX ?= g++

CFLAGS ?= -std=c99 -Wdouble-promotion $(COMMONFLAGS)
CC ?= gcc

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP 
LDLIBS += -lm

-include $(DEPS)
MKDIR_P ?= mkdir -p

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

.DEFAULT_GOAL := 


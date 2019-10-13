TARGET_EXEC ?= embedded

BUILD_DIR ?= ./build
SRC_DIR ?= ./src
INC_DIR ?= ./include

SRCS := $(shell find -L $(SRC_DIR) -name "*.cpp" -or -name "*.c")
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(INC_DIR) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CXXWARNING =  -Wall -Wuninitialized -Wmissing-field-initializers -Werror=pointer-arith
CXXERROR = -Werror-implicit-function-declaration -Werror=return-type
CXXOPTIMIZATION = -O2 -fomit-frame-pointer -funroll-loops -ffast-math -fno-finite-math-only
CXXFLAGS ?= -std=c++11 $(CXXWARNING) $(CXXERROR) $(CXXOPTIMIZATION)
CXX ?= g++

CFLAGS ?= -fPIC -O3 -Wall -Werror -std=c99 -DDEBUG -g
CC ?= gcc

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP 
LDLIBS += -lm

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

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

-include $(DEPS)

MKDIR_P ?= mkdir -p

TARGET_EXEC ?= dmls

BUILD_DIR ?= ./build
SRC_DIRS ?= ./src

SRCS := $(shell find $(SRC_DIRS) -name *.cpp -or -name *.c -or -name *.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CXX ?= g++
CPPWARNING =  -Wall -Wuninitialized -Wmissing-field-initializers -Werror=pointer-arith
CPPERROR = -Werror-implicit-function-declaration -Werror=return-type
CPPOPTIMIZATION = -O2 -fomit-frame-pointer -funroll-loops -ffast-math -fno-finite-math-only
CPPFLAGS ?= $(INC_FLAGS) -MMD -MP -std=c++17 $(CPPWARNING) $(CPPERROR)
LDLIBS += -lstdc++ -lpthread

CFLAGS ?= -fPIC -O3 -Wall -Werror -std=c99
CC ?= gcc


$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)


# assembly
$(BUILD_DIR)/%.s.o: %.s
	$(MKDIR_P) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

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



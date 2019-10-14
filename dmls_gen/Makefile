include ../common.mk

TARGET_EXEC ?= dmls_gen

LDLIBS += -lstdc++ -lm

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

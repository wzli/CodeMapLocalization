include ../common.mk

TARGET_EXEC ?= map_index_gen

LDLIBS += -lstdc++

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

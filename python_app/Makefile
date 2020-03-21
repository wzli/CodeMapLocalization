include ../common.mk

TARGET_LIB ?= libcodemap.so
COMMONFLAGS += -fPIC
$(BUILD_DIR)/$(TARGET_LIB): $(OBJS)
	$(CC) $(OBJS) -shared -Wl,-soname,$(TARGET_LIB) -o $@ $(LDFLAGS) $(LDLIBS)

## these need to be set before including

ifndef TOP
	$(error TOP is not set)
endif
ifndef ASM_SOURCE
	$(error ASM_SOURCE is not set)
endif

ASMEXT=s
OBJEXT=o

OBJFILE=$(BUILD_PATH)/$(TOP).o

# compute 6502 obj from asm
$(OBJFILE): $(ASM_SOURCE)
	@mkdir -p $(BUILD_PATH)
	xa -o $@ $^ -l $(BUILD_PATH)/labels.txt

.PHONY: obj
obj: $(OBJFILE)

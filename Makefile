M=$(PWD)
INC=-I/$(M)/include/*
KBUILD_OPTIONS+=$(KERNEL_SRC)/$(M)

all: modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(INC) $@ $(KBUILD_OPTIONS)

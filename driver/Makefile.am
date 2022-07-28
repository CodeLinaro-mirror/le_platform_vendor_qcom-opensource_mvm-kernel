module = mvm.ko

$(module): mvm.c
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) modules

install-exec-local: $(module)
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) INSTALL_MOD_PATH=$(DESTDIR)$(prefix)/modules modules_install

clean-local:
	-test ! -d "$(KERNEL_DIR)" || $(kmake) clean

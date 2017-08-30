ccflags-y+=-Wfatal-errors
ccflags-y+=-fmax-errors=5

ifdef DEBUG
ccflags-y+=-DDEBUG
endif

obj-m += seco-cec.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

all:	modules

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions \
	modules.order Module.symvers *.tmp *.log

_src = seco-cec.c

checkpatch:
	$(KERNELDIR)/scripts/checkpatch.pl --no-tree --show-types \
		--ignore LINE_CONTINUATIONS \
		--terse -f $(PWD)/$(_src) Makefile

checkpatch2:
	$(KERNELDIR)/scripts/checkpatch.pl --no-tree --show-types \
		--ignore LONG_LINE,LINE_CONTINUATIONS \
		--terse -f $(PWD)/$(_src) Makefile

Lindent:
	$(KERNELDIR)/scripts/Lindent \
		$(PWD)/$(_src)

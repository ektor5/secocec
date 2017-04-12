#ccflags-y+=-Wfatal-errors
ccflags-y+=-DDEBUG
ccflags-y+=-fmax-errors=5

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

_src = secocec.c

checkpatch:
	checkpatch.pl --no-tree --show-types \
		--ignore LINE_CONTINUATIONS \
		--terse -f $(_src) Makefile

checkpatch2:
	checkpatch.pl --no-tree --show-types \
		--ignore LONG_LINE,LINE_CONTINUATIONS \
		--terse -f $(_src) Makefile

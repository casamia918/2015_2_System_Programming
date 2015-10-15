obj-m := chardev.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) -I. modules
	gcc -o profiler profiler.c
	gcc -o stupid stupid_bench.c
	gcc -o clever clever_bench.c
	sudo insmod ./chardev.ko
	sudo mknod /dev/chardev c 1522 0

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm clever stupid profiler
	sudo rm /dev/chardev
	sudo rmmod chardev

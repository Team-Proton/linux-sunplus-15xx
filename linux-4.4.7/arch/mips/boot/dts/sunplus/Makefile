# All DTBs
dtb-$(CONFIG_SPHE15XX)		+= sphe15xx.dtb

# Select a DTB to build in the kernel
obj-$(CONFIG_DTB_SPHE15XX_V1)	+= sphe15xx_v1.dtb.o

# Force kbuild to make empty built-in.o if necessary
obj-				+= dummy.o

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S

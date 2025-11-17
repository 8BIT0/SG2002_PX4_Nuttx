cd /home/zjz/qemu-10.0.3/build
./qemu-system-riscv64 -M virt \
 -m 1G \
 -bios none \
 -nodefaults \
 -kernel /home/zjz/nuttx_px4/nuttx/nuttx \
 -nographic \
 -serial mon:stdio \
 -s -S



 # -serial mon:stdio \
 # -monitor none \


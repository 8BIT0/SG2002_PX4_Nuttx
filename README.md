Auther:8_B!T0

porting NUTTX (886ac) OS to SG2002 little core (700Mhz core)

now only support qemu simulation on ubuntu

sim hardware
    licheerv nano (sg2002)

essential toolchain
    1.riscv64-unknown-elf-gcc
    2.riscv64-unknown-elf-gdb

qemu version 10.0.3

how to run
1. cd to nutx folder input
    tools/configure.sh licheerv_nano:nsh

2. make

3. if build successed leave nuttx folder and goto up level folder execute sh file (terminal 1)
    cd ../
    ./sim_nuttx.sh

4. open a new terminal input command start (terminal 2)
    riscv64-unknown-elf-gdb
    target remote localhost:1234
    c

terminal 1 will print info 

    ABnsh: sysinit: fopen failed: No such file or directory
    nsh: mkrd: boardctl(BOARDIOC_MKRD) failed: Operation not permitted

    NuttShell (NSH) NuttX-11.0.0
    nsh>



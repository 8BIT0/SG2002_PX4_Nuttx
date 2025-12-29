rm -rf ../py_tool/nuttx_1.bin
llvm-objcopy -O binary nuttx nuttx_1.bin
mv nuttx_1.bin ../py_tool

pushd ../py_tool
./../py_tool/pck_fip.sh
popd

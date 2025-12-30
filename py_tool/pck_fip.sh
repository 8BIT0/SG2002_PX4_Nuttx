rm -rf fip.bin

python3 chip_conf.py bin/chip_conf.bin
MONITOR_RUNADDR=0x0000000080000000
BLCP_2ND_RUNADDR=0x000000008fe00000
BLCP_IMG_RUNADDR=0x05200200
BLCP_PARAM_LOADADDR=0
NAND_INFO=00000000
NOR_INFO=$(printf '%72s' | tr ' ' 'FF')

${Q}echo "  [GEN] fip.bin"
python3 fiptool.py -v genfip \
    'fip.bin' \
    --MONITOR_RUNADDR=${MONITOR_RUNADDR} \
    --BLCP_2ND_RUNADDR=${BLCP_2ND_RUNADDR} \
    --CHIP_CONF=bin/chip_conf.bin \
    --NOR_INFO=${NOR_INFO} \
    --NAND_INFO=${NAND_INFO} \
    --BL2=bin/bl2.bin \
    --BLCP_IMG_RUNADDR=${BLCP_IMG_RUNADDR} \
    --BLCP_PARAM_LOADADDR=${BLCP_PARAM_LOADADDR} \
    --BLCP=bin/empty.bin \
    --BLCP_2ND=nuttx_1.bin \
    --MONITOR=bin/fw_dynamic.bin \
    --LOADER_2ND=bin/u-boot-raw.bin \
    --compress=lzma

rm -rf nuttx_1.bin
rm -rf bin/chip_conf.bin

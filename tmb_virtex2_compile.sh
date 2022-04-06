#!/bin/bash
projectpath=/home/cscdev/XilinxProj/TMB_Firmware/TMB_FW_2021_v1
echo "project path ","$projectpath/tmb_virtex2.ise"
source /home/cscdev/Xilinx/Xilinx_Installed/10.1/ISE/settings64.sh
projectfile=$projectpath/tmb_virtex2.ise
syrout=$projectpath/tmb_virtex2.syr
xstout=$projectpath/tmb_virtex2.xst
npdout=$projectpath/tmb_virtex2.ngc
mapdir=$projectpath/mppr_result.dir
bitfile=$projectpath/tmb_virtex2.bit

if [ -f "$syrout" ]; then
   mv $syrout $projectpath/tmb_virtex2_old.syr
fi

if [ -f "$bitfile" ]; then
   mv $bitfile $projectpath/tmb_virtex2_old.bit
fi

xst -ise "$projectfile" -intstyle ise -ifn "$xstout" -ofn "$syrout"
ngdbuild -ise "$projectfile" -intstyle ise -dd _ngo  -nt timestamp -i -p xc2v4000-ff1152-5 "$npdout" tmb_virtex2.ngd
map -ise "$projectfile" -intstyle ise -p xc2v4000-ff1152-5 -timing -logic_opt off -ol high -xe n -t 3 -cm area -pr off -k 4 -tx off -o tmb_virtex2_map.ncd tmb_virtex2.ngd tmb_virtex2.pcf
par -ise "$projectfile" -w -intstyle ise -ol high -t 4 -n 3 tmb_virtex2_map.ncd "$mapdir" tmb_virtex2.pcf
bitgen -ise "$projectfile" -intstyle ise -f tmb_virtex2.ut tmb_virtex2.ncd

#!/bin/bash

arch=arm64
gccarch=aarch64
config=bcmrpi3_defconfig
dts_subdir=dts/broadcom
kernel=Image
tree=ohmbre-4.15.y
base_tree=rpi-4.15.y
srcs_dir=/root/src/ohmkernel
blds_dir=/root/build/ohmkernel
remote_ip=10.0.0.3
wrks_dir=/root/.work/ohmkernel
uppers_dir=/root/.uppers/ohmkernel
base_src_dir=$srcs_dir/$base_tree
src_dir=$srcs_dir/$tree
bld_dir=$blds_dir/$tree
wrk_dir=$wrks_dir/$tree
upper_dir=$uppers_dir/$tree
dist_dir=$srcs_dir/dist
echo "building in $bld_dir ..."
make="make"

make="$make INSTALL_PATH=$dist_dir/boot"
make="$make INSTALL_MOD_PATH=$dist_dir"
make="$make KBUILD_OUTPUT=$bld_dir"
make="$make ARCH=$arch"
make="$make CROSS_COMPILE=/root/src/buildroot/output/host/opt/ext-toolchain/bin/$gccarch-linux-gnu-"
make="$make INITRD=No"
make="$make CCACHE_DIR=$build_dir/.ccache"
make="$make -C $src_dir"
echo "running with make command: $make"
set -e
shopt -s globstar

mkdir -p $dist_dir

setup() {   
    mkdir -p $src_dir
    mkdir -p $wrk_dir
    rm -rf $upper_dir
    mkdir -p $upper_dir

    umount $src_dir > /dev/null 2>&1 || /bin/true
    [[ $(findmnt -M "$src_dir") ]] || mount -t overlay overlay -o lowerdir=$srcs_dir/overlay:$base_src_dir,upperdir=$upper_dir,workdir=$wrk_dir $src_dir
    for f in $(find $src_dir -name \*.patch); do
	patch $(dirname $f)/$(basename $f .patch) $f
    done

    return 0
}

config() {
    $make $config
    $make nconfig
    $make savedefconfig
    #echo "-- config changes below --"
    #diff $bld_dir/defconfig $src_dir/arch/$arch/configs/$config
    cp $bld_dir/defconfig $src_dir/arch/$arch/configs/$config
    pushd $src_dir
    git diff arch/$arch/configs/$config > $OLDPWD/kernel/arch/$arch/configs/$config.patch
    popd
    $make prepare
    return 0
}

build() {
    $make -j10
    sudo dtc -I dts -O dtb -o $bld_dir/arch/$arch/boot/$dts_subdir/dt-blob.bin $src_dir/arch/$arch/boot/$dts_subdir/dt-blob.dts
    version=`$make -f $bld_dir/Makefile kernelrelease 2> /dev/null`
    echo "built version $version"
    return 0
}

dist_modules() {
    $make -j10 modules_install
    return 0
}

dist() {
    cp $bld_dir/arch/$arch/boot/$dts_subdir/ohmbre.dtb $dist_dir/boot/ohmbre.dtb
    cp $bld_dir/arch/$arch/boot/$kernel $dist_dir/boot/ohmbre.img
    cp $src_dir/arch/$arch/boot/config.txt $bld_dir/arch/$arch/boot/config.txt
    cp $src_dir/arch/$arch/boot/cmdline.txt $bld_dir/arch/$arch/boot/cmdline.txt
    cp $bld_dir/arch/$arch/boot/$dts_subdir/dt-blob.bin $dist_dir/boot/dt-blob.bin
    #scp -r $INSTALL_PATH/$tree/* $remote_ip:/boot/$tree/
    
    return 0
}

firmware() {
    wget -O $dist_dir/boot/bootcode.bin https://raw.githubusercontent.com/raspberrypi/firmware/next/boot/bootcode.bin
    wget -O $dist_dir/boot/fixup.dat https://raw.githubusercontent.com/raspberrypi/firmware/next/boot/fixup.dat
    wget -O $dist_dir/boot/start.elf https://raw.githubusercontent.com/raspberrypi/firmware/next/boot/start.elf
}

exportenv() {
    echo "WARNING: changing your env"
    export make=$make
}

setup

do_config=false
do_build=false
do_dist=false
do_dist_modules=false
do_firmware=false
do_env=false

while getopts ":cbmkfe" opt; do
    case ${opt} in
	c ) do_config=true;;
	b ) do_build=true;;
	d ) do_dist=true;;
	m ) do_dist_modules=true;;
	f ) do_firmware=true;;
	e ) do_env=true
    esac
done

if [ $do_config = true ]; then config; fi
if [ $do_build = true ]; then build; fi
if [ $do_dist = true ]; then dist; fi
if [ $do_dist_modules = true ]; then dist_modules; fi
if [ $do_firmware = true ]; then firmware; fi
if [ $do_env = true ]; then exportenv; fi

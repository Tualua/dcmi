# dcmi
Intel DCMI kernel driver for OCP patched for newer kernels

Tested on Quanta Windmill with Ubuntu 18.04.3/16.04.6 with kernel 4.18.20

**dcmitool** is a binary to use DCMI interface. Took it from  https://bugs.launchpad.net/opencompute/+bug/1156667/+attachment/3683163/+files/Intel_driver_and_dcmitool.zip

# How to install

    sudo make install
    sudo modprobe dcmi
    sudo ln -s /usr/lib/x86_64-linux-gnu/libcrypto.so.1.1 /usr/lib/x86_64-linux-gnu/libcrypto.so.6

Your actual version on libcrypto may be different, in this case adjust path in the last line according to your environment (because of kernel 4.18 I used libssl1.1).

You will need to put dcmi in /etc/modules to autoload it


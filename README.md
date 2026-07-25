Qemu fork to interface with SimulIDE.

Building:
======

Configure for ESP32:

.. code-block:: shell

  ./configure --target-list=xtensa-softmmu --extra-cflags=-fPIC --disable-attr --disable-auth-pam --disable-avx2 --disable-avx512bw --disable-blkio --disable-bochs --disable-bpf --disable-brlapi --disable-bzip2 --disable-canokey --disable-cap-ng --disable-capstone --disable-cloop --disable-cocoa --disable-colo-proxy --disable-coreaudio --disable-crypto-afalg --disable-curl --disable-curses --disable-dbus-display --disable-dmg --disable-docs --disable-dsound --disable-fuse --disable-fuse-lseek --disable-gcrypt --disable-gettext --disable-gio --disable-glusterfs --disable-gnutls --disable-gtk --disable-gtk-clipboard --disable-guest-agent --disable-guest-agent-msi --disable-hvf --disable-iconv --disable-jack --disable-keyring --disable-kvm --disable-l2tpv3 --disable-libdaxctl --disable-libdw --disable-libiscsi --disable-libkeyutils --disable-libnfs --disable-libpmem --disable-libssh --disable-libudev --disable-libusb --disable-libvduse --disable-linux-aio --disable-linux-io-uring --disable-lzfse --disable-lzo --disable-malloc-trim --disable-membarrier --disable-modules --disable-mpath --disable-multiprocess --disable-netmap --disable-nettle --disable-numa --disable-nvmm --disable-opengl --disable-oss --disable-pa --disable-parallels --disable-pipewire --disable-png --disable-qcow1 --disable-qed --disable-qga-vss --disable-rbd --disable-rdma --disable-replication --disable-sdl --disable-sdl-image --disable-seccomp --disable-selinux --disable-smartcard --disable-snappy --disable-sndio --disable-sparse --disable-spice --disable-spice-protocol --disable-stack-protector --disable-tcg --disable-tools --disable-tpm --disable-u2f --disable-usb-redir --disable-vde --disable-vdi --disable-vhdx --disable-vhost-crypto --disable-vhost-kernel --disable-vhost-net --disable-vhost-user --disable-vhost-vdpa --disable-virglrenderer --disable-virtfs --disable-vmdk --disable-vmnet --disable-vnc --disable-vnc-jpeg --disable-vnc-sasl --disable-vpc --disable-vte --disable-vvfat --disable-whpx --disable-xen --disable-xkbcommon --disable-zstd --disable-system --disable-user --disable-linux-user --disable-bsd-user --disable-pie --disable-debug-tcg --disable-werror --disable-alsa --disable-debug-info --enable-tcg --enable-system --enable-gcrypt

Configure for ARM:

.. code-block:: shell

  ./configure --target-list=arm-softmmu --extra-cflags=-fPIC --disable-attr --disable-auth-pam --disable-avx2 --disable-avx512bw --disable-blkio --disable-bochs --disable-bpf --disable-brlapi --disable-bzip2 --disable-canokey --disable-cap-ng --disable-capstone --disable-cloop --disable-cocoa --disable-colo-proxy --disable-coreaudio --disable-crypto-afalg --disable-curl --disable-curses --disable-dbus-display --disable-dmg --disable-docs --disable-dsound --disable-fuse --disable-fuse-lseek --disable-gcrypt --disable-gettext --disable-gio --disable-glusterfs --disable-gnutls --disable-gtk --disable-gtk-clipboard --disable-guest-agent --disable-guest-agent-msi --disable-hvf --disable-iconv --disable-jack --disable-keyring --disable-kvm --disable-l2tpv3 --disable-libdaxctl --disable-libdw --disable-libiscsi --disable-libkeyutils --disable-libnfs --disable-libpmem --disable-libssh --disable-libudev --disable-libusb --disable-libvduse --disable-linux-aio --disable-linux-io-uring --disable-lzfse --disable-lzo --disable-malloc-trim --disable-membarrier --disable-modules --disable-mpath --disable-multiprocess --disable-netmap --disable-nettle --disable-numa --disable-nvmm --disable-opengl --disable-oss --disable-pa --disable-parallels --disable-pipewire --disable-png --disable-qcow1 --disable-qed --disable-qga-vss --disable-rbd --disable-rdma --disable-replication --disable-sdl --disable-sdl-image --disable-seccomp --disable-selinux --disable-smartcard --disable-snappy --disable-sndio --disable-sparse --disable-spice --disable-spice-protocol --disable-stack-protector --disable-tcg --disable-tools --disable-tpm --disable-u2f --disable-usb-redir --disable-vde --disable-vdi --disable-vhdx --disable-vhost-crypto --disable-vhost-kernel --disable-vhost-net --disable-vhost-user --disable-vhost-vdpa --disable-virglrenderer --disable-virtfs --disable-vmdk --disable-vmnet --disable-vnc --disable-vnc-jpeg --disable-vnc-sasl --disable-vpc --disable-vte --disable-vvfat --disable-whpx --disable-xen --disable-xkbcommon --disable-zstd --disable-system --disable-user --disable-linux-user --disable-bsd-user --disable-pie --disable-debug-tcg --disable-werror --disable-alsa --disable-debug-info --enable-tcg --enable-system --enable-gcrypt

Build:

.. code-block:: shell

  ninja -C build

======


===========
QEMU README
===========

QEMU is a generic and open source machine & userspace emulator and
virtualizer.

QEMU is capable of emulating a complete machine in software without any
need for hardware virtualization support. By using dynamic translation,
it achieves very good performance. QEMU can also integrate with the Xen
and KVM hypervisors to provide emulated hardware while allowing the
hypervisor to manage the CPU. With hypervisor support, QEMU can achieve
near native performance for CPUs. When QEMU emulates CPUs directly it is
capable of running operating systems made for one machine (e.g. an ARMv7
board) on a different machine (e.g. an x86_64 PC board).

QEMU is also capable of providing userspace API virtualization for Linux
and BSD kernel interfaces. This allows binaries compiled against one
architecture ABI (e.g. the Linux PPC64 ABI) to be run on a host using a
different architecture ABI (e.g. the Linux x86_64 ABI). This does not
involve any hardware emulation, simply CPU and syscall emulation.

QEMU aims to fit into a variety of use cases. It can be invoked directly
by users wishing to have full control over its behaviour and settings.
It also aims to facilitate integration into higher level management
layers, by providing a stable command line interface and monitor API.
It is commonly invoked indirectly via the libvirt library when using
open source applications such as oVirt, OpenStack and virt-manager.

QEMU as a whole is released under the GNU General Public License,
version 2. For full licensing details, consult the LICENSE file.


Documentation
=============

Documentation can be found hosted online at
`<https://www.qemu.org/documentation/>`_. The documentation for the
current development version that is available at
`<https://www.qemu.org/docs/master/>`_ is generated from the ``docs/``
folder in the source tree, and is built by `Sphinx
<https://www.sphinx-doc.org/en/master/>`_.

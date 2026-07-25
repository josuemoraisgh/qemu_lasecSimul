.. _Xtensa-System-emulator:

Xtensa System emulator
----------------------

Two executables cover simulation of both Xtensa endian options,
``qemu-system-xtensa`` and ``qemu-system-xtensaeb``. Two different
machine types are emulated:

-  Xtensa emulator pseudo board \"sim\"

-  Avnet LX60/LX110/LX200 board

The sim pseudo board emulation provides an environment similar to one
provided by the proprietary Tensilica ISS. It supports:

-  A range of Xtensa CPUs, default is the DC232B

-  Console and filesystem access via semihosting calls

The Avnet LX60/LX110/LX200 emulation supports:

-  A range of Xtensa CPUs, default is the DC232B

-  16550 UART

-  OpenCores 10/100 Mbps Ethernet MAC

ESP32 machines
--------------

The ``esp32`` and ``esp32-simul`` machines provide two network frontends.  A
machine can use either frontend or both at the same time:

``open_eth``
  OpenCores Ethernet MAC intended for firmware built with the ESP-IDF
  ``CONFIG_ETH_USE_OPENETH`` option.  The guest keeps its normal ``esp_netif``
  and lwIP stack.  This is the simplest frontend for testing DHCP, DNS,
  TCP/UDP and application protocols without emulating a physical Ethernet
  PHY connection.

``esp32_wifi``
  ESP32 Wi-Fi controller and virtual 802.11 access point.  It is used by
  firmware that calls the ESP-IDF Wi-Fi APIs.

Both frontends connect to ordinary QEMU network backends.  For example,
OpenETH with unprivileged user-mode networking can be started with::

  qemu-system-xtensa \
    -machine esp32 \
    -drive file=flash.bin,if=mtd,format=raw \
    -nic user,model=open_eth,net=192.168.4.0/24

The default libslirp layout for that subnet assigns the guest an address near
``192.168.4.15`` and provides the gateway and DNS relay inside QEMU.  Firmware
must use DHCP instead of assuming that exact address.

To expose a TCP server running on guest port 80 only to the local computer,
add an explicit loopback-bound forwarding rule::

  -nic user,model=open_eth,net=192.168.4.0/24,\
        hostfwd=tcp:127.0.0.1:8180-:80

The same kind of backend can be used with the Wi-Fi model::

  -nic user,model=esp32_wifi,net=192.168.4.0/24

User-mode networking and port forwarding require QEMU to be built with
libslirp.  They do not require a TAP adapter or administrator privileges.
Use ``-netdev stream`` when Ethernet frames must be handled by a separate
gateway process instead of libslirp.

Ethernet and Wi-Fi can coexist as separate ``esp_netif`` interfaces.  Give
each frontend a separate backend and subnet when both use libslirp::

  -nic user,model=open_eth,net=192.168.4.0/24 \
  -nic user,model=esp32_wifi,net=192.168.5.0/24

Do not configure the same frontend more than once: the ESP32 machine contains
one OpenETH controller and one Wi-Fi controller.

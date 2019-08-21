How to use and configure on nanopi
==================================

executable
----------
change paths and user name according to your setup in `bin/start_pyueye_pprzlink`

autorun
-------
copy or merge the content of `etc/rc.local` to the system file `/etc/rc.local`

extra configurations
--------------------
- start blue led blinking by adding to `/etc/rc.local`:
  ```
  echo 1 >/sys/class/leds/blue_led/brightness
  echo heartbeat >/sys/class/leds/blue_led/trigger
  ```
  adjust path according to your distro

- enable USB ethernet gadget:
  ```
  rmmod g_serial (if needed)
  modprobe g_ether
  ```
  or for default, add or replace `g_ether` in `/etc/modules` file
  then set usb0 network interface by adding in `/etc/network/interfaces`

  ```
  allow-hotplug usb0
  iface usb0 inet static
  hwaddress ether 02:78:18:6c:7c:ce
  address 192.168.0.1 (change according to your need)
  netmask 255.255.255.0
  ```

- install dhcp server
  prefer `isc-dhcp` compare to `udhcp` so you don't drag busybox
  ```
  apt install isc-dhcp-server
  ```

  then configure the files `/etc/dhcp/dhcpd.conf` with at leastsomething like:
  ```
  subnet 192.168.1.0 netmask 255.255.255.0 {
      range 192.168.0.2 192.168.0.254;
  } 
  ```

  finaly, configure the interface `usb0` in the file `/etc/default/isc-dhcp-server`


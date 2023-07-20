How to use and configure on Nanopi Air
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


How to use and configure on Nanopi Neo3 (DietPi OS)
======================================
DietPi OS : https://dietpi.com/#download

dependencies
-------

```
sudo apt install python-all-dev libexiv2-dev libboost-python-dev python3-opencv
sudo pip install exiv2 py3exiv2 pyueye
```

You also need the camera drivers. For IDS UI-1251-LE : https://fr.ids-imaging.com/download-details/AB00333.html?os=linux_arm&version=v8&bus=64&floatcalc=hard

manual launch
-------

The `pyueye_pprzlink.py` script must be run as root if you want to save images to the usb device. For example, for serial communication and usb saving:

```
  export PPRZLINK_DIR="/path/pprzlink/lib/v2.0/python"
  sudo --preserve-env=PPRZLINK_DIR pyueye_pprzlink.py -v -s serial -u /media/usb
```

autorun
-------

**Warning:** By default, these services will mount the /dev/sda1 device on /media/usb, where images will be saved.

Change paths and user name according to your setup in `etc/pyueye-pprzlink.service` and `etc/usb-automount.service`. Images will be saved in the working directory defined in pyueye-pprzlink.service (default : /home/pprz/imav/images)

Add the files `etc/pyueye-pprzlink.service` and `etc/usb-automount.service` to the system folder `/etc/systemd/system/`

Enable and start both services :
```
  sudo systemctl enable usb-automount.service
  sudo systemctl start usb-automount.service
  sudo systemctl enable pyueye-pprzlink.service
  sudo systemctl start pyueye-pprzlink.service
```

You can check service status with:
```
sudo journalctl <service_name>
```

serial
----------
With DietPi OS, you have to use the debug serial port. To do this, you need to disable the login console on this port (/dev/ttyS2) by using dietpi-config:
```
  sudo dietpi-config
```

usb
---------
With DietPi OS by default, only one USB pinheader port works with the ueye camera (the other one is set as "otg" in the device-tree, instead of "host")
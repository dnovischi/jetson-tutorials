# Jetson Nano Wi-Fi USB Dongle Installation

This tutorial details how to install wi-fi dongles on the Jetson Nano with which I had success.
The steps discussed where tested with Ubuntu 18.04 and Ubuntu 20.04.


## TP-Link AC600 and AC1300 Adapter Install

1. Plug-in the AC600 or AC1300 adapter into one of the Jetson Nano USB ports. If your using an Jetson Nano A02, you can plug the AC600 into an USB2.0 port.

2. You can check if the dongle was detected via:

```bash
lsusb
```

3. Install prerequisites:

```bash
sudo apt install git dkms net-tools
```

4. Clone the driver repository and navigate into it:

```bash
git clone -b v5.6.4.2 https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au
```

5. Build and install the driver:

```bash
sudo make dkms_install
```

6. Reboot the board:

```bash
 sudo reboot
```

7. Use the network manager command line tool to see if the Wi-Fi is enabled or not:

```bash
sudo nmcli radio wifi
```
8. If the output shows that the Wi-Fi is disabled, you can enable it with the following command:

```bash
sudo nmcli radio wifi on
```

9. To list the available Wi-Fi access points by:

```bash
sudo nmcli dev wifi list
```

10. You can connect to a particular acess point by:

```bash
sudo nmcli dev wifi connect network-ssid password network-password
```

## Notes

Managing connection can also be accomplished via `nmtui` which provides a text based ui.

## References

1. [Driver Installation for NVIDIA® Jetson™ Modules](https://www.forecr.io/blogs/connectivity/tp-link-ac600-ac1300-archer-t2u-t3u-driver-installation-for-jetson-modules)
2. [How to Connect to Wi-Fi Through the Linux Terminal](https://www.makeuseof.com/connect-to-wifi-with-nmcli/)

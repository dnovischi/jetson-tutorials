# Jetson Nano Ubuntu 18.04 Full Install

Its assumed that the setup will done via an Ubuntu enabled PC with access to root privileges. The Jetson Nano setup will proceed in headless mode. Also, I recommend using an endurance SD Card UHS U3, V30 or higher, Class 10.


1. The official intro to the board can be found [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro).
2. To power the Jetson Nano get a power supply that can delliver (at least) 3A at 5V that has:
  - 2.1 Barel Jack output for Jetson Nano B01 or higher
  - USB-C Type output for Jetson Nano A02
3. To configure the Jetson Nano to be powered via the 2.1 barel jack connector you'll need to have a jumper.
4. To connect to the Jetson Nano in headless mode you will need a micro USB cable.
5. Download the latest official Nvidia image from [here](https://developer.nvidia.com/jetson-nano-sd-card-image)
6. Install Balena Etcher on your Ubuntu PC by following the instructions [here](https://github.com/balena-io/etcher#debian-and-ubuntu-based-package-repository-gnulinux-x86x64)
7. Plug-in your micro SD Card into the PC
8. Fire up Etcher, chose the image you downloaded in step 4 and select your micro SD Card as the target (e.g. /dev/mmcblk0).
9. Click Flash, enter the root password and let Etcher do its thing.
10. Unmount the SD Card from the PC.
11. To check if the SD Card is properly umounted do the following:

```bash
$ lsblk
NAME         MAJ:MIN RM   SIZE RO TYPE MOUNTPOINT
mmcblk0      179:0    0  59.5G  0 disk
├─mmcblk0p1  179:1    0  59.5G  0 part
├─mmcblk0p2  179:2    0   128K  0 part
├─mmcblk0p3  179:3    0   448K  0 part
├─mmcblk0p4  179:4    0   576K  0 part
├─mmcblk0p5  179:5    0    64K  0 part
├─mmcblk0p6  179:6    0   192K  0 part
├─mmcblk0p7  179:7    0   384K  0 part
├─mmcblk0p8  259:8    0    64K  0 part
├─mmcblk0p9  259:9    0   448K  0 part
├─mmcblk0p10 259:10   0   448K  0 part
├─mmcblk0p11 259:11   0   768K  0 part
├─mmcblk0p12 259:12   0    64K  0 part
├─mmcblk0p13 259:13   0   192K  0 part
└─mmcblk0p14 259:14   0   128K  0 part
nvme0n1      259:0    0 931.5G  0 disk
├─nvme0n1p1  259:1    0   100M  0 part /boot/efi
├─nvme0n1p2  259:2    0    16M  0 part
├─nvme0n1p3  259:3    0 233.8G  0 part
├─nvme0n1p4  259:4    0   499M  0 part
├─nvme0n1p5  259:5    0  93.1G  0 part /
├─nvme0n1p6  259:6    0  14.9G  0 part [SWAP]
└─nvme0n1p7  259:7    0 589.1G  0 part /home
```
The listing under your SD Card associated device should not have any mount points associated with it. In my case the `mmcblk0` does not show any mount points for any of the partitions (p1 through p14).

12. Un-plugh the SD Card from the PC and plug it in the Jetson Nano. The Nano should be powered off when you do this.
13. Plug-in the jumper in the J48 port (located above the barel jack connector - on its right - for Jetson Nano B01 or below the camera connector for Jetson Nano A02).
14. Plug-in the micro USB cable in the Jetson Nano and the PC
15. Plug-in the USB C or 2.1 barel jack connector.
16. Plug-in the power supply into the mains.
17. At this point the nano will boot up and it will take from 1 to 5 minutes to display something.
18. Install the `screen` utility on the PC:

```bash
  sudo apt install screen
```
19. Make sure your PC user has tty access privileges:

```bash
  sudo usermod -a -G dialout $USER
```
20. Logout and then login for the changes to take effect (Yes, you can also reboot with prefer that).
21. Login into the Jetson Nano to do the initial configuration via screen utility:

```bash
  screen /dev/ttyACM0 115200
```
22. Press Space, then Enter and wait for the text menu to appear. If it does not appear after 5 minutes un-plug the power form Jetson Nano. Wait 30 seconds and re-plug the Jetson Nano power source and repeat step 21. If the trouble persits re-flash the SD Card and repeat from step 12.
23. Press "Ok" in the System Configuration menu that appears (you can navigate with TAB, select with ENTER):

```bash
System Configuration
  ┌─────────────┤ License For Customer Use of NVIDIA Software ├──────────────┐
  │                                                                          │
  │ License For Customer Use of NVIDIA Software
  │
  │ IMPORTANT NOTICE -- READ CAREFULLY: This License For Customer Use of
  │ NVIDIA Software ("LICENSE") is the agreement which governs use of the
  │ software of NVIDIA Corporation and its subsidiaries ("NVIDIA")
  │ downloadable herefrom, including computer software and associated
  │ printed materials ("SOFTWARE").  By downloading, installing, copying,
  │ or otherwise using the SOFTWARE, you agree to be bound by the terms of
  │ this LICENSE.  If you do not agree to the terms of this LICENSE, do not
  │ download the SOFTWARE.
  │
  │ RECITALS
  │
  │ Use of NVIDIAs products requires three elements: the SOFTWARE, the
  │ hardware on a graphics controller board, and a personal computer. The
  │
  │                                  <Ok>
  │                                                                          │
  └──────────────────────────────────────────────────────────────────────────┘
```

24. Select your language (I'm going to leave it as English for now) and hit "Ok":

``` bash
System Configuration
   ┌─────────────────────────┤ Select a language ├──────────────────────────┐
   │ Choose the language to be used for the installation process. The       │
   │ selected language will also be the default language for the installed  │
   │ system.                                                                │
   │                                                                        │
   │ Language:                                                              │
   │                                                                        │
   │              Arabic - ﻲﺑﺮﻋ                                 ↑           │
   │              Asturian - Asturianu                          ▒           │
   │              Basque - Euskara                              ▮           │
   │              Belarusian - Беларуская                       ▒           │
   │              Bosnian - Bosanski                            ▒           │
   │              Bulgarian - Български                         ▒           │
   │              Danish - Dansk                                ▒           │
   │              Dutch - Nederlands                            ▒           │
   │              English - English                             ↓           │
   │                                                                        │
   │                                                                        │
   │                   <Ok>                       <Cancel>                  │
   │                                                                        │
   └────────────────────────────────────────────────────────────────────────┘
```
25. Select your correct area (for other software) to work properly and hit "OK":

```bash
System Configuration
  ┌────────────────────────┤ Select your location ├─────────────────────────┐
  │ The selected location will be used to set your time zone and also for   │
  │ example to help select the system locale. Normally this should be the   │
  │ country where you live.                                                 │
  │                                                                         │
  │ Listed are locations for: Europe. Use the <Go Back> option to select a  │
  │ different continent or region if your location is not listed.           │
  │                                                                         │
  │ Country, territory or area:                                             │
  │                                                                         │
  │                    Romania                           ↑                  │
  │                    Russian Federation                ▒                  │
  │                    San Marino                        ▒                  │
  │                    Serbia                            ▮                  │
  │                    Slovakia                          ▒                  │
  │                    Slovenia                          ↓                  │
  │                                                                         │
  │                                                                         │
  │                   <Ok>                       <Cancel>                   │
  │                                                                         │
  └─────────────────────────────────────────────────────────────────────────┘
```

26. Configure your locale (I'm going to levave it to the default en_US.UTF-8) and hit "OK":

```bash
System Configuration
  ┌──────────────────────────┤ Configure locales ├──────────────────────────┐
  │ There is no locale defined for the combination of language and country  │
  │ you have selected. You can now select your preference from the locales  │
  │ available for the selected language. The locale that will be used is    │
  │ listed in the second column.                                            │
  │                                                                         │
  │ Country to base default locale settings on:                             │
  │                                                                         │
  │                     New Zealand - en_NZ.UTF-8        ↑                  │
  │                     Nigeria - en_NG                  ▒                  │
  │                     Philippines - en_PH.UTF-8        ▒                  │
  │                     Seychelles - en_SC.UTF-8         ▒                  │
  │                     Singapore - en_SG.UTF-8          ▒                  │
  │                     South Africa - en_ZA.UTF-8       ▮                  │
  │                     United Kingdom - en_GB.UTF-8     ▒                  │
  │                     United States - en_US.UTF-8      ↓                  │
  │                                                                         │
  │                                                                         │
  │                   <Ok>                       <Cancel>                   │
  │                                                                         │
  └─────────────────────────────────────────────────────────────────────────┘
```
27. Hit yes for the UTC clock:

```bash
   ┌───────────────────────────┤ Where are you? ├───────────────────────────┐
   │                                                                        │
   │ System clocks are generally set to Coordinated Universal Time (UTC).   │
   │ The operating system uses your time zone to convert system time into   │
   │ local time. This is recommended unless you also use another operating  │
   │ system that expects the clock to be set to local time.                 │
   │                                                                        │
   │ Is the system clock set to UTC?                                        │
   │                                                                        │
   │                   <Yes>                      <No>                      │
   │                                                                        │
   └────────────────────────────────────────────────────────────────────────┘
```
28. Enter your name or the name you want associated with the user we will create and hit "Ok":

```bash
  ┌─────────────────────────────┤ Who are you? ├─────────────────────────────┐
  │ A user account will be created for you to use instead of the root        │
  │ account for non-administrative activities.                               │
  │                                                                          │
  │ Please enter the real name of this user. This information will be used   │
  │ for instance as default origin for emails sent by this user as well as   │
  │ any program which displays or uses the user's real name. Your full name  │
  │ is a reasonable choice.                                                  │
  │                                                                          │
  │ Full name for the new user:                                              │
  │                                                                          │
  │ ________________________________________________________________________ │
  │                                                                          │
  │                   <Ok>                       <Cancel>                    │
  │                                                                          │
  └──────────────────────────────────────────────────────────────────────────┘
```

29. Enter the user name you'll be working with for the Nano and hit "Ok":

```bash
  ┌────────────────────────────┤ Who are you? ├─────────────────────────────┐
  │ Select a username for the new account. Your first name is a reasonable  │
  │ choice. The username should start with a lower-case letter, which can   │
  │ be followed by any combination of numbers and more lower-case letters.  │
  │                                                                         │
  │ Username for your account:                                              │
  │                                                                         │
  │ jetson_________________________________________________________________ │
  │                                                                         │
  │                   <Ok>                       <Cancel>                   │
  │                                                                         │
  └─────────────────────────────────────────────────────────────────────────┘
```

30. Enter the password you want for the user you created (and remember it, your going to need it):

```bash
┌────────────────────────┤ Who are you? ├─────────────────────────┐
│ A good password will contain a mixture of letters, numbers and  │
│ punctuation and should be changed at regular intervals.         │
│                                                                 │
│ Choose a password for the new user:                             │
│                                                                 │
│ _______________________________________________________________ │
│                                                                 │
│                <Ok>                    <Cancel>                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```
31. Re-enter the password and hit "Ok":

```bash
┌────────────────────────────┤ Who are you? ├────────────────────────────┐
│ Please enter the same user password again to verify you have typed it  │
│ correctly.                                                             │
│                                                                        │
│ Re-enter password to verify:                                           │
│                                                                        │
│ ******________________________________________________________________ │
│                                                                        │
│                   <Ok>                       <Cancel>                  │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```
32. If you entered a weak password and still want to use it, hit "Yes" in the following menu:

```bash
   ┌───────────────────────────┤ Who are you? ├────────────────────────────┐
   │                                                                       │
   │ You entered a password that consists of less than eight characters,   │
   │ which is considered too weak. You should choose a stronger password.  │
   │                                                                       │
   │ Use weak password?                                                    │
   │                                                                       │
   │                   <Yes>                      <No>                     │
   │                                                                       │
   └───────────────────────────────────────────────────────────────────────┘
```
33. Delete the default number and hit "Ok":

```bash
  ┌─────────────────────────┤ APP Partition Size ├──────────────────────────┐
  │                                                                         │
  │                                                                         │
  │ Please enter desired size of APP partition in Megabytes (MB). Default   │
  │ value in input field is the maximum size that can be accepted. Enter 0  │
  │ or leave blank to use the maximum size value.                           │
  │                                                                         │
  │ _______________________________________________________________________ │
  │                                                                         │
  │                   <Ok>                       <Cancel>                   │
  │                                                                         │
  └─────────────────────────────────────────────────────────────────────────┘
```
34. Hit "Ok" for the SWAP creation:

```bash

  ┌──────────────────────────┤ Create SWAP File ├───────────────────────────┐
  │                                                                         │
  │ It is recommended to create a 4GB disk SWAP file if you intent to use   │
  │ the device for AI and Deep Learning. For example, training under        │
  │ PyTorch using the GPU. Please note that having a SWAP file may shorten  │
  │ life of SDCARD due to increased writes to the medium. You can manually  │
  │ enable SWAP at a later time by following "Enable Disk SWAP" section in  │
  │ L4T Developer Guide.                                                    │
  │                                                                         │
  │                                 <Ok>                                    │
  │                                                                         │
  └─────────────────────────────────────────────────────────────────────────┘
```
35. Hit "Yes" for the Create SWAP File:

```bash
┌───────┤ Create SWAP File ├───────┐
│                                  │
│                                  │
│                                  │
│ Create SWAP File (Recommended)?  │
│                                  │
│       <Yes>          <No>        │
│                                  │
└──────────────────────────────────┘
```
36. For the primiary network interface select wlan0 (recommend). If the wi-fi adapter is not available, select the ethernet port `eth0` and connect your Nano via an ethernet cable to your router.
As a last resort you can select `usb0`, but please note that I had trouble with this even when following the guides provided with the image:

```bash
┌────────────────────────┤ Network configuration ├─────────────────────────┐
│ Your system has multiple network interfaces. Choose the one to use as    │
│ the primary network interface during the installation. If possible, the  │
│ first connected network interface found has been selected.               │
│                                                                          │
│ Primary network interface:                                               │
│                                                                          │
│                        dummy0: Unknown interface                         │
│                        eth0: Ethernet                                    │
│                        l4tbr0: Unknown interface                         │
│                        rndis0: Unknown interface                         │
│                        usb0: USB net                                     │
│                                                                          │
│                                                                          │
│                   <Ok>                       <Cancel>                    │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```
37. If you encouter the following messge and hit "Ok":

```bash
   ┌───────────────────────┤ Network configuration ├───────────────────────┐
   │                                                                       │
   │ Network autoconfiguration failed                                      │
   │                                                                       │
   │ Your network is probably not using the DHCP protocol. Alternatively,  │
   │ the DHCP server may be slow or some network hardware is not working   │
   │ properly.                                                             │
   │                                                                       │
   │                                <Ok>                                   │
   │                                                                       │
   └───────────────────────────────────────────────────────────────────────┘
```

38. If you selected `usb0` to succeed in the network autoconfiguration you'll need to go to your PC Settings->Network->Profile Settings icon and configure the ipv4 and ipv6 to the "Shared to other computers" option. The go back to the nano and hit "Retry network autoconfiguration".
If not you can restart the tutorial and use an ehternet cable connected to your router.

39. Next configure an unique name for the localhost (especially if you're goinging to use multiple nano's) or have other devices on your network:

```bash
┌────────────────────────┤ Network configuration ├─────────────────────────┐
│ Please enter the hostname for this system.                               │
│                                                                          │
│ The hostname is a single word that identifies your system to the         │
│ network. If you don't know what your hostname should be, consult your    │
│ network administrator. If you are setting up your own home network, you  │
│ can make something up here.                                              │
│                                                                          │
│ Hostname:                                                                │
│                                                                          │
│ localhost_______________________________________________________________ │
│                                                                          │
│                   <Ok>                       <Cancel>                    │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```
40. Select the "MAXN" option and hit "Ok" for the Nvpmodel Mode:

```bash
┌────────────────────────┤ Select Nvpmodel Mode ├─────────────────────────┐
│ If you are unsure which mode to select, keep the default setting. This  │
│ setting can be changed at runtime using the nvpmodel GUI or nvpmodel    │
│ command line utility. Refer to NVIDIA Jetson Linux Developer Guide for  │
│ further information.                                                    │
│                                                                         │
│ Select from one of the below nvpmodel modes:                            │
│                                                                         │
│                                  MAXN                                   │
│                                  5W                                     │
│                                                                         │
│                                                                         │
│                   <Ok>                       <Cancel>                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

41. Let the system do it's thing and do not un-plug the Nano. The nano will reboot it-self when it finishes and you can try to ssh into it by:

```bash
ssh jetson@nano.local
```

If the following message will appear. Enter yes and hit "ENTER:

```bash
The authenticity of host 'nano.local (192.168.55.1)' can't be established.
ECDSA key fingerprint is SHA256:dMwXGkH378pmX8ad9BN84QyaKI+ypc+Q3oFFLOYtxFk.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
```

42. Enter the password you have set for the Nano and hit "Ok"

```bash
jetson@nano.local's password:
```

43. Your now login into the Nano, but we still have to do some upgrading to make it fully functional

```bash
Welcome to Ubuntu 18.04.6 LTS (GNU/Linux 4.9.253-tegra aarch64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage
This system has been minimized by removing packages and content that are
not required on a system that users do not log into.

To restore this content, you can run the 'unminimize' command.

0 updates can be applied immediately.


The programs included with the Ubuntu system are free software;
the exact distribution terms for each program are described in the
individual files in /usr/share/doc/*/copyright.

Ubuntu comes with ABSOLUTELY NO WARRANTY, to the extent permitted by
applicable law.

To run a command as administrator (user "root"), use "sudo <command>".
See "man sudo_root" for details.
```
44. First, install your ssh public key such that we can work easier with the Nano:

```bash
ssh-copy-id -i jetson@nano.local
```
45. Enter the your Nano password and hit "ENTER"
46. Now ssh into you Jetson Nano:

```bash
ssh jetson@nano.local
```
47. Update the package manager list:

```bash
sudo apt update
```
48. Do a full upgrade:

```bash
sudo apt upgrade -y
```
49. If the upgrade ends with an error similar to the one below, try `sudo apt install --fix-broken`.

```bash
dpkg: error processing package nvidia-l4t-bootloader (--configure):
 installed nvidia-l4t-bootloader package post-installation script subprocess returned error exit status 1
Errors were encountered while processing:
 nvidia-l4t-bootloader
E: Sub-process /usr/bin/dpkg returned an error code (1)

```
50. If that doesn't fix the problem do the following:

```bash
sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/backup/ # backup /var/lib/info/
sudo mkdir /var/lib/dpkg/info/ #create new /var/lib/dpkg/info
sudo apt-get update # Update the source list
sudo apt-get -f install # Force install to correct the problem
sudo mv /var/lib/dpkg/info/* /var/lib/dpkg/backup/ #Backup the new structure dpkg/info to old info
sudo rm -rf /var/lib/dpkg/info # Remove the new dpkg structure folder
sudo mv /var/lib/dpkg/backup/ /var/lib/dpkg/info/ #Back with the old
```

51. Install `tensorrt` if left not upgraded:

```bash
sudo apt install tensorrt
```

52. Install the latest jetpack:

```bash
sudo apt install nvidia-jetpack
```

53. Install package manager utilities:

```bash
sudo apt install apt-utils
```
54. Some software packages, especially the CUDA software, requires a gcc, g++ and clang version 8:

```bash
sudo apt install gcc-8 g++-8 clang-8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-8 8
```
55. For ROS installs and other python3 packages install python3 pip:

```bash
sudo apt install python3-pip
```

56. To use `jtop` install the jetson-stats package:

```bash
sudo -H pyhton3 -m pip install jetson-stats
```

57. If you plan to do a distribution upgrade make sure you un-install the following:

```bash
 sudo apt remove --purge chromium-browser chromium-browser-l10n libreoffice-* rhythmbox*
```
58. Remove no longer required packages:

```bash
sudo apt autoremove
```

59. Finally, reboot:

```bash
sudo reboot
```

60. Enjoy.

## Optional Fan Control

1. If you installed a fan on your Jetson Nano and you would like a deamon to manage it, get the jetson-fan-ctl utility:

```bash
git clone https://github.com/Pyrestone/jetson-fan-ctl.git
```

2. Navigate to the repo foloder:

```bash
cd jetson-fan-ctl/
```
3. Install the uitlity:

```bash
sudo ./install.sh
```
4. See the github page for customization information [here](https://github.com/Pyrestone/jetson-fan-ctl)

# Jetson Nano Ubuntu 20.04 Install Through Distribution Upgrade

This tutorial details the steps necesarry to upgrade the official Ubuntu 18.04 Jetson Nano image to Ubuntu 20.04. This setup assumes you already have a clean Ubuntu 18.04 installation that is fully updated. If you don't, please follow the  [Jetson Nano Ubuntu 18.04 Full Install](jetson-nano-ubuntu-18-04-install.md).

Please note that NO librealsense SDK, ROS, Pytorch, TensorFlow or any other framework should have been previously installed. This is because all frameworks have many dependencies which can't be satisified during the distribution upgrade. In all likelihood, you'll end up with a broken OS.

1. First, let's remove some of the packages that cause problems:

```bash
sudo apt-get remove --purge chromium-browser chromium-browser-l10n
```

2. Update and upgrade:

```bash
sudo apt-get update;sudo apt-get upgrade -y
```

3. If durring the upgrade you encounter an error about the nvidia-l4t-bootlader do the following:
```bash
sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/backup/ #move /var/lib/info/`
sudo mkdir /var/lib/dpkg/info/ #create new /var/lib/dpkg/info
sudo apt-get update #update the source list
sudo apt-get -f install #force install to correct the problem
sudo mv /var/lib/dpkg/info/* /var/lib/dpkg/backup/ #Move the new structure dpkg/info to old info
sudo rm -rf /var/lib/dpkg/info #remove the new dpkg structure folder
sudo mv /var/lib/dpkg/backup/ /var/lib/dpkg/info/ #back the old
```

4. Remove unsed packages:
```bash
sudo apt-get autoremove
```

5. If your not confortable with vim, please install nano:

```bash
sudo apt install nano
```

6. You neeed to enable the distribution upgrades for the update manager:

```bash
sudo nano /etc/update-manager/release-upgrades
```

Change the line that states `Prompt=never` to `Prompt=normal`, then save and exit.

7. Update the package manager list:

```bash
sudo apt-get update
```

8. Do a distribution package upgrade:

```bash
sudo apt-get dist-upgrade
```

9. Reboot:

```bash
sudo reboot
```

10. The ssh console might become unavailable durring the upgrade process. Since some user input will be required, to make sure you stay connected through the process use the Jetson Nano serial port:

```bash
screen /dev/ttyACM0 115200
```

loggin using the Jetson Nano user and password.


11. Plese note that the upgrade to Ubuntu 20.04 will last for several hours. During the upgrade check your screen now and then, and answer all the the questions with the default value.

```bash
sudo do-release-upgrade
```

12. When the upgrade finishes and the installation asks for a reboot, enter "N" (!!!DO NOT REBOOT NOW!!!).
  - Edit the `/etc/gdm3/custom.conf` file and check that  `WaylandEnable=false`
  ```bash
  sudo nano /etc/gdm3/custom.conf
  ```
  - Edit the `/etc/X11/xorg.conf` file and uncomment the line `# Driver "nividia"`:
  ```bash
  sudo nano /etc/X11/xorg.conf
  ```
  - Edit `/etc/update-manager/release-upgrades` and change the `Prompt` back to `never`, i.e. `Prompt=never`:
  ```bash
  sudo nano /etc/update-manager/release-upgrades
  ```
  - Reboot:
  ```bash
  sudo reboot
  ```
13. Next, delete the directory /usr/share/vulkan/icd.d to prevent lavapipe warnings when using Jtop:

```bash
sudo rm -rf /usr/share/vulkan/icd.d
```

14. Remove also an annoying circular symbolic link in `/usr/share/applications`:

```bash
 sudo rm /usr/share/applications/vpi1_demos
```

15. Remove distorted logo:

```bash
cd /usr/share/nvpmodel_indicator
sudo mv nv_logo.svg no_logo.svg
```

16. Update the packages:

```bash
sudo apt-get update
```

17. Upgrade the packages:

```bash
sudo apt-get upgrade
```

18. Do a cleanup:

```bash
sudo apt-get autoremove
```

19. Re-enable repositories automatically deactivated during the distribution upgrade, i.e. remove the `#` in front:

```bash
cd /etc/apt/sources.list.d/
sudo nano cuda-l4t-10-2-local.list
sudo nano nvidia-l4t-apt-source.list
sudo nano visionworks-repo.list
sudo nano visionworks-sfm-repo.list
sudo nano visionworks-tracking-repo.list
```

20. Some software packages, especially the CUDA software, requires a gcc, g++ and clang version 8. However verion 8 and 9 of these toolchains have some bugs relating to the optimization options for arm achitectures (i.e. aarch64) like found in rpi and jetson nano. So, to get around various diffrent problems when compilation is required, all toolchains must be available:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-8 g++-8 clang-8 gcc-10 gcc-11 g++-10 g++-11 clang-10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-8 8
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 10
```

## Troubleshooting

### SSH via Wi-Fi Slow or Not Working
1. Turn off the Wi-Fi power saving mode:

```bash
iw dev wlan0 set power_save off
```
2. Make sure the firewall allows port 22 used by ssh:

```bash
sudo apt install ufw
sudo ufw allow ssh
sudo ufw allow 22
sudo ufw enable
sudo ufw status
```

### Apt Package Manager

1. If you run into problems with the package manager do:

```bash
sudo apt --fix-broken install
```

2. If that fails you can force an overwrite for the package:

```bash
sudo dpkg -i --force-overwrite /var/cache/apt/archives/<name_of_the_package>.deb
```

3. Then fix broken:

```bash
sudo apt --fix-broken install
```

4. To check all when well, try to update and upgrade again:

```bash
sudo apt-get update
sudo apt-get upgrade
```

## References

1. [Ubuntu 20.04 on Nano?](https://forums.developer.nvidia.com/t/ubuntu-20-04-on-nano/125451)
2. [Install Ubuntu 20.04 on Jetson Nano](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html)
3. [ROSKY2 - setup operating environment with Jetson Nano Developer 4G kit](https://hackmd.io/@weichih-lin/ROSKY2_setup_environment_jetson_nano_developer_kit)

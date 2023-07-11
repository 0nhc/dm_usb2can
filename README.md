# 1. 设置UDEV Rules
```sh
sudo cp dmbot_usb2can.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
```
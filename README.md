# Bumperbot Setup and Bringup
Below are the steps to setup bumperbot, then bringup the robot.
## 1. Preparation 
### Setup Wifi
Edit the 50-cloud-init.yaml, and enter below lines
```
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                ros_public:
                    password: 9fea575b6a0d4668c666a4b111d7784ca062496a4570715e0d5120714b9b3f90
                SSID:
                    password: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
            dhcp4: true
            optional: true

```
### Enable SSH
```
sudo nano /etc/ssh/sshd_config.d/50-cloud-init.conf
```
```
PasswordAuthentication yes
```

### Disable Auto Update
```
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```
```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```
### Disable Network Sleep, Suspend, Hibernate
```
sudo systemctl mask systemd-networkd-wait-online.service
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```
## 2. Install ROS and download Bumperbot package
```
git clone http://github.com/twming/babybot
```
```
cd babybot
sudo chmod 755 ros2-humble-base-main.sh
```
```
./ros2-humble-base-main.sh
```
## 3. Colcon build Bumperbot
```
colcon build
```
## 4. Setup Bumperbot environment
```
echo "source ~/install/setup.bash" >> ~/.bashrc
```
## 5. Bringup Bumperbot
```
ros2 launch bumperbot_bringup real_robot.launch.py
```

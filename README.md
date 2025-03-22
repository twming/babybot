# Bumperbot Setup
Below are the steps to setup bumperbot
## 1. Preparation 
### Enable SSH
```
sudo nano /etc/ssh/sshd_config
```
```
IPQoS cs0 cs0
```
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

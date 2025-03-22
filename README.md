# Bumperbot Setup
Below are the steps to setup bumperbot

### Enable SSH
```
sudo nano /etc/ssh/sshd_config
```
```
IPQoS cs0 cs0
```
```
sudo nano /etc/ssh/sshd_config.d/60-cloudimg-settings.conf
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

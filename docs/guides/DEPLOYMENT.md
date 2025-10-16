# 🚀 Guide de Déploiement

**Ucaotech DOFbot TRC2025 - Déploiement Jetson Nano Orin**

---

## 📋 Table des Matières

1. [Prérequis](#prérequis)
2. [Préparation Jetson Nano](#préparation-jetson-nano)
3. [Installation Système](#installation-système)
4. [Configuration Réseau](#configuration-réseau)
5. [Installation Dépendances](#installation-dépendances)
6. [Déploiement Application](#déploiement-application)
7. [Optimisation Performance](#optimisation-performance)
8. [Tests Validation](#tests-validation)
9. [Maintenance](#maintenance)
10. [Dépannage](#dépannage)

---

## 📦 Prérequis

### Matériel Requis

- ✅ **Jetson Nano Orin** (8GB RAM recommandé)
- ✅ **Carte microSD** (64GB minimum, Classe 10/U3)
- ✅ **Alimentation** 5V/4A USB-C
- ✅ **Yahboom DOFbot** 6-axis
- ✅ **Intel RealSense D435** (ou compatible)
- ✅ **Câble Ethernet** (configuration initiale)
- ✅ **Routeur WiFi** (déploiement final)

### Logiciels Requis

- **JetPack** 5.1.2+ (Ubuntu 20.04)
- **ROS Noetic**
- **Python** 3.8+
- **CUDA** 11.4+
- **cuDNN** 8.6+

---

## 🖥️ Préparation Jetson Nano

### 1. Téléchargement JetPack

```bash
# Sur ordinateur hôte
# Télécharger JetPack depuis:
# https://developer.nvidia.com/embedded/jetpack

# Ou via NVIDIA SDK Manager
sudo apt install nvidia-sdk-manager
nvidia-sdk-manager
```

### 2. Flash Carte SD

**Méthode 1: Etcher (Simple)**
```bash
# Télécharger Etcher: https://www.balena.io/etcher/
# 1. Insérer carte SD
# 2. Sélectionner image JetPack
# 3. Sélectionner carte SD
# 4. Flash!
```

**Méthode 2: dd (Avancé)**
```bash
# Identifier carte SD
lsblk

# Flash image (⚠️ ATTENTION à /dev/sdX)
sudo dd if=jetpack-image.img of=/dev/sdX bs=4M status=progress
sudo sync
```

### 3. Premier Démarrage

1. **Insérer carte SD** dans Jetson Nano
2. **Connecter**:
   - Écran HDMI
   - Clavier USB
   - Souris USB
   - Ethernet
   - Alimentation
3. **Démarrer** et suivre assistant configuration

**Configuration initiale**:
```
Nom: ucaotech
Utilisateur: ucaotech
Mot de passe: [votre_mot_de_passe_sécurisé]
Hostname: ucaotech-dofbot
Timezone: Africa/Casablanca
```

---

## ⚙️ Installation Système

### 1. Mise à Jour Système

```bash
# Mise à jour packages
sudo apt update
sudo apt upgrade -y

# Installation outils de base
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    htop \
    net-tools \
    python3-pip \
    python3-dev
```

### 2. Configuration Swap (Important!)

```bash
# Créer fichier swap 8GB
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Rendre permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Vérifier
free -h
```

### 3. Installation CUDA (si non inclus)

```bash
# Vérifier CUDA
nvcc --version

# Si absent, installer via JetPack
# Ou manuellement:
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/ /"
sudo apt update
sudo apt install -y cuda-toolkit-11-4
```

### 4. Variables d'Environnement

```bash
# Ajouter à ~/.bashrc
cat >> ~/.bashrc << 'EOF'

# CUDA
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# ROS
source /opt/ros/noetic/setup.bash
source ~/ucaotech_ws/devel/setup.bash

# Python
export PYTHONPATH=$PYTHONPATH:~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts
export PYTHONPATH=$PYTHONPATH:~/ucaotech_ws/src/ucaotech_dofbot_trc2025/src

# Performance
export TF_CPP_MIN_LOG_LEVEL=2
export CUDA_VISIBLE_DEVICES=0
EOF

# Appliquer
source ~/.bashrc
```

---

## 🌐 Configuration Réseau

### 1. Configuration WiFi

```bash
# Lister réseaux disponibles
nmcli device wifi list

# Connexion réseau
sudo nmcli device wifi connect "NOM_RESEAU" password "MOT_DE_PASSE"

# Vérifier connexion
ip addr show wlan0
ping -c 4 google.com
```

### 2. IP Statique (Recommandé)

**Méthode 1: NetworkManager**
```bash
# Configuration IP statique
sudo nmcli connection modify "NOM_CONNEXION" \
    ipv4.addresses 192.168.1.100/24 \
    ipv4.gateway 192.168.1.1 \
    ipv4.dns "8.8.8.8 8.8.4.4" \
    ipv4.method manual

# Redémarrer connexion
sudo nmcli connection down "NOM_CONNEXION"
sudo nmcli connection up "NOM_CONNEXION"
```

**Méthode 2: Fichier netplan**
```bash
# Éditer configuration
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
  wifis:
    wlan0:
      dhcp4: no
      addresses: [192.168.1.101/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "NOM_RESEAU":
          password: "MOT_DE_PASSE"
```

```bash
# Appliquer
sudo netplan apply
```

### 3. Hostname et Découverte

```bash
# Configurer hostname
sudo hostnamectl set-hostname ucaotech-dofbot

# Éditer /etc/hosts
sudo nano /etc/hosts
# Ajouter:
# 127.0.0.1    ucaotech-dofbot

# Installer avahi (mDNS)
sudo apt install -y avahi-daemon avahi-utils

# Test découverte
avahi-browse -a
```

---

## 📚 Installation Dépendances

### 1. Installation ROS Noetic

```bash
# Configuration sources ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# Clé GPG
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installation
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Dépendances ROS
sudo apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialiser rosdep
sudo rosdep init
rosdep update
```

### 2. Packages ROS Additionnels

```bash
# Packages courants
sudo apt install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-realsense2-camera \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-actionlib \
    ros-noetic-control-msgs \
    ros-noetic-trajectory-msgs
```

### 3. Installation Python Packages

```bash
# Mise à jour pip
python3 -m pip install --upgrade pip

# Packages essentiels
pip3 install \
    numpy==1.24.3 \
    opencv-python==4.8.0.74 \
    opencv-contrib-python==4.8.0.74 \
    pyrealsense2==2.54.1 \
    ultralytics==8.0.196 \
    torch==2.0.0 \
    torchvision==0.15.1 \
    rospkg \
    catkin_pkg

# Packages ML/Vision
pip3 install \
    mediapipe==0.10.3 \
    scikit-learn==1.3.0 \
    scipy==1.11.2 \
    matplotlib==3.7.2 \
    pillow==10.0.0

# Packages utilitaires
pip3 install \
    pyyaml==6.0.1 \
    websockets==12.0 \
    asyncio \
    pyserial==3.5
```

### 4. Installation Intel RealSense SDK

```bash
# Ajouter dépôt
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

# Installation
sudo apt update
sudo apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# Vérifier
realsense-viewer
```

### 5. Installation DOFbot SDK

```bash
# Cloner SDK Yahboom
cd ~/
git clone https://github.com/YahboomTechnology/Dofbot-Jetson-Nano.git

# Copier bibliothèque
sudo cp Dofbot-Jetson-Nano/Arm_Lib/Arm_Lib.py /usr/local/lib/python3.8/dist-packages/

# Configurer permissions série
sudo usermod -a -G dialout $USER
# Déconnexion/reconnexion nécessaire
```

---

## 🚀 Déploiement Application

### 1. Création Workspace ROS

```bash
# Créer workspace
mkdir -p ~/ucaotech_ws/src
cd ~/ucaotech_ws/

# Initialiser
catkin_make

# Source
source devel/setup.bash
```

### 2. Cloner Projet

```bash
cd ~/ucaotech_ws/src/

# Cloner depuis GitHub
git clone https://github.com/Badmus2005/uca.git ucaotech_dofbot_trc2025

# Ou copier depuis USB
cp -r /media/ucaotech/USB/ucaotech_dofbot_trc2025 .
```

### 3. Installation Dépendances Projet

```bash
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/

# Installer dépendances Python
pip3 install -r requirements.txt

# Installer dépendances ROS
cd ~/ucaotech_ws/
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Compilation

```bash
cd ~/ucaotech_ws/

# Compilation
catkin_make

# Ou avec options
catkin_make -DCMAKE_BUILD_TYPE=Release

# Source
source devel/setup.bash
```

### 5. Téléchargement Modèles

```bash
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/trc2025_train_models/models/

# Télécharger modèle YOLO (si non inclus)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Ou copier modèle entraîné
cp /path/to/yolov8n_waste.pt .
```

### 6. Configuration Permissions

```bash
# Scripts exécutables
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts/
chmod +x *.py

# Fichiers web
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/web/
chmod 644 *.html *.css *.js
```

---

## ⚡ Optimisation Performance

### 1. Mode Performance Maximale

```bash
# Vérifier modes disponibles
sudo nvpmodel -q

# Activer mode MAXN (performance max)
sudo nvpmodel -m 0

# Activer tous les CPU cores
sudo jetson_clocks

# Vérifier status
sudo jetson_clocks --show
```

### 2. Script Auto-démarrage

**`/etc/systemd/system/ucaotech-robot.service`**
```ini
[Unit]
Description=Ucaotech DOFbot TRC2025
After=network.target

[Service]
Type=simple
User=ucaotech
WorkingDirectory=/home/ucaotech/ucaotech_ws
ExecStartPre=/bin/bash -c 'sudo nvpmodel -m 0'
ExecStartPre=/bin/bash -c 'sudo jetson_clocks'
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/ucaotech/ucaotech_ws/devel/setup.bash && roslaunch ucaotech_dofbot_trc2025 full_system.launch'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# Activer service
sudo systemctl enable ucaotech-robot.service
sudo systemctl start ucaotech-robot.service

# Vérifier status
sudo systemctl status ucaotech-robot.service
```

### 3. Optimisation TensorRT

```bash
# Export modèle en TensorRT
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/trc2025_train_models/

python3 << EOF
from ultralytics import YOLO

# Charger modèle
model = YOLO('models/yolov8n_waste.pt')

# Export TensorRT
model.export(format='engine', device=0, half=True)
EOF

# Fichier généré: yolov8n_waste.engine
```

### 4. Configuration Swappiness

```bash
# Réduire swappiness (performance)
sudo sysctl vm.swappiness=10

# Permanent
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
```

---

## ✅ Tests Validation

### 1. Test Caméra

```bash
# Test RealSense
realsense-viewer

# Test via ROS
roslaunch realsense2_camera rs_camera.launch
rostopic echo /camera/color/image_raw
```

### 2. Test DOFbot

```bash
# Test connexion série
ls -l /dev/ttyUSB* /dev/ttyTHS*

# Test mouvement
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts/
python3 << EOF
from Arm_Lib import Arm_Device
arm = Arm_Device()
time.sleep(1)
arm.Arm_serial_servo_write(1, 90, 500)
print("Test OK!")
EOF
```

### 3. Test Système Complet

```bash
# Lancer système complet
roslaunch ucaotech_dofbot_trc2025 full_system.launch

# Dans autre terminal, vérifier topics
rostopic list
rostopic hz /detected_objects
rostopic hz /joint_states
```

### 4. Test Interface Web

```bash
# Lancer serveur calibration
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts/
python3 calibration_server.py

# Sur ordinateur, ouvrir navigateur:
# http://[JETSON_IP]:8765/
```

---

## 🔧 Maintenance

### 1. Mise à Jour Système

```bash
# Mise à jour régulière
sudo apt update
sudo apt upgrade -y

# Mise à jour projet
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/
git pull origin main

# Recompilation
cd ~/ucaotech_ws/
catkin_make
```

### 2. Logs et Monitoring

```bash
# Logs ROS
roscd ucaotech_dofbot_trc2025
cat ~/.ros/log/latest/*.log

# Monitoring ressources
htop
tegrastats

# Température
watch -n 1 cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

### 3. Backup Configuration

```bash
# Backup workspace
tar -czf ~/ucaotech_backup_$(date +%Y%m%d).tar.gz \
    ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/

# Backup configuration
cp -r ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/config/ \
    ~/config_backup_$(date +%Y%m%d)/
```

---

## 🆘 Dépannage

### Problème 1: Caméra Non Détectée

```bash
# Vérifier connexion USB
lsusb | grep Intel

# Relancer driver
sudo modprobe uvcvideo

# Vérifier permissions
sudo chmod 666 /dev/video*
```

### Problème 2: DOFbot Non Connecté

```bash
# Vérifier port série
ls -l /dev/ttyUSB*

# Tester permissions
sudo chmod 666 /dev/ttyUSB0

# Vérifier groupe dialout
groups | grep dialout
```

### Problème 3: Performance Faible

```bash
# Vérifier mode performance
sudo nvpmodel -q

# Activer mode max
sudo nvpmodel -m 0
sudo jetson_clocks

# Vérifier température
tegrastats
```

### Problème 4: Erreur CUDA

```bash
# Vérifier CUDA
nvcc --version
nvidia-smi

# Vérifier variables
echo $CUDA_HOME
echo $LD_LIBRARY_PATH
```

---

## 📚 Checklist Déploiement

- [ ] Flash JetPack sur carte SD
- [ ] Configuration initiale Jetson
- [ ] Mise à jour système
- [ ] Configuration swap
- [ ] Installation CUDA/cuDNN
- [ ] Configuration réseau (WiFi + IP statique)
- [ ] Installation ROS Noetic
- [ ] Installation dépendances Python
- [ ] Installation RealSense SDK
- [ ] Installation DOFbot SDK
- [ ] Clonage projet
- [ ] Compilation workspace
- [ ] Téléchargement modèles
- [ ] Configuration permissions
- [ ] Activation mode performance
- [ ] Configuration auto-démarrage
- [ ] Tests validation
- [ ] Backup configuration

---

## 📞 Support

Pour aide déploiement : voir [docs/INDEX.md](../INDEX.md)

**Dernière mise à jour : 16 octobre 2025**

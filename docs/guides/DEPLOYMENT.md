# 🚀 Guide de Déploiement

**Ucaotech DOFbot TRC2025 - Déploiement Application**

---

## 📋 Table des Matières

1. [Prérequis](#prérequis)
2. [Vérification Système](#vérification-système)
3. [Déploiement Application](#déploiement-application)
4. [Configuration](#configuration)
5. [Optimisation Performance](#optimisation-performance)
6. [Tests Validation](#tests-validation)
7. [Maintenance](#maintenance)
8. [Dépannage](#dépannage)

---

## 📦 Prérequis

### Matériel

- ✅ **Yahboom DOFbot Kit** (avec Jetson Nano pré-installé)
- ✅ **Intel RealSense D435** (ou compatible)
- ✅ **Connexion Internet** (WiFi ou Ethernet)
- ✅ **Ordinateur PC** pour accéder à l'interface web

### Le DOFbot Yahboom Inclut Déjà

Le kit DOFbot est livré pré-configuré avec :
- ✅ Jetson Nano avec JetPack
- ✅ ROS Noetic installé
- ✅ Python et bibliothèques de base
- ✅ SDK DOFbot (Arm_Lib)
- ✅ Pilotes série et GPIO

**Vous n'avez PAS besoin de réinstaller le système !**

---

## 🔍 Vérification Système

### 1. Premier Démarrage du DOFbot

```bash
# Allumer le DOFbot
# Connexion SSH ou écran direct

# Vérifier la version du système
cat /etc/nv_tegra_release
python3 --version
```

### 2. Vérifier ROS

```bash
# Source ROS
source /opt/ros/noetic/setup.bash

# Vérifier ROS fonctionne
roscore &
sleep 2
rostopic list
pkill roscore
```

### 3. Vérifier DOFbot SDK

```bash
# Test rapide du bras
python3 << EOF
try:
    from Arm_Lib import Arm_Device
    print("✅ Arm_Lib disponible")
    arm = Arm_Device()
    print("✅ Connexion bras OK")
except Exception as e:
    print(f"❌ Erreur: {e}")
EOF
```

### 4. Vérifier Connexion Internet

```bash
# Tester connexion
ping -c 4 google.com

# Obtenir l'IP du Jetson
hostname -I
# Notez cette IP pour l'interface web !
```

---

## 🚀 Déploiement Application

### 1. Mise à Jour Système (Recommandé)

```bash
# Sur le Jetson Nano
sudo apt update
sudo apt upgrade -y
```

### 2. Installation Dépendances Additionnelles

Le DOFbot inclut déjà la plupart des dépendances, mais certains packages sont nécessaires pour l'application TRC2025 :

```bash
# Packages Python pour vision et ML
pip3 install \
    ultralytics==8.0.196 \
    mediapipe==0.10.3 \
    websockets==12.0 \
    pyyaml==6.0.1

# Packages ROS additionnels (si nécessaire)
sudo apt install -y \
    ros-noetic-cv-bridge \
    ros-noetic-realsense2-camera
```

### 3. Installation Intel RealSense SDK

```bash
# Ajouter dépôt Intel
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

# Installation
sudo apt update
sudo apt install -y librealsense2-utils librealsense2-dev

# Vérifier
realsense-viewer
```

### 4. Création Workspace ROS

```bash
# Créer workspace
mkdir -p ~/ucaotech_ws/src
cd ~/ucaotech_ws/

# Initialiser
catkin_make

# Source ROS et workspace
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/ucaotech_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Cloner Projet TRC2025

```bash
cd ~/ucaotech_ws/src/

# Cloner depuis GitHub
git clone https://github.com/Badmus2005/uca.git ucaotech_dofbot_trc2025

# Ou copier depuis clé USB
# cp -r /media/ucaotech/USB/ucaotech_dofbot_trc2025 .
```

### 6. Configuration Permissions

```bash
# Scripts exécutables
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts/
chmod +x *.py

# Fichiers web
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/web/
chmod 644 *.html
```

### 7. Compilation Workspace

```bash
cd ~/ucaotech_ws/

# Compilation
catkin_make

# Source pour appliquer
source devel/setup.bash
```

### 8. Téléchargement Modèles YOLO

```bash
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/trc2025_train_models/models/

# Télécharger modèle YOLO de base (si nécessaire)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Ou copier votre modèle entraîné
# cp /path/to/yolov8n_waste.pt .
```

---

## ⚙️ Configuration

### 1. Configuration Réseau (Optionnel)

Si vous voulez une IP fixe pour accéder facilement à l'interface web :

```bash
# Obtenir l'IP actuelle
hostname -I

# Pour IP statique (optionnel)
sudo nmcli connection modify "NOM_CONNEXION" \
    ipv4.addresses 192.168.1.100/24 \
    ipv4.gateway 192.168.1.1 \
    ipv4.dns "8.8.8.8" \
    ipv4.method manual

# Redémarrer connexion
sudo nmcli connection down "NOM_CONNEXION"
sudo nmcli connection up "NOM_CONNEXION"
```

### 2. Configuration Positions Prédéfinies

```bash
# Éditer fichier de configuration
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/config/
nano positions.yaml
```

Exemple de configuration :
```yaml
home:
  - 90
  - 90
  - 90
  - 90
  - 90
  - 30
observation:
  - 90
  - 45
  - 90
  - 90
  - 90
  - 30
```

### 3. Variables d'Environnement

```bash
# Ajouter au ~/.bashrc (si pas déjà fait)
cat >> ~/.bashrc << 'EOF'

# Workspace TRC2025
export PYTHONPATH=$PYTHONPATH:~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts
export PYTHONPATH=$PYTHONPATH:~/ucaotech_ws/src/ucaotech_dofbot_trc2025/src
EOF

source ~/.bashrc
```

---

## ⚡ Optimisation Performance

### 1. Mode Performance Maximale

Le DOFbot peut fonctionner en différents modes de performance :

```bash
# Vérifier modes disponibles
sudo nvpmodel -q

# Activer mode MAXN (performance maximale)
sudo nvpmodel -m 0

# Activer tous les CPU cores
sudo jetson_clocks

# Vérifier status
sudo jetson_clocks --show
```

### 2. Script de Démarrage Automatique (Optionnel)

Créer un service systemd pour lancer automatiquement le système au démarrage :

**Créer `/etc/systemd/system/ucaotech-robot.service` :**
```bash
sudo nano /etc/systemd/system/ucaotech-robot.service
```

**Contenu du fichier :**
```ini
[Unit]
Description=Ucaotech DOFbot TRC2025
After=network.target

[Service]
Type=simple
User=ucaotech
WorkingDirectory=/home/ucaotech/ucaotech_ws
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/ucaotech/ucaotech_ws/devel/setup.bash && roslaunch ucaotech_dofbot_trc2025 full_system.launch'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Activer le service :**
```bash
# Recharger systemd
sudo systemctl daemon-reload

# Activer le service
sudo systemctl enable ucaotech-robot.service

# Démarrer le service
sudo systemctl start ucaotech-robot.service

# Vérifier le status
sudo systemctl status ucaotech-robot.service
```

### 3. Optimisation Mémoire

```bash
# Réduire swappiness pour meilleures performances
sudo sysctl vm.swappiness=10

# Rendre permanent
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
```

---

## ✅ Tests Validation

### 1. Test Bras DOFbot

```bash
# Test rapide de mouvement
python3 << EOF
from Arm_Lib import Arm_Device
import time

arm = Arm_Device()
time.sleep(1)

# Test mouvement joint 1
arm.Arm_serial_servo_write(1, 90, 1000)
time.sleep(2)

print("✅ Test bras OK!")
EOF
```

### 2. Test Caméra RealSense

```bash
# Lancer viewer RealSense
realsense-viewer

# Ou test via ROS
roslaunch realsense2_camera rs_camera.launch &
sleep 5
rostopic echo /camera/color/image_raw --noarr
pkill -f realsense
```

### 3. Test Interface Web de Calibration

```bash
# Lancer serveur de calibration
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts/
python3 calibration_server.py
```

**Sur votre PC :**
- Ouvrir `web/calibration_interface.html` dans un navigateur
- Configurer l'IP du Jetson (celle obtenue avec `hostname -I`)
- Cliquer sur "Connecter"
- Tester les sliders de joints

### 4. Test Système Complet (Optionnel)

```bash
# Lancer système complet avec ROS
roslaunch ucaotech_dofbot_trc2025 full_system.launch

# Dans autre terminal, vérifier topics
rostopic list
rostopic hz /detected_objects
```

---

## 🔧 Maintenance

### 1. Mise à Jour Application

```bash
# Mettre à jour depuis GitHub
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/
git pull origin main

# Recompilation
cd ~/ucaotech_ws/
catkin_make
source devel/setup.bash
```

### 2. Logs et Monitoring

```bash
# Vérifier logs du serveur de calibration
# Les logs s'affichent dans le terminal où vous lancez le serveur

# Monitoring ressources
htop

# Température Jetson
watch -n 1 cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

### 3. Backup Configuration

```bash
# Backup positions et config
tar -czf ~/backup_config_$(date +%Y%m%d).tar.gz \
    ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/config/

# Copier sur clé USB ou PC
# scp ~/backup_config_*.tar.gz user@pc:/path/to/backup/
```

---

## 🆘 Dépannage

### Problème 1: Caméra RealSense Non Détectée

```bash
# Vérifier connexion USB
lsusb | grep Intel

# Relancer les règles udev
sudo udevadm control --reload-rules
sudo udevadm trigger

# Redémarrer le Jetson si nécessaire
```

### Problème 2: Bras Ne Répond Pas

```bash
# Vérifier connexion série
ls -l /dev/ttyUSB* /dev/ttyTHS*

# Vérifier permissions
sudo chmod 666 /dev/ttyUSB0

# Vérifier que l'utilisateur est dans le groupe dialout
groups | grep dialout
# Si absent:
sudo usermod -a -G dialout $USER
# Puis déconnexion/reconnexion
```

### Problème 3: Interface Web Ne Se Connecte Pas

**Vérifier IP du Jetson :**
```bash
hostname -I
```

**Vérifier serveur calibration lancé :**
```bash
ps aux | grep calibration_server
```

**Vérifier pare-feu :**
```bash
sudo ufw status
# Si actif, autoriser port 8765
sudo ufw allow 8765
```

### Problème 4: Performance Lente

```bash
# Activer mode performance max
sudo nvpmodel -m 0
sudo jetson_clocks

# Vérifier température
tegrastats
# Si surchauffe > 80°C, améliorer ventilation
```

### Problème 5: "Arm_Lib not found"

```bash
# Vérifier installation SDK DOFbot
python3 -c "from Arm_Lib import Arm_Device; print('OK')"

# Si erreur, le SDK Yahboom devrait être pré-installé sur le DOFbot
# Contacter support Yahboom si nécessaire
```

---

## 📚 Checklist Déploiement

### Vérification Système DOFbot
- [ ] DOFbot allumé et fonctionnel
- [ ] ROS Noetic pré-installé vérifié
- [ ] SDK Arm_Lib disponible
- [ ] Connexion Internet active

### Installation Application
- [ ] Workspace ROS créé
- [ ] Projet TRC2025 cloné
- [ ] Dépendances Python installées
- [ ] RealSense SDK installé
- [ ] Compilation workspace réussie
- [ ] Permissions configurées

### Configuration
- [ ] IP Jetson notée (pour interface web)
- [ ] Positions prédéfinies configurées
- [ ] Variables d'environnement ajoutées

### Tests
- [ ] Bras DOFbot répond aux commandes
- [ ] Caméra RealSense détectée
- [ ] Interface web de calibration accessible
- [ ] Connexion PC ↔ Jetson fonctionnelle

### Optimisation (Optionnel)
- [ ] Mode performance activé
- [ ] Service auto-démarrage configuré
- [ ] Backup configuration effectué

---

## 🎯 Démarrage Rapide Post-Installation

Une fois le déploiement terminé, voici comment utiliser le système :

### Lancer l'Interface de Calibration

**Sur le Jetson Nano :**
```bash
cd ~/ucaotech_ws/src/ucaotech_dofbot_trc2025/scripts/
python3 calibration_server.py
```

**Sur votre PC :**
1. Ouvrir `web/calibration_interface.html`
2. Configurer IP du Jetson
3. Connecter et calibrer !

---

## 📞 Support

Pour aide déploiement : voir [docs/INDEX.md](../INDEX.md)

**Dernière mise à jour : 22 octobre 2025**

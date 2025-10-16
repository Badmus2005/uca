# 🚀 QUICKSTART - Démarrage Rapide (5 minutes)

**Ucaotech DOFbot TRC2025 - Guide Ultra-Rapide**

---

## ⚡ Installation Express

```bash
# 1. Cloner le projet
git clone https://github.com/Badmus2005/uca.git
cd uca/ucaotech_dofbot_trc2025

# 2. Installer dépendances
pip3 install -r requirements.txt

# 3. Setup ROS
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
ln -s ~/uca/ucaotech_dofbot_trc2025/ros_package/ucaotech_dofbot_trc2025 .
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 4. Lancer le système
roslaunch ucaotech_dofbot_trc2025 tri.launch
```

**✅ Terminé ! Le système est maintenant opérationnel.**

---

## 🎮 Utilisation Immédiate

### Lancer le Tri Automatique

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ucaotech_dofbot_trc2025 tri.launch
```

**Placez un déchet devant la caméra et observez le tri !**

---

## 🧪 Test Rapide

```bash
# Tester le modèle YOLOv5
python3 tests/test_yolov5_model.py

# Si ça passe → Tout fonctionne ✅
```

---

## 🎯 Prochaines Étapes

- **Calibration** : `python3 scripts/calibrate_positions.py`
- **Interface Web** : Ouvrir `web/calibration_interface.html`
- **Documentation** : Voir [README.md](README.md)

---

**🤖 Prêt pour TRC 2025 ! 🏆**

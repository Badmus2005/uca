# 🤖 Ucaotech DOFbot TRC2025 - Système de Tri Intelligent de Déchets

![TRC2025](https://img.shields.io/badge/TRC-2025-blue) ![Status](https://img.shields.io/badge/status-ready-green) ![Python](https://img.shields.io/badge/python-3.x-blue) ![ROS](https://img.shields.io/badge/ROS-Melodic-brightgreen)

## 📋 Vue d'ensemble

## 📋 Description

Projet développé par l'équipe **Ucaotech** pour la compétition **TRC 2025** (Cotonou, Bénin 🇧🇯). Ce système automatisé de tri des déchets utilise un bras robotique DOFbot à 5 axes équipé d'un Jetson Nano et d'un modèle de vision par ordinateur YOLOv5m pour classifier et trier automatiquement les déchets en 3 catégories.

### 🎯 Objectifs du Projet

- Tri automatique des déchets en 3 catégories : **dangereux**, **ménagers**, **recyclables**
- Précision de classification : **85.2%** (mAP@0.5)
- Système ROS pour coordination temps réel
- Déploiement sur Jetson Nano (optimisé embarqué)

---

## 🏗️ Architecture du Système

```
┌─────────────────────────────────────────────────────────┐
│                    SYSTÈME DOFBOT TRC2025                │
└─────────────────────────────────────────────────────────┘
           │
           │
    ┌──────▼──────┐
    │   CAMÉRA    │ (640x480 @ 10FPS)
    └──────┬──────┘
           │ Images ROS
           ▼
    ┌──────────────┐
    │  VISION NODE │ (YOLOv5m Inference)
    └──────┬───────┘
           │ Classification (class_id + confidence)
           ▼
    ┌────────────────────┐
    │ I2C CONTROLLER NODE│ (Décision + Séquences)
    └──────┬─────────────┘
           │ Commandes I2C
           ▼
    ┌──────────────┐
    │  BRAS DOFBOT │ (5 axes + pince)
    └──────┬───────┘
           │
    ┌──────▼──────────────────────────────┐
    │  3 BACS DE TRI                       │
    │  • Dangereux (électronique)          │
    │  • Ménagers (organiques)             │
    │  • Recyclables (papier/plastique)    │
    └──────────────────────────────────────┘
```

### 🧩 Composants Principaux

| Composant | Technologie | Rôle |
|-----------|-------------|------|
| **Hardware** | Jetson Nano 4GB | Calcul embarqué + inférence GPU |
| **Bras Robotique** | DOFbot 5 axes | Manipulation physique |
| **Caméra** | USB 640×480 | Capture d'images |
| **IA Vision** | YOLOv5m (PyTorch) | Classification des déchets |
| **Middleware** | ROS Melodic | Coordination nœuds temps réel |
| **Contrôle I2C** | Arm_Lib + Python | Communication avec servomoteurs |

---

## 📁 Structure du Projet

```
ucaotech_dofbot_trc2025/
│
├── 📄 README.md                    ← Vous êtes ici
│
├── 📂 config/                      ← Configurations YAML
│   ├── positions.yaml              ← Positions calibrées du bras
│   ├── yolov5_params.yaml          ← Paramètres modèle YOLOv5
│   └── camera_params.yaml          ← Paramètres caméra
│
├── 📂 models/                      ← Modèles d'IA
│   ├── best.pt                     ← Modèle YOLOv5m entraîné (40.6 MB)
│   ├── dataset.yaml                ← Config dataset
│   └── yolov5/                     ← Framework YOLOv5
│       ├── models/                 ← Architecture des modèles
│       │   ├── common.py
│       │   ├── yolo.py
│       │   └── experimental.py
│       └── utils/                  ← Utilitaires
│           ├── general.py
│           ├── torch_utils.py
│           ├── augmentations.py
│           └── dataloaders.py
│
├── 📂 ros_package/                 ← Package ROS principal
│   ├── CMakeLists.txt              ← Configuration CMake
│   ├── package.xml                 ← Métadonnées ROS
│   │
│   ├── 📂 scripts/                 ← Nœuds ROS Python
│   │   ├── vision_node.py          ← Nœud de vision (YOLOv5)
│   │   ├── i2c_controller_node.py  ← Contrôleur principal
│   │   ├── final_camera_node.py    ← Nœud caméra
│   │   └── dofbot_tri_system.py    ← Séquences de tri
│   │
│   ├── 📂 srv/                     ← Services ROS
│   │   └── Classify.srv            ← Service classification
│   │
│   └── 📂 launch/                  ← Fichiers de lancement
│       └── tri.launch              ← Lancement système complet
│
├── 📂 docs/                        ← Documentation
│   ├── README.md                   ← Guide détaillé
│   ├── INSTALLATION.md             ← Instructions installation
│   ├── USAGE.md                    ← Guide d'utilisation
│   └── ARCHITECTURE.md             ← Architecture technique
│
├── 📂 scripts/                     ← Scripts utilitaires
│   ├── deploy_to_jetson.sh         ← Script de déploiement
│   ├── test_model.py               ← Test du modèle
│   └── backup_config.sh            ← Sauvegarde configs
│
├── 📂 tests/                       ← Tests unitaires
│   ├── test_vision.py              ← Tests vision
│   ├── test_movements.py           ← Tests mouvements
│   └── test_integration.py         ← Tests intégration
│
└── 📂 REFERENCES/                  ← Documentation usine (read-only)
    ├── Dofbot_original/            ← Exemples standalone
    └── dofbot_ws_src/              ← Exemples ROS workspace
```

---

## 🚀 Installation Rapide

### Prérequis

- **Matériel** : Jetson Nano 4GB, Bras DOFbot, Caméra USB
- **OS** : Ubuntu 18.04 LTS (JetPack 4.x)
- **ROS** : Melodic (pré-installé avec JetPack)
- **Python** : 3.6+
- **GPU** : CUDA 10.2 (pour inférence)

### Étape 1 : Cloner le Projet

```bash
cd ~/catkin_ws/src
git clone https://github.com/ucaotech/dofbot_trc2025.git ucaotech_dofbot_trc2025
```

### Étape 2 : Installer les Dépendances

```bash
cd ~/catkin_ws/src/ucaotech_dofbot_trc2025

# Dépendances Python
pip3 install -r requirements.txt

# Dépendances ROS (si manquantes)
sudo apt-get update
sudo apt-get install ros-melodic-cv-bridge ros-melodic-sensor-msgs
```

### Étape 3 : Compiler le Package ROS

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Étape 4 : Configurer les Paramètres

Vérifiez et ajustez les fichiers dans `config/` :

```bash
# Vérifier les positions calibrées
nano config/positions.yaml

# Vérifier les paramètres YOLOv5
nano config/yolov5_params.yaml

# Vérifier les paramètres caméra
nano config/camera_params.yaml
```

### Étape 5 : Lancer le Système

```bash
# Lancer tous les nœuds
roslaunch ucaotech_dofbot_trc2025 tri.launch

# OU lancer les nœuds individuellement
# Terminal 1: Caméra
rosrun ucaotech_dofbot_trc2025 final_camera_node.py

# Terminal 2: Vision
rosrun ucaotech_dofbot_trc2025 vision_node.py

# Terminal 3: Contrôleur
rosrun ucaotech_dofbot_trc2025 i2c_controller_node.py
```

---

## 🎮 Utilisation

### Mode Automatique (Tri Continu)

1. Démarrer le système : `roslaunch ucaotech_dofbot_trc2025 tri.launch`
2. Placer les déchets dans la zone de détection
3. Le système détecte, classifie et trie automatiquement

### Mode Service (Classification à la Demande)

```python
import rospy
from ucaotech_dofbot_trc2025.srv import Classify
from sensor_msgs.msg import Image

rospy.init_node('test_client')
classify = rospy.ServiceProxy('/dofbot/classify_image', Classify)

# Envoyer une image pour classification
response = classify(image)
print(f"Classe: {response.class_id}, Confiance: {response.confidence}")
```

### Tests

```bash
# Tester le modèle YOLOv5
python3 scripts/test_model.py

# Tester les mouvements du bras
python3 tests/test_movements.py

# Test d'intégration complet
python3 tests/test_integration.py
```

---

## 📊 Performances

### Modèle YOLOv5m

- **Précision Globale** : 85.2% mAP@0.5
- **Paramètres** : 20.8M
- **Taille** : 40.6 MB
- **Inférence** : ~30ms/image (Jetson Nano GPU)
- **Dataset** : 510 images train + 27 val

### Performances par Classe

| Classe | Précision | Rappel | F1-Score |
|--------|-----------|--------|----------|
| Dangereux | 88.3% | 84.1% | 86.2% |
| Ménagers | 82.7% | 79.5% | 81.1% |
| Recyclables | 84.6% | 89.0% | 86.7% |

### Système Complet

- **Fréquence** : ~3-5 déchets/minute
- **Latence Totale** : ~2-3 secondes/objet
  - Capture : 100ms
  - Inférence : 30ms
  - Décision : 10ms
  - Mouvement : 2-3s

---

## 🔧 Configuration Avancée

### Ajuster le Seuil de Confiance

```yaml
# config/yolov5_params.yaml
inference:
  conf_threshold: 0.6  # Augmenter pour réduire faux positifs (ex: 0.7, 0.8)
```

### Recalibrer les Positions du Bras

```yaml
# config/positions.yaml
bins:
  recyclables:
    joint1: 45  # Ajuster l'angle selon votre setup physique
    joint2: 110
    # ...
```

### Optimiser les Performances

```yaml
# config/yolov5_params.yaml
inference:
  device: "cuda:0"        # GPU (rapide)
  # device: "cpu"         # CPU (lent mais universel)
  half_precision: true    # FP16 pour accélérer sur GPU
```

---

## 🐛 Dépannage

### Problème : Caméra non détectée

```bash
# Vérifier les périphériques vidéo
ls -l /dev/video*

# Tester la caméra
v4l2-ctl --list-devices
```

### Problème : Erreur CUDA / PyTorch

```bash
# Vérifier CUDA
nvcc --version

# Vérifier PyTorch
python3 -c "import torch; print(torch.cuda.is_available())"
```

### Problème : Mouvements du bras incorrects

1. Vérifier les permissions I2C :
   ```bash
   sudo usermod -aG i2c $USER
   sudo chmod 666 /dev/i2c-1
   ```

2. Recalibrer les positions dans `config/positions.yaml`

### Problème : Faible Précision

1. Vérifier l'éclairage (lumière uniforme recommandée)
2. Vérifier la distance objet-caméra (20-50 cm optimal)
3. Augmenter `conf_threshold` dans `yolov5_params.yaml`

---

## 📚 Documentation Complète

- **Installation Détaillée** : [docs/INSTALLATION.md](docs/INSTALLATION.md)
- **Guide Utilisateur** : [docs/USAGE.md](docs/USAGE.md)
- **Architecture Technique** : [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- **API ROS** : [docs/ROS_API.md](docs/ROS_API.md)

---

## 👥 Équipe Ucaotech

- **Développeur Principal** : [Votre Nom]
- **Responsable IA** : [Nom]
- **Responsable Hardware** : [Nom]
- **Superviseur** : [Nom]

---

## 📄 Licence

Ce projet est développé pour la compétition TRC 2025. Tous droits réservés à l'équipe Ucaotech.

---

## 🙏 Remerciements

- **Yahboom** pour le bras robotique DOFbot
- **Ultralytics** pour YOLOv5
- **NVIDIA** pour Jetson Nano
- **ROS Community** pour le framework ROS

---

## 📞 Contact

- **Email** : contact@ucaotech.ci
- **GitHub** : https://github.com/ucaotech
- **Compétition** : TRC 2025, Cotonou, Bénin 🇧🇯

---

**🏆 Bon courage pour TRC 2025 !**

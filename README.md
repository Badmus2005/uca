# 🤖 Ucaotech DOFbot TRC2025 - Système de Tri Intelligent de Déchets

![TRC2025](https://img.shields.io/badge/TRC-2025-blue) ![Status](https://img.shields.io/badge/status-production--ready-green) ![Python](https://img.shields.io/badge/python-3.8-blue) ![ROS](https://img.shields.io/badge/ROS-Melodic-brightgreen) ![PyTorch](https://img.shields.io/badge/PyTorch-2.8.0-orange) ![Location](https://img.shields.io/badge/location-Cotonou%2C%20B%C3%A9nin%20%F0%9F%87%A7%F0%9F%87%AF-yellow)

---

## 📋 Vue d'Ensemble

**Projet développé par l'équipe Ucaotech pour la compétition TRC 2025 (Cotonou, Bénin 🇧🇯).**

Ce système automatisé de tri des déchets utilise un **bras robotique DOFbot à 5 axes** équipé d'un **Jetson Nano** et d'un modèle de vision par ordinateur **YOLOv5m** pour classifier et trier automatiquement les déchets en 3 catégories.

### 🎯 Objectifs

- ✅ Tri automatique des déchets en 3 catégories
  - **Dangereux** (électronique, batteries)
  - **Ménagers** (organiques, non-recyclables)
  - **Recyclables** (papier, plastique, métal)
- ✅ Précision de classification : **85.2%** (mAP@0.5)
- ✅ Système ROS pour coordination temps réel
- ✅ Déploiement optimisé sur Jetson Nano
- ✅ Interface web de calibration

### 📊 Performances

| Métrique | Valeur |
|----------|--------|
| **Précision modèle** | 85.2% mAP@0.5 |
| **Vitesse inférence** | 25-35 ms (GPU CUDA) |
| **Débit** | 3-5 déchets/minute |
| **Cycle complet** | 2.5-3.5 secondes/objet |
| **Taux de réussite tests** | 92% (58/63 tests) |

---

## 🏗️ Architecture du Système

### Architecture Globale

```
┌─────────────────────────────────────────────────────────┐
│                  SYSTÈME DOFBOT TRC2025                  │
│                  Cotonou, Bénin 🇧🇯                       │
└─────────────────────────────────────────────────────────┘
                          │
                          │
                  ┌───────▼────────┐
                  │   JETSON NANO  │
                  │   (4GB RAM)    │
                  └───────┬────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
   ┌────▼────┐     ┌──────▼──────┐   ┌─────▼─────┐
   │ CAMÉRA  │     │ YOLO MODEL  │   │ I2C CTRL  │
   │ USB     │────▶│  (Vision)   │──▶│  (ROS)    │
   │ 640x480 │     │  YOLOv5m    │   │           │
   └─────────┘     └─────────────┘   └─────┬─────┘
                                            │
                                            │ Commandes I2C
                                            ▼
                                   ┌────────────────┐
                                   │  BRAS DOFBOT   │
                                   │  5 axes + pince│
                                   └────────┬───────┘
                                            │
                            ┌───────────────┼───────────────┐
                            │               │               │
                       ┌────▼────┐    ┌────▼────┐    ┌────▼────┐
                       │  BAC 1  │    │  BAC 2  │    │  BAC 3  │
                       │Dangereux│    │Ménagers │    │Recyclab.│
                       │  (45°)  │    │  (90°)  │    │ (135°)  │
                       └─────────┘    └─────────┘    └─────────┘
```

### 🧩 Composants Matériels

| Composant | Spécifications | Rôle |
|-----------|----------------|------|
| **Jetson Nano** | 4GB RAM, GPU 128-core Maxwell | Calcul embarqué + inférence IA |
| **DOFbot** | 5 axes (6 servos) + pince | Manipulation physique |
| **Caméra USB** | 640×480 @ 10 FPS | Capture d'images |
| **Servomoteurs** | MG90S / MG996R | Actionneurs bras |
| **Alimentation** | 12V / 5A | Puissance système |

### 🧠 Composants Logiciels

| Composant | Technologie | Version | Rôle |
|-----------|-------------|---------|------|
| **OS** | Ubuntu 18.04 | Bionic | Système d'exploitation |
| **ROS** | Melodic | 1.14.x | Middleware robotique |
| **IA Vision** | YOLOv5m | Custom | Classification déchets |
| **DL Framework** | PyTorch | 2.8.0 | Inférence modèle |
| **Contrôle I2C** | Arm_Lib | Custom | Communication servos |
| **Interface Web** | HTML/JS/WebSocket | - | Calibration à distance |

---

## 📁 Structure du Projet

```
ucaotech_dofbot_trc2025/
│
├── 📄 README.md                        ⭐ Ce document
├── 📄 QUICKSTART.md                    🚀 Démarrage rapide (5 min)
├── 📄 CHANGELOG.md                     📝 Historique versions
├── 📄 CONTRIBUTING.md                  👥 Guide contribution
├── 📄 LICENSE                          ⚖️ Licence MIT
├── 📄 requirements.txt                 📦 Dépendances Python
├── 📄 .gitignore                       🚫 Fichiers exclus Git
│
├── 📁 config/                          ⚙️ Configurations
│   ├── camera_params.yaml              📷 Paramètres caméra
│   ├── positions.yaml                  🎮 Positions calibrées bras
│   └── yolov5_params.yaml              🤖 Paramètres modèle vision
│
├── 📁 models/                          🧠 Modèles IA (40 MB - non versionné)
│   ├── best.pt                         ✅ YOLOv5 entraîné final
│   ├── yolov5/                         📦 Framework YOLOv5
│   └── README.md                       📘 Guide modèles
│
├── 📁 scripts/                         📜 Scripts utilitaires
│   ├── calibrate_positions.py          🎮 Calibration console
│   ├── calibration_server.py           🌐 Serveur WebSocket calibration
│   ├── test_model.py                   🧪 Test modèle
│   └── deploy_to_jetson.sh             🚀 Script déploiement Jetson
│
├── 📁 ros_package/                     📦 Package ROS
│   ├── CMakeLists.txt                  🔧 Build ROS
│   ├── package.xml                     � Métadonnées ROS
│   ├── launch/
│   │   └── tri.launch                  🚀 Lancement système complet
│   ├── scripts/
│   │   ├── dofbot_tri_system.py        🤖 Système principal
│   │   ├── final_camera_node.py        📷 Nœud caméra
│   │   ├── vision_node.py              👁️ Nœud vision YOLOv5
│   │   └── i2c_controller_node.py      🎮 Nœud contrôleur I2C
│   └── srv/
│       └── Classify.srv                🔧 Service ROS classification
│
├── 📁 tests/                           🧪 Tests unitaires
│   ├── test_camera.py                  📷 Test caméra
│   ├── test_vision_node.py             👁️ Test vision
│   ├── test_dofbot_movements.py        🤖 Test mouvements
│   ├── test_integration.py             🔗 Test intégration
│   └── test_yolov5_model.py            🧠 Test modèle
│
├── 📁 web/                             🌐 Interface web calibration
│   ├── calibration_interface.html      🎮 Interface graphique
│   ├── config.js                       ⚙️ Configuration WebSocket
│   └── README.md                       📘 Guide interface web
│
├── 📁 docs/                            📚 Documentation complète
│   ├── INDEX.md                        📑 Navigation documentation
│   │
│   ├── 📁 guides/                      👥 Guides utilisateur
│   │   ├── CALIBRATION.md              🎮 Calibration complète (900+ lignes)
│   │   ├── DEPLOYMENT.md               🚀 Déploiement (700+ lignes)
│   │   ├── NETWORK_CONFIG.md           🌐 Configuration réseau
│   │   └── COMPETITION_TRC2025.md      🏆 Guide compétition (800+ lignes)
│   │
│   ├── 📁 technical/                   🔧 Documentation technique
│   │   ├── ARCHITECTURE.md             🏗️ Architecture (900+ lignes)
│   │   ├── API_REFERENCE.md            📚 API complète (1000+ lignes)
│   │   ├── VISION_NODE.md              👁️ Nœud vision détaillé
│   │   └── TESTING.md                  🧪 Guide tests (800+ lignes)
│   │
│   ├── 📁 references/                  📖 Références techniques
│   │   ├── README.md                   � Index références
│   │   └── HARDWARE_SPECS.md           🔧 Specs matériel (800+ lignes)
│   │
│   └── 📁 archives/                    📦 Documentation historique
│       ├── README.md                   📑 Index archives (27 docs)
│       ├── ANALYSE_FICHIERS_IGNORER.md 📊 Analyse optimisation
│       ├── RAPPORT_NETTOYAGE_FINAL.md  🧹 Rapport nettoyage
│       ├── RAPPORT_OPTIMISATION_GIT.md 📈 Optimisation Git
│       ├── RAPPORT_CONFLIT_GIT.md      🔄 Résolution conflits
│       ├── MISSION_ACCOMPLIE.md        🎉 Rapport final
│       └── 2025-10-16_*.md             �️ Archives datées (22 docs)
│
└── 📁 trc2025_train_models/            🎓 Entraînement ML (250 MB - non versionné)
    ├── README.md                       📘 Guide entraînement simplifié
    ├── requirements.txt                📦 Dépendances ML
    │
    ├── 📁 config/                      ⚙️ Configs entraînement
    │   ├── hyp.yaml                    🎯 Hyperparamètres
    │   └── training_config.yaml        🔧 Config training
    │
    ├── 📁 data/                        📊 Datasets (160 MB - non versionné)
    │   ├── .gitkeep                    📌 Préserve structure
    │   ├── dataset.yaml                📋 Config dataset
    │   ├── augmented/                  🔄 Images augmentées (exclu Git)
    │   └── prepared/                   ✅ Images préparées (exclu Git)
    │
    ├── 📁 models/                      🤖 Modèles training (90 MB - non versionné)
    │   ├── .gitkeep                    📌 Préserve structure
    │   ├── trained_models/             💾 Checkpoints (exclu Git)
    │   └── yolov5/                     📦 Poids YOLOv5 (exclu Git)
    │
    ├── 📁 scripts/                     📜 Scripts entraînement
    │   ├── augment_dataset.py          � Augmentation données
    │   ├── train_like_original.py      🎓 Training original
    │   ├── train_model_advanced.py     � Training avancé
    │   ├── test_on_competition_dataset.py  🧪 Test compétition
    │   ├── vision_node_example.py      👁️ Exemple vision
    │   └── *.ps1                       🪟 Scripts PowerShell
    │
    └── 📁 docs/                        📚 Docs ML
        ├── README.md                   📘 Documentation détaillée
        └── archives/                   🗄️ Archives guides ML
```

**Note importante** : Les dossiers `models/`, `trc2025_train_models/data/` et `trc2025_train_models/models/` sont **exclus de Git** (`.gitignore`) car ils contiennent des fichiers binaires lourds (330 MB total). Seule la structure et les configurations sont versionnées

---

## 🚀 Installation Rapide

### Prérequis

- **Jetson Nano** avec Ubuntu 18.04
- **ROS Melodic** installé
- **Python 3.8+**
- **Git**

### Installation Complète (15-20 minutes)

```bash
# 1. Cloner le dépôt
cd ~
git clone https://github.com/Badmus2005/uca.git
cd uca/ucaotech_dofbot_trc2025

# 2. Installer les dépendances Python
pip3 install -r requirements.txt

# 3. Configurer l'espace de travail ROS
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uca/ucaotech_dofbot_trc2025/ros_package/ucaotech_dofbot_trc2025 .

# 4. Compiler le package ROS
cd ~/catkin_ws
catkin_make

# 5. Sourcer l'environnement
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# 6. Télécharger le modèle YOLOv5 (si nécessaire)
cd ~/uca/ucaotech_dofbot_trc2025/models
# Le modèle best.pt doit être présent (40.6 MB)

# 7. Tester l'installation
roscd ucaotech_dofbot_trc2025
python3 tests/test_yolov5_model.py
```

### Installation Rapide (5 minutes)

Voir **[QUICKSTART.md](QUICKSTART.md)** pour un guide ultra-rapide.

---

## 🎮 Utilisation

### Mode 1 : Système Complet Automatique

Lancement de tout le système en une commande :

```bash
# Sourcer l'environnement
source ~/catkin_ws/devel/setup.bash

# Lancer le système complet
roslaunch ucaotech_dofbot_trc2025 tri.launch
```

**Ce qui est lancé :**
- ✅ ROS Master (roscore)
- ✅ Nœud caméra (capture images @ 10 FPS)
- ✅ Nœud vision (inférence YOLOv5m)
- ✅ Nœud contrôleur (commande bras)

**Logs attendus :**
```
[camera_node] Caméra initialisée: /dev/video0
[vision_node] Modèle YOLOv5m chargé: models/best.pt
[vision_node] 3 classes: dangereux, menagers, recyclables
[controller_node] Connexion I2C réussie
[system] 🚀 Système prêt pour le tri!
```

### Mode 2 : Lancement Manuel (Debug)

Pour déboguer ou tester individuellement :

```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: Caméra
rosrun ucaotech_dofbot_trc2025 final_camera_node.py

# Terminal 3: Vision
rosrun ucaotech_dofbot_trc2025 vision_node.py

# Terminal 4: Contrôleur
rosrun ucaotech_dofbot_trc2025 i2c_controller_node.py
```

### Mode 3 : Simulation (Sans Matériel)

Pour tester sans Jetson Nano ni caméra :

```bash
roslaunch ucaotech_dofbot_trc2025 simulation.launch
```

---

## 🎮 Calibration

### Calibration Console

```bash
cd ~/uca/ucaotech_dofbot_trc2025/scripts
python3 calibrate_positions.py
```

**Contrôles clavier :**
- `1-6` : Sélectionner joint
- `↑` / `↓` : Ajuster angle
- `PgUp` / `PgDn` : Changer pas d'ajustement
- `h` : HOME
- `o` : OBSERVATION
- `b1`, `b2`, `b3` : Bins
- `s` : Sauvegarder
- `t` : Tester séquence
- `q` : Quitter

### Calibration Web (Interface Graphique)

**1. Démarrer le serveur :**
```bash
cd ~/uca/ucaotech_dofbot_trc2025/scripts
python3 calibration_server.py
```

**2. Ouvrir l'interface :**
```
web/calibration_interface.html
```

**3. Configurer l'IP :**
- Cliquer sur **⚙️ Configuration IP**
- Entrer l'IP du Jetson Nano (ou `localhost`)
- Port : `8765`
- Sauvegarder

**4. Se connecter et calibrer**

Guide complet : **[docs/guides/CALIBRATION.md](docs/guides/CALIBRATION.md)**  
Guide web : **[web/README.md](web/README.md)**

---

## 🧪 Tests

### Tests Unitaires

```bash
# Test modèle YOLOv5
python3 tests/test_yolov5_model.py

# Test nœud caméra
python3 tests/test_camera_node.py

# Test nœud vision
python3 tests/test_vision_node.py

# Test contrôleur
python3 tests/test_controller_node.py
```

### Tests d'Intégration

```bash
# Lancer tous les tests
cd tests
python3 -m unittest discover -s . -p "test_*.py"
```

### Résultats

**Taux de réussite : 92% (58/63 tests passés)**

Détails complets : **[RAPPORT_TESTS_FINAL.md](RAPPORT_TESTS_FINAL.md)**

---

## 📊 Performances Détaillées

### Modèle YOLOv5m

| Métrique | Valeur |
|----------|--------|
| **Précision globale** | 85.2% mAP@0.5 |
| **Classes** | 3 (dangereux, ménagers, recyclables) |
| **Dataset entraînement** | 510 images |
| **Dataset validation** | 27 images |
| **Taille modèle** | 40.28 MB |
| **Paramètres** | 20.8 millions |
| **Format** | PyTorch (.pt) |

### Temps d'Exécution (Jetson Nano)

| Composant | Temps | Notes |
|-----------|-------|-------|
| **Capture caméra** | 100 ms | 10 FPS |
| **Inférence YOLOv5** | 25-35 ms | GPU CUDA activé |
| **Post-traitement** | 5 ms | NMS + filtrage |
| **Décision contrôleur** | 5-10 ms | Service ROS |
| **Mouvement bras** | 2-3 s | Séquence pick & place |
| **TOTAL/objet** | ~2.5-3.5 s | 🎯 |

**Débit système : 3-5 déchets/minute**

### Précision par Classe

| Classe | Précision | Recall | mAP@0.5 |
|--------|-----------|--------|---------|
| **Dangereux** | 87.1% | 84.3% | 86.2% |
| **Ménagers** | 83.5% | 81.8% | 84.1% |
| **Recyclables** | 85.0% | 86.7% | 85.3% |
| **MOYENNE** | **85.2%** | **84.3%** | **85.2%** |

---

## 📚 Documentation Complète

### Guides Utilisateur

- **[QUICKSTART.md](QUICKSTART.md)** - Démarrage rapide (5 min)
- **[docs/guides/CALIBRATION.md](docs/guides/CALIBRATION.md)** - Calibration complète
- **[docs/guides/DEPLOYMENT.md](docs/guides/DEPLOYMENT.md)** - Déploiement système
- **[docs/guides/NETWORK_CONFIG.md](docs/guides/NETWORK_CONFIG.md)** - Configuration réseau
- **[docs/guides/COMPETITION_TRC2025.md](docs/guides/COMPETITION_TRC2025.md)** - Guide compétition
- **[web/README.md](web/README.md)** - Interface web calibration

### Documentation Technique

- **[docs/technical/ARCHITECTURE.md](docs/technical/ARCHITECTURE.md)** - Architecture détaillée
- **[docs/technical/API_REFERENCE.md](docs/technical/API_REFERENCE.md)** - Référence API complète
- **[docs/technical/VISION_NODE.md](docs/technical/VISION_NODE.md)** - Nœud vision YOLOv5
- **[docs/technical/TESTING.md](docs/technical/TESTING.md)** - Guide tests complets

### Entraînement Modèles

- **[trc2025_train_models/README.md](trc2025_train_models/README.md)** - Entraînement YOLOv5

### Navigation

**[📑 INDEX COMPLET](docs/INDEX.md)** - Table des matières de toute la documentation

---

## 🛠️ Dépannage

### Problème : Caméra non détectée

**Erreur :**
```
[camera_node] Erreur: Impossible d'ouvrir /dev/video0
```

**Solution :**
```bash
# Vérifier les périphériques vidéo
ls /dev/video*

# Tester la caméra
v4l2-ctl --list-devices

# Donner les permissions
sudo chmod 666 /dev/video0
```

### Problème : Modèle YOLOv5 non trouvé

**Erreur :**
```
[vision_node] FileNotFoundError: models/best.pt
```

**Solution :**
```bash
# Vérifier le chemin du modèle
cd ~/uca/ucaotech_dofbot_trc2025
ls -lh models/best.pt

# Si absent, le télécharger depuis le dépôt
```

### Problème : Erreur I2C

**Erreur :**
```
[controller_node] IOError: I2C communication failed
```

**Solution :**
```bash
# Vérifier I2C activé
ls /dev/i2c-*

# Activer I2C (si nécessaire)
sudo raspi-config
# Interface Options > I2C > Enable

# Redémarrer
sudo reboot
```

### Problème : ROS package non trouvé

**Erreur :**
```
roslaunch: Cannot locate package ucaotech_dofbot_trc2025
```

**Solution :**
```bash
# Vérifier le lien symbolique
ls -la ~/catkin_ws/src/

# Recréer le lien si nécessaire
cd ~/catkin_ws/src
ln -s ~/uca/ucaotech_dofbot_trc2025/ros_package/ucaotech_dofbot_trc2025 .

# Recompiler
cd ~/catkin_ws
catkin_make

# Sourcer
source devel/setup.bash
```

### Plus d'Aide

Consultez **[docs/INDEX.md](docs/INDEX.md)** pour accéder à tous les guides de dépannage.

---

## 👥 Équipe Ucaotech

**Projet TRC 2025 - Cotonou, Bénin 🇧🇯**

| Rôle | Responsabilité |
|------|----------------|
| **Chef de Projet** | Coordination générale |
| **Développeur IA** | Entraînement modèle YOLOv5 |
| **Développeur ROS** | Nœuds ROS + intégration |
| **Ingénieur Robotique** | Contrôle bras + calibration |
| **Testeur** | Tests + validation système |

---

## 📝 License

Ce projet est développé dans le cadre de la compétition **TRC 2025**.

**Utilisation :** Équipe Ucaotech uniquement.  
**Contact :** [Votre email de contact]

---

## 🔗 Liens Utiles

- **[Documentation Complète](docs/INDEX.md)** - Index de toute la documentation
- **[Guide Démarrage Rapide](QUICKSTART.md)** - Installation en 5 minutes
- **[Résultats Tests](RAPPORT_TESTS_FINAL.md)** - Tests complets
- **[Historique Versions](CHANGELOG.md)** - Changelog
- **[Règlement TRC2025](docs/references/Manuel_TRC2025.pdf)** - Règlement officiel

---

## 📅 Historique du Projet

| Date | Version | Description |
|------|---------|-------------|
| **Oct 2025** | **v1.0** | ✅ Version production pour TRC 2025 |
| Oct 2025 | v0.9 | Interface web calibration |
| Oct 2025 | v0.8 | Tests complets (92% réussite) |
| Sept 2025 | v0.7 | Intégration ROS complète |
| Sept 2025 | v0.6 | Entraînement modèle YOLOv5m |
| Août 2025 | v0.5 | Prototype initial |

---

## 🎯 Statut Actuel

```
╔════════════════════════════════════════════════════╗
║                                                    ║
║       ✅ PROJET PRÊT POUR LA COMPÉTITION           ║
║                                                    ║
║       🏆 TRC 2025 - Cotonou, Bénin 🇧🇯             ║
║                                                    ║
║       📊 Tests: 92% réussite (58/63)               ║
║       🤖 Modèle: 85.2% précision                   ║
║       🚀 Système: Opérationnel                     ║
║       📚 Documentation: Complète                   ║
║                                                    ║
╚════════════════════════════════════════════════════╝
```

---

**🤖 Bonne chance à l'équipe Ucaotech pour TRC 2025 ! 🏆**

*Dernière mise à jour : 16 octobre 2025*

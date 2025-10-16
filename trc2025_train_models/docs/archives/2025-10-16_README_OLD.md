# 🤖 Projet DOFBot - Tri Automatique de Déchets

**TRC2025 - TEKBOT Robotics Challenge**  
Système de classification de déchets avec YOLOv5 pour robot DOFBot Jetson Nano

---

## 🎯 Vue d'ensemble

Système de reconnaissance et classification automatique de déchets en 3 catégories :
- **Dangereux** 🔴 : Objets potentiellement nocifs (batteries, amiante, acides...)
- **Ménagers** ⚫ : Déchets domestiques ordinaires
- **Recyclables** 🟢 : Matériaux recyclables (plastique, carton, verre...)

### 📊 Performances actuelles

| Métrique | Valeur | Status |
|----------|--------|--------|
| **Précision validation** | 85.2% | ✅ Bon |
| **Classe dangereuse** | 100% (9/9) | ⭐ Parfait |
| **Classe ménagers** | 77.8% (7/9) | ⚠️ À surveiller |
| **Classe recyclables** | 77.8% (7/9) | ⚠️ À surveiller |
| **Score estimé compétition** | +160 points | ✅ Compétitif |

---

## 🛠️ Technologies

- **Framework** : YOLOv5 (Ultralytics)
- **Langage** : Python 3.13.5
- **Deep Learning** : PyTorch 2.8.0
- **Modèle** : YOLOv5m (20.8M paramètres)
- **Dataset** : 510 images train + 27 images validation

---

## 📁 Structure du projet

```
dofbot_tri_complete/
├── data/
│   ├── augmented/train/        # 510 images augmentées
│   ├── prepared/val/           # 27 images validation
│   └── dataset.yaml            # Config dataset
│
├── models/
│   ├── trained_models/
│   │   └── garbage_classifier_v1/    # Modèle principal (85.2%)
│   │       └── weights/
│   │           └── best.pt           # Meilleur modèle (40.6 MB)
│   └── yolov5/                       # Framework YOLOv5
│
├── scripts/
│   ├── test_on_competition_dataset.py      # Test validation
│   ├── augment_dataset.py                  # Augmentation dataset
│   ├── train_like_original.py              # Réentraînement
│   ├── prepare_deployment.ps1              # Préparer déploiement
│   └── vision_node_example.py              # Code ROS exemple
│
├── docs/                               # 📚 Documentation complète
│   ├── INDEX.md                        # 🗂️ Navigation docs
│   ├── DEPLOIEMENT_RAPIDE_TLDR.md     # ⚡ START HERE
│   ├── DEPLOIEMENT_ROBOT_JETSON.md    # 📖 Guide complet
│   ├── ARCHITECTURE_DEPLOIEMENT_RESUME.md
│   ├── PROJET_STATUS.md                # 📊 État du projet
│   ├── STRATEGIE_FINALE_TRC2025.md     # 🏆 Stratégie compétition
│   ├── PLAN_ACTION_SUITE.md            # 📋 Prochaines étapes
│   └── ...
│
├── config/                             # Configuration
├── README.md                           # Ce fichier
└── requirements.txt                    # Dépendances Python
```

---

## 🚀 Quick Start

### 1️⃣ Tester le modèle

```bash
python scripts/test_on_competition_dataset.py
```

**Résultat attendu :**
```
✅ 85.2% accuracy (23/27 images correctes)
   - dangereux: 100% (9/9)
   - menagers: 77.8% (7/9)
   - recyclables: 77.8% (7/9)
```

### 2️⃣ Réentraîner (optionnel)

```bash
python scripts/train_like_original.py
```

Durée : 10-12 heures (CPU)

### 3️⃣ Déployer sur robot

**Sur PC Windows :**
```powershell
.\scripts\prepare_deployment.ps1
# → Crée deploy_to_robot/ (~42 MB)
```

**Transférer vers Jetson :**
```bash
scp -r deploy_to_robot/* jetson@<IP>:~/transfer/
```

**Installer sur robot :**
```bash
cd ~/catkin_ws/src/dofbot_tri/
mkdir -p models/yolov5/{models,utils}
cp ~/transfer/best.pt models/
cp ~/transfer/dataset.yaml models/
cp -r ~/transfer/yolov5/* models/yolov5/
```

---

## 📚 Documentation

### 🎯 Par besoin

| Besoin | Document | Temps lecture |
|--------|----------|---------------|
| **Déployer sur robot** | [`docs/DEPLOIEMENT_RAPIDE_TLDR.md`](docs/DEPLOIEMENT_RAPIDE_TLDR.md) | ⚡ 5 min |
| **Guide complet déploiement** | [`docs/DEPLOIEMENT_ROBOT_JETSON.md`](docs/DEPLOIEMENT_ROBOT_JETSON.md) | 📖 20 min |
| **État du projet** | [`docs/PROJET_STATUS.md`](docs/PROJET_STATUS.md) | 📊 10 min |
| **Stratégie compétition** | [`docs/STRATEGIE_FINALE_TRC2025.md`](docs/STRATEGIE_FINALE_TRC2025.md) | 🏆 15 min |
| **Prochaines étapes** | [`docs/PLAN_ACTION_SUITE.md`](docs/PLAN_ACTION_SUITE.md) | 📋 5 min |
| **Navigation complète** | [`docs/INDEX.md`](docs/INDEX.md) | 🗂️ Index |

### 🔗 Liens rapides

- **Manuel officiel TRC2025** : `docs/Manuel de Jeu - TRC25 V3.pdf`
- **Mission bonus (+50pts)** : `docs/MISSION_BONUS_OBJET_INFECTE.md`
- **Analyse compétition** : `docs/ANALYSE_COMPETITION_TRC2025.md`

---

## 🤖 Architecture Robot (ROS)

### Structure requise sur Jetson Nano

```
~/catkin_ws/src/dofbot_tri/
├── CMakeLists.txt
├── package.xml
├── srv/
│   └── Classify.srv              # Service classification
├── scripts/
│   ├── vision_node.py            # Node vision (charge modèle)
│   └── i2c_controller_node.py    # Contrôle bras
├── launch/
│   └── tri.launch                # Launch file
├── models/                        # ⭐ À créer (voir docs)
│   ├── best.pt                   # Modèle YOLOv5 (40.6 MB)
│   ├── dataset.yaml              # Config classes
│   └── yolov5/                   # Framework
│       ├── models/
│       └── utils/
└── config/
    └── arm_positions.yaml
```

**Voir documentation :** [`docs/DEPLOIEMENT_RAPIDE_TLDR.md`](docs/DEPLOIEMENT_RAPIDE_TLDR.md)

---

## 🔧 Installation locale

### Prérequis

- Python 3.8+ (testé avec 3.13.5)
- PyTorch 2.0+
- 8 GB RAM minimum
- 5 GB espace disque

### Installation

```bash
# Cloner le repo
git clone <repo-url>
cd dofbot_tri_complete

# Installer dépendances
pip install -r requirements.txt

# Tester
python scripts/test_on_competition_dataset.py
```

---

## 📊 Historique du projet

### Version actuelle : Post-cleanup v1.0

**Modèle :**
- Architecture : YOLOv5m
- Entraînement : 100 epochs
- Dataset : 510 images train (augmentées) + 27 validation
- Performance : 85.2% validation

**Optimisations effectuées :**
- ✅ Augmentation dataset (102 → 510 images)
- ✅ Nettoyage projet (~700 MB libérés)
- ✅ Documentation complète organisée
- ✅ Scripts de déploiement automatiques

**Prochaines étapes :**
1. Tests physiques avec cubes réels
2. Décision réentraînement (si <80% physique)
3. Optimisation vitesse (TensorRT)
4. Intégration complète DOFBot

---

## 🏆 Compétition TRC2025

### Contexte

- **Lieu** : Abidjan, Côte d'Ivoire
- **Date** : TRC2025
- **Challenge** : Tri automatique de 90 cubes
- **Temps** : 5 minutes

### Scoring

| Type déchet | Points | Erreur |
|-------------|--------|--------|
| Dangereux | +15 pts | -20 pts |
| Recyclable | +10 pts | -20 pts |
| Ménager | +5 pts | -20 pts |

### Stratégie

**Qualité > Quantité**
- Trier 40-50 cubes avec 85-90% précision
- Score estimé : +360 à +425 points
- Focus sur classe dangereuse (100% détection)

**Voir stratégie complète :** [`docs/STRATEGIE_FINALE_TRC2025.md`](docs/STRATEGIE_FINALE_TRC2025.md)

---

## 🛠️ Scripts utiles

### Test et validation

```bash
# Tester sur dataset validation
python scripts/test_on_competition_dataset.py

# Augmenter le dataset (déjà fait)
python scripts/augment_dataset.py
```

### Entraînement

```bash
# Réentraîner avec mêmes paramètres
python scripts/train_like_original.py

# Entraînement avancé
python scripts/train_model_advanced.py
```

### Déploiement

```powershell
# Préparer package pour robot (Windows)
.\scripts\prepare_deployment.ps1
```

---

## 🐛 Troubleshooting

### Modèle ne charge pas

```bash
# Vérifier le fichier existe
ls -lh models/trained_models/garbage_classifier_v1/weights/best.pt
# Doit afficher ~40.6 MB

# Tester le chargement
python -c "import torch; m=torch.hub.load('ultralytics/yolov5','custom',path='models/trained_models/garbage_classifier_v1/weights/best.pt'); print(m.names)"
```

### Erreur "No module named 'yaml'"

```bash
pip install PyYAML
```

### Performance lente

- Utiliser GPU si disponible : `device='cuda:0'`
- Réduire taille image : `img_size=416`
- Exporter en TensorRT (Jetson)

---

## 📝 Notes importantes

### Dataset

- **Train** : 510 images (augmentées)
- **Validation** : 27 images (jamais modifiées)
- **Source** : Dataset ECOCITY officiel TRC2025
- **Classes** : dangereux, menagers, recyclables

### Modèle

- **Fichier** : `models/trained_models/garbage_classifier_v1/weights/best.pt`
- **Taille** : 40.6 MB
- **Format** : PyTorch (.pt)
- **Exports** : ONNX, TFLite, TorchScript disponibles

### Backup

- **Backup sécurité** : `BACKUP_FINAL_20251011_201347/` (232 MB)
- **Ne pas supprimer** jusqu'après compétition !

---

## 🤝 Contribution

Ce projet est développé pour la compétition TRC2025.

**Équipe :** [Votre équipe]  
**Contact :** [Votre contact]

---

## 📄 Licence

- Projet : [Votre licence]
- YOLOv5 : AGPL-3.0 (Ultralytics)

---

## 🎉 Remerciements

- **Ultralytics** pour YOLOv5
- **TEKBOT** pour l'organisation TRC2025
- **PyTorch** pour le framework deep learning

---

## 🚀 Prochaines étapes

### Priorité 1 : Tests physiques (MAINTENANT)

1. Imprimer 10-15 images ECOCITY sur papier
2. Coller sur cubes 3×3 cm
3. Tester avec DOFBot + caméra + modèle actuel
4. Mesurer précision réelle

**Critère décision :**
- Si ≥80% → Optimiser vitesse
- Si <80% → Réentraîner (10-12h)

### Priorité 2 : Optimisation (après tests)

- Export TensorRT pour Jetson Nano
- Optimisation trajectoires bras
- Tests chronométrés complets

### Priorité 3 : Mission bonus (optionnel)

- Contacter organisateur pour image "cube infecté"
- Ajouter 4ème classe au modèle
- Fine-tune 10-20 epochs

**Voir plan complet :** [`docs/PLAN_ACTION_SUITE.md`](docs/PLAN_ACTION_SUITE.md)

---

<div align="center">

**🏆 OBJECTIF : 1ère place pour le TRC2025 ! 🚀**

[📚 Documentation](docs/) | [⚡ Déploiement](docs/DEPLOIEMENT_RAPIDE_TLDR.md) | [🏆 Stratégie](docs/STRATEGIE_FINALE_TRC2025.md)

</div>

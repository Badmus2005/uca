# ⚡ DÉPLOIEMENT RAPIDE - TL;DR

> **Question :** Mon dossier `models/` sur le robot doit contenir quoi ?  
> **Réponse courte :** 3 choses essentielles (42 MB total)

---

## 🎯 RÉPONSE DIRECTE

Votre architecture robot doit ressembler à ça :

```
~/catkin_ws/src/dofbot_tri/
├── scripts/
│   └── vision_node.py          ← Utilise le modèle
├── models/                      ← ⭐ VOTRE QUESTION
│   ├── best.pt                  ← 1️⃣ Votre modèle (40.6 MB)
│   ├── dataset.yaml            ← 2️⃣ Config classes (1 KB)
│   └── yolov5/                  ← 3️⃣ Framework (1.5 MB)
│       ├── models/
│       └── utils/
└── config/
    └── arm_positions.yaml
```

---

## 📦 3 FICHIERS ESSENTIELS À COPIER

| # | Quoi | D'où (votre PC) | Vers (robot) | Taille |
|---|------|-----------------|--------------|--------|
| 1️⃣ | **Modèle** | `models/trained_models/garbage_classifier_v1/weights/best.pt` | `~/catkin_ws/src/dofbot_tri/models/best.pt` | 40.6 MB |
| 2️⃣ | **Classes** | `data/dataset.yaml` | `~/catkin_ws/src/dofbot_tri/models/dataset.yaml` | 1 KB |
| 3️⃣ | **Framework** | `models/yolov5/` (dossiers models/ et utils/) | `~/catkin_ws/src/dofbot_tri/models/yolov5/` | 1.5 MB |

**Total : ~42 MB**

---

## 🚀 PROCÉDURE EN 3 ÉTAPES

### Étape 1 : Préparer (sur votre PC Windows)
```powershell
cd D:\TRC2025\docTel\TRC_Doc\TRC\dofbot_tri_complete
.\scripts\prepare_deployment.ps1
# → Crée deploy_to_robot/ avec tout dedans
```

### Étape 2 : Transférer (PC → Jetson)
```bash
# Via SCP
scp -r deploy_to_robot/* jetson@<IP_JETSON>:~/transfer/

# OU via USB
# Copier deploy_to_robot/ sur clé USB
```

### Étape 3 : Installer (sur Jetson Nano)
```bash
ssh jetson@<IP_JETSON>

# Créer structure
cd ~/catkin_ws/src/dofbot_tri/
mkdir -p models/yolov5/{models,utils}

# Copier
cp ~/transfer/best.pt models/
cp ~/transfer/dataset.yaml models/
cp -r ~/transfer/yolov5/models/* models/yolov5/models/
cp -r ~/transfer/yolov5/utils/* models/yolov5/utils/
cp ~/transfer/yolov5/__init__.py models/yolov5/

# Vérifier
ls -lh models/best.pt  # Doit afficher ~41M
```

**C'est tout ! ✅**

---

## 💻 CODE POUR `vision_node.py`

Votre `vision_node.py` charge le modèle comme ça :

```python
#!/usr/bin/env python3
import sys
from pathlib import Path

# Ajouter YOLOv5 au path
FILE = Path(__file__).resolve()
YOLOV5_ROOT = FILE.parents[1] / 'models' / 'yolov5'
sys.path.append(str(YOLOV5_ROOT))

# Importer YOLOv5
from models.common import DetectMultiBackend
from utils.general import non_max_suppression

# Charger le modèle
model_path = str(FILE.parents[1] / 'models' / 'best.pt')
model = DetectMultiBackend(model_path, device='cuda:0')

# Utiliser pour classifier
# pred = model(image)
```

**Exemple complet :** `scripts/vision_node_example.py`

---

## 📚 DOCUMENTATION COMPLÈTE

| Document | Usage |
|----------|-------|
| `docs/DEPLOIEMENT_ROBOT_JETSON.md` | 📖 Guide complet détaillé (22 KB) |
| `docs/ARCHITECTURE_DEPLOIEMENT_RESUME.md` | 🗺️ Schémas et flux (13 KB) |
| `scripts/prepare_deployment.ps1` | 🔧 Script automatique (10 KB) |
| `scripts/vision_node_example.py` | 💻 Code ROS complet (14 KB) |

---

## ❓ FAQ RAPIDE

### Q1 : Je copie tout le dossier `yolov5/` ?
**R :** Non ! Seulement ces sous-dossiers :
- `yolov5/models/` (architectures)
- `yolov5/utils/` (utilitaires)
- `yolov5/__init__.py`

Le script `prepare_deployment.ps1` s'en occupe automatiquement.

---

### Q2 : Pourquoi 42 MB seulement ?
**R :** Car on ne copie que l'essentiel :
- ❌ Pas les images d'entraînement
- ❌ Pas les logs
- ❌ Pas les backups
- ✅ Juste le modèle + code nécessaire

---

### Q3 : Et le fichier `last.pt` ?
**R :** Pas besoin ! `best.pt` suffit (c'est le meilleur modèle).

---

### Q4 : Comment tester que ça marche ?
**R :** Sur le Jetson après installation :
```bash
cd ~/catkin_ws/src/dofbot_tri/models/
python3 -c "
import torch
m = torch.hub.load('yolov5', 'custom', path='best.pt', source='local')
print('✅ OK:', m.names)
"
```

Devrait afficher :
```
✅ OK: ['dangereux', 'menagers', 'recyclables']
```

---

### Q5 : Ça marche sur CPU aussi ?
**R :** Oui mais lent (500ms). GPU recommandé (200ms).

Changer dans `vision_node.py` :
```python
# CPU
model = DetectMultiBackend(model_path, device='cpu')

# GPU (recommandé sur Jetson)
model = DetectMultiBackend(model_path, device='cuda:0')
```

---

## 🎯 EN RÉSUMÉ

**Votre question :** *"mon dossier models/ doit renfermer quoi precisement..."*

**Réponse :**
```
models/
├── best.pt           ← Votre modèle entraîné
├── dataset.yaml     ← Vos 3 classes
└── yolov5/          ← Code YOLOv5 minimal
    ├── models/
    └── utils/
```

**Action :** Exécuter `.\scripts\prepare_deployment.ps1` qui prépare tout automatiquement ! 🚀

---

**Besoin de plus de détails ?** → `docs/DEPLOIEMENT_ROBOT_JETSON.md`  
**Prêt à déployer ?** → `.\scripts\prepare_deployment.ps1` 🎉

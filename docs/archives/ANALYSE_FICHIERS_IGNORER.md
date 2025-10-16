# 📊 Analyse des Fichiers à Ignorer sur GitHub

**Date** : 16 octobre 2025  
**Objectif** : Identifier les dossiers/fichiers à exclure du repository GitHub

---

## 🔍 Analyse par Taille

### Dossiers Volumineux Détectés

| Dossier | Taille | Fichiers | Recommandation |
|---------|--------|----------|----------------|
| **REFERENCES/** | **1.34 GB** | **3,738** | ❌ **À EXCLURE** |
| **trc2025_train_models/data/** | 160 MB | 1,078 | ❌ **À EXCLURE** |
| **trc2025_train_models/models/** | 91 MB | 227 | ❌ **À EXCLURE** |
| **models/** (racine) | 41 MB | 19 | ❌ **À EXCLURE** |
| **docs/references/** | ~1 MB | 2 | ⚠️ **PARTIEL** (.pdf exclus) |

**Total à exclure** : ~1.63 GB

---

## 📁 Analyse Détaillée par Dossier

### 1. ❌ REFERENCES/ (1.34 GB - 3,738 fichiers)

**Contenu** :
- `Dofbot_original/` - Code source original du fabricant
- `dofbot_ws_examples/` - Exemples ROS workspace
- `dofbot_ws_src/` - Source code ROS packages (ar_track_alvar, etc.)

**Raison d'exclusion** :
- ✅ Code de référence du fabricant (disponible publiquement)
- ✅ Exemples non utilisés dans le projet
- ✅ Taille excessive (1.34 GB)
- ✅ Code ROS tiers (ar_track_alvar, yahboomcar_description, etc.)

**Recommandation** : **EXCLURE TOTALEMENT**
- Créer un README.md dans REFERENCES/ expliquant où trouver ces ressources
- Ajouter `REFERENCES/` au .gitignore

---

### 2. ❌ trc2025_train_models/data/ (160 MB - 1,078 fichiers)

**Contenu** :
- `augmented/` - Images augmentées pour entraînement
- `prepared/` - Dataset préparé
- `dataset.yaml` - Configuration (à garder)

**Raison d'exclusion** :
- ✅ Images de training (volumineuses)
- ✅ Données spécifiques à votre robot
- ✅ Copyright potentiel (images TRC2025)
- ✅ Chaque équipe a son propre dataset

**Recommandation** : **EXCLURE (sauf dataset.yaml)**
- Garder `dataset.yaml` et `.gitkeep`
- Exclure `augmented/` et `prepared/`

---

### 3. ❌ trc2025_train_models/models/ (91 MB - 227 fichiers)

**Contenu** :
- `trained_models/` - Modèles YOLOv5 entraînés (*.pt)
- `yolov5/` - Poids YOLOv5 pré-entraînés
- `class_names.json` - Noms des classes (à garder)

**Raison d'exclusion** :
- ✅ Fichiers binaires lourds (*.pt, *.pth)
- ✅ Modèles spécifiques à votre entraînement
- ✅ GitHub limite à 100 MB par fichier
- ✅ Pas nécessaire pour reproduire le projet

**Recommandation** : **EXCLURE (sauf class_names.json)**
- Garder `class_names.json` et `.gitkeep`
- Exclure tous les *.pt, *.pth, *.weights

---

### 4. ❌ models/ (racine - 41 MB - 19 fichiers)

**Contenu** :
- Modèles ML entraînés du projet principal

**Raison d'exclusion** :
- ✅ Fichiers binaires lourds
- ✅ Peuvent être régénérés via entraînement

**Recommandation** : **EXCLURE**
- Créer un README.md expliquant comment entraîner/télécharger les modèles

---

### 5. ⚠️ docs/references/ (1 MB)

**Contenu** :
- `HARDWARE_SPECS.md` ✅ À garder
- `Manuel de Jeu - TRC25 V3.pdf` ❌ À exclure (copyright)

**Recommandation** : **PARTIEL**
- Garder les .md
- Exclure les .pdf

---

## 📝 Autres Fichiers à Exclure

### Configurations Spécifiques Robot

```
config/robot_*.yaml          # Configuration robot spécifique
config/calibration_*.json    # Données de calibration
config/camera_calibration_*.npz  # Calibration caméra
```

### Fichiers Temporaires/Cache

```
__pycache__/                 # Python cache
*.pyc, *.pyo                 # Python compiled
.pytest_cache/               # Pytest cache
logs/                        # Logs d'exécution
*.log                        # Fichiers log
```

### Environnements Virtuels

```
venv/                        # Virtual environment
ENV/, env/, .venv            # Autres venv
```

---

## ✅ Résumé des Actions

### À Ajouter au .gitignore

```gitignore
# ==========================================
# DOSSIERS DE RÉFÉRENCE (1.34 GB)
# ==========================================
REFERENCES/

# ==========================================
# MODÈLES ML (130+ MB)
# ==========================================
models/
trc2025_train_models/models/trained_models/
trc2025_train_models/models/yolov5/
*.pt
*.pth
*.weights
*.onnx
*.engine
*.h5
*.pb

# ==========================================
# DATASETS (160+ MB)
# ==========================================
trc2025_train_models/data/augmented/
trc2025_train_models/data/prepared/
trc2025_train_models/data/**/*.jpg
trc2025_train_models/data/**/*.png
trc2025_train_models/data/**/*.jpeg

# ==========================================
# PDF (Copyright)
# ==========================================
docs/references/*.pdf
Manuel*.pdf

# ==========================================
# CONFIGS ROBOT SPÉCIFIQUES
# ==========================================
config/robot_*.yaml
config/calibration_*.json
config/camera_calibration_*.npz
```

---

## 📋 README à Créer

### REFERENCES/README.md

```markdown
# 📚 REFERENCES

⚠️ **Ce dossier n'est PAS versionné sur GitHub** (1.34 GB)

## Contenu Local

- `Dofbot_original/` - Code source original Yahboom DOFbot
- `dofbot_ws_examples/` - Exemples ROS workspace
- `dofbot_ws_src/` - Packages ROS tiers

## Comment Obtenir Ces Fichiers

### DOFbot Original
Télécharger depuis : [Yahboom DOFbot](https://github.com/YahboomTechnology/Dofbot-jetson_nano)

### Packages ROS
```bash
sudo apt-get install ros-noetic-ar-track-alvar
# Autres packages selon besoins
```

## Pourquoi Pas sur GitHub ?

1. **Taille** : 1.34 GB (limite GitHub)
2. **Redondance** : Code disponible publiquement
3. **Copyright** : Code tiers avec licences variées
4. **Inutile** : Référence uniquement, pas utilisé directement
```

### models/README.md

```markdown
# 🤖 Modèles ML

⚠️ **Ce dossier n'est PAS versionné sur GitHub** (41 MB)

## Modèles Entraînés

Les modèles entraînés ne sont pas versionnés car :
- Fichiers binaires volumineux (> 40 MB)
- Spécifiques à votre dataset
- Peuvent être régénérés

## Comment Obtenir les Modèles

### Option 1 : Entraîner
```bash
cd trc2025_train_models
python scripts/train_yolo.py
```

### Option 2 : Télécharger (si disponible)
Voir `trc2025_train_models/README.md` section "Pre-trained Models"

## Formats Supportés

- YOLOv5 (*.pt, *.pth)
- ONNX (*.onnx)
- TensorRT (*.engine)
```

---

## 🎯 Bénéfices

### Avant Exclusion
- **Taille repo** : ~1.63 GB
- **Fichiers** : ~5,000+
- **Clone time** : 10-15 minutes
- **GitHub LFS** : Requis

### Après Exclusion
- **Taille repo** : ~50-100 MB
- **Fichiers** : ~200-300
- **Clone time** : 30-60 secondes
- **GitHub LFS** : Non requis

---

## ✅ Checklist Finale

- [ ] Ajouter REFERENCES/ au .gitignore
- [ ] Créer REFERENCES/README.md
- [ ] Ajouter models/ au .gitignore
- [ ] Créer models/README.md
- [ ] Exclure trc2025_train_models/data/augmented/
- [ ] Exclure trc2025_train_models/data/prepared/
- [ ] Exclure trc2025_train_models/models/trained_models/
- [ ] Exclure trc2025_train_models/models/yolov5/
- [ ] Exclure docs/references/*.pdf
- [ ] Créer .gitkeep dans dossiers vides
- [ ] Tester avec `git status` (doit montrer < 100 MB)

---

**🎊 Résultat : Repository GitHub propre, léger et professionnel !**

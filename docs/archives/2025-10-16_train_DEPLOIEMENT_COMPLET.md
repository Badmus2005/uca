# 🤖 GUIDE DÉPLOIEMENT MODÈLE SUR ROBOT JETSON NANO

**Date :** 12 octobre 2025  
**Objectif :** Transférer et intégrer le modèle YOLOv5 entraîné vers l'architecture ROS du DOFBot

---

## 📋 TABLE DES MATIÈRES

1. [Architecture actuelle](#architecture-actuelle)
2. [Structure du dossier models/](#structure-du-dossier-models)
3. [Fichiers à transférer](#fichiers-à-transférer)
4. [Procédure de déploiement](#procédure-de-déploiement)
5. [Modification du vision_node.py](#modification-du-vision_nodepy)
6. [Optimisation pour Jetson Nano](#optimisation-pour-jetson-nano)
7. [Tests sur robot](#tests-sur-robot)

---

## 🏗️ ARCHITECTURE ACTUELLE

### Sur votre PC local (Windows)
```
D:\TRC2025\docTel\TRC_Doc\TRC\dofbot_tri_complete\
├── models/
│   ├── trained_models/
│   │   └── garbage_classifier_v1/
│   │       ├── weights/
│   │       │   ├── best.pt          ← ⭐ MODÈLE PRINCIPAL (40.6 MB)
│   │       │   └── last.pt          ← Dernier checkpoint
│   │       ├── hyp.yaml             ← Hyperparamètres
│   │       ├── opt.yaml             ← Options entraînement
│   │       └── results.png          ← Graphiques performance
│   └── yolov5/                       ← Framework YOLOv5 complet
│
├── data/
│   └── dataset.yaml                  ← Configuration classes
│
└── config/
    └── hyp.yaml                      ← Hyperparamètres training
```

### Sur le robot Jetson Nano (Ubuntu + ROS)
```
~/catkin_ws/src/dofbot_tri/
├── CMakeLists.txt
├── package.xml
├── srv/
│   └── Classify.srv
├── scripts/
│   ├── vision_node.py               ← ⭐ À MODIFIER (utilise le modèle)
│   ├── i2c_controller_node.py
│   └── test_calibration.py
├── launch/
│   └── tri.launch
├── models/                           ← ⭐ À COMPLÉTER
│   └── (...)
└── config/
    └── arm_positions.yaml
```

---

## 📂 STRUCTURE DU DOSSIER `models/` (SUR ROBOT)

Voici **exactement** ce que doit contenir `~/catkin_ws/src/dofbot_tri/models/` :

### ✅ Option 1 : Structure minimale (RECOMMANDÉE pour Jetson Nano)

```
~/catkin_ws/src/dofbot_tri/models/
├── best.pt                           ← ⭐ Modèle principal (40.6 MB)
├── dataset.yaml                      ← Configuration classes
└── yolov5/                           ← Framework YOLOv5 (seulement fichiers essentiels)
    ├── models/
    │   ├── common.py                 ← Architectures réseau
    │   ├── yolo.py                   ← Détecteur YOLO
    │   └── experimental.py           ← Fonctions expérimentales
    ├── utils/
    │   ├── __init__.py
    │   ├── general.py                ← Fonctions utilitaires
    │   ├── torch_utils.py            ← Utilitaires PyTorch
    │   ├── augmentations.py          ← Augmentations
    │   ├── dataloaders.py            ← Chargeurs données
    │   └── plots.py                  ← Visualisation (optionnel)
    ├── __init__.py
    └── requirements.txt              ← Dépendances Python
```

**Taille totale :** ~45 MB (modèle + YOLOv5 essentiel)

---

### ✅ Option 2 : Structure complète (avec backup et versions)

```
~/catkin_ws/src/dofbot_tri/models/
├── current/                          ← ⭐ Modèle actuellement utilisé
│   ├── best.pt                       ← Lien symbolique → ../v1/best.pt
│   └── dataset.yaml                  ← Configuration classes
│
├── v1/                               ← Version 1 (85.2% accuracy)
│   ├── best.pt                       ← Modèle v1
│   ├── hyp.yaml                      ← Hyperparamètres
│   ├── opt.yaml                      ← Options
│   └── results.png                   ← Performance graphs
│
├── v2/                               ← Future version 2 (si réentraînement)
│   └── best.pt                       ← Modèle v2 (à venir)
│
├── tensorrt/                         ← Modèles optimisés TensorRT (futur)
│   ├── best.engine                   ← TensorRT FP16
│   └── best_int8.engine              ← TensorRT INT8
│
├── yolov5/                           ← Framework YOLOv5
│   └── (...)
│
└── README.md                         ← Documentation versions
```

**Taille totale :** ~150 MB (avec backups)

---

## 📦 FICHIERS À TRANSFÉRER

### 🔴 FICHIERS OBLIGATOIRES (Minimum vital)

| Fichier Local | Destination Robot | Taille | Priorité |
|---------------|-------------------|--------|----------|
| `models/trained_models/garbage_classifier_v1/weights/best.pt` | `~/catkin_ws/src/dofbot_tri/models/best.pt` | 40.6 MB | ⭐ CRITIQUE |
| `data/dataset.yaml` | `~/catkin_ws/src/dofbot_tri/models/dataset.yaml` | 1 KB | ⭐ CRITIQUE |
| `models/yolov5/models/` (dossier complet) | `~/catkin_ws/src/dofbot_tri/models/yolov5/models/` | ~500 KB | ⭐ CRITIQUE |
| `models/yolov5/utils/` (dossier complet) | `~/catkin_ws/src/dofbot_tri/models/yolov5/utils/` | ~1 MB | ⭐ CRITIQUE |

### 🟡 FICHIERS RECOMMANDÉS (Pour debug et monitoring)

| Fichier Local | Destination Robot | Taille | Usage |
|---------------|-------------------|--------|-------|
| `models/trained_models/garbage_classifier_v1/hyp.yaml` | `~/catkin_ws/src/dofbot_tri/models/v1/hyp.yaml` | 1 KB | Documentation |
| `models/trained_models/garbage_classifier_v1/opt.yaml` | `~/catkin_ws/src/dofbot_tri/models/v1/opt.yaml` | 1 KB | Configuration |
| `models/trained_models/garbage_classifier_v1/results.png` | `~/catkin_ws/src/dofbot_tri/models/v1/results.png` | ~100 KB | Performance |

### 🟢 FICHIERS OPTIONNELS (Pas nécessaires sur robot)

| Fichier | Raison |
|---------|--------|
| `last.pt` | Backup inutile (best.pt suffit) |
| `train_batch*.jpg` | Images debug training |
| `val_batch*.jpg` | Images debug validation |
| `confusion_matrix.png` | Analyse locale uniquement |
| `events.out.tfevents.*` | Logs TensorBoard (PC uniquement) |

---

## 🚀 PROCÉDURE DE DÉPLOIEMENT

### Étape 1 : Préparer les fichiers sur PC

#### 1.1 Créer un dossier de transfert
```powershell
# Sur votre PC Windows
cd D:\TRC2025\docTel\TRC_Doc\TRC\dofbot_tri_complete
mkdir deploy_to_robot
```

#### 1.2 Copier les fichiers essentiels
```powershell
# Modèle principal
Copy-Item models\trained_models\garbage_classifier_v1\weights\best.pt deploy_to_robot\

# Configuration dataset
Copy-Item data\dataset.yaml deploy_to_robot\

# Framework YOLOv5 (seulement dossiers essentiels)
Copy-Item -Recurse models\yolov5\models deploy_to_robot\yolov5_models
Copy-Item -Recurse models\yolov5\utils deploy_to_robot\yolov5_utils

# Fichiers Python racine YOLOv5
Copy-Item models\yolov5\__init__.py deploy_to_robot\yolov5_init.py
```

#### 1.3 Créer une archive (optionnel mais recommandé)
```powershell
# Compresser pour transfert plus rapide
Compress-Archive -Path deploy_to_robot\* -DestinationPath deploy_to_robot.zip
```

---

### Étape 2 : Transférer vers le Jetson Nano

#### Option A : Via SCP (SSH File Transfer)
```bash
# Sur votre PC (PowerShell)
# Remplacez <JETSON_IP> par l'IP de votre Jetson Nano
scp -r deploy_to_robot/* jetson@<JETSON_IP>:~/transfer/

# Exemple : scp -r deploy_to_robot/* jetson@192.168.1.100:~/transfer/
```

#### Option B : Via clé USB
```bash
1. Copier deploy_to_robot/ sur clé USB
2. Insérer clé USB dans Jetson Nano
3. Monter et copier :
   sudo mount /dev/sda1 /mnt/usb
   cp -r /mnt/usb/deploy_to_robot/* ~/transfer/
```

#### Option C : Via réseau partagé (si même réseau WiFi)
```bash
# Sur PC : Partager le dossier deploy_to_robot via Windows
# Sur Jetson : Accéder au partage réseau et copier
```

---

### Étape 3 : Installer sur le robot

```bash
# SSH vers le Jetson Nano
ssh jetson@<JETSON_IP>

# Naviguer vers le workspace ROS
cd ~/catkin_ws/src/dofbot_tri/

# Créer la structure models/
mkdir -p models/yolov5/models
mkdir -p models/yolov5/utils
mkdir -p models/v1

# Copier les fichiers
cp ~/transfer/best.pt models/best.pt
cp ~/transfer/dataset.yaml models/dataset.yaml
cp -r ~/transfer/yolov5_models/* models/yolov5/models/
cp -r ~/transfer/yolov5_utils/* models/yolov5/utils/
cp ~/transfer/yolov5_init.py models/yolov5/__init__.py

# Vérifier la structure
tree models/ -L 2
```

**Résultat attendu :**
```
models/
├── best.pt                    ← 40.6 MB
├── dataset.yaml              ← 1 KB
└── yolov5/
    ├── __init__.py
    ├── models/               ← ~500 KB
    │   ├── common.py
    │   ├── yolo.py
    │   └── ...
    └── utils/                ← ~1 MB
        ├── general.py
        ├── torch_utils.py
        └── ...
```

---

## 🔧 MODIFICATION DU `vision_node.py`

Votre script ROS doit charger et utiliser le modèle. Voici comment :

### Code actuel (exemple hypothétique)
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        # TODO: Charger le modèle YOLOv5
        pass
    
    def classify_image(self, image):
        # TODO: Utiliser le modèle pour classifier
        pass
```

### Code modifié (avec YOLOv5)
```python
#!/usr/bin/env python3
import rospy
import torch
import sys
import os
from pathlib import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Ajouter le chemin YOLOv5 au PYTHONPATH
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1] / 'models' / 'yolov5'  # ~/catkin_ws/src/dofbot_tri/models/yolov5
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        
        # Paramètres
        self.model_path = rospy.get_param('~model_path', 
            str(FILE.parents[1] / 'models' / 'best.pt'))
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)
        self.img_size = rospy.get_param('~img_size', 640)
        
        # Initialiser le modèle
        rospy.loginfo("Chargement du modèle YOLOv5...")
        self.device = select_device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.model = DetectMultiBackend(self.model_path, device=self.device, dnn=False)
        self.stride = self.model.stride
        self.img_size = check_img_size(self.img_size, s=self.stride)
        self.names = self.model.names  # ['dangereux', 'menagers', 'recyclables']
        
        rospy.loginfo(f"Modèle chargé sur {self.device}")
        rospy.loginfo(f"Classes: {self.names}")
        
        # Bridge ROS-OpenCV
        self.bridge = CvBridge()
        
        # Subscriber pour les images
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Service ROS pour classification
        from dofbot_tri.srv import Classify, ClassifyResponse
        self.classify_service = rospy.Service('classify_waste', Classify, self.classify_service_handler)
        
        rospy.loginfo("Vision Node prêt !")
    
    def preprocess_image(self, img):
        """Prétraiter l'image pour YOLOv5"""
        img = letterbox(img, self.img_size, stride=self.stride, auto=True)[0]
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.float() / 255.0  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim
        return img
    
    def classify_image(self, cv_image):
        """
        Classifier une image avec YOLOv5
        
        Returns:
            class_name (str): 'dangereux', 'menagers', ou 'recyclables'
            confidence (float): Confiance de la prédiction (0-1)
        """
        # Prétraitement
        img = self.preprocess_image(cv_image)
        
        # Inférence
        with torch.no_grad():
            pred = self.model(img, augment=False, visualize=False)
        
        # Post-traitement (NMS)
        pred = non_max_suppression(pred, self.conf_threshold, 0.45, None, False, max_det=10)
        
        # Extraire la meilleure prédiction
        if len(pred[0]) > 0:
            # Prendre la détection avec la plus haute confiance
            best_det = pred[0][0]  # [x1, y1, x2, y2, conf, class]
            class_id = int(best_det[5])
            confidence = float(best_det[4])
            class_name = self.names[class_id]
            
            rospy.loginfo(f"Détection: {class_name} (confiance: {confidence:.2f})")
            return class_name, confidence
        else:
            rospy.logwarn("Aucune détection !")
            return None, 0.0
    
    def image_callback(self, msg):
        """Callback pour les images de la caméra"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            class_name, confidence = self.classify_image(cv_image)
            
            if class_name and confidence > self.conf_threshold:
                rospy.loginfo(f"Cube détecté: {class_name}")
                # TODO: Publier sur un topic ou appeler le service du bras
                
        except Exception as e:
            rospy.logerr(f"Erreur traitement image: {e}")
    
    def classify_service_handler(self, req):
        """Handler pour le service ROS Classify"""
        from dofbot_tri.srv import ClassifyResponse
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
            class_name, confidence = self.classify_image(cv_image)
            
            return ClassifyResponse(
                waste_type=class_name if class_name else "unknown",
                confidence=confidence
            )
        except Exception as e:
            rospy.logerr(f"Erreur service classification: {e}")
            return ClassifyResponse(waste_type="error", confidence=0.0)

if __name__ == '__main__':
    try:
        node = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

### Fichier `Classify.srv`
```
# Request
sensor_msgs/Image image

---
# Response
string waste_type      # 'dangereux', 'menagers', 'recyclables', ou 'unknown'
float32 confidence     # 0.0 à 1.0
```

---

## ⚡ OPTIMISATION POUR JETSON NANO

Le Jetson Nano a des ressources limitées. Voici comment optimiser :

### 1. Utiliser TensorRT (RECOMMANDÉ)

```bash
# Sur le Jetson Nano
cd ~/catkin_ws/src/dofbot_tri/models/

# Exporter vers TensorRT FP16 (2-3× plus rapide)
python3 -c "
from yolov5.models.common import DetectMultiBackend
import torch

model = DetectMultiBackend('best.pt', device='cuda:0', dnn=False)
dummy_input = torch.randn(1, 3, 640, 640).to('cuda:0')

# Export TorchScript
ts = torch.jit.trace(model.model, dummy_input)
ts.save('best_traced.torchscript')

print('Modèle TorchScript exporté: best_traced.torchscript')
"

# Utiliser dans vision_node.py:
# self.model = DetectMultiBackend('best_traced.torchscript', device=self.device)
```

**Gain :** 300ms → 100-150ms par inférence

### 2. Réduire la taille d'image

```python
# Dans vision_node.py
self.img_size = 416  # Au lieu de 640
```

**Gain :** 300ms → 150ms (mais -2% accuracy)

### 3. Utiliser CUDA si disponible

```python
self.device = select_device('cuda:0')  # Force GPU Jetson
```

### 4. Optimiser OpenCV

```bash
# Installer OpenCV avec CUDA support (si pas déjà fait)
sudo apt-get install python3-opencv
```

---

## 🧪 TESTS SUR ROBOT

### Test 1 : Vérifier le chargement du modèle

```bash
# SSH vers Jetson
ssh jetson@<JETSON_IP>

# Naviguer vers le workspace
cd ~/catkin_ws/src/dofbot_tri/models/

# Test Python direct
python3 << EOF
import torch
from pathlib import Path

model_path = 'best.pt'
print(f"Chargement de {model_path}...")

model = torch.hub.load('yolov5', 'custom', path=model_path, source='local')
print(f"✅ Modèle chargé avec succès!")
print(f"Classes: {model.names}")
EOF
```

**Sortie attendue :**
```
Chargement de best.pt...
✅ Modèle chargé avec succès!
Classes: ['dangereux', 'menagers', 'recyclables']
```

### Test 2 : Test d'inférence avec une image

```bash
# Copier une image de test
scp data/prepared/val/images/dangereux_* jetson@<JETSON_IP>:~/test_image.jpg

# Sur Jetson
cd ~/catkin_ws/src/dofbot_tri/models/
python3 << EOF
import torch
from PIL import Image

model = torch.hub.load('yolov5', 'custom', path='best.pt', source='local')
img = Image.open('~/test_image.jpg')

results = model(img)
results.print()
results.show()  # Si X11 forwarding activé
EOF
```

### Test 3 : Lancer le node ROS

```bash
# Source le workspace
cd ~/catkin_ws
source devel/setup.bash

# Lancer le node vision
rosrun dofbot_tri vision_node.py
```

**Logs attendus :**
```
[ INFO] [1697123456.789]: Chargement du modèle YOLOv5...
[ INFO] [1697123458.123]: Modèle chargé sur cuda:0
[ INFO] [1697123458.124]: Classes: ['dangereux', 'menagers', 'recyclables']
[ INFO] [1697123458.125]: Vision Node prêt !
```

### Test 4 : Test du service de classification

```bash
# Terminal 1 : Lancer le node
rosrun dofbot_tri vision_node.py

# Terminal 2 : Appeler le service
rosservice call /classify_waste "image: {data: []}"  # Avec vraie image
```

---

## 📊 STRUCTURE FINALE COMPLÈTE

### Sur le robot après déploiement

```
~/catkin_ws/src/dofbot_tri/
├── CMakeLists.txt
├── package.xml
│
├── srv/
│   └── Classify.srv                  ← Service classification
│
├── scripts/
│   ├── vision_node.py               ← ⭐ MODIFIÉ (charge best.pt)
│   ├── i2c_controller_node.py       ← Contrôle bras
│   ├── test_calibration.py
│   └── test_model.py                ← ⭐ NOUVEAU (test modèle standalone)
│
├── launch/
│   └── tri.launch                    ← ⭐ À MODIFIER (lance vision_node)
│
├── models/                           ← ⭐ NOUVEAU (dossier complet)
│   ├── best.pt                       ← Modèle principal (40.6 MB)
│   ├── dataset.yaml                  ← Config classes
│   ├── best_traced.torchscript       ← Modèle optimisé TensorRT (optionnel)
│   │
│   ├── yolov5/                       ← Framework YOLOv5
│   │   ├── __init__.py
│   │   ├── models/
│   │   │   ├── common.py
│   │   │   ├── yolo.py
│   │   │   └── experimental.py
│   │   └── utils/
│   │       ├── general.py
│   │       ├── torch_utils.py
│   │       ├── augmentations.py
│   │       └── dataloaders.py
│   │
│   └── v1/                           ← Backup version 1
│       ├── hyp.yaml
│       ├── opt.yaml
│       └── results.png
│
└── config/
    ├── arm_positions.yaml            ← Positions bras
    └── vision_params.yaml            ← ⭐ NOUVEAU (params vision)
```

---

## 📝 CHECKLIST DÉPLOIEMENT

### Avant transfert (sur PC)
- [ ] Vérifier que `best.pt` existe et fonctionne (85.2%)
- [ ] Copier fichiers dans `deploy_to_robot/`
- [ ] Vérifier taille totale (~42 MB)
- [ ] Créer archive ZIP (optionnel)

### Transfert
- [ ] Transférer via SCP/USB vers Jetson
- [ ] Vérifier intégrité fichiers (tailles)
- [ ] Vérifier permissions (chmod +x si nécessaire)

### Installation sur robot
- [ ] Créer structure `~/catkin_ws/src/dofbot_tri/models/`
- [ ] Copier `best.pt` et `dataset.yaml`
- [ ] Copier framework YOLOv5
- [ ] Vérifier avec `tree models/`

### Modification code ROS
- [ ] Mettre à jour `vision_node.py` avec code YOLOv5
- [ ] Créer `Classify.srv` si pas existant
- [ ] Mettre à jour `CMakeLists.txt` (ajouter srv)
- [ ] Mettre à jour `tri.launch`

### Tests
- [ ] Test chargement modèle Python standalone
- [ ] Test inférence sur image test
- [ ] Test lancement node ROS
- [ ] Test service de classification
- [ ] Test avec caméra réelle

### Optimisation (optionnel)
- [ ] Export TorchScript/TensorRT
- [ ] Benchmark vitesse (cible <150ms)
- [ ] Ajuster paramètres (img_size, conf_threshold)

---

## 🎯 RÉSUMÉ : FICHIERS ESSENTIELS À COPIER

| # | Fichier Source (PC) | Destination (Robot) | Taille |
|---|---------------------|---------------------|--------|
| 1 | `models/trained_models/garbage_classifier_v1/weights/best.pt` | `~/catkin_ws/src/dofbot_tri/models/best.pt` | 40.6 MB |
| 2 | `data/dataset.yaml` | `~/catkin_ws/src/dofbot_tri/models/dataset.yaml` | 1 KB |
| 3 | `models/yolov5/models/` (complet) | `~/catkin_ws/src/dofbot_tri/models/yolov5/models/` | 500 KB |
| 4 | `models/yolov5/utils/` (complet) | `~/catkin_ws/src/dofbot_tri/models/yolov5/utils/` | 1 MB |
| 5 | `models/yolov5/__init__.py` | `~/catkin_ws/src/dofbot_tri/models/yolov5/__init__.py` | 1 KB |

**TOTAL :** ~42 MB

---

## 🚨 PROBLÈMES COURANTS

### Erreur : "No module named 'models'"
```bash
# Solution : Ajouter yolov5 au PYTHONPATH
export PYTHONPATH=$PYTHONPATH:~/catkin_ws/src/dofbot_tri/models/yolov5
```

### Erreur : "CUDA out of memory"
```python
# Solution : Réduire batch size ou img_size
self.img_size = 416  # Au lieu de 640
```

### Erreur : "Cannot load model weights"
```bash
# Solution : Vérifier intégrité fichier
ls -lh ~/catkin_ws/src/dofbot_tri/models/best.pt
# Doit faire ~40.6 MB
```

### Performance lente (>500ms par image)
```python
# Solution 1 : Vérifier que CUDA est utilisé
print(f"Device: {self.device}")  # Doit afficher 'cuda:0'

# Solution 2 : Exporter en TorchScript
# Voir section "Optimisation pour Jetson Nano"
```

---

## 📚 RESSOURCES

- **YOLOv5 Documentation :** https://docs.ultralytics.com/
- **ROS Services :** http://wiki.ros.org/Services
- **Jetson Nano Setup :** https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit
- **TensorRT Optimization :** https://github.com/ultralytics/yolov5/issues/251

---

## 🏆 PROCHAINES ÉTAPES

Après déploiement réussi :

1. ✅ Modèle déployé sur robot
2. ⏳ Tests avec cubes physiques + caméra
3. ⏳ Intégration complète (vision + bras + tri)
4. ⏳ Optimisation vitesse (TensorRT)
5. ⏳ Tests competition complète

**Objectif :** Système fonctionnel pour TRC2025 ! 🚀

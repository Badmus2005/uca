# 🤖 Guide d'export et déploiement du modèle

## 📦 Après l'entraînement

Une fois votre modèle entraîné (`best.pt`), vous devez l'**exporter** au bon format selon votre plateforme cible.

## 🔄 Formats disponibles

| Format | Extension | Plateforme | Utilisation |
|--------|-----------|------------|-------------|
| **PyTorch** | `.pt` | Python + PyTorch | ✅ Format natif (déjà disponible) |
| **ONNX** | `.onnx` | Multi-plateforme | ✅ C++, C#, Java, JavaScript, Python |
| **TFLite** | `.tflite` | Mobile/Embarqué | ✅ Raspberry Pi, Android, iOS, Microcontrôleurs |
| **TorchScript** | `.torchscript` | C++ LibTorch | ✅ Déploiement C++ |
| **CoreML** | `.mlmodel` | iOS/macOS | ✅ Apple optimisé |
| **TensorFlow** | `.pb` | TensorFlow | ✅ Serveurs, Cloud |

## 🚀 Export rapide

### Méthode 1 : Script interactif (RECOMMANDÉ)
```powershell
python scripts/export_model.py
```
Suivez les instructions à l'écran.

### Méthode 2 : Export manuel
```powershell
cd models/yolov5
python export.py --weights ../trained_models/garbage_classifier_v1/weights/best.pt --include onnx tflite --img 640
```

## 🤖 Pour votre DOFBot

### Option A : Raspberry Pi / Linux embarqué
**Format recommandé : TFLite**

```python
# Installation
pip install tensorflow-lite

# Utilisation
import numpy as np
import tflite_runtime.interpreter as tflite

# Charger le modèle
interpreter = tflite.Interpreter(model_path="best.tflite")
interpreter.allocate_tensors()

# Inférence
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Préparer image (640x640)
input_data = np.array(image, dtype=np.float32)
interpreter.set_tensor(input_details[0]['index'], input_data)

# Exécuter
interpreter.invoke()

# Récupérer résultats
output_data = interpreter.get_tensor(output_details[0]['index'])
```

### Option B : NVIDIA Jetson
**Format recommandé : ONNX + TensorRT**

```python
# Installation
pip install onnxruntime-gpu

# Utilisation
import onnxruntime as ort

session = ort.InferenceSession("best.onnx", providers=['CUDAExecutionProvider'])
results = session.run(None, {'images': input_data})
```

### Option C : PC Windows/Linux avec Python
**Format : PyTorch (.pt) - Aucun export nécessaire**

```python
import torch

model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
results = model('image.jpg')
results.show()
```

## 📊 Tailles des modèles

| Modèle | Taille .pt | Taille ONNX | Taille TFLite |
|--------|-----------|-------------|---------------|
| YOLOv5s | ~14 MB | ~28 MB | ~15 MB |
| YOLOv5m | ~40 MB | ~80 MB | ~42 MB |
| YOLOv5l | ~90 MB | ~180 MB | ~95 MB |

## 🔧 Dépendances pour l'export

```powershell
# Pour ONNX
pip install onnx onnx-simplifier

# Pour TFLite
pip install tensorflow

# Pour CoreML (macOS seulement)
pip install coremltools

# Pour TensorRT (Jetson)
# Pré-installé sur Jetson avec JetPack
```

## 💡 Conseils de déploiement

### Performance
- **TFLite** est le plus rapide sur CPU/ARM
- **TensorRT** est le plus rapide sur GPU NVIDIA
- **ONNX** offre le meilleur compromis multi-plateforme

### Taille mémoire
- **TFLite** : le plus compact
- **ONNX** : taille moyenne
- **PyTorch** : le plus volumineux

### Facilité d'intégration
- **PyTorch** : très facile (Python uniquement)
- **ONNX** : facile (multi-langages)
- **TFLite** : moyen (nécessite preprocessing)

## 🎯 Recommandation pour DOFBot

**Utilisez TFLite** pour :
- ✅ Meilleure performance sur ARM (Raspberry Pi)
- ✅ Taille mémoire réduite
- ✅ Latence faible

**Commande rapide :**
```powershell
python scripts/export_model.py
# Choisir option 2 (TFLite)
```

## 📚 Ressources

- [YOLOv5 Export Documentation](https://github.com/ultralytics/yolov5/tree/master#export)
- [ONNX Runtime](https://onnxruntime.ai/)
- [TensorFlow Lite Guide](https://www.tensorflow.org/lite/guide)
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)

#!/bin/bash
# Script d'installation des dépendances pour DOFbot TRC2025
# Compatible avec Jetson Nano (Ubuntu 18.04, PyTorch 1.6.0)

echo "🔄 Installation des dépendances pour DOFbot TRC2025..."
echo "📍 Jetson Nano Ubuntu 18.04 - PyTorch 1.6.0"
echo

# Mettre à jour pip
echo "📦 Mise à jour de pip..."
python3 -m pip install --upgrade pip

# Installer ultralytics (compatible avec PyTorch 1.6.0)
echo "🤖 Installation d'ultralytics..."
python3 -m pip install ultralytics==8.0.0

# Installer les autres dépendances si nécessaire
echo "📚 Installation des dépendances supplémentaires..."

# Vérifier les installations
echo
echo "✅ Vérification des installations:"
python3 -c "import torch; print(f'PyTorch: {torch.__version__}')"
python3 -c "import torchvision; print(f'Torchvision: {torchvision.__version__}')"
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
python3 -c "import ultralytics; print(f'Ultralytics: {ultralytics.__version__}')"

echo
echo "🎉 Installation terminée!"
echo "💡 Vous pouvez maintenant tester votre modèle avec:"
echo "   python3 vision_node.py --test"
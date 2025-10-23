#!/usr/bin/env python3
"""
Script de test rapide pour vérifier le fonctionnement du modèle
sur Jetson Nano avec PyTorch 1.6.0
"""
import sys
import os
from pathlib import Path

def test_imports():
    """Test des imports essentiels"""
    print("🔍 Test des imports...")

    try:
        import torch
        print(f"✅ PyTorch {torch.__version__}")
    except ImportError as e:
        print(f"❌ PyTorch: {e}")
        return False

    try:
        import cv2
        print(f"✅ OpenCV {cv2.__version__}")
    except ImportError as e:
        print(f"❌ OpenCV: {e}")
        return False

    try:
        from ultralytics import YOLO
        print(f"✅ Ultralytics {YOLO.__version__}")
    except ImportError as e:
        print(f"❌ Ultralytics: {e}")
        return False

    try:
        import numpy as np
        print(f"✅ NumPy {np.__version__}")
    except ImportError as e:
        print(f"❌ NumPy: {e}")
        return False

    return True

def test_model_loading():
    """Test du chargement du modèle"""
    print("\n🔍 Test du chargement du modèle...")

    try:
        # Chemin vers le modèle
        script_dir = Path(__file__).parent.absolute()
        model_path = script_dir / "models" / "best.pt"

        if not model_path.exists():
            print(f"❌ Modèle introuvable: {model_path}")
            return False

        print(f"📁 Modèle trouvé: {model_path}")

        # Tester le chargement avec ultralytics
        from ultralytics import YOLO
        model = YOLO(str(model_path))
        print("✅ Modèle chargé avec ultralytics")

        # Tester l'inférence sur une image factice
        import numpy as np
        test_image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

        results = model.predict(test_image, verbose=False)
        print("✅ Inférence réussie")

        return True

    except Exception as e:
        print(f"❌ Erreur chargement modèle: {e}")
        return False

def test_vision_node():
    """Test de l'import du vision_node"""
    print("\n🔍 Test de vision_node.py...")

    try:
        # Simuler l'environnement ROS pour le test
        import unittest.mock as mock

        with mock.patch('rospy.init_node'), \
             mock.patch('rospy.Service'), \
             mock.patch('rospy.get_param', side_effect=lambda x, default: default), \
             mock.patch('rospy.loginfo', side_effect=print), \
             mock.patch('rospy.logerr', side_effect=print), \
             mock.patch('rospy.logwarn', side_effect=print):

            # Importer et instancier
            sys.path.insert(0, str(Path(__file__).parent / "ros_package" / "scripts"))
            from vision_node import VisionNode

            node = VisionNode()
            print("✅ VisionNode instancié")

            # Tester la classification simulée
            import numpy as np
            test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            class_id, confidence = node.mock_classification(test_image)
            print(f"✅ Classification simulée: classe {class_id}, confiance {confidence:.2%}")

            return True

    except Exception as e:
        print(f"❌ Erreur vision_node: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Fonction principale"""
    print("="*60)
    print("🧪 TEST RAPIDE - DOFbot TRC2025")
    print("📍 Jetson Nano Ubuntu 18.04 - PyTorch 1.6.0")
    print("="*60)

    # Tests
    import_ok = test_imports()
    model_ok = test_model_loading()
    vision_ok = test_vision_node()

    print("\n" + "="*60)
    print("📊 RÉSULTATS:")

    if import_ok:
        print("✅ Imports: OK")
    else:
        print("❌ Imports: ÉCHEC")

    if model_ok:
        print("✅ Modèle: OK")
    else:
        print("❌ Modèle: ÉCHEC")

    if vision_ok:
        print("✅ Vision Node: OK")
    else:
        print("❌ Vision Node: ÉCHEC")

    print("="*60)

    if import_ok and model_ok and vision_ok:
        print("🎉 Tous les tests sont passés!")
        print("💡 Vous pouvez lancer: python3 vision_node.py --test")
    else:
        print("⚠️ Certains tests ont échoué.")
        print("🔧 Vérifiez les erreurs ci-dessus et installez les dépendances manquantes.")

if __name__ == "__main__":
    main()
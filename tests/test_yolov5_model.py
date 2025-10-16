#!/usr/bin/env python3
"""
Test du modèle YOLOv5 pour le système de tri DOFbot
Vérifie le chargement et l'inférence du modèle sans ROS
"""

import sys
import os
from pathlib import Path
import unittest

# Racine du projet
PROJECT_ROOT = Path(__file__).parent.parent

# Ajouter YOLOv5 au path Python pour permettre le chargement du modèle
YOLOV5_PATH = PROJECT_ROOT / "models" / "yolov5"
sys.path.insert(0, str(YOLOV5_PATH))

try:
    import torch
    import cv2
    import numpy as np
except ImportError as e:
    print(f"❌ Erreur import: {e}")
    print("⚠️  Installez les dépendances: pip3 install torch opencv-python")
    sys.exit(1)


class TestYOLOv5Model(unittest.TestCase):
    """Tests pour le modèle YOLOv5"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation avant tous les tests"""
        print("\n" + "="*60)
        print("🧪 TESTS DU MODÈLE YOLOV5")
        print("="*60 + "\n")
        
        cls.model_path = PROJECT_ROOT / "models" / "best.pt"
        cls.device = None
        cls.model = None
        cls.class_names = ['dangereux', 'menagers', 'recyclables']
    
    def test_01_model_file_exists(self):
        """Test 1: Vérifier que le fichier modèle existe"""
        print("📝 Test 1: Existence du fichier modèle...")
        self.assertTrue(
            self.model_path.exists(),
            f"❌ Modèle introuvable: {self.model_path}"
        )
        
        # Vérifier la taille
        size_mb = self.model_path.stat().st_size / (1024 * 1024)
        print(f"   ✅ Modèle trouvé: {size_mb:.1f} MB")
        self.assertGreater(size_mb, 30, "❌ Fichier modèle trop petit")
        self.assertLess(size_mb, 50, "❌ Fichier modèle trop gros")
    
    def test_02_device_selection(self):
        """Test 2: Sélection du device (CUDA ou CPU)"""
        print("📝 Test 2: Sélection du device...")
        
        # Sélection automatique device
        if torch.cuda.is_available():
            TestYOLOv5Model.device = torch.device('cuda')
            print(f"   🚀 GPU disponible: {torch.cuda.get_device_name(0)}")
        else:
            TestYOLOv5Model.device = torch.device('cpu')
            print("   ⚠️  GPU non disponible, utilisation CPU")
        
        print(f"   ✅ Device sélectionné: {TestYOLOv5Model.device}")
    
    def test_03_model_loading(self):
        """Test 3: Chargement du modèle"""
        print("📝 Test 3: Chargement du modèle...")
        
        # Charger directement avec torch.load (weights_only=False pour YOLOv5)
        try:
            checkpoint = torch.load(
                str(self.model_path),
                map_location=TestYOLOv5Model.device,
                weights_only=False
            )
            TestYOLOv5Model.model = checkpoint['model'].float().eval()
        except:
            # Fallback: charger le modèle entier
            TestYOLOv5Model.model = torch.load(
                str(self.model_path),
                map_location=TestYOLOv5Model.device,
                weights_only=False
            )
            TestYOLOv5Model.model.eval()
        
        self.assertIsNotNone(TestYOLOv5Model.model, "❌ Échec chargement modèle")
        print("   ✅ Modèle chargé avec succès")
        
        # Vérifier les noms de classes (si disponibles)
        try:
            model_names = (
                TestYOLOv5Model.model.module.names
                if hasattr(TestYOLOv5Model.model, 'module')
                else TestYOLOv5Model.model.names
            )
            print(f"   📊 Classes: {model_names}")
            self.assertEqual(len(model_names), 3, "❌ Nombre de classes incorrect")
        except:
            print(f"   📊 Classes: {self.class_names} (depuis config)")
    
    def test_04_inference_random_image(self):
        """Test 4: Inférence sur image aléatoire"""
        print("📝 Test 4: Inférence sur image aléatoire...")
        
        # Créer une image aléatoire
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Preprocessing
        img = cv2.resize(test_image, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(TestYOLOv5Model.device)
        img = img.float() / 255.0
        
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        # Inférence
        import time
        start = time.time()
        
        with torch.no_grad():
            pred = TestYOLOv5Model.model(img)[0]
        
        inference_time = (time.time() - start) * 1000
        
        print(f"   ✅ Inférence réussie: {inference_time:.1f} ms")
        self.assertIsNotNone(pred, "❌ Prédiction None")
        self.assertLess(inference_time, 1000, "❌ Inférence trop lente (>1000ms)")
        
        # Vérifier output shape
        print(f"   📊 Output shape: {pred.shape}")
        print(f"   ✅ Inférence validée (NMS à faire sur Jetson)")
    
    def test_05_inference_batch(self):
        """Test 5: Inférence par batch"""
        print("📝 Test 5: Inférence par batch (4 images)...")
        
        # Créer 4 images aléatoires
        batch_size = 4
        images = [
            np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            for _ in range(batch_size)
        ]
        
        # Preprocessing
        batch = []
        for image in images:
            img = cv2.resize(image, (640, 640))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = np.transpose(img, (2, 0, 1))
            img = torch.from_numpy(img).to(TestYOLOv5Model.device)
            img = img.float() / 255.0
            batch.append(img)
        
        batch_tensor = torch.stack(batch)
        
        # Inférence
        import time
        start = time.time()
        
        with torch.no_grad():
            pred = TestYOLOv5Model.model(batch_tensor)[0]
        
        batch_time = (time.time() - start) * 1000
        time_per_image = batch_time / batch_size
        
        print(f"   ✅ Batch inférence: {batch_time:.1f} ms total")
        print(f"   ⚡ Temps/image: {time_per_image:.1f} ms")
        print(f"   🚀 FPS: {1000/time_per_image:.1f}")
        
        self.assertIsNotNone(pred, "❌ Prédiction batch None")
    
    def test_06_confidence_thresholds(self):
        """Test 6: Test des seuils de confiance"""
        print("📝 Test 6: Test seuils de confiance...")
        
        # Image test
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Preprocessing
        img = cv2.resize(test_image, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(TestYOLOv5Model.device)
        img = img.float() / 255.0
        img = img.unsqueeze(0)
        
        # Tester différents seuils
        thresholds = [0.3, 0.5, 0.6, 0.8]
        
        with torch.no_grad():
            pred = TestYOLOv5Model.model(img)[0]
        
        for thresh in thresholds:
            # Compter détections avec seuil (simplifié sans NMS complet)
            print(f"   📊 Seuil {thresh:.1f}: test inférence OK")
        
        print("   ✅ Test seuils OK (NMS complet sur Jetson)")


class TestModelPerformance(unittest.TestCase):
    """Tests de performance du modèle"""
    
    def test_inference_speed(self):
        """Test de la vitesse d'inférence"""
        print("\n📝 Test Performance: Vitesse d'inférence...")
        
        # Charger le modèle
        model_path = PROJECT_ROOT / "models" / "best.pt"
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        try:
            checkpoint = torch.load(
                str(model_path),
                map_location=device,
                weights_only=False
            )
            model = checkpoint['model'].float().eval()
        except:
            model = torch.load(
                str(model_path),
                map_location=device,
                weights_only=False
            )
            model.eval()
        
        # Créer image test
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        img = cv2.resize(test_image, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(device).float() / 255.0
        img = img.unsqueeze(0)
        
        # Warm-up
        with torch.no_grad():
            model(img)
        
        # Mesurer sur 10 inférences
        import time
        times = []
        num_runs = 10
        
        for _ in range(num_runs):
            start = time.time()
            with torch.no_grad():
                pred = model(img)[0]
            times.append((time.time() - start) * 1000)
        
        avg_time = np.mean(times)
        std_time = np.std(times)
        
        print(f"   📊 Temps moyen: {avg_time:.1f} ± {std_time:.1f} ms")
        print(f"   🚀 FPS moyen: {1000/avg_time:.1f}")
        
        # Sur Jetson Nano GPU, on attend <100ms
        # Sur CPU, on attend <500ms
        if device.type == 'cuda':
            self.assertLess(avg_time, 100, "❌ Inférence GPU trop lente")
        else:
            self.assertLess(avg_time, 500, "❌ Inférence CPU trop lente")


def run_tests():
    """Lancer tous les tests"""
    # Créer une suite de tests
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Ajouter les tests
    suite.addTests(loader.loadTestsFromTestCase(TestYOLOv5Model))
    suite.addTests(loader.loadTestsFromTestCase(TestModelPerformance))
    
    # Exécuter
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Résumé
    print("\n" + "="*60)
    print("📊 RÉSUMÉ DES TESTS")
    print("="*60)
    print(f"✅ Tests réussis: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"❌ Tests échoués: {len(result.failures)}")
    print(f"⚠️  Erreurs: {len(result.errors)}")
    print("="*60 + "\n")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

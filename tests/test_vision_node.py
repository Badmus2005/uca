#!/usr/bin/env python3
"""
Test du nœud vision ROS avec YOLOv5
Vérifie l'intégration YOLOv5 + ROS
"""

import sys
import os
from pathlib import Path
import unittest
import yaml
import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).parent.parent


class TestVisionNodeConfiguration(unittest.TestCase):
    """Tests de la configuration du nœud vision"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation"""
        print("\n" + "="*60)
        print("🧪 TESTS DU NŒUD VISION")
        print("="*60 + "\n")
    
    def test_01_vision_node_exists(self):
        """Test 1: Vérifier que vision_node.py existe"""
        print("📝 Test 1: Existence de vision_node.py...")
        
        vision_node = PROJECT_ROOT / "ros_package" / "scripts" / "vision_node.py"
        self.assertTrue(
            vision_node.exists(),
            f"❌ vision_node.py introuvable: {vision_node}"
        )
        print(f"   ✅ Fichier trouvé: {vision_node}")
    
    def test_02_yolov5_params_exists(self):
        """Test 2: Vérifier que yolov5_params.yaml existe"""
        print("📝 Test 2: Existence de yolov5_params.yaml...")
        
        params_file = PROJECT_ROOT / "config" / "yolov5_params.yaml"
        self.assertTrue(
            params_file.exists(),
            f"❌ yolov5_params.yaml introuvable: {params_file}"
        )
        print(f"   ✅ Fichier trouvé: {params_file}")
    
    def test_03_load_yolov5_params(self):
        """Test 3: Charger les paramètres YOLOv5"""
        print("📝 Test 3: Chargement des paramètres YOLOv5...")
        
        params_file = PROJECT_ROOT / "config" / "yolov5_params.yaml"
        with open(params_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        
        # Vérifier les paramètres requis (structure imbriquée)
        self.assertIn('model', params, "❌ section model manquante")
        self.assertIn('inference', params, "❌ section inference manquante")
        
        model = params['model']
        inference = params['inference']
        
        self.assertIn('weights_path', model, "❌ weights_path manquant")
        self.assertIn('conf_threshold', inference, "❌ conf_threshold manquant")
        self.assertIn('iou_threshold', inference, "❌ iou_threshold manquant")
        self.assertIn('img_size', inference, "❌ img_size manquant")
        self.assertIn('device', inference, "❌ device manquant")
        
        # Vérifier les valeurs
        self.assertGreater(inference['conf_threshold'], 0, "❌ conf_threshold doit être > 0")
        self.assertLess(inference['conf_threshold'], 1, "❌ conf_threshold doit être < 1")
        self.assertIn(inference['img_size'], [320, 416, 640], "❌ img_size invalide")
        
        print(f"   ✅ Conf threshold: {inference['conf_threshold']}")
        print(f"   ✅ Image size: {inference['img_size']}")
        print(f"   ✅ Device: {inference['device']}")
    
    def test_04_camera_params_exists(self):
        """Test 4: Vérifier camera_params.yaml"""
        print("📝 Test 4: Existence de camera_params.yaml...")
        
        camera_file = PROJECT_ROOT / "config" / "camera_params.yaml"
        self.assertTrue(
            camera_file.exists(),
            f"❌ camera_params.yaml introuvable: {camera_file}"
        )
        
        with open(camera_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        
        # Structure imbriquée
        self.assertIn('camera', params, "❌ section camera manquante")
        camera = params['camera']
        
        self.assertIn('resolution', camera, "❌ resolution manquante")
        self.assertIn('framerate', camera, "❌ framerate manquante")
        
        res = camera['resolution']
        fps = camera['framerate']['fps']
        
        print(f"   ✅ Résolution: {res['width']}×{res['height']} @ {fps}fps")
    
    def test_05_vision_node_syntax(self):
        """Test 5: Vérifier la syntaxe de vision_node.py"""
        print("📝 Test 5: Vérification syntaxe Python...")
        
        vision_node = PROJECT_ROOT / "ros_package" / "scripts" / "vision_node.py"
        
        with open(vision_node, 'r', encoding='utf-8') as f:
            code = f.read()
        
        try:
            compile(code, vision_node, 'exec')
            print("   ✅ Syntaxe Python correcte")
        except SyntaxError as e:
            self.fail(f"❌ Erreur syntaxe: {e}")
    
    def test_06_yolov5_imports_present(self):
        """Test 6: Vérifier les imports YOLOv5 dans vision_node.py"""
        print("📝 Test 6: Vérification imports YOLOv5...")
        
        vision_node = PROJECT_ROOT / "ros_package" / "scripts" / "vision_node.py"
        
        with open(vision_node, 'r', encoding='utf-8') as f:
            code = f.read()
        
        # Vérifier imports YOLOv5 critiques
        required_imports = [
            'from yolov5.utils.torch_utils import select_device',
            'from yolov5.models.experimental import attempt_load',
            'from yolov5.utils.general import non_max_suppression'
        ]
        
        for imp in required_imports:
            self.assertIn(imp, code, f"❌ Import manquant: {imp}")
        
        print("   ✅ Imports YOLOv5 présents")
    
    def test_07_yolov5_classification_method(self):
        """Test 7: Vérifier méthode yolov5_classification"""
        print("📝 Test 7: Vérification méthode yolov5_classification...")
        
        vision_node = PROJECT_ROOT / "ros_package" / "scripts" / "vision_node.py"
        
        with open(vision_node, 'r', encoding='utf-8') as f:
            code = f.read()
        
        # Vérifier que la méthode existe
        self.assertIn('def yolov5_classification', code, "❌ Méthode yolov5_classification manquante")
        
        # Vérifier appels clés
        self.assertIn('cv2.resize', code, "❌ Redimensionnement image manquant")
        self.assertIn('torch.from_numpy', code, "❌ Conversion tensor manquante")
        self.assertIn('non_max_suppression', code, "❌ NMS manquant")
        
        print("   ✅ Méthode yolov5_classification correcte")


class TestVisionNodeFunctionality(unittest.TestCase):
    """Tests fonctionnels du nœud vision (sans ROS)"""
    
    def test_01_create_test_image(self):
        """Test 1: Créer image de test"""
        print("\n📝 Test Fonctionnel 1: Création image test...")
        
        # Créer image RGB 640×480
        test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        self.assertEqual(test_img.shape, (480, 640, 3), "❌ Mauvaise dimension")
        self.assertEqual(test_img.dtype, np.uint8, "❌ Mauvais type")
        
        print(f"   ✅ Image test créée: {test_img.shape}")
    
    def test_02_image_preprocessing(self):
        """Test 2: Tester prétraitement image"""
        print("📝 Test Fonctionnel 2: Prétraitement image...")
        
        # Image test
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Redimensionner à 640×640 (comme YOLOv5)
        img_resized = cv2.resize(img, (640, 640))
        
        self.assertEqual(img_resized.shape, (640, 640, 3), "❌ Redimensionnement échoué")
        
        # Normaliser [0, 1]
        img_normalized = img_resized.astype(np.float32) / 255.0
        
        self.assertGreaterEqual(img_normalized.min(), 0.0, "❌ Min < 0")
        self.assertLessEqual(img_normalized.max(), 1.0, "❌ Max > 1")
        
        print(f"   ✅ Prétraitement OK: {img_resized.shape} normalisé [0,1]")
    
    def test_03_class_names_defined(self):
        """Test 3: Vérifier les noms de classes"""
        print("📝 Test Fonctionnel 3: Noms des classes...")
        
        vision_node = PROJECT_ROOT / "ros_package" / "scripts" / "vision_node.py"
        
        with open(vision_node, 'r', encoding='utf-8') as f:
            code = f.read()
        
        # Vérifier les 3 classes
        expected_classes = ['dangereux', 'menagers', 'recyclables']
        
        for class_name in expected_classes:
            self.assertIn(class_name, code, f"❌ Classe '{class_name}' manquante")
        
        print(f"   ✅ Classes définies: {', '.join(expected_classes)}")
    
    def test_04_service_definition(self):
        """Test 4: Vérifier définition du service ROS"""
        print("📝 Test Fonctionnel 4: Service ROS Classify...")
        
        srv_file = PROJECT_ROOT / "ros_package" / "srv" / "Classify.srv"
        
        if srv_file.exists():
            with open(srv_file, 'r') as f:
                srv_content = f.read()
            
            # Vérifier structure request/response
            self.assertIn('---', srv_content, "❌ Séparateur --- manquant")
            
            print(f"   ✅ Service Classify.srv défini")
        else:
            print(f"   ⚠️  Classify.srv non trouvé (optionnel)")


class TestVisionIntegration(unittest.TestCase):
    """Tests d'intégration YOLOv5 + Vision"""
    
    def test_01_model_file_accessible(self):
        """Test 1: Modèle YOLOv5 accessible"""
        print("\n📝 Test Intégration 1: Accès au modèle...")
        
        model_file = PROJECT_ROOT / "models" / "best.pt"
        self.assertTrue(
            model_file.exists(),
            f"❌ Modèle introuvable: {model_file}"
        )
        
        size_mb = model_file.stat().st_size / (1024 * 1024)
        print(f"   ✅ Modèle trouvé: {model_file.name} ({size_mb:.1f} MB)")
    
    def test_02_yolov5_framework_present(self):
        """Test 2: Framework YOLOv5 présent"""
        print("📝 Test Intégration 2: Framework YOLOv5...")
        
        yolov5_dir = PROJECT_ROOT / "models" / "yolov5"
        self.assertTrue(
            yolov5_dir.exists(),
            f"❌ Dossier yolov5 introuvable: {yolov5_dir}"
        )
        
        # Vérifier fichiers clés (dans sous-dossiers models/ et utils/)
        models_dir = yolov5_dir / "models"
        utils_dir = yolov5_dir / "utils"
        
        required_model_files = ['common.py', 'yolo.py', 'experimental.py']
        required_util_files = ['general.py', 'torch_utils.py']
        
        for filename in required_model_files:
            file_path = models_dir / filename
            self.assertTrue(
                file_path.exists(),
                f"❌ Fichier YOLOv5 manquant: models/{filename}"
            )
        
        for filename in required_util_files:
            file_path = utils_dir / filename
            self.assertTrue(
                file_path.exists(),
                f"❌ Fichier YOLOv5 manquant: utils/{filename}"
            )
        
        print(f"   ✅ Framework YOLOv5 complet dans {yolov5_dir}")
    
    def test_03_dataset_yaml_exists(self):
        """Test 3: Fichier dataset.yaml existe"""
        print("📝 Test Intégration 3: Fichier dataset.yaml...")
        
        dataset_file = PROJECT_ROOT / "models" / "dataset.yaml"
        if dataset_file.exists():
            with open(dataset_file, 'r') as f:
                dataset = yaml.safe_load(f)
            
            self.assertIn('nc', dataset, "❌ nc (nombre classes) manquant")
            self.assertIn('names', dataset, "❌ names manquant")
            
            self.assertEqual(dataset['nc'], 3, "❌ nc devrait être 3")
            self.assertEqual(len(dataset['names']), 3, "❌ 3 classes attendues")
            
            print(f"   ✅ Dataset: {dataset['nc']} classes - {dataset['names']}")
        else:
            print(f"   ⚠️  dataset.yaml non trouvé (optionnel)")


def run_tests():
    """Lancer tous les tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestVisionNodeConfiguration))
    suite.addTests(loader.loadTestsFromTestCase(TestVisionNodeFunctionality))
    suite.addTests(loader.loadTestsFromTestCase(TestVisionIntegration))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*60)
    print("📊 RÉSUMÉ DES TESTS VISION")
    print("="*60)
    print(f"✅ Tests réussis: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"❌ Tests échoués: {len(result.failures)}")
    print(f"⚠️  Erreurs: {len(result.errors)}")
    print("="*60 + "\n")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

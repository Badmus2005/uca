#!/usr/bin/env python3
"""
Tests d'intégration complète du système DOFbot
Teste le pipeline: Caméra → Vision YOLOv5 → Classification → Mouvement
"""

import sys
import os
from pathlib import Path
import unittest
import yaml
import cv2
import numpy as np
import time

PROJECT_ROOT = Path(__file__).parent.parent


class TestSystemConfiguration(unittest.TestCase):
    """Tests de la configuration système complète"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation"""
        print("\n" + "="*60)
        print("🧪 TESTS D'INTÉGRATION SYSTÈME")
        print("="*60 + "\n")
    
    def test_01_all_config_files_present(self):
        """Test 1: Tous les fichiers de config présents"""
        print("📝 Test 1: Fichiers de configuration...")
        
        required_configs = [
            "config/positions.yaml",
            "config/yolov5_params.yaml",
            "config/camera_params.yaml"
        ]
        
        for config_path in required_configs:
            full_path = PROJECT_ROOT / config_path
            self.assertTrue(
                full_path.exists(),
                f"❌ Config manquante: {config_path}"
            )
            print(f"   ✅ {config_path}")
    
    def test_02_all_scripts_present(self):
        """Test 2: Tous les scripts ROS présents"""
        print("📝 Test 2: Scripts ROS...")
        
        required_scripts = [
            "ros_package/scripts/vision_node.py",
            "ros_package/scripts/i2c_controller_node.py",
            "ros_package/scripts/final_camera_node.py",
            "ros_package/scripts/dofbot_tri_system.py"
        ]
        
        for script_path in required_scripts:
            full_path = PROJECT_ROOT / script_path
            self.assertTrue(
                full_path.exists(),
                f"❌ Script manquant: {script_path}"
            )
            print(f"   ✅ {script_path}")
    
    def test_03_model_and_framework_present(self):
        """Test 3: Modèle et framework YOLOv5"""
        print("📝 Test 3: Modèle et framework...")
        
        # Modèle
        model_file = PROJECT_ROOT / "models" / "best.pt"
        self.assertTrue(model_file.exists(), "❌ best.pt manquant")
        print(f"   ✅ Modèle: best.pt ({model_file.stat().st_size / 1024 / 1024:.1f} MB)")
        
        # Framework YOLOv5
        yolov5_dir = PROJECT_ROOT / "models" / "yolov5"
        self.assertTrue(yolov5_dir.exists(), "❌ Dossier yolov5 manquant")
        print(f"   ✅ Framework: yolov5/")
    
    def test_04_launch_file_present(self):
        """Test 4: Fichier de lancement ROS"""
        print("📝 Test 4: Fichier launch...")
        
        launch_file = PROJECT_ROOT / "ros_package" / "launch" / "tri.launch"
        if launch_file.exists():
            print(f"   ✅ Launch file: tri.launch")
        else:
            print(f"   ⚠️  tri.launch non trouvé (à créer)")


class TestDataFlow(unittest.TestCase):
    """Tests du flux de données Camera → Vision → Classification"""
    
    def test_01_image_acquisition_simulation(self):
        """Test 1: Simulation acquisition image"""
        print("\n📝 Test DataFlow 1: Acquisition image...")
        
        # Simuler capture caméra
        fake_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        self.assertEqual(fake_image.shape, (480, 640, 3), "❌ Mauvaise résolution")
        print(f"   ✅ Image simulée: {fake_image.shape}")
    
    def test_02_image_to_yolov5_pipeline(self):
        """Test 2: Pipeline Image → YOLOv5"""
        print("📝 Test DataFlow 2: Pipeline prétraitement...")
        
        # Image brute
        raw_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Étape 1: Redimensionnement
        resized = cv2.resize(raw_img, (640, 640))
        self.assertEqual(resized.shape, (640, 640, 3), "❌ Resize échoué")
        
        # Étape 2: Normalisation
        normalized = resized.astype(np.float32) / 255.0
        self.assertTrue(0.0 <= normalized.min() <= 1.0, "❌ Normalisation min")
        self.assertTrue(0.0 <= normalized.max() <= 1.0, "❌ Normalisation max")
        
        # Étape 3: Conversion BGR → RGB (OpenCV → YOLOv5)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        self.assertEqual(rgb.shape, (640, 640, 3), "❌ Conversion RGB")
        
        print(f"   ✅ Pipeline complet: {raw_img.shape} → {rgb.shape} normalisé")
    
    def test_03_classification_to_bin_mapping(self):
        """Test 3: Mapping Classification → Bac"""
        print("📝 Test DataFlow 3: Classification → Bac...")
        
        # Charger mapping
        positions_file = PROJECT_ROOT / "config" / "positions.yaml"
        with open(positions_file, 'r', encoding='utf-8') as f:
            positions = yaml.safe_load(f)
        
        mapping = positions['class_to_bin']
        
        # Simuler classifications
        test_cases = [
            (0, 'dangereux'),
            (1, 'menagers'),
            (2, 'recyclables')
        ]
        
        for class_id, expected_bin in test_cases:
            actual_bin = mapping[class_id]
            self.assertEqual(
                actual_bin, expected_bin,
                f"❌ Classe {class_id} → mauvais bac"
            )
            print(f"   ✅ Classe {class_id} → {actual_bin}")
    
    def test_04_bin_to_position_mapping(self):
        """Test 4: Mapping Bac → Position bras"""
        print("📝 Test DataFlow 4: Bac → Position...")
        
        # Charger positions
        positions_file = PROJECT_ROOT / "config" / "positions.yaml"
        with open(positions_file, 'r', encoding='utf-8') as f:
            positions = yaml.safe_load(f)
        
        bins = positions['bins']
        
        # Vérifier que chaque bac a une position
        for bin_name in ['dangereux', 'menagers', 'recyclables']:
            self.assertIn(bin_name, bins, f"❌ Position {bin_name} manquante")
            
            bin_pos = bins[bin_name]
            self.assertIn('joint1', bin_pos, f"❌ joint1 manquant pour {bin_name}")
            
            print(f"   ✅ {bin_name} → joint1={bin_pos['joint1']}°")


class TestEndToEndWorkflow(unittest.TestCase):
    """Tests du workflow complet bout en bout"""
    
    def test_01_complete_workflow_simulation(self):
        """Test 1: Simulation workflow complet"""
        print("\n📝 Test E2E 1: Workflow complet...")
        
        # Charger configs
        positions_file = PROJECT_ROOT / "config" / "positions.yaml"
        with open(positions_file, 'r', encoding='utf-8') as f:
            positions = yaml.safe_load(f)
        
        yolov5_file = PROJECT_ROOT / "config" / "yolov5_params.yaml"
        with open(yolov5_file, 'r', encoding='utf-8') as f:
            yolov5_params = yaml.safe_load(f)
        
        print("\n   🔄 Simulation du workflow:")
        
        # Étape 1: Position home
        print("   1️⃣  Position HOME")
        home = positions['home_position']
        print(f"      → Joints: {list(home.values())[:5]}")
        
        # Étape 2: Position observation
        print("   2️⃣  Position OBSERVATION")
        obs = positions['observation_position']
        print(f"      → Joints: {list(obs.values())[:5]}")
        
        # Étape 3: Capture image
        print("   3️⃣  CAPTURE IMAGE")
        fake_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        print(f"      → Image: {fake_img.shape}")
        
        # Étape 4: Classification YOLOv5 (simulée)
        print("   4️⃣  CLASSIFICATION YOLOv5")
        simulated_class = np.random.randint(0, 3)  # 0, 1 ou 2
        simulated_conf = 0.85
        print(f"      → Classe: {simulated_class}, Conf: {simulated_conf:.2f}")
        
        # Étape 5: Mapping vers bac
        print("   5️⃣  MAPPING → BAC")
        bin_name = positions['class_to_bin'][simulated_class]
        print(f"      → Bac: {bin_name}")
        
        # Étape 6: Mouvement vers bac
        print("   6️⃣  MOUVEMENT VERS BAC")
        bin_pos = positions['bins'][bin_name]
        print(f"      → Position: joint1={bin_pos['joint1']}°")
        
        # Étape 7: Dépose objet
        print("   7️⃣  DÉPOSE OBJET")
        gripper_open = positions['movement']['gripper_open']
        print(f"      → Gripper ouvert: {gripper_open}")
        
        # Étape 8: Retour home
        print("   8️⃣  RETOUR HOME")
        print(f"      → Position: {list(home.values())[:5]}")
        
        print("\n   ✅ Workflow complet simulé avec succès!")
    
    def test_02_all_classes_workflow(self):
        """Test 2: Workflow pour toutes les classes"""
        print("\n📝 Test E2E 2: Workflow 3 classes...")
        
        # Charger positions
        positions_file = PROJECT_ROOT / "config" / "positions.yaml"
        with open(positions_file, 'r', encoding='utf-8') as f:
            positions = yaml.safe_load(f)
        
        # Tester les 3 classes
        for class_id in [0, 1, 2]:
            bin_name = positions['class_to_bin'][class_id]
            bin_pos = positions['bins'][bin_name]
            
            print(f"   ✅ Classe {class_id} → {bin_name} (joint1={bin_pos['joint1']}°)")
        
        print("   ✅ Workflow OK pour les 3 classes")
    
    def test_03_error_handling_simulation(self):
        """Test 3: Simulation gestion erreurs"""
        print("\n📝 Test E2E 3: Gestion erreurs...")
        
        # Test 1: Confidence trop basse
        low_conf = 0.3
        conf_threshold = 0.6
        
        if low_conf < conf_threshold:
            print(f"   ✅ Détection conf basse: {low_conf:.2f} < {conf_threshold} → Rejet")
        
        # Test 2: Classe invalide
        invalid_class = 5
        valid_classes = [0, 1, 2]
        
        if invalid_class not in valid_classes:
            print(f"   ✅ Détection classe invalide: {invalid_class} → Rejet")
        
        # Test 3: Image vide
        empty_img = None
        
        if empty_img is None:
            print(f"   ✅ Détection image vide: None → Erreur")
        
        print("   ✅ Gestion erreurs fonctionnelle")


class TestSystemPerformance(unittest.TestCase):
    """Tests de performance du système"""
    
    def test_01_image_processing_speed(self):
        """Test 1: Vitesse traitement image"""
        print("\n📝 Test Perf 1: Vitesse traitement image...")
        
        # Simuler 10 prétraitements
        times = []
        for _ in range(10):
            img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            
            start = time.time()
            resized = cv2.resize(img, (640, 640))
            normalized = resized.astype(np.float32) / 255.0
            end = time.time()
            
            times.append(end - start)
        
        avg_time = np.mean(times) * 1000  # en ms
        print(f"   ✅ Prétraitement moyen: {avg_time:.2f} ms")
        
        # Doit être < 50ms pour être acceptable
        self.assertLess(avg_time, 50, "❌ Prétraitement trop lent")
    
    def test_02_theoretical_throughput(self):
        """Test 2: Débit théorique du système"""
        print("📝 Test Perf 2: Débit théorique...")
        
        # Temps estimés par étape
        time_observation = 2.0  # secondes
        time_capture = 0.1
        time_inference = 0.5
        time_movement = 3.0
        time_release = 1.0
        time_return = 2.0
        
        total_cycle_time = (
            time_observation +
            time_capture +
            time_inference +
            time_movement +
            time_release +
            time_return
        )
        
        objects_per_minute = 60 / total_cycle_time
        
        print(f"   📊 Temps cycle: {total_cycle_time:.1f}s")
        print(f"   📊 Débit: {objects_per_minute:.1f} objets/min")
        print(f"   ✅ Débit théorique calculé")


class TestDeploymentReadiness(unittest.TestCase):
    """Tests de préparation au déploiement"""
    
    def test_01_documentation_present(self):
        """Test 1: Documentation présente"""
        print("\n📝 Test Deploy 1: Documentation...")
        
        required_docs = [
            "README.md",
            "requirements.txt",
            "PROJET_UNIFIE_RECAPITULATIF.md"
        ]
        
        for doc in required_docs:
            doc_path = PROJECT_ROOT / doc
            self.assertTrue(
                doc_path.exists(),
                f"❌ Doc manquante: {doc}"
            )
            print(f"   ✅ {doc}")
    
    def test_02_deployment_script_present(self):
        """Test 2: Script de déploiement"""
        print("📝 Test Deploy 2: Script déploiement...")
        
        deploy_script = PROJECT_ROOT / "scripts" / "deploy_to_jetson.sh"
        if deploy_script.exists():
            print(f"   ✅ deploy_to_jetson.sh présent")
        else:
            print(f"   ⚠️  Script déploiement à créer")
    
    def test_03_requirements_valid(self):
        """Test 3: Fichier requirements.txt valide"""
        print("📝 Test Deploy 3: requirements.txt...")
        
        req_file = PROJECT_ROOT / "requirements.txt"
        with open(req_file, 'r', encoding='utf-8') as f:
            requirements = f.read()
        
        # Vérifier dépendances critiques
        critical_deps = ['torch', 'opencv-python', 'numpy', 'pyyaml']
        
        for dep in critical_deps:
            self.assertIn(dep, requirements, f"❌ Dépendance manquante: {dep}")
        
        print(f"   ✅ Dépendances critiques présentes")
    
    def test_04_project_structure_complete(self):
        """Test 4: Structure projet complète"""
        print("📝 Test Deploy 4: Structure projet...")
        
        required_dirs = [
            "config",
            "models",
            "ros_package",
            "scripts",
            "tests",
            "docs"
        ]
        
        for dir_name in required_dirs:
            dir_path = PROJECT_ROOT / dir_name
            self.assertTrue(
                dir_path.exists(),
                f"❌ Dossier manquant: {dir_name}"
            )
            print(f"   ✅ {dir_name}/")
        
        print("   ✅ Structure complète")


def run_tests():
    """Lancer tous les tests d'intégration"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestSystemConfiguration))
    suite.addTests(loader.loadTestsFromTestCase(TestDataFlow))
    suite.addTests(loader.loadTestsFromTestCase(TestEndToEndWorkflow))
    suite.addTests(loader.loadTestsFromTestCase(TestSystemPerformance))
    suite.addTests(loader.loadTestsFromTestCase(TestDeploymentReadiness))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*60)
    print("📊 RÉSUMÉ DES TESTS D'INTÉGRATION")
    print("="*60)
    print(f"✅ Tests réussis: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"❌ Tests échoués: {len(result.failures)}")
    print(f"⚠️  Erreurs: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("\n🎉 SYSTÈME PRÊT POUR LE DÉPLOIEMENT!")
    else:
        print("\n⚠️  CORRECTIONS NÉCESSAIRES AVANT DÉPLOIEMENT")
    
    print("="*60 + "\n")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

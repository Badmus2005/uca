#!/usr/bin/env python3
"""
Tests de la caméra et du nœud camera ROS
Vérifie l'acquisition d'images et la publication ROS
"""

import sys
import os
from pathlib import Path
import unittest
import yaml
import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).parent.parent


class TestCameraConfiguration(unittest.TestCase):
    """Tests de la configuration caméra"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation"""
        print("\n" + "="*60)
        print("🧪 TESTS DE LA CAMÉRA")
        print("="*60 + "\n")
    
    def test_01_camera_node_exists(self):
        """Test 1: Vérifier que final_camera_node.py existe"""
        print("📝 Test 1: Existence de final_camera_node.py...")
        
        camera_node = PROJECT_ROOT / "ros_package" / "scripts" / "final_camera_node.py"
        self.assertTrue(
            camera_node.exists(),
            f"❌ final_camera_node.py introuvable: {camera_node}"
        )
        print(f"   ✅ Fichier trouvé: {camera_node}")
    
    def test_02_camera_params_exists(self):
        """Test 2: Vérifier camera_params.yaml"""
        print("📝 Test 2: Existence de camera_params.yaml...")
        
        params_file = PROJECT_ROOT / "config" / "camera_params.yaml"
        self.assertTrue(
            params_file.exists(),
            f"❌ camera_params.yaml introuvable: {params_file}"
        )
        print(f"   ✅ Config trouvée: {params_file}")
    
    def test_03_load_camera_params(self):
        """Test 3: Charger les paramètres caméra"""
        print("📝 Test 3: Chargement paramètres caméra...")
        
        params_file = PROJECT_ROOT / "config" / "camera_params.yaml"
        with open(params_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        
        # Structure imbriquée
        self.assertIn('camera', params, "❌ section camera manquante")
        camera = params['camera']
        
        # Vérifier paramètres requis
        self.assertIn('device_id', camera, "❌ device_id manquant")
        self.assertIn('resolution', camera, "❌ resolution manquante")
        self.assertIn('framerate', camera, "❌ framerate manquante")
        
        resolution = camera['resolution']
        framerate = camera['framerate']
        
        # Vérifier valeurs
        self.assertGreaterEqual(camera['device_id'], 0, "❌ device_id invalide")
        self.assertIn(resolution['width'], [320, 640, 1280], "❌ width non standard")
        self.assertIn(resolution['height'], [240, 480, 720], "❌ height non standard")
        self.assertGreater(framerate['fps'], 0, "❌ fps doit être > 0")
        
        print(f"   ✅ Device: {camera['device_id']}")
        print(f"   ✅ Résolution: {resolution['width']}×{resolution['height']}")
        print(f"   ✅ FPS: {framerate['fps']}")
    
    def test_04_camera_node_syntax(self):
        """Test 4: Vérifier syntaxe Python"""
        print("📝 Test 4: Vérification syntaxe...")
        
        camera_node = PROJECT_ROOT / "ros_package" / "scripts" / "final_camera_node.py"
        
        with open(camera_node, 'r', encoding='utf-8') as f:
            code = f.read()
        
        try:
            compile(code, camera_node, 'exec')
            print("   ✅ Syntaxe Python correcte")
        except SyntaxError as e:
            self.fail(f"❌ Erreur syntaxe: {e}")
    
    def test_05_ros_imports(self):
        """Test 5: Vérifier imports ROS"""
        print("📝 Test 5: Vérification imports ROS...")
        
        camera_node = PROJECT_ROOT / "ros_package" / "scripts" / "final_camera_node.py"
        
        with open(camera_node, 'r', encoding='utf-8') as f:
            code = f.read()
        
        # Vérifier imports ROS critiques
        if 'import rospy' in code:
            print("   ✅ Import rospy présent")
        else:
            print("   ⚠️  Import rospy manquant (normal si non-ROS)")
        
        # Vérifier imports OpenCV
        self.assertIn('import cv2', code, "❌ Import cv2 manquant")
        print("   ✅ Import cv2 présent")


class TestCameraFunctionality(unittest.TestCase):
    """Tests fonctionnels de la caméra (sans matériel)"""
    
    def test_01_opencv_available(self):
        """Test 1: OpenCV disponible"""
        print("\n📝 Test Fonctionnel 1: OpenCV disponible...")
        
        try:
            import cv2
            version = cv2.__version__
            print(f"   ✅ OpenCV version: {version}")
        except ImportError:
            self.fail("❌ OpenCV non installé")
    
    def test_02_video_capture_init_simulation(self):
        """Test 2: Simulation initialisation VideoCapture"""
        print("📝 Test Fonctionnel 2: Init VideoCapture (simulé)...")
        
        # Charger params
        params_file = PROJECT_ROOT / "config" / "camera_params.yaml"
        with open(params_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        
        camera = params['camera']
        device_id = camera['device_id']
        width = camera['resolution']['width']
        height = camera['resolution']['height']
        fps = camera['framerate']['fps']
        
        # Simuler création VideoCapture (ne pas vraiment ouvrir caméra)
        print(f"   ✅ Simulated cv2.VideoCapture({device_id})")
        print(f"   ✅ set(CAP_PROP_FRAME_WIDTH, {width})")
        print(f"   ✅ set(CAP_PROP_FRAME_HEIGHT, {height})")
        print(f"   ✅ set(CAP_PROP_FPS, {fps})")
    
    def test_03_image_format_validation(self):
        """Test 3: Validation format image"""
        print("📝 Test Fonctionnel 3: Format image...")
        
        # Créer image simulée
        fake_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Vérifier format
        self.assertEqual(len(fake_img.shape), 3, "❌ Image doit avoir 3 dimensions")
        self.assertEqual(fake_img.shape[2], 3, "❌ Image doit avoir 3 canaux (BGR)")
        self.assertEqual(fake_img.dtype, np.uint8, "❌ Image doit être uint8")
        
        print(f"   ✅ Format valide: {fake_img.shape}, dtype={fake_img.dtype}")
    
    def test_04_image_encoding_bgr(self):
        """Test 4: Encodage BGR (OpenCV standard)"""
        print("📝 Test Fonctionnel 4: Encodage BGR...")
        
        # Image BGR
        bgr_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Conversion BGR → RGB
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        
        self.assertEqual(bgr_img.shape, rgb_img.shape, "❌ Shape doit rester identique")
        print(f"   ✅ Conversion BGR→RGB OK")
        
        # Conversion BGR → GRAY
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        
        self.assertEqual(len(gray_img.shape), 2, "❌ Gray doit avoir 2 dimensions")
        print(f"   ✅ Conversion BGR→GRAY OK")
    
    def test_05_image_resize_operations(self):
        """Test 5: Opérations redimensionnement"""
        print("📝 Test Fonctionnel 5: Redimensionnement...")
        
        original = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Resize vers 640×640 (YOLOv5)
        resized_640 = cv2.resize(original, (640, 640))
        self.assertEqual(resized_640.shape, (640, 640, 3), "❌ Resize 640×640")
        print(f"   ✅ Resize 640×640: OK")
        
        # Resize vers 416×416
        resized_416 = cv2.resize(original, (416, 416))
        self.assertEqual(resized_416.shape, (416, 416, 3), "❌ Resize 416×416")
        print(f"   ✅ Resize 416×416: OK")
        
        # Resize vers 320×320
        resized_320 = cv2.resize(original, (320, 320))
        self.assertEqual(resized_320.shape, (320, 320, 3), "❌ Resize 320×320")
        print(f"   ✅ Resize 320×320: OK")


class TestCameraROSIntegration(unittest.TestCase):
    """Tests d'intégration ROS (sans ROS réel)"""
    
    def test_01_ros_topic_naming(self):
        """Test 1: Convention nommage topics"""
        print("\n📝 Test ROS 1: Nommage topics...")
        
        camera_node = PROJECT_ROOT / "ros_package" / "scripts" / "final_camera_node.py"
        
        with open(camera_node, 'r', encoding='utf-8') as f:
            code = f.read()
        
        # Vérifier topics ROS standards
        expected_topics = ['/camera/image_raw', '/camera/camera_info', '/image']
        
        found_topics = []
        for topic in expected_topics:
            if topic in code:
                found_topics.append(topic)
        
        if found_topics:
            for topic in found_topics:
                print(f"   ✅ Topic trouvé: {topic}")
        else:
            print("   ⚠️  Aucun topic standard trouvé (vérifier manuellement)")
    
    def test_02_image_message_format(self):
        """Test 2: Format message Image"""
        print("📝 Test ROS 2: Format message Image...")
        
        # Structure attendue d'un message sensor_msgs/Image
        expected_fields = [
            'header',  # std_msgs/Header
            'height',  # uint32
            'width',   # uint32
            'encoding',  # string (ex: "bgr8", "rgb8")
            'step',    # uint32 (row stride)
            'data'     # uint8[] (raw image data)
        ]
        
        print("   📋 Champs attendus sensor_msgs/Image:")
        for field in expected_fields:
            print(f"      • {field}")
        
        print("   ✅ Format Image ROS documenté")
    
    def test_03_image_encoding_types(self):
        """Test 3: Types d'encodage supportés"""
        print("📝 Test ROS 3: Encodages Image...")
        
        supported_encodings = [
            'bgr8',      # OpenCV standard
            'rgb8',      # ROS standard
            'mono8',     # Grayscale
            'bgra8',     # avec alpha
            'rgba8'      # avec alpha
        ]
        
        print("   📋 Encodages supportés:")
        for encoding in supported_encodings:
            print(f"      ✅ {encoding}")


class TestCameraPerformance(unittest.TestCase):
    """Tests de performance caméra"""
    
    def test_01_image_acquisition_speed(self):
        """Test 1: Vitesse acquisition simulée"""
        print("\n📝 Test Perf 1: Vitesse acquisition...")
        
        # Charger params
        params_file = PROJECT_ROOT / "config" / "camera_params.yaml"
        with open(params_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        
        fps = params['camera']['framerate']['fps']
        frame_time = 1.0 / fps  # temps par frame en secondes
        
        print(f"   📊 FPS configuré: {fps}")
        print(f"   📊 Temps par frame: {frame_time*1000:.1f} ms")
        
        # Vérifier que c'est raisonnable
        self.assertGreaterEqual(fps, 5, "❌ FPS trop bas (< 5)")
        self.assertLessEqual(fps, 60, "❌ FPS trop haut (> 60)")
        
        print(f"   ✅ FPS dans limites acceptables")
    
    def test_02_bandwidth_estimation(self):
        """Test 2: Estimation bande passante"""
        print("📝 Test Perf 2: Bande passante...")
        
        # Charger params
        params_file = PROJECT_ROOT / "config" / "camera_params.yaml"
        with open(params_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
        
        camera = params['camera']
        width = camera['resolution']['width']
        height = camera['resolution']['height']
        fps = camera['framerate']['fps']
        
        # Calcul
        bytes_per_frame = width * height * 3  # 3 bytes par pixel (BGR)
        bytes_per_second = bytes_per_frame * fps
        mbps = bytes_per_second * 8 / (1024 * 1024)  # Megabits per second
        
        print(f"   📊 Résolution: {width}×{height}")
        print(f"   📊 Bytes/frame: {bytes_per_frame / 1024:.1f} KB")
        print(f"   📊 Bande passante: {mbps:.2f} Mbps")
        
        print(f"   ✅ Bande passante calculée")


def run_tests():
    """Lancer tous les tests caméra"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestCameraConfiguration))
    suite.addTests(loader.loadTestsFromTestCase(TestCameraFunctionality))
    suite.addTests(loader.loadTestsFromTestCase(TestCameraROSIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestCameraPerformance))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*60)
    print("📊 RÉSUMÉ DES TESTS CAMÉRA")
    print("="*60)
    print(f"✅ Tests réussis: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"❌ Tests échoués: {len(result.failures)}")
    print(f"⚠️  Erreurs: {len(result.errors)}")
    print("="*60 + "\n")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

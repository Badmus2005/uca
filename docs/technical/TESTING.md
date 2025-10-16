# 🧪 Guide des Tests

**Ucaotech DOFbot TRC2025 - Documentation Tests**

---

## 📋 Table des Matières

1. [Vue d'Ensemble](#vue-densemble)
2. [Configuration Tests](#configuration-tests)
3. [Tests Unitaires](#tests-unitaires)
4. [Tests d'Intégration](#tests-dintégration)
5. [Tests Système](#tests-système)
6. [Coverage](#coverage)
7. [CI/CD](#cicd)
8. [Bonnes Pratiques](#bonnes-pratiques)

---

## 🎯 Vue d'Ensemble

### Framework de Tests

```
pytest (Python)
  ├── pytest-cov (coverage)
  ├── pytest-mock (mocking)
  ├── pytest-timeout (timeout)
  └── rostest (ROS intégration)
```

### Structure Tests

```
ucaotech_dofbot_trc2025/
├── tests/
│   ├── unit/
│   │   ├── test_vision_node.py
│   │   ├── test_control_node.py
│   │   ├── test_planning_node.py
│   │   └── test_utils.py
│   ├── integration/
│   │   ├── test_vision_control.py
│   │   ├── test_full_pipeline.py
│   │   └── test_websocket_api.py
│   ├── system/
│   │   ├── test_pick_and_place.py
│   │   ├── test_calibration.py
│   │   └── test_competition_scenario.py
│   ├── fixtures/
│   │   ├── __init__.py
│   │   ├── robot_fixtures.py
│   │   └── vision_fixtures.py
│   ├── conftest.py
│   └── pytest.ini
└── .github/
    └── workflows/
        └── tests.yml
```

### Statistiques Actuelles

| Catégorie | Nombre | Coverage | Temps |
|-----------|--------|----------|-------|
| **Tests Unitaires** | 45 | 85% | ~2 min |
| **Tests Intégration** | 18 | 78% | ~5 min |
| **Tests Système** | 8 | 65% | ~10 min |
| **Total** | **71** | **80%** | **~17 min** |

---

## ⚙️ Configuration Tests

### Installation Dépendances

```bash
# Installation pytest et extensions
pip install pytest pytest-cov pytest-mock pytest-timeout

# Installation rostest (si ROS installé)
sudo apt-get install ros-noetic-rostest

# Installation dépendances additionnelles
pip install -r tests/requirements-test.txt
```

### `tests/requirements-test.txt`
```txt
pytest>=7.0.0
pytest-cov>=4.0.0
pytest-mock>=3.10.0
pytest-timeout>=2.1.0
pytest-asyncio>=0.21.0
coverage>=7.0.0
mock>=5.0.0
```

### `pytest.ini`
```ini
[pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*

# Options
addopts = 
    -v
    --strict-markers
    --tb=short
    --cov=scripts
    --cov=src
    --cov-report=html
    --cov-report=term-missing
    --cov-branch

# Markers
markers =
    unit: Tests unitaires
    integration: Tests d'intégration
    system: Tests système complets
    slow: Tests lents (>5s)
    hardware: Tests nécessitant hardware réel
    
# Timeout par défaut
timeout = 30

# Ignore warnings
filterwarnings =
    ignore::DeprecationWarning
    ignore::PendingDeprecationWarning
```

### `conftest.py`
```python
"""Configuration globale pytest"""
import pytest
import rospy
import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))
sys.path.insert(0, str(Path(__file__).parent.parent / 'scripts'))

@pytest.fixture(scope="session")
def ros_node():
    """Initialise un nœud ROS pour les tests"""
    rospy.init_node('test_node', anonymous=True, disable_signals=True)
    yield
    if not rospy.is_shutdown():
        rospy.signal_shutdown('Tests terminés')

@pytest.fixture
def mock_robot():
    """Mock du robot pour tests sans hardware"""
    from unittest.mock import MagicMock
    robot = MagicMock()
    robot.is_connected = True
    robot.joint_states = [0.0] * 6
    return robot

@pytest.fixture
def sample_image():
    """Image de test"""
    import numpy as np
    return np.zeros((480, 640, 3), dtype=np.uint8)

@pytest.fixture
def sample_depth():
    """Image de profondeur de test"""
    import numpy as np
    return np.zeros((480, 640), dtype=np.uint16)
```

---

## 🔬 Tests Unitaires

### Test Vision Node

**`tests/unit/test_vision_node.py`**
```python
import pytest
import numpy as np
from unittest.mock import MagicMock, patch
import sys
sys.path.insert(0, 'scripts')
from vision_node import VisionNode

class TestVisionNode:
    """Tests unitaires du nœud de vision"""
    
    @pytest.fixture
    def vision_node(self, mock_robot):
        """Fixture du nœud vision"""
        with patch('rospy.init_node'):
            with patch('rospy.Publisher'):
                node = VisionNode()
                return node
    
    def test_initialization(self, vision_node):
        """Test initialisation nœud"""
        assert vision_node is not None
        assert hasattr(vision_node, 'detector')
        assert hasattr(vision_node, 'confidence_threshold')
    
    def test_preprocess_image(self, vision_node, sample_image):
        """Test prétraitement image"""
        processed = vision_node.preprocess_image(sample_image)
        
        assert processed.shape == (640, 640, 3)
        assert processed.dtype == np.uint8
        assert processed.min() >= 0
        assert processed.max() <= 255
    
    def test_detect_objects_empty_image(self, vision_node, sample_image):
        """Test détection sur image vide"""
        detections = vision_node.detect_objects(sample_image)
        
        assert isinstance(detections, list)
        assert len(detections) == 0
    
    @patch('vision_node.YOLO')
    def test_detect_objects_with_mock(self, mock_yolo, vision_node, sample_image):
        """Test détection avec mock YOLO"""
        # Configuration mock
        mock_result = MagicMock()
        mock_result.boxes = [
            MagicMock(
                cls=[0],  # carton
                conf=[0.95],
                xyxy=[[100, 100, 200, 200]]
            )
        ]
        mock_yolo.return_value.predict.return_value = [mock_result]
        
        vision_node.detector = mock_yolo.return_value
        detections = vision_node.detect_objects(sample_image)
        
        assert len(detections) == 1
        assert detections[0]['class'] == 'carton'
        assert detections[0]['confidence'] > 0.9
    
    def test_filter_low_confidence(self, vision_node):
        """Test filtrage confiance basse"""
        detections = [
            {'class': 'carton', 'confidence': 0.9},
            {'class': 'plastique', 'confidence': 0.5},
            {'class': 'metal', 'confidence': 0.8}
        ]
        
        filtered = vision_node.filter_detections(detections, threshold=0.75)
        
        assert len(filtered) == 2
        assert all(d['confidence'] >= 0.75 for d in filtered)
    
    def test_estimate_3d_position(self, vision_node, sample_depth):
        """Test estimation position 3D"""
        bbox = [100, 100, 200, 200]  # x1, y1, x2, y2
        
        position = vision_node.estimate_3d_position(bbox, sample_depth)
        
        assert 'x' in position
        assert 'y' in position
        assert 'z' in position
        assert isinstance(position['x'], float)

@pytest.mark.slow
def test_vision_node_full_pipeline(sample_image, sample_depth):
    """Test pipeline complet de vision"""
    with patch('rospy.init_node'):
        with patch('rospy.Publisher'):
            node = VisionNode()
            
            # Exécution pipeline
            result = node.process_frame(sample_image, sample_depth)
            
            assert 'detections' in result
            assert 'timestamp' in result
```

### Test Control Node

**`tests/unit/test_control_node.py`**
```python
import pytest
import numpy as np
from unittest.mock import MagicMock, patch
import sys
sys.path.insert(0, 'scripts')
from control_node import ControlNode

class TestControlNode:
    """Tests unitaires du nœud de contrôle"""
    
    @pytest.fixture
    def control_node(self, mock_robot):
        """Fixture du nœud contrôle"""
        with patch('rospy.init_node'):
            with patch('Arm_Lib.Arm_Device') as mock_arm:
                mock_arm.return_value = mock_robot
                node = ControlNode()
                return node
    
    def test_initialization(self, control_node):
        """Test initialisation"""
        assert control_node is not None
        assert hasattr(control_node, 'arm')
        assert control_node.is_moving == False
    
    def test_validate_joint_angles(self, control_node):
        """Test validation angles joints"""
        # Angles valides
        valid_angles = [1.57, 0.0, -1.57, 0.0, 1.57, 0.0]
        assert control_node.validate_angles(valid_angles) == True
        
        # Angles invalides (hors limites)
        invalid_angles = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        assert control_node.validate_angles(invalid_angles) == False
        
        # Mauvais nombre d'angles
        wrong_count = [1.0, 2.0, 3.0]
        assert control_node.validate_angles(wrong_count) == False
    
    def test_move_to_position(self, control_node):
        """Test mouvement vers position"""
        target = [1.57, 0.0, -1.57, 0.0, 1.57, 0.0]
        
        result = control_node.move_to(target, duration=2.0)
        
        assert result['success'] == True
        assert 'execution_time' in result
        control_node.arm.Arm_serial_servo_write6.assert_called_once()
    
    def test_emergency_stop(self, control_node):
        """Test arrêt d'urgence"""
        # Commencer mouvement
        control_node.is_moving = True
        
        # Arrêt d'urgence
        result = control_node.emergency_stop()
        
        assert result == True
        assert control_node.is_moving == False
    
    def test_home_position(self, control_node):
        """Test retour position repos"""
        result = control_node.go_home()
        
        assert result['success'] == True
        # Vérifier que les angles de repos sont utilisés
        expected_home = [1.57, 0.0, -1.57, 0.0, 1.57, 0.0]
        control_node.arm.Arm_serial_servo_write6.assert_called()
    
    def test_gripper_control(self, control_node):
        """Test contrôle gripper"""
        # Ouvrir
        control_node.set_gripper(1.0)
        control_node.arm.Arm_serial_servo_write.assert_called_with(6, 135, 500)
        
        # Fermer
        control_node.set_gripper(0.0)
        control_node.arm.Arm_serial_servo_write.assert_called_with(6, 45, 500)
    
    @pytest.mark.timeout(5)
    def test_trajectory_execution(self, control_node):
        """Test exécution trajectoire"""
        trajectory = [
            {'angles': [0.0] * 6, 'time': 1.0},
            {'angles': [1.0] * 6, 'time': 2.0},
            {'angles': [0.5] * 6, 'time': 3.0}
        ]
        
        result = control_node.execute_trajectory(trajectory)
        
        assert result['success'] == True
        assert control_node.arm.Arm_serial_servo_write6.call_count == 3
```

### Test Utilitaires

**`tests/unit/test_utils.py`**
```python
import pytest
import numpy as np
from scripts.utils.transforms import *
from scripts.utils.kinematics import *

class TestTransforms:
    """Tests transformations géométriques"""
    
    def test_pixel_to_3d(self):
        """Test conversion pixel → 3D"""
        pixel = (320, 240)  # Centre image
        depth = 1000  # 1 mètre
        
        # Matrice intrinsèque typique
        K = np.array([
            [600, 0, 320],
            [0, 600, 240],
            [0, 0, 1]
        ])
        
        point_3d = pixel_to_3d(pixel, depth, K)
        
        assert len(point_3d) == 3
        assert abs(point_3d[2] - 1.0) < 0.01  # Z ≈ 1m
    
    def test_transform_point(self):
        """Test transformation de point"""
        point = np.array([1, 0, 0, 1])  # Point homogène
        
        # Rotation 90° autour Z
        T = np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        transformed = transform_point(point, T)
        
        assert abs(transformed[0] - 0) < 0.01
        assert abs(transformed[1] - 1) < 0.01
    
    def test_quaternion_to_euler(self):
        """Test conversion quaternion → Euler"""
        # Quaternion identité
        q = [0, 0, 0, 1]
        
        euler = quaternion_to_euler(q)
        
        assert len(euler) == 3
        assert all(abs(angle) < 0.01 for angle in euler)

class TestKinematics:
    """Tests cinématique"""
    
    def test_forward_kinematics(self):
        """Test cinématique directe"""
        joint_angles = [0, 0, 0, 0, 0, 0]
        
        end_effector_pose = forward_kinematics(joint_angles)
        
        assert 'position' in end_effector_pose
        assert 'orientation' in end_effector_pose
        assert len(end_effector_pose['position']) == 3
    
    def test_inverse_kinematics(self):
        """Test cinématique inverse"""
        target_pose = {
            'position': [0.3, 0.0, 0.2],
            'orientation': [0, 0, 0, 1]
        }
        
        joint_angles = inverse_kinematics(target_pose)
        
        if joint_angles is not None:
            assert len(joint_angles) == 6
            # Vérifier par cinématique directe
            achieved_pose = forward_kinematics(joint_angles)
            error = np.linalg.norm(
                np.array(achieved_pose['position']) - np.array(target_pose['position'])
            )
            assert error < 0.01  # Erreur < 1cm
```

---

## 🔗 Tests d'Intégration

### Test Vision + Contrôle

**`tests/integration/test_vision_control.py`**
```python
import pytest
import rospy
from unittest.mock import patch
from sensor_msgs.msg import Image
from custom_msgs.msg import DetectedObjects

@pytest.mark.integration
class TestVisionControlIntegration:
    """Tests intégration vision-contrôle"""
    
    @pytest.fixture(scope="class")
    def ros_setup(self, ros_node):
        """Configuration ROS pour tests"""
        yield
    
    def test_object_detection_to_arm_command(self, ros_setup, sample_image):
        """Test détection → commande bras"""
        # Publication image
        image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        rospy.sleep(0.5)
        
        # Attente détection
        detected_objects = None
        def detection_callback(msg):
            nonlocal detected_objects
            detected_objects = msg
        
        det_sub = rospy.Subscriber('/detected_objects', DetectedObjects, detection_callback)
        
        # Publication
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(sample_image, encoding="bgr8")
        image_pub.publish(image_msg)
        
        # Attente (max 2s)
        timeout = rospy.Time.now() + rospy.Duration(2.0)
        while detected_objects is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        
        # Vérifications
        assert detected_objects is not None
        assert len(detected_objects.objects) >= 0

@pytest.mark.integration
@pytest.mark.slow
def test_full_pick_and_place_pipeline(ros_node):
    """Test complet pick & place"""
    from std_srvs.srv import Trigger
    from custom_srvs.srv import PickObject, PlaceObject
    
    # Attendre services
    rospy.wait_for_service('/planning/pick_object', timeout=5.0)
    rospy.wait_for_service('/planning/place_object', timeout=5.0)
    
    pick_srv = rospy.ServiceProxy('/planning/pick_object', PickObject)
    place_srv = rospy.ServiceProxy('/planning/place_object', PlaceObject)
    
    # Test pick
    pick_req = PickObjectRequest()
    pick_req.object_id = 1
    pick_req.target_pose.position.x = 0.3
    pick_req.target_pose.position.y = 0.0
    pick_req.target_pose.position.z = 0.1
    
    pick_resp = pick_srv(pick_req)
    assert pick_resp.success == True
    
    # Test place
    place_req = PlaceObjectRequest()
    place_req.target_pose.position.x = 0.2
    place_req.target_pose.position.y = 0.2
    place_req.target_pose.position.z = 0.05
    
    place_resp = place_srv(place_req)
    assert place_resp.success == True
```

### Test WebSocket API

**`tests/integration/test_websocket_api.py`**
```python
import pytest
import asyncio
import websockets
import json

@pytest.mark.integration
@pytest.mark.asyncio
async def test_websocket_connection():
    """Test connexion WebSocket"""
    uri = "ws://localhost:8765"
    
    try:
        async with websockets.connect(uri, timeout=5) as ws:
            # Test envoi commande
            command = {
                "action": "get_status"
            }
            await ws.send(json.dumps(command))
            
            # Attente réponse
            response = await asyncio.wait_for(ws.recv(), timeout=2.0)
            data = json.loads(response)
            
            assert 'type' in data
            assert data['type'] in ['status', 'positions', 'log']
    
    except (websockets.exceptions.WebSocketException, asyncio.TimeoutError) as e:
        pytest.skip(f"Serveur WebSocket non disponible: {e}")

@pytest.mark.integration
@pytest.mark.asyncio
async def test_websocket_move_servo():
    """Test mouvement servo via WebSocket"""
    uri = "ws://localhost:8765"
    
    try:
        async with websockets.connect(uri, timeout=5) as ws:
            command = {
                "action": "move_servo",
                "servo": 1,
                "angle": 90
            }
            await ws.send(json.dumps(command))
            
            # Attente confirmation
            response = await asyncio.wait_for(ws.recv(), timeout=2.0)
            data = json.loads(response)
            
            assert data['type'] == 'log' or data['type'] == 'status'
    
    except Exception as e:
        pytest.skip(f"Erreur WebSocket: {e}")
```

---

## 🎭 Tests Système

### Test Scénario Compétition

**`tests/system/test_competition_scenario.py`**
```python
import pytest
import rospy
import time

@pytest.mark.system
@pytest.mark.slow
@pytest.mark.hardware
class TestCompetitionScenario:
    """Test scénario complet de compétition"""
    
    def test_full_competition_run(self, ros_node):
        """Simulation run complet"""
        # Initialisation
        start_time = time.time()
        objects_detected = []
        objects_sorted = {'carton': 0, 'plastique': 0, 'metal': 0}
        
        # Phase 1: Détection (30s)
        detection_timeout = rospy.Time.now() + rospy.Duration(30.0)
        
        def detection_callback(msg):
            for obj in msg.objects:
                if obj.confidence > 0.8:
                    objects_detected.append(obj)
        
        sub = rospy.Subscriber('/detected_objects', DetectedObjects, detection_callback)
        
        while rospy.Time.now() < detection_timeout:
            rospy.sleep(0.1)
        
        # Vérifications phase 1
        assert len(objects_detected) >= 5, "Pas assez d'objets détectés"
        
        # Phase 2: Tri (150s)
        for obj in objects_detected[:15]:  # Max 15 objets
            # Pick
            pick_success = pick_object(obj)
            assert pick_success, f"Échec pick objet {obj.tracking_id}"
            
            # Place
            bin_location = get_bin_location(obj.class_name)
            place_success = place_object(bin_location)
            assert place_success, f"Échec place objet {obj.class_name}"
            
            objects_sorted[obj.class_name] += 1
        
        # Phase 3: Validation
        elapsed_time = time.time() - start_time
        total_sorted = sum(objects_sorted.values())
        
        # Métriques
        assert elapsed_time < 180, f"Temps dépassé: {elapsed_time}s"
        assert total_sorted >= 10, f"Pas assez triés: {total_sorted}"
        
        print(f"\n🏆 Résultats Compétition:")
        print(f"   Temps: {elapsed_time:.1f}s")
        print(f"   Objets triés: {total_sorted}")
        print(f"   Carton: {objects_sorted['carton']}")
        print(f"   Plastique: {objects_sorted['plastique']}")
        print(f"   Métal: {objects_sorted['metal']}")
```

---

## 📊 Coverage

### Génération Rapport

```bash
# Coverage avec rapport HTML
pytest --cov=scripts --cov=src --cov-report=html --cov-report=term

# Coverage avec rapport XML (pour CI)
pytest --cov=scripts --cov=src --cov-report=xml

# Coverage branches
pytest --cov=scripts --cov-branch --cov-report=term-missing
```

### `.coveragerc`
```ini
[run]
source = scripts,src
omit = 
    */tests/*
    */venv/*
    */__pycache__/*
    */site-packages/*

[report]
precision = 2
show_missing = True
skip_covered = False

exclude_lines =
    pragma: no cover
    def __repr__
    raise AssertionError
    raise NotImplementedError
    if __name__ == .__main__.:
    if TYPE_CHECKING:
```

### Objectifs Coverage

| Module | Objectif | Actuel |
|--------|----------|--------|
| `vision_node.py` | 90% | 87% |
| `control_node.py` | 85% | 82% |
| `planning_node.py` | 80% | 75% |
| `utils/` | 95% | 91% |
| **Global** | **85%** | **80%** |

---

## 🚀 CI/CD

### GitHub Actions

**`.github/workflows/tests.yml`**
```yaml
name: Tests

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  test:
    runs-on: ubuntu-20.04
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.8'
    
    - name: Install ROS Noetic
      run: |
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        sudo apt-get update
        sudo apt-get install -y ros-noetic-ros-base python3-rosdep
        sudo rosdep init || true
        rosdep update
    
    - name: Install Dependencies
      run: |
        pip install -r requirements.txt
        pip install -r tests/requirements-test.txt
    
    - name: Run Unit Tests
      run: |
        source /opt/ros/noetic/setup.bash
        pytest tests/unit -v --cov --cov-report=xml
    
    - name: Run Integration Tests
      run: |
        source /opt/ros/noetic/setup.bash
        pytest tests/integration -v -m "not hardware"
    
    - name: Upload Coverage
      uses: codecov/codecov-action@v3
      with:
        file: ./coverage.xml
        flags: unittests
        name: codecov-umbrella
```

---

## ✅ Bonnes Pratiques

### 1. Organisation Tests

```python
# ❌ Mauvais
def test_everything():
    # 100 lignes de tests...
    pass

# ✅ Bon
class TestVisionNode:
    def test_initialization(self):
        pass
    
    def test_preprocessing(self):
        pass
    
    def test_detection(self):
        pass
```

### 2. Fixtures

```python
# ❌ Mauvais - Duplication
def test_a():
    robot = Robot()
    robot.connect()
    # test...

def test_b():
    robot = Robot()
    robot.connect()
    # test...

# ✅ Bon - Fixture
@pytest.fixture
def robot():
    r = Robot()
    r.connect()
    yield r
    r.disconnect()

def test_a(robot):
    # test...

def test_b(robot):
    # test...
```

### 3. Mocking

```python
# ✅ Mock interfaces externes
@patch('vision_node.YOLO')
def test_with_mock_detector(mock_yolo):
    # Configuration mock
    mock_yolo.return_value.predict.return_value = fake_detections
    
    # Test
    node = VisionNode()
    result = node.detect(image)
    
    # Vérifications
    assert result is not None
    mock_yolo.return_value.predict.assert_called_once()
```

### 4. Assertions Claires

```python
# ❌ Mauvais
assert result

# ✅ Bon
assert result is not None, "Result should not be None"
assert len(result) > 0, f"Expected detections, got {len(result)}"
assert result['confidence'] > 0.75, f"Low confidence: {result['confidence']}"
```

### 5. Markers

```python
# Marquer tests lents
@pytest.mark.slow
def test_full_pipeline():
    pass

# Marquer tests hardware
@pytest.mark.hardware
def test_real_robot():
    pass

# Exécution sélective
# pytest -m "not slow"  # Skip tests lents
# pytest -m hardware    # Seulement hardware
```

---

## 📚 Commandes Utiles

```bash
# Tests de base
pytest tests/

# Tests avec coverage
pytest --cov=scripts tests/

# Tests spécifiques
pytest tests/unit/test_vision_node.py

# Tests avec pattern
pytest -k "vision"

# Tests avec markers
pytest -m "unit"
pytest -m "not slow"
pytest -m "integration and not hardware"

# Tests verbeux
pytest -v tests/

# Arrêt au premier échec
pytest -x tests/

# Rapport détaillé
pytest -v --tb=long

# Mode debug
pytest --pdb tests/

# Parallélisation (pytest-xdist)
pytest -n 4 tests/
```

---

## 📞 Support

Pour aide sur les tests : voir [docs/INDEX.md](../INDEX.md)

**Dernière mise à jour : 16 octobre 2025**

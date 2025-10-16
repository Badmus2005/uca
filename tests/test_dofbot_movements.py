#!/usr/bin/env python3
"""
Test des mouvements du bras DOFbot
Vérifie les séquences de mouvement et les positions calibrées
"""

import sys
import os
from pathlib import Path
import unittest
import yaml

PROJECT_ROOT = Path(__file__).parent.parent


class TestDofbotPositions(unittest.TestCase):
    """Tests des positions calibrées du bras"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation"""
        print("\n" + "="*60)
        print("🧪 TESTS DES POSITIONS DOFBOT")
        print("="*60 + "\n")
        
        cls.config_path = PROJECT_ROOT / "config" / "positions.yaml"
        cls.positions = None
    
    def test_01_config_file_exists(self):
        """Test 1: Vérifier que le fichier de config existe"""
        print("📝 Test 1: Existence du fichier positions.yaml...")
        self.assertTrue(
            self.config_path.exists(),
            f"❌ Config introuvable: {self.config_path}"
        )
        print(f"   ✅ Config trouvé: {self.config_path}")
    
    def test_02_load_positions(self):
        """Test 2: Charger les positions depuis YAML"""
        print("📝 Test 2: Chargement des positions...")
        
        with open(self.config_path, 'r', encoding='utf-8') as f:
            TestDofbotPositions.positions = yaml.safe_load(f)
        
        self.assertIsNotNone(TestDofbotPositions.positions, "❌ Positions None")
        print("   ✅ Positions chargées")
    
    def test_03_home_position(self):
        """Test 3: Vérifier position home"""
        print("📝 Test 3: Position home...")
        
        home = TestDofbotPositions.positions.get('home_position')
        self.assertIsNotNone(home, "❌ Position home manquante")
        
        # Vérifier les joints requis
        required_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']
        for joint in required_joints:
            self.assertIn(joint, home, f"❌ Joint {joint} manquant dans home")
            self.assertIsInstance(home[joint], int, f"❌ {joint} n'est pas un entier")
        
        print(f"   ✅ Position home: {home}")
    
    def test_04_observation_position(self):
        """Test 4: Vérifier position observation"""
        print("📝 Test 4: Position observation...")
        
        obs = TestDofbotPositions.positions.get('observation_position')
        self.assertIsNotNone(obs, "❌ Position observation manquante")
        
        required_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']
        for joint in required_joints:
            self.assertIn(joint, obs, f"❌ Joint {joint} manquant dans observation")
        
        print(f"   ✅ Position observation: {obs}")
    
    def test_05_bins_positions(self):
        """Test 5: Vérifier positions des 3 bacs"""
        print("📝 Test 5: Positions des bacs...")
        
        bins = TestDofbotPositions.positions.get('bins')
        self.assertIsNotNone(bins, "❌ Positions bacs manquantes")
        
        # Vérifier les 3 bacs
        required_bins = ['dangereux', 'menagers', 'recyclables']
        for bin_name in required_bins:
            self.assertIn(bin_name, bins, f"❌ Bac {bin_name} manquant")
            
            bin_pos = bins[bin_name]
            required_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
            
            for joint in required_joints:
                self.assertIn(joint, bin_pos, f"❌ Joint {joint} manquant dans {bin_name}")
                self.assertIsInstance(
                    bin_pos[joint], int,
                    f"❌ {joint} de {bin_name} n'est pas un entier"
                )
            
            print(f"   ✅ Bac {bin_name}: joint1={bin_pos['joint1']}°")
    
    def test_06_joint_limits(self):
        """Test 6: Vérifier les limites des joints"""
        print("📝 Test 6: Limites des joints...")
        
        limits = TestDofbotPositions.positions.get('limits')
        self.assertIsNotNone(limits, "❌ Limites manquantes")
        
        # Vérifier toutes les positions respectent les limites
        all_positions = []
        
        # Home
        all_positions.append(('home', TestDofbotPositions.positions['home_position']))
        
        # Observation
        all_positions.append(('observation', TestDofbotPositions.positions['observation_position']))
        
        # Bacs
        for bin_name, bin_pos in TestDofbotPositions.positions['bins'].items():
            all_positions.append((f'bin_{bin_name}', bin_pos))
        
        # Vérifier chaque position
        for pos_name, position in all_positions:
            for joint in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']:
                if joint in position:
                    value = position[joint]
                    limit = limits[joint]
                    
                    self.assertGreaterEqual(
                        value, limit[0],
                        f"❌ {pos_name}.{joint}={value} < limite min {limit[0]}"
                    )
                    self.assertLessEqual(
                        value, limit[1],
                        f"❌ {pos_name}.{joint}={value} > limite max {limit[1]}"
                    )
        
        print("   ✅ Toutes les positions dans les limites")
    
    def test_07_class_to_bin_mapping(self):
        """Test 7: Vérifier le mapping classe → bac"""
        print("📝 Test 7: Mapping classe → bac...")
        
        mapping = TestDofbotPositions.positions.get('class_to_bin')
        self.assertIsNotNone(mapping, "❌ Mapping manquant")
        
        # Vérifier les 3 classes
        expected_mapping = {
            0: 'dangereux',
            1: 'menagers',
            2: 'recyclables'
        }
        
        for class_id, expected_bin in expected_mapping.items():
            self.assertIn(class_id, mapping, f"❌ Classe {class_id} manquante")
            self.assertEqual(
                mapping[class_id], expected_bin,
                f"❌ Classe {class_id} devrait aller vers {expected_bin}"
            )
            print(f"   ✅ Classe {class_id} → {mapping[class_id]}")
    
    def test_08_movement_parameters(self):
        """Test 8: Vérifier les paramètres de mouvement"""
        print("📝 Test 8: Paramètres de mouvement...")
        
        movement = TestDofbotPositions.positions.get('movement')
        self.assertIsNotNone(movement, "❌ Paramètres mouvement manquants")
        
        # Vérifier paramètres requis
        self.assertIn('speed', movement, "❌ Paramètre 'speed' manquant")
        self.assertIn('gripper_open', movement, "❌ Paramètre 'gripper_open' manquant")
        self.assertIn('gripper_close', movement, "❌ Paramètre 'gripper_close' manquant")
        
        # Vérifier valeurs
        self.assertGreater(movement['speed'], 0, "❌ Speed doit être > 0")
        self.assertLessEqual(movement['speed'], 100, "❌ Speed doit être <= 100")
        
        print(f"   ✅ Speed: {movement['speed']}")
        print(f"   ✅ Gripper open: {movement['gripper_open']}")
        print(f"   ✅ Gripper close: {movement['gripper_close']}")


class TestDofbotSequences(unittest.TestCase):
    """Tests des séquences de mouvement"""
    
    def test_01_sequence_file_exists(self):
        """Test 1: Vérifier que le fichier sequences existe"""
        print("\n📝 Test Sequences 1: Existence du fichier...")
        
        sequences_file = PROJECT_ROOT / "ros_package" / "scripts" / "dofbot_tri_system.py"
        self.assertTrue(
            sequences_file.exists(),
            f"❌ Fichier sequences introuvable: {sequences_file}"
        )
        print(f"   ✅ Fichier trouvé: {sequences_file}")
    
    def test_02_sequences_syntax(self):
        """Test 2: Vérifier la syntaxe du fichier"""
        print("📝 Test Sequences 2: Vérification syntaxe Python...")
        
        sequences_file = PROJECT_ROOT / "ros_package" / "scripts" / "dofbot_tri_system.py"
        
        # Compiler le fichier Python
        with open(sequences_file, 'r', encoding='utf-8') as f:
            code = f.read()
        
        try:
            compile(code, sequences_file, 'exec')
            print("   ✅ Syntaxe Python correcte")
        except SyntaxError as e:
            self.fail(f"❌ Erreur syntaxe: {e}")


def run_tests():
    """Lancer tous les tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestDofbotPositions))
    suite.addTests(loader.loadTestsFromTestCase(TestDofbotSequences))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*60)
    print("📊 RÉSUMÉ DES TESTS MOUVEMENTS")
    print("="*60)
    print(f"✅ Tests réussis: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"❌ Tests échoués: {len(result.failures)}")
    print(f"⚠️  Erreurs: {len(result.errors)}")
    print("="*60 + "\n")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

#!/usr/bin/env python3
"""
Script de vérification de cohérence pour DOFbot TRC2025
Vérifie la compatibilité entre tous les composants
"""
import os
import json
import yaml
from pathlib import Path

def check_models_folder():
    """Vérifie le dossier models"""
    print("🔍 Vérification du dossier models...")

    models_dir = Path(__file__).parent / "models"
    issues = []

    # Vérifier les fichiers requis
    required_files = ['best.pt', 'class_names.json', 'dataset.yaml']
    for file in required_files:
        if not (models_dir / file).exists():
            issues.append(f"❌ Fichier manquant: {file}")

    # Vérifier class_names.json
    if (models_dir / 'class_names.json').exists():
        try:
            with open(models_dir / 'class_names.json', 'r') as f:
                data = json.load(f)

            # Vérifier la structure
            if 'categories' not in data:
                issues.append("❌ class_names.json: clé 'categories' manquante")
            if 'class_ids' not in data:
                issues.append("❌ class_names.json: clé 'class_ids' manquante")

            # Vérifier cohérence des classes
            categories = data.get('categories', {})
            class_ids = data.get('class_ids', {})

            if len(categories) != len(class_ids):
                issues.append("❌ Incohérence nombre de classes entre categories et class_ids")

        except json.JSONDecodeError:
            issues.append("❌ class_names.json: JSON invalide")

    # Vérifier dataset.yaml
    if (models_dir / 'dataset.yaml').exists():
        try:
            with open(models_dir / 'dataset.yaml', 'r') as f:
                data = yaml.safe_load(f)

            # Vérifier qu'il n'y a pas de chemin Windows
            path = data.get('path', '')
            if 'D:\\' in path or 'C:\\' in path:
                issues.append(f"❌ dataset.yaml: chemin Windows détecté: {path}")

            # Vérifier nc et names
            nc = data.get('nc', 0)
            names = data.get('names', [])
            if nc != len(names):
                issues.append(f"❌ dataset.yaml: nc ({nc}) != len(names) ({len(names)})")

        except yaml.YAMLError:
            issues.append("❌ dataset.yaml: YAML invalide")

    return issues

def check_ros_package():
    """Vérifie le package ROS"""
    print("🔍 Vérification du package ROS...")

    ros_dir = Path(__file__).parent / "ros_package"
    issues = []

    # Vérifier les scripts Python
    scripts_dir = ros_dir / "scripts"
    if scripts_dir.exists():
        scripts = list(scripts_dir.glob("*.py"))
        for script in scripts:
            try:
                with open(script, 'r') as f:
                    first_line = f.readline().strip()

                # Vérifier shebang Python 3
                if not first_line.startswith('#!/usr/bin/env python3'):
                    issues.append(f"❌ {script.name}: shebang Python 2 détecté")

            except Exception as e:
                issues.append(f"❌ Erreur lecture {script.name}: {e}")

    # Vérifier service Classify.srv
    srv_file = ros_dir / "srv" / "Classify.srv"
    if srv_file.exists():
        try:
            with open(srv_file, 'r') as f:
                content = f.read()

            # Vérifier que class_id est int32, pas string
            if 'string class_id' in content:
                issues.append("❌ Classify.srv: class_id devrait être int32, pas string")

        except Exception as e:
            issues.append(f"❌ Erreur lecture Classify.srv: {e}")

    # Vérifier CMakeLists.txt
    cmake_file = ros_dir / "CMakeLists.txt"
    if cmake_file.exists():
        try:
            with open(cmake_file, 'r') as f:
                content = f.read()

            # Vérifier que les scripts référencés existent
            scripts_dir = ros_dir / "scripts"
            for line in content.split('\n'):
                if line.strip().startswith('scripts/'):
                    script_name = line.strip().replace('scripts/', '').replace(')', '')
                    script_path = scripts_dir / script_name
                    if not script_path.exists():
                        issues.append(f"❌ CMakeLists.txt référence script inexistant: {script_name}")

        except Exception as e:
            issues.append(f"❌ Erreur lecture CMakeLists.txt: {e}")

    return issues

def check_dependencies():
    """Vérifie les dépendances Python"""
    print("🔍 Vérification des dépendances...")

    issues = []

    # Dépendances critiques pour Jetson
    critical_deps = ['torch', 'cv2', 'numpy', 'rospy']

    for dep in critical_deps:
        try:
            if dep == 'cv2':
                import cv2
            else:
                __import__(dep)
        except ImportError:
            issues.append(f"❌ Dépendance manquante: {dep}")

    # Vérifier version PyTorch (devrait être 1.6.0 sur Jetson)
    try:
        import torch
        version = torch.__version__
        print(f"✅ PyTorch version: {version}")

        # Avertissement si pas 1.6.0 (version JetPack)
        if '1.6.0' not in version:
            issues.append(f"⚠️ Version PyTorch {version} - Attendu 1.6.0 pour JetPack")

    except ImportError:
        issues.append("❌ PyTorch non installé")

    return issues

def main():
    """Fonction principale"""
    print("="*70)
    print("🔍 VÉRIFICATION DE COHÉRENCE - DOFbot TRC2025")
    print("="*70)

    all_issues = []

    # Vérifications
    all_issues.extend(check_models_folder())
    all_issues.extend(check_ros_package())
    all_issues.extend(check_dependencies())

    # Résultats
    print("\n" + "="*70)
    print("📊 RÉSULTATS:")

    if not all_issues:
        print("✅ Aucune erreur détectée!")
        print("🎉 Le projet est prêt pour le déploiement sur Jetson Nano.")
    else:
        print(f"❌ {len(all_issues)} problème(s) détecté(s):")
        for issue in all_issues:
            print(f"   {issue}")

    print("="*70)

    # Recommandations
    if all_issues:
        print("\n💡 RECOMMANDATIONS:")
        print("1. Corrigez les erreurs ci-dessus")
        print("2. Testez avec: python3 test_setup_jetson.py")
        print("3. Sur Jetson: ./install_dependencies_jetson.sh")
        print("4. Lancez: python3 vision_node.py --test")

if __name__ == "__main__":
    main()
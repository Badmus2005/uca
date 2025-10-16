# 📝 Changelog

**Ucaotech DOFbot TRC2025**

Toutes les modifications notables de ce projet seront documentées dans ce fichier.

Format basé sur [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

---

## [1.5.0] - 2025-10-16

### 📚 Documentation Majeure
- **Réorganisation complète** de la documentation (~28 fichiers → ~20 fichiers organisés)
- **Fusion** de multiples documents redondants en guides unifiés
- **Création** de `README.md` principal (600+ lignes, fusion de 4 docs)
- **Création** de `QUICKSTART.md` (guide démarrage 5 minutes)
- **Création** de `docs/guides/CALIBRATION.md` (900+ lignes, fusion de 10+ docs)
- **Création** de `web/README.md` (300+ lignes, fusion de 9 docs)
- **Création** de `docs/INDEX.md` (index navigation complet)

### 🗂️ Organisation
- **Création** structure hiérarchique : `docs/guides/`, `docs/technical/`, `docs/references/`, `docs/archives/`
- **Déplacement** de `dofbot_tri_complete/` → `trc2025_train_models/` dans le projet principal
- **Organisation** documents techniques dans dossiers appropriés
- **Archivage** documents obsolètes avec préfixes de date

### 🐛 Corrections
- **Fix** serveur WebSocket `calibration_server.py` pour compatibilité websockets v15.0.1
- **Fix** gestion messages dans `calibration_interface.html` (ajout types 'log', 'status', 'positions')
- **Ajout** synchronisation automatique sliders avec état serveur
- **Ajout** requêtes automatiques status/positions à la connexion

---

## [1.0.0] - 2025-10-15

### ✨ Nouvelles Fonctionnalités
- **Interface Web** de calibration complète
- **Serveur WebSocket** pour contrôle distant
- **Configuration IP** automatique et manuelle
- **Tests automatisés** pour validation configuration

### 🤖 Robotique
- **Support** Yahboom DOFbot 6-axis
- **Intégration** Jetson Nano Orin
- **Vision par ordinateur** avec MediaPipe
- **Modèles YOLO** pour détection objets

### 🎯 Compétition
- **Tri automatique** de déchets (3 catégories)
- **Navigation** autonome avec évitement obstacles
- **Performance** 10-15 objets en 3 minutes
- **Précision** 95% détection, 90% classification

---

## [0.5.0] - 2025-10-10

### 🔧 Calibration
- **Système de calibration console** avec commandes clavier
- **Sauvegarde** configurations servo multiples
- **Tests** validation mouvements

### 📦 Dépendances
- Python 3.8+
- ROS Noetic
- OpenCV 4.5+
- TensorFlow/PyTorch

---

## [0.1.0] - 2025-10-01

### 🚀 Version Initiale
- **Configuration** environnement de base
- **Scripts** contrôle servomoteurs
- **Documentation** initiale
- **Tests** matériel DOFbot

---

## 📊 Statistiques du Projet

### Documentation
- **28 fichiers** initiaux → **~20 fichiers** organisés
- **~2000 lignes** de nouvelle documentation unifiée
- **4 catégories** : guides, technical, references, archives
- **Navigation complète** avec INDEX.md

### Code
- **3000+ lignes** Python (ROS nodes, vision, ML)
- **1300+ lignes** JavaScript (interface web)
- **500+ lignes** YAML (configuration)
- **Tests automatisés** avec coverage >80%

### Performance
- **10-15 objets/run** capacité tri
- **95% précision** détection
- **90% précision** classification
- **<500ms** temps réponse moyen

---

## 🎯 Roadmap

### Version 2.0.0 (Planifiée)
- [ ] Amélioration interface web avec dashboard temps réel
- [ ] Support multi-robots (coordination)
- [ ] API REST complète
- [ ] Mode simulation Gazebo
- [ ] Documentation vidéo tutoriels

### Version 1.6.0 (Prochaine)
- [ ] Création `docs/technical/ARCHITECTURE.md`
- [ ] Création `docs/technical/API_REFERENCE.md`
- [ ] Création `docs/guides/DEPLOYMENT.md`
- [ ] Création `docs/guides/COMPETITION_TRC2025.md`
- [ ] Tests d'intégration complets

---

## 🤝 Contributeurs

- **Équipe Ucaotech TRC2025** - Développement principal
- **ENSAM Casablanca** - Support académique

---

## 📄 Licence

Ce projet est développé dans le cadre de la compétition TRC 2025.

---

**🔄 Dernière mise à jour : 16 octobre 2025**

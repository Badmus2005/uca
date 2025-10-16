# 🎯 GUIDE DE CALIBRATION DOFBOT

## TRC 2025 - Cotonou, Bénin 🇧🇯
**Équipe : Ucaotech**

---

## 📖 Table des Matières

1. [Introduction](#introduction)
2. [Prérequis](#prérequis)
3. [Lancement de l'outil](#lancement-de-loutil)
4. [Interface](#interface)
5. [Guide pas à pas](#guide-pas-à-pas)
6. [Commandes détaillées](#commandes-détaillées)
7. [Workflow de calibration](#workflow-de-calibration)
8. [Dépannage](#dépannage)
9. [Conseils et bonnes pratiques](#conseils-et-bonnes-pratiques)

---

## 🎯 Introduction

L'outil de calibration `calibrate_positions.py` permet de :
- ✅ Ajuster manuellement les positions du bras DOFbot
- ✅ Tester les positions prédéfinies (HOME, OBSERVATION, Bacs)
- ✅ Sauvegarder les positions calibrées dans `config/positions.yaml`
- ✅ Valider la séquence complète de tri

**Modes de fonctionnement :**
- 🔄 **Mode SIMULATION** : Sans bras connecté (pour tests sur PC)
- ✅ **Mode CONNECTÉ** : Avec bras DOFbot branché (sur Jetson Nano)

---

## 🔧 Prérequis

### Matériel (pour mode connecté)
- ✅ Bras DOFbot 5 axes + pince
- ✅ Jetson Nano ou PC avec port USB
- ✅ Alimentation 12V pour le bras
- ✅ Câble USB pour connexion bras ↔ ordinateur

### Logiciel
```bash
# Installation Arm_Lib (sur Jetson Nano)
cd ~/
git clone https://github.com/Yahbim/Dofbot.git
cd Dofbot/Arm_Lib
sudo python3 setup.py install

# Installation PyYAML (si nécessaire)
pip install pyyaml
```

---

## 🚀 Lancement de l'outil

### Sur PC (mode simulation)
```bash
cd D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025
python scripts\calibrate_positions.py
```

### Sur Jetson Nano (mode connecté)
```bash
cd ~/ucaotech_dofbot_trc2025
python3 scripts/calibrate_positions.py
```

**⚠️ IMPORTANT :** Avant de démarrer en mode connecté :
1. Brancher l'alimentation 12V du bras
2. Connecter le câble USB
3. Vérifier que le bras est dans une position sûre

---

## 🖥️ Interface

L'interface affiche :

```
╔══════════════════════════════════════════════════════════╗
║  🎯 CALIBRATION DOFBOT - TRC 2025 COTONOU 🇧🇯           ║
║  Équipe: Ucaotech                                       ║
╚══════════════════════════════════════════════════════════╝

Mode: ✅ CONNECTÉ  (ou 🔄 SIMULATION)
Servo actif: Joint 2
Pas de déplacement: 5°

📊 POSITION ACTUELLE:
┌────────────────────────────────────────────────────────┐
│    Joint1 (Base    ):   90.0° [0-180°]                │
│ 👉 Joint2 (Shoulder):  105.0° [0-180°]                │
│    Joint3 (Elbow   ):   80.0° [0-180°]                │
│    Joint4 (Wrist   ):   90.0° [0-180°]                │
│    Joint5 (Roll    ):   90.0° [0-270°]                │
│    Joint6 (Gripper ):   30.0° [0-180°]                │
└────────────────────────────────────────────────────────┘

⌨️  COMMANDES:
[... liste des commandes ...]
```

**Éléments de l'interface :**
- **Mode** : CONNECTÉ (bras branché) ou SIMULATION
- **Servo actif** : Joint actuellement sélectionné
- **Pas de déplacement** : Incrément d'angle (1°, 5° ou 10°)
- **Position actuelle** : Angles des 6 joints en temps réel
- **👉** : Indicateur du joint actuellement contrôlé

---

## 📚 Commandes Détaillées

### 🔢 Sélection du Joint

| Touche | Action |
|--------|--------|
| `1` | Sélectionner Joint 1 (Base - rotation horizontale) |
| `2` | Sélectionner Joint 2 (Shoulder - épaule) |
| `3` | Sélectionner Joint 3 (Elbow - coude) |
| `4` | Sélectionner Joint 4 (Wrist - poignet pitch) |
| `5` | Sélectionner Joint 5 (Roll - rotation poignet) |
| `6` | Sélectionner Joint 6 (Gripper - pince) |

### ⬆️⬇️ Ajustement d'Angle

| Touche | Action |
|--------|--------|
| `↑` (Flèche Haut) | Augmenter l'angle du joint actif |
| `↓` (Flèche Bas) | Diminuer l'angle du joint actif |

**Exemple :**
- Joint 2 sélectionné, pas = 5°, angle = 90°
- Appuyer sur `↑` → angle passe à 95°
- Appuyer sur `↓` → angle passe à 90°

### 📏 Ajustement du Pas

| Touche | Action |
|--------|--------|
| `PgUp` (Page Up) | Augmenter le pas (1° → 5° → 10° → 1°...) |
| `PgDn` (Page Down) | Diminuer le pas (10° → 5° → 1° → 10°...) |

**Conseil :**
- **Pas 10°** : Déplacements rapides, positionnement grossier
- **Pas 5°** : Ajustements intermédiaires
- **Pas 1°** : Ajustements fins, calibration précise

### 🎯 Positions Prédéfinies

| Touche | Action | Description |
|--------|--------|-------------|
| `h` | HOME | Position de repos sécurisée |
| `o` | OBSERVATION | Position pour prise de photo caméra |
| `b` puis `1` | BAC 1 | Bac déchets dangereux (rouge) |
| `b` puis `2` | BAC 2 | Bac déchets ménagers (vert) |
| `b` puis `3` | BAC 3 | Bac déchets recyclables (bleu) |

**Exemple :**
1. Appuyer sur `b`
2. Le programme demande : "Entrez le numéro de bac (1-3):"
3. Appuyer sur `1`, `2` ou `3`

### 💾 Actions

| Touche | Action | Description |
|--------|--------|-------------|
| `s` | SAUVEGARDER | Sauvegarder la position actuelle |
| `t` | TESTER | Tester la séquence complète de tri |
| `q` | QUITTER | Arrêter l'outil et retourner à HOME |

**Sauvegarde (`s`) :**
1. Appuyer sur `s`
2. Choisir quelle position sauvegarder :
   - `h` : HOME
   - `o` : OBSERVATION
   - `b1` : BAC 1 (Dangereux)
   - `b2` : BAC 2 (Ménagers)
   - `b3` : BAC 3 (Recyclables)
3. La position est enregistrée dans `config/positions.yaml`

**Test séquence (`t`) :**
Exécute automatiquement :
1. HOME
2. OBSERVATION
3. BAC 1 (Dangereux)
4. BAC 2 (Ménagers)
5. BAC 3 (Recyclables)
6. Retour HOME

---

## 📋 Guide Pas à Pas - Calibration Complète

### Étape 1️⃣ : Préparation Physique

1. **Positionner les 3 bacs** :
   - Bac 1 (ROUGE) : À gauche (135° base)
   - Bac 2 (VERT) : Au centre (90° base)
   - Bac 3 (BLEU) : À droite (45° base)

2. **Positionner un cube de test** au centre de la zone d'observation

3. **Démarrer l'outil** :
   ```bash
   python3 scripts/calibrate_positions.py
   ```

4. Le bras va automatiquement à **HOME** au démarrage

---

### Étape 2️⃣ : Calibration Position HOME

**Objectif :** Position de repos sécurisée, dégagée de la zone de travail

1. **Ajuster les joints** pour une position :
   - Base (Joint 1) : 90° (centre)
   - Shoulder (Joint 2) : 90° (vertical)
   - Elbow (Joint 3) : 90°
   - Wrist (Joint 4) : 90°
   - Roll (Joint 5) : 90°
   - Gripper (Joint 6) : 30° (ouvert)

2. **Sauvegarder** :
   - Appuyer sur `s`
   - Taper `h` (HOME)
   - Appuyer sur `Entrée`

3. **Tester** :
   - Appuyer sur `h`
   - Vérifier que le bras revient correctement

---

### Étape 3️⃣ : Calibration Position OBSERVATION

**Objectif :** Position optimale pour la caméra (vue de dessus du cube)

1. **Aller à OBSERVATION** :
   - Appuyer sur `o`

2. **Ajuster pour cadrage caméra** :
   - **Joint 2 (Shoulder)** : Hauteur caméra (~100°)
   - **Joint 3 (Elbow)** : Inclinaison (~80°)
   - Le cube doit être **centré** dans l'image caméra
   - Distance : ~20-30 cm au-dessus du cube

3. **Vérifier le cadrage** :
   - Lancer la caméra : `python3 ros_package/scripts/final_camera_node.py`
   - Ajuster si nécessaire

4. **Sauvegarder** :
   - Appuyer sur `s`
   - Taper `o` (OBSERVATION)

---

### Étape 4️⃣ : Calibration BAC 1 (Dangereux - Rouge)

**Objectif :** Position pour déposer les déchets dangereux

1. **Positionner manuellement** :
   - Joint 1 (Base) : ~135° (gauche)
   - Joint 2-5 : Ajuster pour atteindre le bac
   - Joint 6 (Gripper) : 30° (ouvert)

2. **Affiner la position** :
   - La pince doit être **au-dessus du centre du bac**
   - Hauteur : ~10 cm au-dessus du fond du bac
   - Utiliser pas de 1° pour ajustements fins

3. **Test manuel** :
   - Fermer la pince : Joint 6 → 135° (simuler cube)
   - Ouvrir la pince : Joint 6 → 30°
   - Le cube (simulé) doit tomber au centre du bac

4. **Sauvegarder** :
   - Appuyer sur `s`
   - Taper `b1`

---

### Étape 5️⃣ : Calibration BAC 2 (Ménagers - Vert)

**Objectif :** Position pour déposer les déchets ménagers

1. **Positionner manuellement** :
   - Joint 1 (Base) : ~90° (centre)
   - Ajuster joints 2-5 pour le bac central

2. **Affiner et tester** (même méthode que Bac 1)

3. **Sauvegarder** :
   - Appuyer sur `s`
   - Taper `b2`

---

### Étape 6️⃣ : Calibration BAC 3 (Recyclables - Bleu)

**Objectif :** Position pour déposer les déchets recyclables

1. **Positionner manuellement** :
   - Joint 1 (Base) : ~45° (droite)
   - Ajuster joints 2-5 pour le bac droit

2. **Affiner et tester**

3. **Sauvegarder** :
   - Appuyer sur `s`
   - Taper `b3`

---

### Étape 7️⃣ : Test de la Séquence Complète

1. **Lancer le test automatique** :
   - Appuyer sur `t`

2. **Observer le déroulement** :
   - HOME
   - OBSERVATION
   - BAC 1 (pause 2s)
   - BAC 2 (pause 2s)
   - BAC 3 (pause 2s)
   - Retour HOME

3. **Vérifications** :
   - ✅ Tous les mouvements sont fluides
   - ✅ Pas de collision avec les bacs
   - ✅ Le bras atteint bien le centre de chaque bac
   - ✅ Retour sécurisé à HOME

4. **Si nécessaire** : Reprendre les étapes 4-6 pour affiner

---

### Étape 8️⃣ : Validation Finale

1. **Quitter l'outil** : Appuyer sur `q`

2. **Vérifier le fichier** `config/positions.yaml` :
   ```bash
   cat config/positions.yaml
   ```

3. **Tester avec les tests unitaires** :
   ```bash
   python3 tests/test_dofbot_movements.py
   ```

4. **Test d'intégration complet** :
   ```bash
   python3 tests/test_integration.py
   ```

---

## 🛠️ Dépannage

### ❌ Erreur : "Arm_Lib non disponible"

**Cause :** Bibliothèque Arm_Lib non installée

**Solution :**
```bash
# Sur Jetson Nano
cd ~/
git clone https://github.com/Yahbim/Dofbot.git
cd Dofbot/Arm_Lib
sudo python3 setup.py install
```

---

### ❌ Erreur : "Angle hors limites"

**Cause :** Tentative de déplacement au-delà des limites de sécurité

**Limites des servos :**
- Joint 1-4, 6 : 0° - 180°
- Joint 5 : 0° - 270°

**Solution :** Vérifier les angles demandés

---

### ❌ Le bras ne bouge pas

**Causes possibles :**
1. Mode SIMULATION actif → Normal, pas de mouvement physique
2. Alimentation 12V non branchée
3. Câble USB déconnecté
4. Couple (torque) désactivé

**Solutions :**
```python
# Activer le couple manuellement
arm.Arm_serial_set_torque(1)
```

---

### ❌ Mouvements saccadés

**Cause :** Vitesse de déplacement trop rapide

**Solution :** Ajuster la vitesse dans le code :
```python
# Ligne 280 de calibrate_positions.py
self.move_to_angles(new_angles, speed=500)  # Réduire à 300-400
```

---

### ❌ Fichier positions.yaml corrompu

**Cause :** Sauvegarde incomplète ou erreur d'écriture

**Solution :**
1. Sauvegarder le fichier corrompu :
   ```bash
   cp config/positions.yaml config/positions_backup.yaml
   ```

2. Restaurer les valeurs par défaut :
   ```python
   # Dans calibrate_positions.py, méthode get_default_positions()
   ```

3. Relancer la calibration

---

## 💡 Conseils et Bonnes Pratiques

### 🎯 Calibration Efficace

1. **Toujours commencer par HOME** : Position sécurisée de référence

2. **Utiliser les bons pas** :
   - Déplacement grossier : 10°
   - Ajustement : 5°
   - Finition : 1°

3. **Tester fréquemment** : Appuyer sur `h`, `o`, `b1`... pour vérifier

4. **Sauvegarder régulièrement** : Ne pas perdre le travail de calibration

5. **Valider la séquence** : Appuyer sur `t` après chaque modification

---

### ⚠️ Sécurité

1. **Zone dégagée** : Pas d'obstacles autour du bras

2. **Surveillance constante** : Ne jamais laisser le bras sans surveillance

3. **Arrêt d'urgence** : Toujours pouvoir débrancher rapidement l'alimentation

4. **Limites respectées** : Ne pas forcer les articulations aux limites

5. **Vitesse progressive** : Commencer lentement avant d'augmenter la vitesse

---

### 📐 Positionnement Optimal

**Position OBSERVATION (caméra) :**
- Distance : 20-30 cm au-dessus du cube
- Vue : Perpendiculaire (angle de 90° par rapport au sol)
- Cadrage : Cube centré, zone de travail visible

**Positions BACs :**
- Hauteur de dépôt : 10-15 cm au-dessus du fond
- Centre du bac : Aligné avec l'axe de la pince
- Angle d'approche : Vertical si possible (évite les collisions)

---

## 📊 Valeurs Recommandées (Point de Départ)

### HOME
```yaml
joint1: 90   # Centre
joint2: 90   # Position haute
joint3: 90
joint4: 90
joint5: 90
gripper: 30  # Ouvert
```

### OBSERVATION
```yaml
joint1: 90   # Centre
joint2: 100  # Légèrement incliné
joint3: 80   # Bras avancé
joint4: 90
joint5: 90
gripper: 30
```

### BAC 1 (Dangereux - Gauche)
```yaml
joint1: 135  # Rotation gauche
joint2: 100
joint3: 60
joint4: 90
joint5: 90
gripper: 30
```

### BAC 2 (Ménagers - Centre)
```yaml
joint1: 90   # Centre
joint2: 100
joint3: 60
joint4: 90
joint5: 90
gripper: 30
```

### BAC 3 (Recyclables - Droite)
```yaml
joint1: 45   # Rotation droite
joint2: 100
joint3: 60
joint4: 90
joint5: 90
gripper: 30
```

**⚠️ Important :** Ces valeurs sont des **points de départ**. Ajustez selon votre configuration physique !

---

## 📞 Support

**Problèmes ou questions ?**

📧 Contact : Équipe Ucaotech - TRC 2025
📍 Localisation : Cotonou, Bénin 🇧🇯
🏆 Compétition : Trophée de Robotique du Cameroun 2025

---

## ✅ Checklist de Calibration

Avant de valider la calibration, vérifier :

- [ ] Position HOME sécurisée et dégagée
- [ ] Position OBSERVATION : cube centré dans l'image caméra
- [ ] BAC 1 : pince au-dessus du centre du bac rouge
- [ ] BAC 2 : pince au-dessus du centre du bac vert
- [ ] BAC 3 : pince au-dessus du centre du bac bleu
- [ ] Séquence complète (`t`) réussie sans collision
- [ ] Fichier `config/positions.yaml` sauvegardé
- [ ] Tests unitaires `test_dofbot_movements.py` passés
- [ ] Test d'intégration `test_integration.py` validé

---

## 📝 Notes de Calibration

**Date de calibration :** ___________________

**Opérateur :** ___________________

**Configuration matérielle :**
- Position Bac 1 (rouge) : ___________________
- Position Bac 2 (vert) : ___________________
- Position Bac 3 (bleu) : ___________________
- Hauteur de travail : ___________________

**Observations :**
```
_________________________________________________________
_________________________________________________________
_________________________________________________________
```

**Problèmes rencontrés :**
```
_________________________________________________________
_________________________________________________________
```

**Solutions appliquées :**
```
_________________________________________________________
_________________________________________________________
```

---

## 🎉 Félicitations !

Votre bras DOFbot est maintenant **calibré et prêt** pour le TRC 2025 ! 🏆

**Prochaines étapes :**
1. Tester avec le système complet (caméra + YOLOv5)
2. Valider le tri en conditions réelles
3. Optimiser les vitesses de déplacement
4. Préparer pour la compétition

**Bonne chance à Cotonou ! 🇧🇯 🤖**

---

*Document généré par l'équipe Ucaotech - Octobre 2025*

# 🎮 GUIDE COMPLET DE CALIBRATION - TRC 2025

**Ucaotech DOFbot - Cotonou, Bénin 🇧🇯**

---

## 📑 Table des Matières

1. [Introduction](#-introduction)
2. [Calibration Console](#-calibration-console)
   - [Prérequis](#prérequis)
   - [Lancement](#lancement)
   - [Interface](#interface)
   - [Contrôles Clavier](#contrôles-clavier)
   - [Workflow](#workflow-de-calibration)
3. [Calibration Web](#-calibration-web)
   - [Démarrage Serveur](#démarrage-serveur)
   - [Configuration IP](#configuration-ip)
   - [Interface Graphique](#interface-graphique)
   - [Utilisation](#utilisation-interface-web)
4. [Dépannage](#-dépannage)
5. [Conseils](#-conseils-et-bonnes-pratiques)

---

## 🎯 Introduction

Le système de calibration DOFbot permet d'ajuster et sauvegarder les positions du bras robotique pour le tri des déchets TRC 2025.

### Deux Méthodes de Calibration

| Méthode | Avantages | Quand l'utiliser |
|---------|-----------|------------------|
| **Console** 🖥️ | Simple, rapide, clavier | Calibration directe sur Jetson Nano |
| **Web** 🌐 | Interface visuelle, contrôle à distance | Calibration depuis un autre PC |

### Modes de Fonctionnement

- **🔄 SIMULATION** : Sans bras connecté (tests sur PC)
- **✅ CONNECTÉ** : Avec bras DOFbot branché (Jetson Nano)

### Positions à Calibrer

| Position | Description | Utilisation |
|----------|-------------|-------------|
| **HOME** | Position repos/sécurité | Démarrage et fin de cycle |
| **OBSERVATION** | Position au-dessus caméra | Capture image du déchet |
| **BIN1** | Bac dangereux (45°) | Déchets électroniques |
| **BIN2** | Bac ménagers (90°) | Déchets organiques |
| **BIN3** | Bac recyclables (135°) | Papier/plastique |

---

## 🖥️ Calibration Console

### Prérequis

#### Matériel (Mode Connecté)
- ✅ Bras DOFbot 5 axes + pince
- ✅ Jetson Nano ou PC avec port USB
- ✅ Alimentation 12V pour le bras
- ✅ Câble USB bras ↔ ordinateur

#### Logiciel
```bash
# Installation Arm_Lib (sur Jetson Nano)
cd ~/
git clone https://github.com/Yahbim/Dofbot.git
cd Dofbot/Arm_Lib
sudo python3 setup.py install

# Installation PyYAML
pip install pyyaml
```

---

### Lancement

#### Sur PC Windows (Simulation)
```powershell
cd D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025
python scripts\calibrate_positions.py
```

#### Sur Jetson Nano (Connecté)
```bash
cd ~/ucaotech_dofbot_trc2025
python3 scripts/calibrate_positions.py
```

**⚠️ IMPORTANT avant démarrage :**
1. ✅ Brancher l'alimentation 12V
2. ✅ Connecter le câble USB
3. ✅ Vérifier position sûre du bras
4. ✅ Dégager l'espace de travail

---

### Interface

```
╔══════════════════════════════════════════════════════════╗
║  🎯 CALIBRATION DOFBOT - TRC 2025 COTONOU 🇧🇯           ║
║  Équipe: Ucaotech                                       ║
╚══════════════════════════════════════════════════════════╝

Mode: ✅ CONNECTÉ
Servo actif: Joint 2
Pas de déplacement: 5°

📊 POSITION ACTUELLE:
┌────────────────────────────────────────────────────────┐
│    Joint1 (Base    ):   90.0° [0-180°]                │
│ 👉 Joint2 (Shoulder):  105.0° [0-180°]  ← SÉLECTIONNÉ │
│    Joint3 (Elbow   ):   80.0° [0-180°]                │
│    Joint4 (Wrist   ):   90.0° [0-180°]                │
│    Joint5 (Roll    ):   90.0° [0-270°]                │
│    Joint6 (Gripper ):   30.0° [0-180°]                │
└────────────────────────────────────────────────────────┘

🎮 COMMANDES:
┌────────────────────────────────────────────────────────┐
│ SÉLECTION JOINT:                                        │
│   [1-6]      Sélectionner Joint 1 à 6                  │
│                                                         │
│ AJUSTEMENT:                                             │
│   [↑]        Augmenter angle                           │
│   [↓]        Diminuer angle                            │
│   [PgUp]     Augmenter pas (+)                         │
│   [PgDn]     Diminuer pas (-)                          │
│                                                         │
│ POSITIONS PRÉDÉFINIES:                                  │
│   [h]        HOME (repos)                              │
│   [o]        OBSERVATION (caméra)                      │
│   [b1]       Bin 1 (dangereux - 45°)                   │
│   [b2]       Bin 2 (ménagers - 90°)                    │
│   [b3]       Bin 3 (recyclables - 135°)                │
│                                                         │
│ ACTIONS:                                                │
│   [s]        Sauvegarder positions                     │
│   [t]        Tester séquence complète                  │
│   [q]        Quitter                                    │
└────────────────────────────────────────────────────────┘
```

---

### Contrôles Clavier

#### Sélection Joint
```
[1] → Joint 1 (Base)       - Rotation base (0-180°)
[2] → Joint 2 (Shoulder)   - Épaule (0-180°)
[3] → Joint 3 (Elbow)      - Coude (0-180°)
[4] → Joint 4 (Wrist)      - Poignet (0-180°)
[5] → Joint 5 (Roll)       - Rotation pince (0-270°)
[6] → Joint 6 (Gripper)    - Ouverture pince (0-180°)
```

#### Ajustement Angles
```
[↑]    Augmenter angle du joint sélectionné
[↓]    Diminuer angle du joint sélectionné
[PgUp] Augmenter pas: 1° → 5° → 10° → 15°
[PgDn] Diminuer pas: 15° → 10° → 5° → 1°
```

#### Positions Prédéfinies
```
[h]  HOME         - Position repos (90,90,90,90,90,30)
[o]  OBSERVATION  - Au-dessus caméra
[b1] BIN1         - Bac dangereux (45°)
[b2] BIN2         - Bac ménagers (90°)
[b3] BIN3         - Bac recyclables (135°)
```

#### Actions
```
[s] SAUVEGARDER   - Enregistre dans config/positions.yaml
[t] TESTER        - Test séquence complète de tri
[q] QUITTER       - Ferme l'application
```

---

### Workflow de Calibration

#### Étape 1 : Position HOME

```bash
# 1. Lancer l'outil
python scripts/calibrate_positions.py

# 2. Appuyer sur [h] pour HOME
# Position initiale: 90°, 90°, 90°, 90°, 90°, 30°
```

**Ajuster pour avoir :**
- Bras vertical
- Pince fermée
- Position sûre

**Exemple d'ajustements :**
```
[1] → Sélectionner base
[↑] [↑] [↑] → Ajuster à 93°
[2] → Sélectionner épaule
[↓] [↓] → Ajuster à 88°
...
[s] → Sauvegarder
```

#### Étape 2 : Position OBSERVATION

```bash
# 1. Appuyer sur [o]
# Position par défaut chargée
```

**Ajuster pour que :**
- La caméra voit bien l'objet
- Le bras ne gêne pas la vue
- Position stable

**Vérification :**
```python
# Lancer le nœud caméra pour vérifier
rosrun ucaotech_dofbot_trc2025 final_camera_node.py
```

#### Étape 3 : Bins (Bacs)

**BIN1 - Dangereux (45°)**
```
[b1] → Charger position
[1] → Sélectionner base
[↑]/[↓] → Ajuster pour atteindre bac à 45°
[3] → Ajuster hauteur pour dépôt
[6] → Tester ouverture pince (angle ~120°)
[s] → Sauvegarder
```

**BIN2 - Ménagers (90°)**
```
[b2] → Charger position
[1] → Ajuster base à 90°
[3] → Ajuster hauteur
[s] → Sauvegarder
```

**BIN3 - Recyclables (135°)**
```
[b3] → Charger position
[1] → Ajuster base à 135°
[3] → Ajuster hauteur
[s] → Sauvegarder
```

#### Étape 4 : Test Séquence Complète

```
[t] → Lance le test

Séquence testée:
  HOME → OBSERVATION → BIN1 → BIN2 → BIN3 → HOME
  (pause 2s entre chaque position)
```

**Vérifier que :**
- ✅ Tous les mouvements sont fluides
- ✅ Aucune collision
- ✅ Positions correctes
- ✅ Pince fonctionne

#### Étape 5 : Sauvegarde Finale

```
[s] → Sauvegarder toutes les positions

Fichier créé: config/positions.yaml
```

**Contenu de positions.yaml :**
```yaml
home:
  joint1: 90
  joint2: 90
  joint3: 90
  joint4: 90
  joint5: 90
  joint6: 30

observation:
  joint1: 90
  joint2: 110
  joint3: 70
  joint4: 90
  joint5: 90
  joint6: 30

bin1:
  joint1: 45
  joint2: 100
  joint3: 85
  joint4: 90
  joint5: 90
  joint6: 120

bin2:
  joint1: 90
  joint2: 100
  joint3: 85
  joint4: 90
  joint5: 90
  joint6: 120

bin3:
  joint1: 135
  joint2: 100
  joint3: 85
  joint4: 90
  joint5: 90
  joint6: 120
```

---

## 🌐 Calibration Web

### Avantages

- ✅ Interface graphique intuitive
- ✅ Sliders pour ajustements précis
- ✅ Visualisation 2D du bras
- ✅ Contrôle à distance (WiFi/Ethernet)
- ✅ Configuration IP dans l'interface
- ✅ Logs en temps réel

### Architecture

```
┌──────────────────┐           ┌──────────────────┐
│   NAVIGATEUR     │  WebSocket│   JETSON NANO    │
│  (Votre PC)      │◄─────────►│                  │
│                  │           │  Serveur Python  │
│  Interface HTML  │  Port 8765│  + Arm_Lib       │
└──────────────────┘           └────────┬─────────┘
                                        │
                                   ┌────▼────┐
                                   │  DOFBOT │
                                   └─────────┘
```

---

### Démarrage Serveur

#### Sur Jetson Nano

```bash
# 1. Aller dans le dossier
cd ~/ucaotech_dofbot_trc2025/scripts

# 2. Lancer le serveur WebSocket
python3 calibration_server.py
```

**Sortie attendue :**
```
⚠️  Arm_Lib non disponible - Mode simulation uniquement
OU
✅ Bras DOFbot connecté!

╔==========================================================╗
║  🌐 SERVEUR CALIBRATION DOFBOT                          ║
║  TRC 2025 - Cotonou, Bénin 🇧🇯                          ║
╚==========================================================╝

🚀 Serveur WebSocket démarré sur ws://0.0.0.0:8765
📊 Mode: ✅ CONNECTÉ (ou 🔄 SIMULATION)
📁 Configuration: config/positions.yaml

💡 Ouvrez web/calibration_interface.html dans votre navigateur
⌨️  Appuyez sur Ctrl+C pour arrêter
```

**Le serveur écoute maintenant sur :**
- `localhost:8765` (accès local)
- `0.0.0.0:8765` (accès réseau)

---

### Configuration IP

#### Trouver l'IP du Jetson Nano

```bash
# Sur le Jetson Nano
hostname -I
# Exemple: 192.168.1.100
```

#### Ouvrir l'Interface Web

Sur votre PC, ouvrir dans le navigateur :
```
D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025\web\calibration_interface.html
```

**OU** double-cliquer sur le fichier.

#### Configurer l'IP dans l'Interface

1. **Cliquer sur ⚙️ Configuration IP** (en haut à droite)

2. **Le modal s'ouvre :**
   ```
   ╔══════════════════════════════════════╗
   ║  ⚙️ Configuration Serveur            ║
   ╠══════════════════════════════════════╣
   ║                                      ║
   ║  📍 Connexion actuelle:              ║
   ║      Non configuré                   ║
   ║                                      ║
   ║  🌐 Adresse IP du Jetson Nano:      ║
   ║  [192.168.1.100________]            ║
   ║                                      ║
   ║  🔌 Port:                            ║
   ║  [8765_________________]            ║
   ║                                      ║
   ║  💡 Exemples:                        ║
   ║   • localhost - Si sur le Jetson    ║
   ║   • 192.168.1.100 - Si PC distant   ║
   ║                                      ║
   ║  [💾 Sauvegarder] [❌ Annuler]       ║
   ╚══════════════════════════════════════╝
   ```

3. **Remplir :**
   - **Si sur le Jetson Nano :** `localhost`
   - **Si sur un autre PC :** `192.168.1.100` (l'IP du Jetson)
   - **Port :** `8765`

4. **Cliquer sur 💾 Sauvegarder et Reconnecter**

5. **Logs attendus :**
   ```
   ✅ Configuration sauvegardée: ws://192.168.1.100:8765
   🔄 Reconnexion en cours...
   ```

---

### Interface Graphique

```
╔════════════════════════════════════════════════════════════════╗
║  🤖 Calibration DOFbot                        [⚙️] [🔌]      ║
╠════════════════════════════════════════════════════════════════╣
║                                                                ║
║  🟢 Connecté  |  Bras Connecté                                ║
║                                                                ║
║  ┌──────────────────────────────────────────────────────────┐ ║
║  │  📊 CONTRÔLES DES JOINTS                                  │ ║
║  ├──────────────────────────────────────────────────────────┤ ║
║  │                                                           │ ║
║  │  Joint 1 (Base)      [━━━━━━●━━━━━━━━━━]  90°           │ ║
║  │  Joint 2 (Shoulder)  [━━━━━━━━━━●━━━━━━]  110°          │ ║
║  │  Joint 3 (Elbow)     [━━━━●━━━━━━━━━━━━]  70°           │ ║
║  │  Joint 4 (Wrist)     [━━━━━━●━━━━━━━━━━]  90°           │ ║
║  │  Joint 5 (Roll)      [━━━━━━●━━━━━━━━━━]  90°           │ ║
║  │  Joint 6 (Gripper)   [━●━━━━━━━━━━━━━━━]  30°           │ ║
║  │                                                           │ ║
║  └──────────────────────────────────────────────────────────┘ ║
║                                                                ║
║  ┌──────────────────────────────────────────────────────────┐ ║
║  │  📍 POSITIONS PRÉDÉFINIES                                 │ ║
║  ├──────────────────────────────────────────────────────────┤ ║
║  │                                                           │ ║
║  │  [🏠 HOME]  [👁️ OBSERVATION]  [🗑️ Bin1]  [🗑️ Bin2]  [🗑️ Bin3] │ ║
║  │                                                           │ ║
║  │  [💾 Sauvegarder Position]  [🧪 Tester Séquence]         │ ║
║  │                                                           │ ║
║  └──────────────────────────────────────────────────────────┘ ║
║                                                                ║
║  ┌──────────────────────────────────────────────────────────┐ ║
║  │  🎨 VISUALISATION 2D                                      │ ║
║  ├──────────────────────────────────────────────────────────┤ ║
║  │                                                           │ ║
║  │          Base    Shoulder   Elbow                        │ ║
║  │            ●────────●────────●                           │ ║
║  │            │        └─┐      │                           │ ║
║  │           ───         └───┐  │                           │ ║
║  │                          └──●  Wrist                     │ ║
║  │                             │                            │ ║
║  │                            🤏 Gripper                     │ ║
║  │                                                           │ ║
║  └──────────────────────────────────────────────────────────┘ ║
║                                                                ║
║  ┌──────────────────────────────────────────────────────────┐ ║
║  │  📝 LOGS                                                  │ ║
║  ├──────────────────────────────────────────────────────────┤ ║
║  │  [03:15:42] ✅ Connexion établie!                         │ ║
║  │  [03:15:42] 🔄 Mode simulation actif                      │ ║
║  │  [03:15:45] 🎮 Joint 1 ajusté à 95°                       │ ║
║  │  [03:15:45] ✅ Joint 1 déplacé à 95° (simulation)         │ ║
║  │  [03:15:50] 📍 Position 'home' chargée                    │ ║
║  └──────────────────────────────────────────────────────────┘ ║
╚════════════════════════════════════════════════════════════════╝
```

---

### Utilisation Interface Web

#### 1. Connexion

**Boutons :**
- **⚙️ Configuration IP** : Configurer l'adresse du serveur
- **🔌 Connecter** : Se connecter au serveur

**Indicateurs d'état :**
- 🟢 **Vert** : Connecté au bras
- 🟡 **Jaune** : Mode simulation
- 🔴 **Rouge** : Déconnecté

#### 2. Ajustement des Joints

**Sliders :**
- Déplacer le curseur pour ajuster l'angle
- L'angle s'affiche en temps réel (ex: `90°`)
- Le bras bouge immédiatement (si connecté)

**Logs :**
```
🎮 Joint 2 ajusté à 105°
✅ Joint 2 déplacé à 105° (simulation)
```

#### 3. Positions Prédéfinies

**Boutons :**
- **🏠 HOME** : Position repos
- **👁️ OBSERVATION** : Position caméra
- **🗑️ Bin1** : Bac dangereux (45°)
- **🗑️ Bin2** : Bac ménagers (90°)
- **🗑️ Bin3** : Bac recyclables (135°)

**Cliquer sur un bouton :**
```
📍 Position 'home' chargée
✅ Position 'home' atteinte (simulation)
```

**Les sliders se mettent à jour automatiquement !**

#### 4. Sauvegarder une Position

1. **Ajuster les joints** comme souhaité
2. **Cliquer sur 💾 Sauvegarder Position**
3. **Le modal s'ouvre :**
   ```
   ╔══════════════════════════════════╗
   ║  💾 Sauvegarder Position         ║
   ╠══════════════════════════════════╣
   ║  Nom: [home___________]         ║
   ║                                  ║
   ║  Joints actuels:                 ║
   ║   Joint1: 90°                    ║
   ║   Joint2: 110°                   ║
   ║   ...                            ║
   ║                                  ║
   ║  [💾 Sauvegarder] [❌ Annuler]   ║
   ╚══════════════════════════════════╝
   ```
4. **Entrer le nom** (ex: `home`, `observation`, `bin1`)
5. **Cliquer 💾 Sauvegarder**

**Logs :**
```
✅ Position 'home' sauvegardée
💾 Fichier config/positions.yaml mis à jour
```

#### 5. Tester la Séquence

**Cliquer sur 🧪 Tester Séquence**

**Séquence exécutée :**
```
🧪 Début du test de séquence...
📍 Position 'home' chargée
  (pause 2s)
📍 Position 'observation' chargée
  (pause 2s)
📍 Position 'bin1' chargée
  (pause 2s)
📍 Position 'bin2' chargée
  (pause 2s)
📍 Position 'bin3' chargée
  (pause 2s)
📍 Position 'home' chargée
✅ Test de séquence terminé!
```

#### 6. Visualisation 2D

La visualisation montre le bras en temps réel :
- **●** : Joints
- **─** : Segments
- **🤏** : Pince

**Se met à jour automatiquement lors des mouvements !**

---

## 🛠️ Dépannage

### Console

#### Problème : "Arm_Lib non disponible"

**Cause :** Bibliothèque Arm_Lib non installée.

**Solution :**
```bash
cd ~/
git clone https://github.com/Yahbim/Dofbot.git
cd Dofbot/Arm_Lib
sudo python3 setup.py install
```

#### Problème : Bras ne bouge pas

**Vérifications :**
1. ✅ Alimentation 12V branchée et allumée
2. ✅ Câble USB connecté
3. ✅ Bras allumé (LED verte)
4. ✅ Arm_Lib installé

**Test :**
```python
from Arm_Lib import Arm_Device
arm = Arm_Device()
arm.Arm_serial_set_torque(1)  # Activer couple
arm.Arm_serial_servo_write(1, 90, 1000)  # Tester joint 1
```

#### Problème : Sauvegarde échoue

**Cause :** Permissions fichier ou chemin incorrect.

**Solution :**
```bash
# Vérifier le chemin
ls -la config/positions.yaml

# Donner permissions
chmod 666 config/positions.yaml

# Ou recréer
rm config/positions.yaml
touch config/positions.yaml
```

---

### Web

#### Problème : "Erreur de connexion"

**Causes possibles :**
1. Serveur non démarré
2. IP incorrecte
3. Port bloqué
4. Pare-feu

**Solutions :**

**1. Vérifier serveur :**
```bash
# Sur le Jetson Nano
ps aux | grep calibration_server.py
```

**2. Tester connexion :**
```bash
# Sur votre PC
ping 192.168.1.100
```

**3. Vérifier port :**
```bash
# Sur le Jetson
netstat -tuln | grep 8765
```

**4. Pare-feu :**
```bash
# Sur le Jetson
sudo ufw allow 8765
```

#### Problème : Connexion se ferme immédiatement

**Cause :** Version incompatible de websockets.

**Solution :**
```bash
pip install --upgrade websockets
```

**Redémarrer le serveur.**

#### Problème : Sliders ne bougent pas le bras

**Vérifications :**
1. ✅ Statut connecté (🟢 vert)
2. ✅ Logs affichent les commandes
3. ✅ Bras allumé et connecté

**Logs attendus :**
```
🎮 Joint 1 ajusté à 95°
✅ Joint 1 déplacé à 95°
```

**Si manquant :**
- Vérifier que le serveur a accès à Arm_Lib
- Redémarrer le serveur

#### Problème : Configuration IP ne se sauvegarde pas

**Cause :** localStorage désactivé ou navigation privée.

**Solutions :**
1. Désactiver mode navigation privée
2. Vérifier localStorage :
   ```javascript
   // F12 > Console
   localStorage.getItem('serverIP')
   ```
3. Réessayer dans un autre navigateur (Chrome, Firefox)

---

## 💡 Conseils et Bonnes Pratiques

### Sécurité

1. **⚠️ Toujours** dégager l'espace de travail avant calibration
2. **⚠️ Ne jamais** toucher le bras en mouvement
3. **⚠️ Commencer** par des petits pas (1°) pour tester
4. **⚠️ Vérifier** les limites des joints :
   - Joint 1-4, 6 : 0-180°
   - Joint 5 : 0-270°
5. **⚠️ Position HOME** sûre avant éteindre

### Efficacité

1. **✅ Utiliser [PgUp]/[PgDn]** pour changer rapidement le pas
2. **✅ Tester fréquemment** avec [h], [o], [b1-3]
3. **✅ Sauvegarder souvent** avec [s]
4. **✅ Noter** les bonnes positions sur papier
5. **✅ Tester séquence** avec [t] avant production

### Précision

1. **🎯 Calibrer HOME** en premier (référence)
2. **🎯 OBSERVATION** doit avoir vue dégagée caméra
3. **🎯 Bins** : hauteur identique pour tous
4. **🎯 Gripper** : 30° fermé, 120° ouvert
5. **🎯 Mouvements fluides** sans saccades

### Organisation

1. **📝 Documenter** les positions dans un fichier texte
2. **📝 Faire backup** de positions.yaml avant modification
3. **📝 Tester** sur table avant compétition
4. **📝 Vérifier** alignement bacs avec positions calibrées

---

## 📊 Checklist de Calibration

### Préparation

- [ ] Bras DOFbot installé et stable
- [ ] Alimentation 12V branchée
- [ ] Câble USB connecté
- [ ] Caméra positionnée correctement
- [ ] 3 bacs alignés (45°, 90°, 135°)
- [ ] Espace de travail dégagé
- [ ] Arm_Lib installé (si mode connecté)

### Calibration

- [ ] Lancer l'outil (console ou web)
- [ ] Configurer IP (si web)
- [ ] Connecter au serveur (si web)
- [ ] Calibrer HOME (position repos)
- [ ] Calibrer OBSERVATION (vue caméra)
- [ ] Calibrer BIN1 (dangereux - 45°)
- [ ] Calibrer BIN2 (ménagers - 90°)
- [ ] Calibrer BIN3 (recyclables - 135°)
- [ ] Sauvegarder positions
- [ ] Tester séquence complète
- [ ] Vérifier aucune collision
- [ ] Refaire si nécessaire

### Validation

- [ ] Tous les mouvements fluides
- [ ] Positions précises
- [ ] Pince fonctionne (ferme/ouvre)
- [ ] Pas de collision
- [ ] Fichier positions.yaml créé
- [ ] Test séquence OK
- [ ] Prêt pour production ! ✅

---

## 🎯 Résumé

| Méthode | Commande | Avantage |
|---------|----------|----------|
| **Console** | `python scripts/calibrate_positions.py` | Simple, rapide, clavier |
| **Web** | Serveur + Interface HTML | Visuel, à distance, intuitif |

**Positions à calibrer :**
- HOME, OBSERVATION, BIN1, BIN2, BIN3

**Fichier créé :**
- `config/positions.yaml`

**Test final :**
- Séquence complète HOME → OBS → BIN1 → BIN2 → BIN3 → HOME

---

## 📚 Voir Aussi

- **[README.md](../README.md)** - Documentation principale
- **[web/README.md](../web/README.md)** - Guide interface web détaillé
- **[docs/guides/DEPLOYMENT.md](DEPLOYMENT.md)** - Guide déploiement
- **[docs/INDEX.md](../INDEX.md)** - Index complet documentation

---

**🤖 Bonne calibration pour TRC 2025 ! 🏆**

*Guide créé le 16 octobre 2025 - Équipe Ucaotech, Cotonou, Bénin 🇧🇯*

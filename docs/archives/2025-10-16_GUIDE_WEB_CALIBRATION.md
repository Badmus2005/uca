# 🌐 Interface Web de Calibration - Guide Rapide

## TRC 2025 - Cotonou, Bénin 🇧🇯

---

## 🚀 Démarrage Rapide

### Étape 1 : Installer les dépendances

```bash
# Installer WebSockets
pip install websockets

# (PyYAML déjà installé)
```

### Étape 2 : Démarrer le serveur

```bash
cd D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025
python scripts\calibration_server.py
```

**Vous devriez voir :**
```
╔══════════════════════════════════════════════════════════╗
║  🌐 SERVEUR CALIBRATION DOFBOT                          ║
║  TRC 2025 - Cotonou, Bénin 🇧🇯                          ║
╚══════════════════════════════════════════════════════════╝

🚀 Serveur WebSocket démarré sur ws://localhost:8765
📊 Mode: 🔄 SIMULATION
📁 Configuration: ../config/positions.yaml

💡 Ouvrez web/calibration_interface.html dans votre navigateur
⌨️  Appuyez sur Ctrl+C pour arrêter
```

### Étape 3 : Ouvrir l'interface web

**Option 1 - Double-clic :**
- Naviguer vers : `D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025\web\`
- Double-cliquer sur `calibration_interface.html`

**Option 2 - Navigateur :**
- Ouvrir votre navigateur (Chrome, Firefox, Edge)
- Taper : `file:///D:/TRC2025/docTel/TRC_Doc/TRC/ucaotech_dofbot_trc2025/web/calibration_interface.html`

### Étape 4 : Connecter au serveur

1. Cliquer sur le bouton **"🔌 Connecter"** en haut à droite
2. Le statut devrait passer de "Mode Simulation" à "Connecté" (point vert)
3. Le log affichera : "✅ Connecté au serveur de calibration"

---

## 🎮 Utilisation de l'Interface

### **Panel Gauche - Contrôle des Joints**

#### Sliders (Curseurs)
- **Joint 1** : Base (rotation horizontale) - 0° à 180°
- **Joint 2** : Shoulder (épaule) - 0° à 180°
- **Joint 3** : Elbow (coude) - 0° à 180°
- **Joint 4** : Wrist (poignet) - 0° à 180°
- **Joint 5** : Roll (rotation poignet) - 0° à 270°
- **Joint 6** : Gripper (pince) - 0° (fermé) à 180° (ouvert)

#### Boutons d'Ajustement Rapide
Chaque joint dispose de 4 boutons :
- **-10°** : Diminuer de 10 degrés
- **-1°** : Diminuer de 1 degré (ajustement fin)
- **+1°** : Augmenter de 1 degré (ajustement fin)
- **+10°** : Augmenter de 10 degrés

#### Boutons Spéciaux (Joint 6 - Pince)
- **Ouvrir** : Met la pince à 30° (position ouverte)
- **Fermer** : Met la pince à 135° (position fermée)

---

### **Panel Droit - Positions & Actions**

#### Positions Prédéfinies
- **🏠 HOME** : Position de repos sécurisée
- **👁️ OBSERVATION** : Position pour capture caméra
- **🗑️ BAC 1 (Dangereux)** : Bac rouge à gauche
- **🗑️ BAC 2 (Ménagers)** : Bac vert au centre
- **🗑️ BAC 3 (Recyclables)** : Bac bleu à droite

#### Boutons d'Action
- **💾 Sauvegarder Position** : Ouvre un modal pour choisir quelle position sauvegarder
- **🧪 Test Séquence** : Exécute automatiquement HOME → OBSERVATION → BAC1 → BAC2 → BAC3 → HOME
- **🔄 Réinitialiser** : Retour à la position HOME par défaut
- **📥 Exporter Config** : Télécharge un fichier JSON avec toutes les positions

#### Visualisation
- **Coordonnées X, Y, Z** : Position estimée de la pince (en mm)
- **Représentation 2D du bras** : Animation en temps réel
- **Panel de log** : Historique des actions et messages

---

## 📝 Workflow de Calibration Visuel

### 1️⃣ Calibrer Position HOME

1. Utiliser les sliders pour ajuster chaque joint
2. Objectif : Position sûre, dégagée, stable
3. Cliquer sur **"💾 Sauvegarder Position"**
4. Choisir **"🏠 HOME"**
5. Vérifier : Cliquer sur **"🏠 HOME"** dans les positions prédéfinies

### 2️⃣ Calibrer Position OBSERVATION

1. Cliquer sur **"👁️ OBSERVATION"** (ira à la position par défaut)
2. Ajuster pour cadrage optimal caméra :
   - Joint 2 (Shoulder) : ~100° pour hauteur
   - Joint 3 (Elbow) : ~80° pour angle
3. Le cube doit être **centré** sous la caméra
4. Sauvegarder : **"💾 Sauvegarder"** → **"👁️ OBSERVATION"**

### 3️⃣ Calibrer BAC 1 (Dangereux)

1. Cliquer sur **"🗑️ BAC 1"** (position initiale)
2. Ajuster manuellement :
   - Joint 1 : ~135° (rotation vers la gauche)
   - Joints 2-4 : Ajuster pour atteindre le bac
   - Joint 6 : 30° (pince ouverte)
3. La pince doit être **au-dessus du centre du bac**
4. Sauvegarder : **"💾 Sauvegarder"** → **"🗑️ BAC 1"**

### 4️⃣ Calibrer BAC 2 et BAC 3

- Répéter le processus pour les bacs 2 (centre, 90°) et 3 (droite, 45°)

### 5️⃣ Test Final

1. Cliquer sur **"🧪 Test Séquence"**
2. Observer le déroulement automatique
3. Vérifier : Pas de collision, positions précises
4. Si nécessaire, ajuster et re-sauvegarder

### 6️⃣ Export

1. Cliquer sur **"📥 Exporter Config"**
2. Un fichier JSON sera téléchargé : `dofbot_calibration_XXXXXX.json`
3. Conserver comme backup

---

## ⌨️ Raccourcis Clavier

| Touche | Action |
|--------|--------|
| `1-6` | Sélectionner joint 1 à 6 (futur) |
| `H` | Aller à HOME |
| `O` | Aller à OBSERVATION |
| `T` | Lancer test de séquence |

---

## 🎨 Avantages de l'Interface Web

### ✅ Par rapport à l'interface console :

1. **Visualisation en temps réel** : Voir le bras bouger
2. **Contrôle précis** : Sliders pour ajustements visuels
3. **Feedback immédiat** : Coordonnées X/Y/Z calculées
4. **Multi-plateforme** : Fonctionne sur PC, tablette, smartphone
5. **Pas de dépendance** : Juste un navigateur web
6. **Log coloré** : Messages success/warning/error distincts
7. **Export facile** : Sauvegarde JSON en un clic

### 🎯 Utilisation recommandée :

- **PC** : Interface web (confort visuel)
- **Jetson Nano sans écran** : Interface console (SSH)
- **Tablette/Smartphone** : Interface web (démo mobile)

---

## 🔧 Dépannage

### ❌ "Échec de connexion au serveur"

**Cause** : Le serveur `calibration_server.py` n'est pas démarré

**Solution** :
```bash
python scripts\calibration_server.py
```

### ❌ "WebSocket connection failed"

**Cause** : Mauvaise URL de connexion

**Solution** : Vérifier que le serveur écoute sur `ws://localhost:8765`

Si sur Jetson Nano, modifier dans `calibration_interface.html` ligne 891 :
```javascript
ws = new WebSocket('ws://192.168.x.x:8765');  // IP du Jetson
```

### ❌ Les mouvements ne se déclenchent pas

**Cause** : Mode simulation actif

**Vérification** : 
- Statut affiche "🔄 Mode Simulation"
- Le bras physique n'est pas connecté
- C'est **normal** : L'interface fonctionne en simulation

**Pour mode réel** : Lancer sur Jetson Nano avec Arm_Lib installé

### ❌ L'interface ne se charge pas

**Cause** : Fichier HTML corrompu ou navigateur incompatible

**Solution** :
- Utiliser Chrome, Firefox ou Edge (version récente)
- Ouvrir la console développeur (F12) pour voir les erreurs
- Vérifier le chemin du fichier

---

## 📊 Comparaison Interfaces

| Critère | Interface Console | Interface Web |
|---------|-------------------|---------------|
| **Visualisation** | ❌ Texte uniquement | ✅ Graphique 2D |
| **Facilité** | ⚠️ Touches spéciales | ✅ Clics souris |
| **Précision** | ✅ Contrôle fin | ✅ Sliders + boutons |
| **Feedback** | ⚠️ Texte | ✅ Visuel + log |
| **Multi-device** | ❌ PC uniquement | ✅ PC/Tablette/Mobile |
| **Installation** | ✅ Aucune | ⚠️ Serveur WebSocket |
| **Utilisation SSH** | ✅ Parfait | ❌ Impossible |
| **Apprentissage** | ⚠️ 5-10 min | ✅ 2 min |

**Recommandation** :
- **Première calibration** : Interface Web (plus intuitive)
- **Ajustements rapides** : Interface Console (plus rapide si SSH)
- **Démonstration** : Interface Web (plus impressionnante)

---

## 🎥 Vidéo Tutoriel (Bientôt)

📹 Une vidéo de démonstration sera créée avant la compétition

**Contenu prévu** :
1. Installation complète
2. Démarrage du serveur
3. Calibration pas à pas
4. Test séquence complète
5. Export et validation

---

## 📞 Support

**Questions ?** Contactez l'équipe Ucaotech

**TRC 2025** - Trophée de Robotique du Cameroun
📍 Cotonou, Bénin 🇧🇯

---

## ✅ Checklist de Démarrage

Avant d'utiliser l'interface web :

- [ ] Python 3.x installé
- [ ] `pip install websockets` exécuté
- [ ] Serveur `calibration_server.py` démarré
- [ ] Fichier `calibration_interface.html` ouvert dans le navigateur
- [ ] Bouton "🔌 Connecter" cliqué
- [ ] Statut affiche "Connecté" avec point vert
- [ ] Log affiche "✅ Connecté au serveur de calibration"

**Prêt à calibrer ! 🎉**

---

*Document créé le 16 octobre 2025*
*Équipe Ucaotech - TRC 2025 Cotonou*

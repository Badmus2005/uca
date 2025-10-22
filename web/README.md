# 🌐 Interface Web de Calibration - Guide Complet

**Ucaotech DOFbot TRC2025 - Cotonou, Bénin 🇧🇯**

---

## 📋 Vue d'Ensemble

Interface web pour calibrer le bras DOFbot à distance avec :
- ✅ Contrôle par sliders (6 joints)
- ✅ Visualisation 2D en temps réel
- ✅ Configuration IP dans l'interface
- ✅ Positions prédéfinies (HOME, OBS, Bins)
- ✅ Logs colorés
- ✅ Test de séquence

---

## 🚀 Démarrage Rapide

### Étape 1 : Démarrer le Serveur

**Sur le Jetson Nano :**
```bash
cd ~/ucaotech_dofbot_trc2025/scripts
python3 calibration_server.py
```

**Sortie :**
```
🚀 Serveur WebSocket démarré sur ws://0.0.0.0:8765
📊 Mode: 🔄 SIMULATION
💡 Ouvrez web/calibration_interface.html dans votre navigateur
```

### Étape 2 : Ouvrir l'Interface

**lancez le serveur local si vous etes sur votre ordinateur :**
```
cd D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025\web; python -m http.server 8080
```

### Étape 3 : Configurer l'IP

1. **Cliquer sur ⚙️ Configuration IP** (en haut)
2. **Entrer l'IP du Jetson :**
   - Si sur le Jetson : `localhost`
   - Si sur un autre PC : `192.168.1.100` (exemple)
3. **Port :** `8765`
4. **Cliquer 💾 Sauvegarder**

### Étape 4 : Connecter

1. **Cliquer sur 🔌 Connecter**
2. **Vérifier les logs :**
   ```
   ✅ Connexion établie!
   🔄 Mode simulation actif
   ```

### Étape 5 : Calibrer !

- **Bouger les sliders** pour ajuster les joints
- **Cliquer sur les boutons** (HOME, OBS, Bins)
- **Sauvegarder** les positions

---

## 🎮 Utilisation

### Contrôles des Joints

**Sliders (1-6) :**
- Joint 1 (Base) : 0-180°
- Joint 2 (Shoulder) : 0-180°
- Joint 3 (Elbow) : 0-180°
- Joint 4 (Wrist) : 0-180°
- Joint 5 (Roll) : 0-270°
- Joint 6 (Gripper) : 0-180°

**Bouger un slider :**
- Déplace le bras en temps réel
- Affiche l'angle (ex: `105°`)
- Log : `🎮 Joint 2 ajusté à 105°`

### Positions Prédéfinies

**Boutons :**
- **🏠 HOME** : Position repos (90,90,90,90,90,30)
- **👁️ OBSERVATION** : Au-dessus caméra
- **🗑️ Bin1** : Bac dangereux (45°)
- **🗑️ Bin2** : Bac ménagers (90°)
- **🗑️ Bin3** : Bac recyclables (135°)

**Cliquer → Tous les sliders se mettent à jour !**

### Sauvegarder une Position

1. **Ajuster les joints**
2. **Cliquer 💾 Sauvegarder Position**
3. **Entrer le nom** (ex: `home`)
4. **Cliquer Sauvegarder**
5. **Fichier config/positions.yaml mis à jour**

### Tester la Séquence

**Cliquer 🧪 Tester Séquence**

Exécute : HOME → OBS → BIN1 → BIN2 → BIN3 → HOME

### Visualisation 2D

**Affiche le bras en temps réel :**
- Segments colorés
- Joints (cercles)
- Pince
- Se met à jour automatiquement

### Logs

**Panel en bas :**
- ✅ Vert : Succès
- ⚠️ Jaune : Avertissement
- ❌ Rouge : Erreur
- ℹ️ Bleu : Info

---

## ⚙️ Configuration IP

### Pourquoi Configurer ?

Pour contrôler le bras depuis **un autre PC** que le Jetson Nano.

### Comment Trouver l'IP du Jetson ?

**Sur le Jetson Nano :**
```bash
hostname -I
# Exemple : 192.168.1.100
```

### Configuration dans l'Interface

**1. Cliquer ⚙️ Configuration IP**

**2. Remplir :**
```
Adresse IP : 192.168.1.100
Port : 8765
```

**3. Cliquer 💾 Sauvegarder et Reconnecter**

**4. La config est sauvegardée dans le navigateur (localStorage)**

### Exemples

| Situation | IP | Port |
|-----------|-----|------|
| Interface sur le Jetson | `localhost` | 8765 |
| Interface sur un PC du réseau | `192.168.1.100` | 8765 |
| Jetson en point d'accès WiFi | `10.42.0.1` | 8765 |

---

## 🛠️ Dépannage

### Connexion Échoue

**Problème :** `❌ Erreur de connexion`

**Solutions :**

1. **Vérifier le serveur :**
   ```bash
   # Sur le Jetson
   ps aux | grep calibration_server
   ```

2. **Vérifier l'IP :**
   ```bash
   ping 192.168.1.100
   ```

3. **Vérifier le port :**
   ```bash
   netstat -tuln | grep 8765
   ```

4. **Pare-feu :**
   ```bash
   sudo ufw allow 8765
   ```

### Connexion se Ferme Immédiatement

**Problème :** `✅ Connexion établie!` puis `⚠️ Connexion fermée`

**Cause :** Version incompatible websockets.

**Solution :**
```bash
pip install --upgrade websockets
# Redémarrer le serveur
```

### Sliders ne Bougent pas le Bras

**Vérifications :**
- ✅ Statut **🟢 Connecté** ?
- ✅ Logs affichent les commandes ?
- ✅ Bras allumé ?

**Test :**
```python
# Sur le Jetson
from Arm_Lib import Arm_Device
arm = Arm_Device()
arm.Arm_serial_set_torque(1)
arm.Arm_serial_servo_write(1, 90, 1000)
```

### Configuration IP ne se Sauvegarde pas

**Cause :** Mode navigation privée ou localStorage désactivé.

**Solutions :**
1. Désactiver mode privé
2. Tester avec `localStorage.getItem('serverIP')` (F12 > Console)
3. Changer de navigateur

---

## 📊 Historique des Corrections

### 16 Oct 2025 - v1.0

**Problème résolu :** Connexion se fermait immédiatement

**Cause :** 
```python
# Ancienne signature (incompatible websockets 15.0.1)
async def handle_client(self, websocket, path):

# Nouvelle signature
async def handle_client(self, websocket):
```

**Correctifs appliqués :**
1. ✅ Signature `handle_client()` mise à jour
2. ✅ Gestion complète des messages (`log`, `status`, `positions`)
3. ✅ Demande automatique statut à la connexion
4. ✅ Synchronisation sliders avec serveur
5. ✅ Configuration IP dans l'interface
6. ✅ Sauvegarde localStorage

**Résultat :** Connexion stable, système opérationnel ✅

---

## 🎯 Checklist d'Utilisation

### Préparation
- [ ] Serveur démarré sur Jetson
- [ ] IP du Jetson connue
- [ ] Interface ouverte dans navigateur
- [ ] Configuration IP faite
- [ ] Connexion établie (🟢 vert)

### Calibration
- [ ] Tester chaque slider
- [ ] Calibrer HOME
- [ ] Calibrer OBSERVATION
- [ ] Calibrer BIN1
- [ ] Calibrer BIN2
- [ ] Calibrer BIN3
- [ ] Sauvegarder toutes les positions
- [ ] Tester séquence complète
- [ ] Vérifier aucune collision

### Validation
- [ ] Tous les mouvements fluides
- [ ] Positions correctes
- [ ] Pince fonctionne
- [ ] Séquence OK
- [ ] Prêt pour production ✅

---

## 📚 Voir Aussi

- **[README.md](../README.md)** - Documentation principale
- **[docs/guides/CALIBRATION.md](../docs/guides/CALIBRATION.md)** - Guide calibration complet
- **[QUICKSTART.md](../QUICKSTART.md)** - Démarrage rapide
- **[docs/INDEX.md](../docs/INDEX.md)** - Index documentation

---

## 🔗 Fichiers Importants

| Fichier | Rôle |
|---------|------|
| `calibration_interface.html` | Interface graphique |
| `config.js` | Configuration (optionnel) |
| `../scripts/calibration_server.py` | Serveur WebSocket |
| `../config/positions.yaml` | Positions sauvegardées |

---

**🤖 Interface prête pour TRC 2025 ! 🏆**

*Guide créé le 16 octobre 2025 - Équipe Ucaotech*

---

## 📞 Support

**En cas de problème :**
1. Consulter la section [Dépannage](#-dépannage)
2. Vérifier les logs du serveur
3. Vérifier les logs de l'interface (F12)
4. Consulter le guide complet : [docs/guides/CALIBRATION.md](../docs/guides/CALIBRATION.md)

# ⚙️ Configuration IP Directement dans l'Interface

## 🎉 Nouvelle Fonctionnalité Ajoutée !

Vous pouvez maintenant **configurer l'adresse IP du serveur directement dans l'interface web**, sans avoir besoin de modifier aucun fichier de code !

---

## 📋 Changements Apportés

### 1. **Bouton Configuration dans la Barre de Statut**
   - Un nouveau bouton **⚙️ Configuration IP** a été ajouté à côté du bouton "Connecter"
   - Cliquez dessus pour ouvrir le modal de configuration

### 2. **Modal de Configuration**
   - Interface conviviale pour saisir:
     - 🌐 **Adresse IP** du Jetson Nano (ou `localhost`)
     - 🔌 **Port** du serveur WebSocket (par défaut 8765)
   - Affiche l'adresse de connexion actuelle
   - Exemples d'utilisation intégrés

### 3. **Sauvegarde Automatique**
   - La configuration est **sauvegardée dans le navigateur** (localStorage)
   - Persistante même après fermeture/réouverture du navigateur
   - Plus besoin de modifier `config.js` !

### 4. **Validation Intelligente**
   - Validation automatique du format IP (IPv4)
   - Vérification de la validité du port (1-65535)
   - Messages d'erreur clairs en cas de saisie incorrecte

### 5. **Reconnexion Automatique**
   - Si vous êtes déjà connecté, l'interface se reconnecte automatiquement avec la nouvelle IP
   - Sinon, un message vous invite à cliquer sur "Connecter"

---

## 🚀 Comment Utiliser

### **Première Utilisation**

1. **Ouvrez l'interface web**
   ```
   D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025\web\calibration_interface.html
   ```
   
   Double-cliquez sur le fichier pour l'ouvrir dans votre navigateur.

2. **Configurez l'adresse IP**
   - Cliquez sur le bouton **⚙️ Configuration IP**
   - Le modal s'ouvre avec un formulaire

3. **Entrez les informations**
   
   **Si vous êtes SUR le Jetson Nano:**
   ```
   Adresse IP: localhost
   Port: 8765
   ```
   
   **Si vous êtes sur un AUTRE PC du réseau:**
   ```
   Adresse IP: 192.168.1.100  (IP du Jetson Nano)
   Port: 8765
   ```
   
   Pour trouver l'IP du Jetson Nano:
   ```bash
   hostname -I
   ```

4. **Sauvegardez**
   - Cliquez sur **💾 Sauvegarder et Reconnecter**
   - La configuration est automatiquement enregistrée
   - L'interface tente de se connecter

5. **Testez la connexion**
   - Si pas encore connecté, cliquez sur **🔌 Connecter**
   - Le statut devient vert si la connexion réussit
   - Les logs affichent les détails de connexion

---

## 🔍 Exemples de Configuration

### **Configuration Locale (sur le Jetson Nano)**
```
🌐 Adresse IP: localhost
🔌 Port: 8765
```
**Résultat:** `ws://localhost:8765`

---

### **Configuration Réseau (depuis un autre PC)**

**Trouver l'IP du Jetson Nano:**
```bash
# Sur le Jetson Nano, exécutez:
hostname -I
# Exemple de sortie: 192.168.1.100
```

**Dans l'interface:**
```
🌐 Adresse IP: 192.168.1.100
🔌 Port: 8765
```
**Résultat:** `ws://192.168.1.100:8765`

---

### **Configuration avec IP statique**
```
🌐 Adresse IP: 10.42.0.50
🔌 Port: 8765
```
**Résultat:** `ws://10.42.0.50:8765`

---

## ⚠️ Validation et Erreurs

### **Adresses IP Valides**
- ✅ `localhost`
- ✅ `192.168.1.100`
- ✅ `10.0.0.5`
- ✅ `172.16.254.1`

### **Adresses IP Invalides**
- ❌ `192.168.1` (incomplète)
- ❌ `300.168.1.100` (octet > 255)
- ❌ `jetson-nano` (nom d'hôte non supporté, utilisez IP)
- ❌ `192.168.1.100:8765` (le port doit être saisi séparément)

### **Ports Valides**
- ✅ `8765` (port par défaut)
- ✅ `8080`, `3000`, `9000` (ports courants)
- ❌ `99999` (> 65535)
- ❌ `abc` (non numérique)

---

## 🛠️ Résolution de Problèmes

### **Le modal ne s'ouvre pas**
- Vérifiez que vous avez bien cliqué sur **⚙️ Configuration IP**
- Rafraîchissez la page (F5)
- Vérifiez la console du navigateur (F12)

### **"Adresse IP invalide"**
- Vérifiez le format: `192.168.1.100` (4 nombres séparés par des points)
- Chaque nombre doit être entre 0 et 255
- Ou utilisez simplement `localhost`

### **La connexion échoue après sauvegarde**
1. **Vérifiez que le serveur est démarré:**
   ```powershell
   cd D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025\scripts
   python calibration_server.py
   ```

2. **Vérifiez l'IP du Jetson Nano:**
   ```bash
   hostname -I
   ```

3. **Testez la connexion réseau:**
   ```powershell
   ping 192.168.1.100
   ```

4. **Vérifiez le pare-feu:**
   - Le port 8765 doit être ouvert sur le Jetson Nano

### **La configuration ne se sauvegarde pas**
- Vérifiez que le navigateur supporte localStorage
- Ne naviguez pas en mode privé/incognito
- Essayez un autre navigateur (Chrome, Firefox, Edge)

### **Anciennes valeurs apparaissent**
- Cliquez sur **⚙️ Configuration IP** pour voir/modifier
- Ou effacez le localStorage du navigateur:
  - F12 → Console → `localStorage.clear()`
  - Rafraîchissez la page

---

## 📊 Avantages de cette Méthode

### **Avant (avec config.js)**
```javascript
// Il fallait modifier ce fichier à chaque changement d'IP
const SERVER_IP = '192.168.1.100';
const SERVER_PORT = 8765;
```
❌ Modification de code nécessaire  
❌ Risque d'erreur de syntaxe  
❌ Nécessite de connaître JavaScript  

### **Maintenant (interface graphique)**
```
1. Clic sur ⚙️ Configuration IP
2. Saisie visuelle des valeurs
3. Sauvegarde automatique
```
✅ Pas de code à modifier  
✅ Interface intuitive  
✅ Validation automatique  
✅ Persistance dans le navigateur  

---

## 🔄 Workflow Complet

```
┌─────────────────────────────────────┐
│  1. Ouvrir calibration_interface.html│
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  2. Cliquer sur ⚙️ Configuration IP  │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  3. Saisir IP et Port                │
│     - localhost (si sur Jetson)      │
│     - 192.168.x.x (si sur autre PC)  │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  4. Cliquer "Sauvegarder"            │
│     ✅ Validation automatique         │
│     💾 Sauvegarde dans localStorage   │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  5. Reconnexion automatique          │
│     (si déjà connecté)               │
│     OU                               │
│     Cliquer "🔌 Connecter"           │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  ✅ Connexion établie!                │
│  🟢 Statut: Connecté                 │
│  🎮 Prêt pour le calibrage           │
└─────────────────────────────────────┘
```

---

## 💡 Conseils Pro

1. **Configuration unique par navigateur**
   - Chaque navigateur garde sa propre configuration
   - Chrome et Firefox ont des localStorage séparés

2. **Test rapide de l'IP**
   - Avant de configurer, testez avec `ping <IP>`
   - Assurez-vous que le Jetson Nano est accessible

3. **Mode hors-ligne**
   - L'interface fonctionne toujours en mode simulation
   - Pratique pour tester l'interface sans matériel

4. **Changement d'IP rapide**
   - Rouvrez le modal à tout moment
   - Changez l'IP sans recharger la page

5. **Export de configuration**
   - Une fois configuré, vous pouvez bookmarker l'URL
   - La config est préservée au prochain accès

---

## 📝 Fichiers Modifiés

### **calibration_interface.html**
- **Ligne ~470**: Ajout du CSS pour le modal et le bouton
- **Ligne ~498**: Ajout du bouton ⚙️ Configuration IP dans la barre de statut
- **Ligne ~1072**: Modification de `connectToRobot()` pour utiliser localStorage
- **Ligne ~958**: Ajout de toutes les fonctions de gestion du modal:
  - `openIPModal()`
  - `closeIPModal()`
  - `validateIP()`
  - `saveIPConfiguration()`
  - Event listener au chargement de la page
- **Ligne ~1180**: Ajout du HTML du modal avant `</body>`

**Total: ~100 lignes de code ajoutées**

---

## 🎯 Récapitulatif

| Fonctionnalité | Statut |
|----------------|--------|
| Bouton Configuration IP | ✅ Ajouté |
| Modal de saisie | ✅ Créé |
| Validation IP/Port | ✅ Implémenté |
| Sauvegarde localStorage | ✅ Fonctionnel |
| Reconnexion automatique | ✅ Actif |
| Affichage IP actuelle | ✅ Intégré |
| Exemples d'utilisation | ✅ Inclus |
| Messages d'erreur clairs | ✅ Implémentés |

---

## 🎉 Prêt pour TRC 2025 Cotonou !

Avec cette interface améliorée, vous pouvez:
- ✅ Configurer facilement l'IP depuis n'importe quel PC
- ✅ Pas besoin de modifier le code
- ✅ Interface utilisateur intuitive
- ✅ Prêt pour la compétition à **Cotonou, Bénin** 🇧🇯

**Bonne chance pour la compétition ! 🤖🏆**

---

## 📞 Aide Supplémentaire

Si vous avez des questions ou rencontrez des problèmes:
1. Consultez les logs dans l'interface (panneau en bas)
2. Ouvrez la console du navigateur (F12)
3. Vérifiez que le serveur est bien démarré
4. Testez d'abord avec `localhost` avant de passer au réseau

---

*Document créé le 2025 - TRC 2025 Cotonou, Bénin 🇧🇯*

# 🌐 CONFIGURATION ACCÈS DISTANT - Interface Web de Calibration

## TRC 2025 - Cotonou, Bénin 🇧🇯

---

## ✅ OUI, vous pouvez contrôler le bras depuis un autre PC !

L'interface web peut être utilisée depuis **n'importe quel appareil** sur le même réseau :
- 💻 **PC Windows/Mac/Linux**
- 📱 **Tablette** (Android/iOS)
- 📱 **Smartphone** (Android/iOS)
- 🖥️ **Autre ordinateur portable**

---

## 🔧 CONFIGURATION EN 3 ÉTAPES

### **Étape 1️⃣ : Trouver l'adresse IP du Jetson Nano**

#### Sur le Jetson Nano (bras robotique) :

```bash
# Commande 1 : ifconfig
ifconfig

# OU Commande 2 : hostname
hostname -I
```

**Vous verrez quelque chose comme :**
```
wlan0: 192.168.1.100    # ← Notez cette adresse !
```

**Exemples d'adresses IP courantes :**
- `192.168.1.x` (réseau domestique)
- `192.168.0.x` (réseau domestique alternatif)
- `10.0.0.x` (réseau d'entreprise)
- `172.16.x.x` (réseau d'entreprise)

---

### **Étape 2️⃣ : Modifier le serveur (déjà fait !)**

✅ **DÉJÀ MODIFIÉ !** Le serveur écoute maintenant sur `0.0.0.0` au lieu de `localhost`.

**Vérification :**
Ouvrir `scripts/calibration_server.py` ligne 281, vous devriez voir :
```python
async def start_server(self, host='0.0.0.0', port=8765):
```

✅ Si c'est `'0.0.0.0'` → Parfait !  
❌ Si c'est `'localhost'` → Changez-le en `'0.0.0.0'`

---

### **Étape 3️⃣ : Modifier l'interface web**

#### Option A : Modification manuelle (Recommandé)

**Ouvrir :** `web/calibration_interface.html`

**Chercher ligne 1004 :**
```javascript
ws = new WebSocket('ws://localhost:8765');
```

**Remplacer par :**
```javascript
ws = new WebSocket('ws://192.168.1.100:8765');  // ← Votre IP Jetson
```

**Exemple complet :**
```javascript
function connectToRobot() {
    addLog('🔌 Tentative de connexion au serveur...', 'warning');

    try {
        // MODIFIER CETTE LIGNE AVEC L'IP DU JETSON
        ws = new WebSocket('ws://192.168.1.100:8765');  // ← Votre IP ici

        ws.onopen = function () {
            isConnected = true;
            // ... reste du code
        };
```

#### Option B : Saisie dynamique (Plus flexible)

Je vais créer une version avec **saisie d'IP** dans l'interface elle-même !

---

## 🚀 UTILISATION APRÈS CONFIGURATION

### **Sur le Jetson Nano (bras) :**

1. Démarrer le serveur :
```bash
cd ~/ucaotech_dofbot_trc2025
python3 scripts/calibration_server.py
```

2. Vérifier le message :
```
🚀 Serveur WebSocket démarré sur ws://0.0.0.0:8765
📊 Mode: ✅ CONNECTÉ
```

3. **Noter l'IP du Jetson** :
```bash
hostname -I
# Exemple : 192.168.1.100
```

### **Sur votre PC (ou tablette) :**

1. Ouvrir le fichier `calibration_interface.html`
2. Cliquer sur "🔌 Connecter"
3. Le log devrait afficher :
   - `🔌 Tentative de connexion...`
   - `✅ Connexion établie!`

---

## 🔥 VERSION DYNAMIQUE (avec saisie IP)

Je vais créer une version améliorée où vous pouvez **saisir l'IP directement** dans l'interface !

Voulez-vous que je crée cette version ? Elle ajoutera :
- ✅ Un champ de saisie pour l'IP
- ✅ Sauvegarde de l'IP dans le navigateur (localStorage)
- ✅ Bouton "🔌 Connecter" qui demande l'IP si non définie
- ✅ Plus besoin de modifier le code !

---

## 🔒 SÉCURITÉ & PARE-FEU

### **Sur le Jetson Nano :**

Si la connexion échoue, vérifier le pare-feu :

```bash
# Autoriser le port 8765
sudo ufw allow 8765/tcp

# OU désactiver le pare-feu temporairement (test)
sudo ufw disable
```

### **Sur votre PC Windows :**

Si le pare-feu Windows bloque :
1. Ouvrir "Pare-feu Windows Defender"
2. Autoriser les connexions sortantes sur le port 8765

---

## 📡 TEST DE CONNEXION

### **Test 1 : Ping**

Depuis votre PC, vérifier que le Jetson est accessible :

```bash
# Windows PowerShell
ping 192.168.1.100

# Résultat attendu :
# Réponse de 192.168.1.100 : octets=32 temps=5ms TTL=64
```

### **Test 2 : Telnet (optionnel)**

```bash
# Windows PowerShell
Test-NetConnection -ComputerName 192.168.1.100 -Port 8765

# OU
telnet 192.168.1.100 8765
```

---

## 🎯 EXEMPLES DE CONFIGURATION

### **Réseau domestique typique :**

```
Router Wi-Fi
  │
  ├─ Jetson Nano : 192.168.1.100
  └─ Votre PC    : 192.168.1.50

Configuration :
  Serveur Jetson : 0.0.0.0:8765
  Interface PC   : ws://192.168.1.100:8765
```

### **Hotspot smartphone :**

```
Smartphone (Hotspot)
  │
  ├─ Jetson Nano : 192.168.43.100
  └─ Votre PC    : 192.168.43.50

Configuration :
  Serveur Jetson : 0.0.0.0:8765
  Interface PC   : ws://192.168.43.100:8765
```

### **Réseau Ethernet direct :**

```
PC ←──[Câble Ethernet]──→ Jetson Nano

Jetson : 192.168.0.1
PC     : 192.168.0.2

Configuration :
  Serveur Jetson : 0.0.0.0:8765
  Interface PC   : ws://192.168.0.1:8765
```

---

## 🐛 DÉPANNAGE

### ❌ "WebSocket connection failed"

**Causes possibles :**
1. IP incorrecte
2. Serveur non démarré sur le Jetson
3. Pare-feu bloque le port 8765
4. Jetson et PC sur des réseaux différents

**Solutions :**
```bash
# 1. Vérifier IP du Jetson
hostname -I

# 2. Vérifier que le serveur tourne
ps aux | grep calibration_server

# 3. Tester le ping
ping 192.168.1.100

# 4. Autoriser le port
sudo ufw allow 8765/tcp
```

### ❌ "Connection refused"

**Cause :** Le serveur n'écoute que sur localhost

**Solution :** Vérifier que `host='0.0.0.0'` dans `calibration_server.py`

### ❌ "Connection timeout"

**Cause :** Réseaux différents (PC en Wi-Fi, Jetson en Ethernet)

**Solution :** Mettre les deux sur le même réseau

---

## 📱 UTILISATION SUR TABLETTE/SMARTPHONE

### **Avantages :**
- ✅ Mobilité totale
- ✅ Contrôle tactile intuitif
- ✅ Démonstration impressionnante

### **Configuration :**

1. **Sur le Jetson :** Démarrer le serveur
2. **Sur la tablette :** Ouvrir Chrome/Safari
3. **Taper l'adresse :**
   ```
   file:///sdcard/Download/calibration_interface.html
   ```
   OU utiliser un serveur HTTP simple :
   ```bash
   # Sur le Jetson
   cd ~/ucaotech_dofbot_trc2025/web
   python3 -m http.server 8080
   
   # Sur la tablette, aller à :
   # http://192.168.1.100:8080/calibration_interface.html
   ```

---

## 🎓 RECOMMANDATIONS

### **Pour la compétition TRC 2025 :**

1. **Setup principal :**
   - Jetson Nano connecté au bras
   - Tablette pour contrôle visuel
   - PC de secours si problème

2. **Configuration réseau :**
   - Créer un hotspot Wi-Fi dédié
   - IP fixes pour éviter les changements
   - Tester la veille de la compétition

3. **Backup :**
   - Interface console toujours disponible (SSH)
   - Fichier HTML sur USB
   - Documentation imprimée

---

## ✅ CHECKLIST DE CONFIGURATION

Avant la compétition, vérifier :

- [ ] Serveur modifié : `host='0.0.0.0'`
- [ ] IP du Jetson connue : `_______________`
- [ ] Interface HTML modifiée avec l'IP
- [ ] Test de connexion PC → Jetson réussi
- [ ] Pare-feu configuré (port 8765 ouvert)
- [ ] Test de contrôle du bras depuis PC
- [ ] Test sur tablette (optionnel)
- [ ] Documentation imprimée disponible

---

## 🔥 BONUS : VERSION AVEC SAISIE DYNAMIQUE

Voulez-vous que je crée une version améliorée où vous pouvez **changer l'IP directement dans l'interface** sans modifier le code ?

**Fonctionnalités :**
- 🔧 Champ de saisie IP dans l'interface
- 💾 Sauvegarde automatique de l'IP
- 🔄 Bouton "Changer serveur"
- ✅ Plus besoin de modifier le HTML !

---

## 📞 SUPPORT

**Questions ?** Consultez :
- `docs/GUIDE_WEB_CALIBRATION.md` - Guide principal
- `docs/SYNTHESE_COMPLETE.md` - Vue d'ensemble
- Ce document - Configuration réseau

---

**TRC 2025 - Équipe Ucaotech**  
**Cotonou, Bénin 🇧🇯**

*Document créé le 16 octobre 2025*

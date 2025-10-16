# 🌐 GUIDE RAPIDE - Configuration Réseau

## ✅ Comment utiliser l'interface depuis un autre PC

---

## 📋 RÉSUMÉ EN 3 ÉTAPES

### 1️⃣ **Sur le Jetson Nano** (bras robotique)

Trouver son adresse IP :
```bash
hostname -I
```

Résultat exemple : `192.168.1.100`

### 2️⃣ **Sur votre PC** (interface web)

Ouvrir le fichier `web/config.js` et modifier :

```javascript
const SERVER_IP = '192.168.1.100';  // ← Mettre l'IP du Jetson ici
```

**C'est tout !** Le reste est automatique.

### 3️⃣ **Démarrer et connecter**

**Sur le Jetson :**
```bash
python3 scripts/calibration_server.py
```

**Sur votre PC :**
- Ouvrir `web/calibration_interface.html`
- Cliquer sur "🔌 Connecter"
- Contrôler le bras à distance ! 🎮

---

## 🎯 EXEMPLES DE CONFIGURATION

### Configuration 1 : Utilisation locale (défaut)
```javascript
// Dans config.js
const SERVER_IP = 'localhost';
```
→ PC et serveur sur la **même machine**

### Configuration 2 : Réseau Wi-Fi domestique
```javascript
// Dans config.js
const SERVER_IP = '192.168.1.100';
```
→ Jetson et PC sur le **même réseau Wi-Fi**

### Configuration 3 : Hotspot smartphone
```javascript
// Dans config.js
const SERVER_IP = '192.168.43.100';
```
→ Créer un hotspot depuis votre téléphone

### Configuration 4 : Connexion Ethernet directe
```javascript
// Dans config.js
const SERVER_IP = '192.168.0.1';
```
→ Câble Ethernet entre Jetson et PC

---

## 🔧 MODIFICATION DÉTAILLÉE

### Fichier : `web/config.js`

**AVANT (local uniquement) :**
```javascript
const SERVER_IP = 'localhost';
const SERVER_PORT = 8765;
```

**APRÈS (accès distant) :**
```javascript
const SERVER_IP = '192.168.1.100';  // ← IP du Jetson
const SERVER_PORT = 8765;            // ← Ne pas changer
```

**Enregistrer** le fichier, puis **rafraîchir** la page web (F5).

---

## ✅ VÉRIFICATION

### Test 1 : Ping depuis votre PC
```bash
ping 192.168.1.100
```

✅ **OK** : `Réponse de 192.168.1.100 : octets=32 temps=5ms`  
❌ **KO** : `Délai d'attente de la demande dépassé`

### Test 2 : Connexion WebSocket

Ouvrir l'interface web :
1. Le log doit afficher : `🔌 Tentative de connexion à ws://192.168.1.100:8765...`
2. Puis : `✅ Connexion établie!`

---

## 🐛 DÉPANNAGE RAPIDE

### ❌ "WebSocket connection failed"

**Solution 1 :** Vérifier l'IP
```bash
# Sur le Jetson
hostname -I
```

**Solution 2 :** Vérifier que le serveur tourne
```bash
# Sur le Jetson
ps aux | grep calibration_server
```

**Solution 3 :** Autoriser le port dans le pare-feu
```bash
# Sur le Jetson
sudo ufw allow 8765/tcp
```

### ❌ "Connection timeout"

**Cause :** Jetson et PC sur des réseaux différents

**Solution :** Mettre les deux sur le **même réseau** (Wi-Fi ou Ethernet)

### ❌ "Connection refused"

**Cause :** Le serveur n'écoute que sur localhost

**Solution :** Vérifier que `calibration_server.py` a bien `host='0.0.0.0'`

---

## 📱 UTILISATION SUR TABLETTE/SMARTPHONE

### Option 1 : Fichier local

1. Copier `calibration_interface.html` et `config.js` sur la tablette
2. Ouvrir avec Chrome/Safari
3. Cliquer sur "🔌 Connecter"

### Option 2 : Serveur HTTP (recommandé)

**Sur le Jetson :**
```bash
cd ~/ucaotech_dofbot_trc2025/web
python3 -m http.server 8080
```

**Sur la tablette :**
- Ouvrir le navigateur
- Aller à : `http://192.168.1.100:8080/calibration_interface.html`
- Contrôler le bras ! 🎮

---

## 🎓 CONFIGURATION POUR LA COMPÉTITION

### Setup recommandé :

```
┌─────────────────┐
│  ROUTEUR Wi-Fi  │ (hotspot dédié)
│   (fixe IP)     │
└────────┬────────┘
         │
    ┌────┴────┐
    │         │
┌───▼────┐ ┌─▼──────┐
│ Jetson │ │ Tablette│
│  Nano  │ │ Contrôle│
└────────┘ └─────────┘
192.168.1.100  192.168.1.50
```

**Avantages :**
- ✅ Réseau isolé (pas de conflits)
- ✅ IP fixes (pas de changement)
- ✅ Contrôle mobile avec la tablette
- ✅ Backup avec PC en cas de problème

---

## 🔒 SÉCURITÉ

### ⚠️ ATTENTION

- Le serveur WebSocket n'a **pas d'authentification**
- N'importe qui sur le réseau peut se connecter
- Pour la compétition : utiliser un réseau **fermé** (hotspot dédié)

---

## ✅ CHECKLIST

Avant la compétition :

- [ ] IP du Jetson connue : `_______________`
- [ ] Fichier `config.js` modifié avec l'IP
- [ ] Serveur `calibration_server.py` modifié (`host='0.0.0.0'`)
- [ ] Test ping PC → Jetson : OK
- [ ] Test connexion WebSocket : OK
- [ ] Test contrôle du bras : OK
- [ ] Test sur tablette (optionnel) : OK

---

## 📞 POUR ALLER PLUS LOIN

**Guides détaillés :**
- `docs/CONFIGURATION_RESEAU.md` - Guide complet réseau
- `docs/GUIDE_WEB_CALIBRATION.md` - Guide interface web
- `docs/SYNTHESE_COMPLETE.md` - Vue d'ensemble

---

**TRC 2025 - Équipe Ucaotech**  
**Cotonou, Bénin 🇧🇯**

*Contrôlez votre bras de n'importe où ! 🌐🤖*

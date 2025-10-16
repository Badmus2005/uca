# 🧪 Test de la Configuration IP

## Ouvrez l'interface maintenant !

Double-cliquez sur ce fichier pour l'ouvrir:
```
D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025\web\calibration_interface.html
```

## ✅ Checklist de Test

### 1. **Ouverture du Modal**
- [ ] Ouvrir calibration_interface.html dans le navigateur
- [ ] Vérifier la présence du bouton **⚙️ Configuration IP** dans la barre de statut
- [ ] Cliquer sur le bouton
- [ ] Le modal doit s'ouvrir avec:
  - Un champ pour l'IP
  - Un champ pour le port (8765 par défaut)
  - Des exemples d'utilisation
  - L'affichage de la connexion actuelle
  - Deux boutons: "Sauvegarder" et "Annuler"

### 2. **Validation des Entrées**

**Test IP Valide:**
- [ ] Entrer `localhost` → Doit être accepté ✅
- [ ] Entrer `192.168.1.100` → Doit être accepté ✅
- [ ] Entrer `10.0.0.5` → Doit être accepté ✅

**Test IP Invalide:**
- [ ] Entrer `192.168.1` → Doit afficher erreur ❌
- [ ] Entrer `300.168.1.1` → Doit afficher erreur ❌
- [ ] Entrer `abc.def.ghi.jkl` → Doit afficher erreur ❌

**Test Port Valide:**
- [ ] Entrer `8765` → Doit être accepté ✅
- [ ] Entrer `3000` → Doit être accepté ✅

**Test Port Invalide:**
- [ ] Entrer `99999` → Doit afficher erreur ❌
- [ ] Entrer `-1` → Doit afficher erreur ❌
- [ ] Entrer `abc` → Doit afficher erreur ❌

### 3. **Sauvegarde dans localStorage**
- [ ] Configurer avec `localhost` et port `8765`
- [ ] Cliquer "Sauvegarder"
- [ ] Vérifier le message de succès dans les logs
- [ ] Fermer le navigateur complètement
- [ ] Rouvrir calibration_interface.html
- [ ] Ouvrir le modal → Les valeurs doivent être préservées ✅

### 4. **Test de la Console Navigateur**
- [ ] Appuyer sur F12 pour ouvrir la console
- [ ] Taper: `localStorage.getItem('serverIP')`
- [ ] Doit afficher: `"localhost"` (ou votre IP configurée)
- [ ] Taper: `localStorage.getItem('serverPort')`
- [ ] Doit afficher: `"8765"` (ou votre port configuré)

### 5. **Changement d'IP**
- [ ] Ouvrir le modal
- [ ] Changer l'IP de `localhost` à `192.168.1.100`
- [ ] Sauvegarder
- [ ] Vérifier que le message affiche la nouvelle IP
- [ ] Rouvrir le modal → Doit afficher `192.168.1.100` ✅

### 6. **Bouton Annuler**
- [ ] Ouvrir le modal
- [ ] Modifier l'IP
- [ ] Cliquer "Annuler"
- [ ] Rouvrir le modal → Les anciennes valeurs doivent être conservées ✅

### 7. **Reconnexion Automatique**

**Si le serveur est démarré:**
```powershell
cd D:\TRC2025\docTel\TRC_Doc\TRC\ucaotech_dofbot_trc2025\scripts
python calibration_server.py
```

- [ ] Dans l'interface, cliquer "🔌 Connecter"
- [ ] Vérifier que le statut devient vert "Connecté"
- [ ] Ouvrir le modal de configuration
- [ ] Changer l'IP (exemple: `127.0.0.1` au lieu de `localhost`)
- [ ] Sauvegarder
- [ ] L'interface doit:
  1. Se déconnecter automatiquement
  2. Attendre 1 seconde
  3. Se reconnecter avec la nouvelle IP

### 8. **Messages dans les Logs**

Vérifier que les messages suivants apparaissent:
- [ ] `⚙️ Modal de configuration ouvert` (à l'ouverture)
- [ ] `❌ Configuration annulée` (si on clique Annuler)
- [ ] `✅ Configuration sauvegardée: ws://...` (après sauvegarde)
- [ ] `🔄 Reconnexion en cours...` (si déjà connecté)
- [ ] `📍 Configuration chargée: ws://...` (au chargement de la page)

### 9. **Test Réseau (si Jetson Nano disponible)**

**Sur le Jetson Nano:**
```bash
cd /chemin/vers/projet/scripts
python3 calibration_server.py
```

**Trouver l'IP du Jetson:**
```bash
hostname -I
# Exemple: 192.168.1.100
```

**Sur votre PC:**
- [ ] Ouvrir calibration_interface.html
- [ ] Cliquer "⚙️ Configuration IP"
- [ ] Entrer l'IP du Jetson (ex: `192.168.1.100`)
- [ ] Port: `8765`
- [ ] Sauvegarder
- [ ] Cliquer "🔌 Connecter"
- [ ] Vérifier que la connexion s'établit ✅

### 10. **Test de Persistance Multi-Navigateurs**

- [ ] Configurer dans Chrome avec une IP
- [ ] Ouvrir la même page dans Firefox
- [ ] Firefox doit avoir sa propre configuration (pas de synchronisation)
- [ ] C'est normal: chaque navigateur a son localStorage séparé

## 🎯 Résultats Attendus

Si tous les tests passent, vous devez observer:

1. ✅ **Interface conviviale** sans besoin de toucher au code
2. ✅ **Validation robuste** des entrées
3. ✅ **Persistance** de la configuration
4. ✅ **Reconnexion automatique** après changement d'IP
5. ✅ **Messages clairs** dans les logs
6. ✅ **Flexibilité** entre `localhost` et IP réseau

## 🐛 Problèmes Courants

### Le modal ne s'affiche pas
**Solution:**
- Rafraîchir la page (F5 ou Ctrl+F5)
- Vider le cache du navigateur
- Vérifier la console (F12) pour les erreurs JavaScript

### Les valeurs ne se sauvent pas
**Solution:**
- Désactiver le mode navigation privée
- Vérifier que localStorage est activé dans les paramètres du navigateur
- Tester avec `localStorage.setItem('test', 'ok')` dans la console

### La connexion échoue après sauvegarde
**Solution:**
- Vérifier que le serveur est bien démarré
- Tester avec `ping <IP>` pour vérifier la connectivité réseau
- Vérifier le pare-feu (port 8765 doit être ouvert)

## 📊 Rapport de Test

Remplissez après vos tests:

```
Date: _______________
Navigateur utilisé: _______________
Version du navigateur: _______________

Tests réussis: __ / 10
Tests échoués: __ / 10

Commentaires:
_________________________________
_________________________________
_________________________________

Signature: _______________
```

---

## 🎉 Félicitations !

Si tous les tests passent, votre interface est **prête pour la production** ! 🚀

Vous pouvez maintenant:
- Utiliser l'interface sur n'importe quel PC du réseau
- Changer d'IP en quelques clics
- Ne jamais toucher au code pour la configuration réseau

**Prêt pour TRC 2025 Cotonou, Bénin ! 🇧🇯🤖**

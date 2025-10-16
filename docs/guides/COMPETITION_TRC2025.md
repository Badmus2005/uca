# 🏆 Guide Compétition TRC 2025

**Ucaotech DOFbot TRC2025 - Guide Compétition**

---

## 📋 Table des Matières

1. [À Propos de TRC 2025](#à-propos-de-trc-2025)
2. [Règlement](#règlement)
3. [Stratégie](#stratégie)
4. [Préparation](#préparation)
5. [Checklist Jour J](#checklist-jour-j)
6. [Optimisations](#optimisations)
7. [Scénarios de Secours](#scénarios-de-secours)
8. [Post-Compétition](#post-compétition)

---

## 🎯 À Propos de TRC 2025

### Présentation

La **TRC (Technological Robot Competition) 2025** est une compétition de robotique organisée au Maroc, mettant en avant l'innovation et les compétences techniques dans le domaine de la robotique autonome.

**Équipe Ucaotech**
- 🏫 **École**: ENSAM Casablanca
- 👥 **Membres**: [Noms équipe]
- 🤖 **Robot**: DOFbot 6-axis avec vision
- 🎯 **Objectif**: Tri automatique de déchets

### Thème 2025: Tri Intelligent de Déchets

**Mission**: Trier automatiquement des déchets en 3 catégories:
- 📦 **Carton** → Bac bleu
- 🧴 **Plastique** → Bac jaune
- 🔩 **Métal** → Bac vert

### Contraintes

| Contrainte | Valeur |
|------------|--------|
| **Temps** | 3 minutes maximum |
| **Zone de jeu** | 2m x 2m |
| **Objets** | 15-20 déchets variés |
| **Autonomie** | 100% autonome (pas de télécommande) |
| **Énergie** | Batterie uniquement |

---

## 📜 Règlement

### 1. Déroulement Épreuve

#### Phase 1: Installation (5 minutes)
- Placement du robot en zone de départ
- Vérification connexions
- Test rapide systèmes
- Validation arbitres

#### Phase 2: Compétition (3 minutes)
```
00:00 - Départ (signal arbitre)
00:05 - Début détection objets
00:15 - Premier objet trié
03:00 - Fin (arrêt automatique)
```

#### Phase 3: Validation (2 minutes)
- Comptage objets par bac
- Vérification catégories
- Calcul score

### 2. Système de Points

#### Points Positifs
| Action | Points |
|--------|--------|
| Objet correctement trié | **+10** |
| Objet trié en <10s | **+2** (bonus vitesse) |
| Bac correct identifié | **+5** |
| Retour zone départ | **+5** |
| Temps restant (par 10s) | **+1** |

#### Pénalités
| Faute | Points |
|-------|--------|
| Objet mal trié | **-5** |
| Objet hors bac | **-3** |
| Sortie zone de jeu | **-10** |
| Contact humain | **-15** |
| Arrêt d'urgence | **-20** |

#### Bonus Spéciaux
- 🥇 **+50** : Tous objets triés
- 🎯 **+30** : 100% précision
- ⚡ **+20** : Temps < 2 minutes

### 3. Règles Techniques

#### Dimensions Robot
- Taille max: **40cm x 40cm x 50cm** (replié)
- Poids max: **10 kg**
- Pas de parties dangereuses

#### Autonomie
- ✅ Détection autonome
- ✅ Décision autonome
- ✅ Mouvement autonome
- ❌ Pas de contrôle externe
- ❌ Pas d'intervention humaine

#### Sécurité
- Bouton d'arrêt d'urgence obligatoire
- Vitesse max: 0.5 m/s
- Force gripper limitée
- Protection batteries

### 4. Zone de Jeu

```
┌─────────────────────────────────────┐
│         Zone de Jeu 2m x 2m         │
│                                     │
│  🟦 Bac Carton    🟨 Bac Plastique  │
│     (0.3, 1.5)       (1.0, 1.5)    │
│                                     │
│  🟩 Bac Métal      📦 Zone Objets   │
│     (1.7, 1.5)       (1.0, 0.5)    │
│                                     │
│  🤖 Zone Départ                     │
│     (1.0, 0.1)                      │
└─────────────────────────────────────┘
```

---

## 🎮 Stratégie

### 1. Approche Globale

**Stratégie Adoptée**: **Balayage Systématique + Tri Prioritaire**

#### Avantages
- ✅ Couverture complète zone
- ✅ Pas d'objets oubliés
- ✅ Ordre prédictible
- ✅ Facile à débugger

#### Inconvénients
- ⚠️ Pas optimisé pour vitesse pure
- ⚠️ Mouvements répétitifs

### 2. Phases Opérationnelles

#### Phase 1: Scan Initial (10-15s)
```python
def scan_zone():
    """Balayage complet zone de jeu"""
    # Rotation 360° avec détection
    for angle in range(0, 360, 30):
        rotate(angle)
        detect_objects()
        map_positions()
    
    # Prioriser objets
    prioritize_by_distance()
```

**Objectif**: Détecter maximum d'objets

#### Phase 2: Tri Prioritaire (120-150s)
```python
def execute_sorting():
    """Tri des objets par priorité"""
    for obj in sorted_objects:
        # Pick
        success = pick_object(obj)
        if not success:
            continue  # Passer au suivant
        
        # Identify
        category = confirm_category(obj)
        
        # Place
        bin_pos = get_bin_position(category)
        place_object(bin_pos)
        
        # Update count
        update_statistics()
```

**Priorités**:
1. 🎯 Objets proches (< 0.5m)
2. 💯 Objets haute confiance (> 0.9)
3. 📦 Catégorie manquante (diversité)
4. ⚡ Objets faciles à saisir

#### Phase 3: Retour (5-10s)
```python
def return_home():
    """Retour position départ"""
    navigate_to(START_POSITION)
    confirm_arrival()
    shutdown_systems()
```

### 3. Décisions Tactiques

#### Scénario 1: Objet Difficile
```
Si confiance < 0.75:
  → Analyser sous autre angle
  → Si toujours < 0.75: SKIP
```

#### Scénario 2: Temps Limité
```
Si temps restant < 30s:
  → Trier seulement objets proches
  → Ignorer objets lointains
  → Retour immédiat
```

#### Scénario 3: Gripper Échoue
```
Si échec pick après 2 tentatives:
  → Marquer objet comme problématique
  → Passer au suivant
  → Retenter à la fin si temps
```

### 4. Optimisations Trajectoire

#### Optimisation 1: Groupement Spatial
```python
def cluster_objects(objects):
    """Grouper objets proches"""
    from sklearn.cluster import DBSCAN
    
    positions = [[obj.x, obj.y] for obj in objects]
    clustering = DBSCAN(eps=0.3, min_samples=2).fit(positions)
    
    # Trier par clusters
    return sort_by_clusters(objects, clustering.labels_)
```

#### Optimisation 2: Trajectoire Shortest Path
```python
def optimize_path(objects, bins):
    """Optimiser ordre de visite"""
    from scipy.spatial.distance import cdist
    
    # Calcul distances
    distances = cdist(object_positions, bin_positions)
    
    # TSP approximation (greedy)
    path = nearest_neighbor_tsp(distances)
    
    return path
```

---

## 🛠️ Préparation

### 1. Entraînement (2 semaines avant)

#### Semaine -2
- [x] Test modèle sur objets réels compétition
- [x] Calibration fine caméra
- [x] Réglage gripper (force, vitesse)
- [x] Tests pick & place répétitifs (50+ essais)
- [x] Mesure temps moyen par objet

#### Semaine -1
- [x] Simulation runs complets (10+)
- [x] Optimisation trajectoires
- [x] Tests robustesse (conditions variées)
- [x] Préparation plan B
- [x] Vérification batterie

### 2. Tests Validation

#### Test 1: Précision Détection
```bash
# Objectif: >95% précision
python3 scripts/test_detection.py --samples 100
```

**Critères**:
- Carton: >95% précision
- Plastique: >92% précision
- Métal: >95% précision

#### Test 2: Vitesse Tri
```bash
# Objectif: <12s par objet
python3 scripts/test_sorting_speed.py --objects 15
```

**Métriques**:
- Temps moyen pick: <5s
- Temps moyen place: <4s
- Temps navigation: <3s

#### Test 3: Fiabilité Gripper
```bash
# Objectif: >90% succès
python3 scripts/test_gripper.py --trials 50
```

**Résultats Attendus**:
- Succès pick: >90%
- Succès hold: >95%
- Succès place: >98%

### 3. Matériel

#### Checklist Matériel Essentiel
- [ ] 🤖 DOFbot complet (vérifié)
- [ ] 🔋 Batteries chargées (2 sets minimum)
- [ ] 📷 Caméra RealSense + câble
- [ ] 💻 Jetson Nano + carte SD backup
- [ ] 🔌 Câbles alimentation
- [ ] 🔧 Kit outils (tournevis, pinces)
- [ ] 📱 Routeur WiFi portable
- [ ] 🖱️ Clavier/souris sans fil

#### Checklist Matériel Secours
- [ ] 🔋 Batterie supplémentaire
- [ ] 💾 Carte SD backup (système complet)
- [ ] 🔌 Câbles USB additionnels
- [ ] 🔩 Vis et fixations de rechange
- [ ] 🧰 Multimètre
- [ ] 📋 Documentation imprimée

---

## ✅ Checklist Jour J

### J-1 (Veille)

#### Technique
- [ ] Charge complète toutes batteries
- [ ] Backup final code sur USB
- [ ] Test complet système
- [ ] Vérification calibration
- [ ] Préparation matériel transport

#### Logistique
- [ ] Vérification inscription
- [ ] Plan d'accès au lieu
- [ ] Horaires passage
- [ ] Contact organisateurs
- [ ] Hébergement confirmé

### Jour J - Matin

#### Arrivée (H-2h)
- [ ] Check-in organisation
- [ ] Installation stand
- [ ] Déballage matériel
- [ ] Branchements
- [ ] Test rapide connexions

#### Avant Passage (H-30min)
- [ ] Charge batterie 100%
- [ ] Vérification caméra
- [ ] Test mouvements robot
- [ ] Vérification code version finale
- [ ] Brief équipe

### Pendant Compétition

#### Phase Installation (5min)
```
T-5min  ✓ Placement robot zone départ
T-4min  ✓ Connexion système
T-3min  ✓ Lancement ROS nodes
T-2min  ✓ Test caméra
T-1min  ✓ Calibration rapide
T-0min  ✓ Mode autonome → GO!
```

#### Phase Run (3min)
```
00:00  🟢 Départ
00:10  📊 Premier objet détecté
00:20  📦 Premier objet trié
01:00  📊 5 objets triés
02:00  📊 10 objets triés
02:50  🏠 Retour zone départ
03:00  🏁 Fin
```

#### Surveillance
- 👀 Observer écran debug
- 📊 Compter objets triés
- ⏱️ Monitorer temps restant
- 🚨 Prêt pour arrêt d'urgence

### Après Passage

#### Immédiat
- [ ] Sauvegarde logs
- [ ] Photos/vidéos run
- [ ] Notes problèmes rencontrés
- [ ] Recharge batteries

#### Débriefing (15min)
- Qu'est-ce qui a bien marché ? ✅
- Qu'est-ce qui a échoué ? ❌
- Améliorations possibles ? 💡
- Score obtenu vs. attendu ? 📊

---

## ⚡ Optimisations

### 1. Optimisations Logicielles

#### Optimisation Détection
```python
# Version optimisée pour compétition
class CompetitionVisionNode:
    def __init__(self):
        # Modèle TensorRT (2x plus rapide)
        self.model = YOLO('models/yolov8n_waste.engine')
        
        # Paramètres agressifs
        self.conf_threshold = 0.70  # Réduit pour vitesse
        self.nms_threshold = 0.35
        
        # Cache résultats
        self.detection_cache = {}
    
    def detect_optimized(self, image):
        """Détection optimisée"""
        # Réduire résolution si besoin
        if self.time_remaining < 60:
            image = cv2.resize(image, (416, 416))
        
        # Détection
        results = self.model(image, verbose=False)
        
        return self.filter_results(results)
```

#### Optimisation Contrôle
```python
# Mouvements parallélisés
class FastController:
    def pick_and_navigate(self, obj, next_obj):
        """Pick objet actuel + naviguer vers suivant"""
        # Thread 1: Pick actuel
        pick_thread = threading.Thread(
            target=self.pick_object,
            args=(obj,)
        )
        
        # Thread 2: Planifier trajet suivant
        plan_thread = threading.Thread(
            target=self.plan_trajectory,
            args=(next_obj,)
        )
        
        pick_thread.start()
        plan_thread.start()
        
        pick_thread.join()
        plan_thread.join()
```

### 2. Optimisations Matérielles

#### Optimisation Performance Jetson
```bash
#!/bin/bash
# competition_boost.sh

# Mode performance MAX
sudo nvpmodel -m 0
sudo jetson_clocks

# Désactiver GUI (économie ressources)
sudo systemctl stop gdm3

# Priorité processus ROS
sudo renice -n -10 -p $(pgrep -f ros)

# Nettoyage cache
sync; echo 3 | sudo tee /proc/sys/vm/drop_caches
```

#### Optimisation Énergie
```yaml
# config/power_management.yaml
battery_monitoring:
  check_interval: 10  # secondes
  low_battery_threshold: 20  # %
  critical_threshold: 10  # %
  
actions:
  low_battery:
    - reduce_speed: 0.7
    - disable_debug_display: true
  
  critical_battery:
    - emergency_return: true
    - save_state: true
```

### 3. Tuning Paramètres Compétition

**`config/competition_params.yaml`**
```yaml
vision:
  confidence_threshold: 0.70  # Réduit pour vitesse
  processing_rate: 20         # Hz (réduit de 30)
  image_resolution: [640, 480]  # Réduit de 1280x720

control:
  max_velocity: 0.8           # m/s (augmenté)
  acceleration: 3.0           # m/s² (augmenté)
  gripper_speed: 1.5          # Rapide
  
navigation:
  obstacle_margin: 0.03       # m (réduit)
  path_planning_timeout: 0.5  # s (réduit)

strategy:
  max_pick_attempts: 2        # Limite tentatives
  skip_low_confidence: 0.70
  prioritize_close_objects: true
  max_sorting_time: 170       # s (garder 10s retour)
```

---

## 🆘 Scénarios de Secours

### Scénario 1: Caméra Défaillante

**Symptômes**:
- Pas d'image
- Image figée
- FPS très faible

**Actions**:
```bash
# 1. Redémarrer node caméra
rosnode kill /camera_node
rosrun realsense2_camera rs_camera.launch

# 2. Si échec: Débrancher/rebrancher USB
# 3. Si échec: Reboot système
sudo reboot
```

**Plan B**: Mode manuel (si règlement permet)

### Scénario 2: Gripper Bloqué

**Symptômes**:
- Gripper ne bouge pas
- Erreurs servo

**Actions**:
```python
# Reset gripper
arm.Arm_serial_servo_write(6, 90, 1000)
time.sleep(2)

# Test ouverture/fermeture
for pos in [135, 45, 90]:
    arm.Arm_serial_servo_write(6, pos, 500)
    time.sleep(1)
```

**Plan B**: Sauter objets nécessitant gripper précis

### Scénario 3: Batterie Faible

**Symptômes**:
- Tension < 11V
- Mouvements ralentis

**Actions**:
```python
def battery_emergency():
    """Mode économie batterie"""
    # Réduire vitesse
    set_velocity_limit(0.5)
    
    # Désactiver debug
    disable_debug_displays()
    
    # Trier objets proches seulement
    filter_objects(max_distance=0.5)
    
    # Retour anticipé
    if battery < 15:
        emergency_return()
```

### Scénario 4: Objet Coincé

**Symptômes**:
- Objet dans gripper ne lâche pas
- Détection objet persistante

**Actions**:
1. Forcer ouverture gripper
2. Secouer doucement
3. Si échec: Continuer avec objet
4. Tenter de déposer dans bon bac quand même

### Scénario 5: Perte WiFi

**Symptômes**:
- Impossible accéder interface web
- Logs non disponibles

**Impact**: ⚠️ Faible (système autonome)

**Actions**:
- Mode autonome continue
- Pas d'intervention possible
- Monitorer via écran HDMI si possible

---

## 📊 Post-Compétition

### 1. Analyse Immédiate

#### Métriques à Collecter
```python
# Après run, sauvegarder:
metrics = {
    'total_time': run_duration,
    'objects_detected': len(all_detections),
    'objects_sorted': len(successfully_sorted),
    'accuracy': correct_sorts / total_sorts,
    'average_pick_time': sum(pick_times) / len(pick_times),
    'average_place_time': sum(place_times) / len(place_times),
    'errors': error_log,
    'battery_consumption': start_voltage - end_voltage
}

save_metrics('run_metrics.json', metrics)
```

#### Questions Clés
- ✅ Quels objets bien détectés ?
- ❌ Quels objets ratés ?
- ⚡ Temps par étape conforme ?
- 🔋 Consommation batterie acceptable ?
- 🐛 Bugs rencontrés ?

### 2. Rapport Détaillé

**Template Rapport**:
```markdown
# Rapport Compétition TRC 2025

## Résultats
- Score: XX points
- Classement: XX/YY
- Objets triés: XX/15

## Performance
- Temps: XX secondes
- Précision: XX%
- Objets par minute: XX

## Points Positifs
- [Liste succès]

## Points d'Amélioration
- [Liste échecs]

## Leçons Apprises
- [Liste insights]

## Actions Futures
- [Liste améliorations]
```

### 3. Améliorations Futures

#### Court Terme (1 mois)
- [ ] Corriger bugs identifiés
- [ ] Améliorer précision détection
- [ ] Optimiser trajectoires
- [ ] Tests robustesse

#### Moyen Terme (3 mois)
- [ ] Nouveau modèle ML amélioré
- [ ] Refonte planification trajectoire
- [ ] Interface monitoring avancée
- [ ] Documentation complète

#### Long Terme (6 mois)
- [ ] Multi-robots collaboration
- [ ] IA apprentissage en ligne
- [ ] Préhension adaptative
- [ ] Généralisation autres objets

---

## 🎯 Objectifs de Performance

### Objectifs Minimaux (Bronze 🥉)
- ✅ 8+ objets triés
- ✅ 60%+ précision
- ✅ Pas de sortie zone
- ✅ Système stable

**Score Estimé**: 80-100 points

### Objectifs Cibles (Argent 🥈)
- ✅ 12+ objets triés
- ✅ 80%+ précision
- ✅ Temps < 2min30
- ✅ Bonus vitesse

**Score Estimé**: 120-150 points

### Objectifs Stretch (Or 🥇)
- ✅ 15+ objets triés
- ✅ 95%+ précision
- ✅ Temps < 2min
- ✅ Tous bonus

**Score Estimé**: 180-220 points

---

## 📞 Contacts Utiles

### Organisation TRC 2025
- 📧 Email: contact@trc2025.ma
- 📱 Tél: +212 XXX XXX XXX
- 🌐 Site: www.trc2025.ma

### Équipe Ucaotech
- 👨‍💻 Chef équipe: [Nom]
- 📧 Email équipe: ucaotech@ensam.ac.ma
- 📱 Urgence: [Numéro]

### Support Technique
- 🔧 Yahboom: support@yahboom.com
- 📷 Intel RealSense: realsense.support@intel.com
- 🤖 ROS: answers.ros.org

---

## 🏁 Conclusion

**Préparation = Succès**

La compétition TRC 2025 est l'aboutissement de mois de travail. Avec :
- ✅ Une stratégie solide
- ✅ Des tests approfondis
- ✅ Une préparation rigoureuse
- ✅ Un plan de secours

**Notre équipe Ucaotech est prête à relever le défi ! 🚀**

---

**Bon courage à toute l'équipe ! 💪**

**Dernière mise à jour : 16 octobre 2025**

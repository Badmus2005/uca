#!/bin/bash
# Script de déploiement du projet ucaotech_dofbot_trc2025 sur Jetson Nano
# Usage: ./deploy_to_jetson.sh <jetson_ip> [jetson_user]

set -e  # Arrêter en cas d'erreur

# Couleurs pour l'affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Paramètres
JETSON_IP="${1:-}"
JETSON_USER="${2:-jetson}"
JETSON_WS="/home/${JETSON_USER}/catkin_ws"
PROJECT_NAME="ucaotech_dofbot_trc2025"

# Fonction d'affichage
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Vérifier les arguments
if [ -z "$JETSON_IP" ]; then
    print_error "Usage: $0 <jetson_ip> [jetson_user]"
    print_error "Exemple: $0 192.168.1.100 jetson"
    exit 1
fi

print_status "🚀 Déploiement du projet $PROJECT_NAME vers $JETSON_USER@$JETSON_IP"

# 1. Vérifier la connexion SSH
print_status "📡 Vérification de la connexion SSH..."
if ! ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" "echo 'SSH OK'" &>/dev/null; then
    print_error "Impossible de se connecter à $JETSON_USER@$JETSON_IP"
    print_error "Vérifiez l'adresse IP et que SSH est activé sur le Jetson"
    exit 1
fi
print_status "✅ Connexion SSH OK"

# 2. Vérifier que catkin_ws existe
print_status "📂 Vérification du workspace ROS..."
ssh "$JETSON_USER@$JETSON_IP" "
    if [ ! -d \"$JETSON_WS\" ]; then
        echo 'Création du workspace catkin...'
        mkdir -p $JETSON_WS/src
        cd $JETSON_WS
        catkin_make
        echo 'source $JETSON_WS/devel/setup.bash' >> ~/.bashrc
    fi
"
print_status "✅ Workspace ROS OK"

# 3. Créer une archive du projet (exclure les dossiers inutiles)
print_status "📦 Création de l'archive du projet..."
TEMP_DIR=$(mktemp -d)
ARCHIVE_NAME="${PROJECT_NAME}_$(date +%Y%m%d_%H%M%S).tar.gz"
ARCHIVE_PATH="${TEMP_DIR}/${ARCHIVE_NAME}"

tar -czf "$ARCHIVE_PATH" \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='.git' \
    --exclude='build' \
    --exclude='devel' \
    --exclude='*.log' \
    --exclude='REFERENCES' \
    -C .. "$PROJECT_NAME"

print_status "✅ Archive créée: $ARCHIVE_NAME ($(du -h "$ARCHIVE_PATH" | cut -f1))"

# 4. Transférer l'archive vers le Jetson
print_status "📤 Transfert de l'archive vers le Jetson..."
scp "$ARCHIVE_PATH" "$JETSON_USER@$JETSON_IP:/tmp/"
print_status "✅ Transfert terminé"

# 5. Extraire et installer sur le Jetson
print_status "⚙️  Installation sur le Jetson..."
ssh "$JETSON_USER@$JETSON_IP" << 'ENDSSH'
    set -e
    
    echo "📂 Extraction de l'archive..."
    cd /tmp
    tar -xzf $(ls -t ucaotech_dofbot_trc2025_*.tar.gz | head -1)
    
    echo "📋 Sauvegarde de l'ancienne version (si existe)..."
    if [ -d "$JETSON_WS/src/$PROJECT_NAME" ]; then
        mv "$JETSON_WS/src/$PROJECT_NAME" "$JETSON_WS/src/${PROJECT_NAME}_backup_$(date +%Y%m%d_%H%M%S)"
    fi
    
    echo "📦 Copie du nouveau projet..."
    mv "/tmp/$PROJECT_NAME" "$JETSON_WS/src/"
    
    echo "🔨 Compilation du package ROS..."
    cd "$JETSON_WS"
    source /opt/ros/melodic/setup.bash
    catkin_make --pkg $PROJECT_NAME
    
    echo "🔧 Configuration des permissions I2C..."
    sudo usermod -aG i2c $USER || true
    sudo chmod 666 /dev/i2c-1 2>/dev/null || true
    
    echo "📝 Vérification des dépendances Python..."
    # Vérifier si requirements.txt existe
    if [ -f "$JETSON_WS/src/$PROJECT_NAME/requirements.txt" ]; then
        echo "Installation des dépendances Python..."
        pip3 install -r "$JETSON_WS/src/$PROJECT_NAME/requirements.txt" --user
    else
        echo "⚠️  Pas de requirements.txt trouvé, installation manuelle des dépendances:"
        echo "   - PyTorch (déjà installé avec JetPack)"
        echo "   - OpenCV (déjà installé avec JetPack)"
        echo "   - Autres: pip3 install pyyaml numpy pillow --user"
    fi
    
    echo "🧹 Nettoyage..."
    rm -f /tmp/ucaotech_dofbot_trc2025_*.tar.gz
    
    echo "✅ Installation terminée !"
ENDSSH

print_status "✅ Déploiement terminé !"

# 6. Afficher les instructions de test
cat << EOF

${GREEN}╔═══════════════════════════════════════════════════════════════╗
║              🎉 DÉPLOIEMENT RÉUSSI !                          ║
╚═══════════════════════════════════════════════════════════════╝${NC}

📋 Prochaines étapes sur le Jetson Nano:

1. Se connecter au Jetson:
   ${YELLOW}ssh $JETSON_USER@$JETSON_IP${NC}

2. Sourcer l'environnement ROS:
   ${YELLOW}source ~/catkin_ws/devel/setup.bash${NC}

3. Vérifier l'installation:
   ${YELLOW}rospack find $PROJECT_NAME${NC}

4. Tester la caméra:
   ${YELLOW}rosrun $PROJECT_NAME final_camera_node.py${NC}

5. Tester le modèle YOLOv5:
   ${YELLOW}cd ~/catkin_ws/src/$PROJECT_NAME
   python3 scripts/test_model.py${NC}

6. Lancer le système complet:
   ${YELLOW}roslaunch $PROJECT_NAME tri.launch${NC}

${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}

📦 Archive locale sauvegardée: $ARCHIVE_PATH
🗑️  Pour supprimer l'archive: rm -f "$ARCHIVE_PATH"

${GREEN}🏆 Bon courage pour TRC 2025 ! 🤖${NC}

EOF

# Nettoyer
rm -rf "$TEMP_DIR"

exit 0

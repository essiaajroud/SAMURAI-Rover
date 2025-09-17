# SAMURAI Project - ROS2 Enabled

## Description

SAMURAI is an AI-powered detection and tracking system, now integrated with ROS2 for robotics applications.

## Features

- YOLOv8-based object detection
- Real-time tracking system
- ROS2 integration (Foxy/Humble)
- Web-based dashboard
- MLOps pipeline with Jenkins
- Simulation support through Gazebo

## Requirements

- ROS2 (Foxy or Humble)
- Python 3.8+
- CUDA-capable GPU (optional)
- Node.js 14+

## Installation

### ROS2 Setup

```bash
# For Foxy
source /opt/ros/foxy/setup.bash

# For Humble
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt install ros-$ROS_DISTRO-rosbridge-server
```

### Project Setup

```bash
# Clone repository
git clone https://github.com/yourusername/SAMURAI.git
cd SAMURAI

# Build ROS2 workspace
cd ros2_ws
colcon build
source install/setup.bash

# Install frontend dependencies
cd ../client
npm install
```

## Usage

### Launch Options

1. Full System (with simulation):

```bash
ros2 launch samurai_launch full_system.launch.py
```

2. Dashboard:

```bash
cd client
npm run serve
```

3. Testing without hardware:

```bash
ros2 launch samurai_simulation simulation.launch.py
```

## Architecture

- `/ros2_ws`: ROS2 workspace
  - `/src/samurai_detection`: Detection node
  - `/src/samurai_tracking`: Tracking node
  - `/src/samurai_simulation`: Simulation package
- `/client`: Web dashboard
- `/mlops`: MLOps pipeline scripts
- `/server`: Backend services and models

## Development

### Structure des données

```javascript
// Détection
{
  id: 1,
  label: "Person",
  confidence: 0.92,
  x: 215,
  y: 304,
  speed: 5.3,
  distance: 12.4,
  timestamp: 1640995200000
}

// Trajectoire
{
  id: 1,
  label: "Person",
  startTime: 1640995200000,
  lastSeen: 1640995260000,
  points: [
    { x: 215, y: 304, timestamp: 1640995200000 },
    { x: 220, y: 310, timestamp: 1640995210000 }
  ]
}
```

### Ajout de nouvelles fonctionnalités

1. **Backend** : Ajouter les endpoints dans `app.py`
2. **Frontend** : Créer les composants dans `client/src/components/`
3. **Base de données** : Modifier les modèles si nécessaire
4. **Tests** : Vérifier la compatibilité

## 🐛 Dépannage

### Problèmes courants

**Backend ne démarre pas :**

- Vérifier que Python 3.7+ est installé
- Vérifier que les dépendances sont installées
- Vérifier que le port 5000 est libre

**Frontend ne se connecte pas au backend :**

- Vérifier que le backend est démarré
- Vérifier l'URL dans la configuration
- Vérifier les logs de la console

**Données non sauvegardées :**

- Vérifier la connexion à la base de données
- Vérifier les permissions d'écriture
- Vérifier les logs du serveur

## 📈 Performance

### Optimisations recommandées

- Utiliser une base de données PostgreSQL pour la production
- Implémenter la pagination pour les grandes quantités de données
- Ajouter un cache Redis pour les requêtes fréquentes
- Optimiser les requêtes de base de données

## 🤝 Contribution

1. Fork le projet
2. Créer une branche pour votre fonctionnalité
3. Commiter vos changements
4. Pousser vers la branche
5. Ouvrir une Pull Request

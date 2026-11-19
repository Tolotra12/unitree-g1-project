#Projet Unitree G1 EDU - ROS 2

# Documentation technique - Environnement et simulation du G1

Exemple de README à placer à la racine du dépôt GitHub. Les noms de paquets, de dossiers et de topics sont des exemples : à adapter à votre projet.

## Objectif

Ce dépôt contient l'environnement de développement et la simulation de l'humanoïde Unitree G1 : ROS 2 Humble, Docker, MuJoCo et Isaac Sim. Il sert de base aux briques suivantes : traitement des données, vision et navigation.

## Prérequis

| Élément | Version | Remarque |
| --- | --- | --- |
| Système | Ubuntu 22.04 | Installé en natif |
| ROS 2 | Humble | Installé dans le conteneur Docker |
| Docker | 24 ou plus | Avec NVIDIA Container Toolkit |
| GPU | NVIDIA, pilote récent | Nécessaire pour Isaac Sim |
| Simulateurs | MuJoCo, Isaac Sim | Voir la section Installation |

## Installation

1. Cloner le dépôt avec ses sous-modules :

```bash
git clone --recursive https://github.com/<organisation>/<depot>.git
cd <depot>
```

2. Construire l'image Docker :

```bash
docker build -t g1-dev -f docker/Dockerfile .
```

3. Lancer le conteneur avec accès au GPU et au réseau :

```bash
docker run -it --rm --gpus all --net=host \
  -v $(pwd)/ros2_ws:/root/ros2_ws g1-dev
```

4. Compiler le workspace ROS 2 à l'intérieur du conteneur :

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Structure du dépôt

| Dossier | Contenu |
| --- | --- |
| `docker/` | Dockerfile et scripts de lancement |
| `ros2_ws/src/` | Paquets ROS 2 du projet |
| `models/` | Description du G1 (URDF, MJCF, USD) |
| `config/` | Paramètres YAML (capteurs, EKF, SLAM) |
| `docs/` | Documentation détaillée (architecture, essais, décisions) |

## Lancer la simulation

```bash
ros2 launch g1_bringup sim.launch.py
```

Pour piloter le robot à la manette, dans un second terminal du conteneur :

```bash
ros2 launch g1_teleop joy.launch.py
```

On peut alors visualiser le robot dans RViz2 : l'arbre `tf` et les articulations doivent s'afficher.

## Topics ROS 2 principaux

Noms donnés à titre d'exemple : à vérifier avec `ros2 topic list`.

| Topic | Type | Rôle |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | État des articulations |
| `/imu` | `sensor_msgs/Imu` | Centrale inertielle |
| `/points` | `sensor_msgs/PointCloud2` | Nuage de points du LiDAR |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Image couleur |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Image de profondeur |
| `/cmd_vel` | `geometry_msgs/Twist` | Commande de vitesse |
| `/tf` | `tf2_msgs/TFMessage` | Repères du robot |

## Configuration

- Les paramètres des capteurs, de l'EKF et du SLAM sont dans `config/*.yaml`.
- Toute modification de paramètre est notée dans le journal de travail ci-dessous, avec la raison.

## Problèmes connus

| Symptôme | Cause probable | Solution |
| --- | --- | --- |
| Isaac Sim ne démarre pas dans Docker | Accès GPU absent | Vérifier `--gpus all` et le NVIDIA Container Toolkit |
| Aucun topic visible dans RViz | Bridge ROS 2 non actif | Vérifier que l'extension est activée dans Isaac Sim |
| Le robot tombe au démarrage | Contacts des pieds mal réglés | Contrôler friction et paramètres de contact du modèle |

## Journal de travail

Une entrée par jour de travail utile : ce qui a été fait, ce qui a été décidé, ce qui reste.

| Date | Fait | Décision ou remarque | À faire |
| --- | --- | --- | --- |
| 2026-11-12 | Image Docker construite, ROS 2 Humble opérationnel | Tout le projet tourne dans le conteneur pour rester reproductible | Importer le modèle du G1 |
| 2026-11-16 | Modèle du G1 chargé dans MuJoCo | Commencer par MuJoCo, plus rapide pour tester la marche | Convertir au format USD pour Isaac Sim |
| 2026-11-18 | Environnement validé de bout en bout | Phase 1 terminée | Ajouter les capteurs en simulation |

## Comment contribuer

- Une branche par ticket, nommée `feature/<sujet>`.
- Un message de commit court qui dit ce qui change et pourquoi.
- Mettre à jour ce document dans la même modification de code.

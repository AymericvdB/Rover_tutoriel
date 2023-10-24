# Modelisation d'un petit véhicule terrestre sous ROS et Gazebo

Ce package applique le tutoriel décrit dans la documentation du package nav2 de ROS2, en utilisant GazeboSim à la place de GazeboClassic;
Voir le [tutoriel](https://navigation.ros.org/setup_guides/index.html) original.


## Instructions (version courte)

## Instructions (version longue)

### Installation

Installer ROS2 Humble : suivre ces ([instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
Installer Gazebosim (Ignition) Fortress : suivre ces ([instructions](https://gazebosim.org/docs/fortress/install_ubuntu))

Installer le package 'ros_gz' (qui comprend entre autres 'ros_gz_bridge') : ' sudo apt install ros-humble-ros-gz'
Installer le package 'robot_localization' : 'sudo apt install ros-humble-robot-localization'
Installer le package 'slam_toolbox' : 'sudo apt install ros-humble-slam-toolbox'
Installer le package 'nav2' : 'sudo apt install ros-humble-navigation2'
                              'sudo apt install ros-humble-nav2-bringup'

Nota : 
    Plusieurs distributions de Gazebosim peuvent coexister; il faut simplement préciser la version à lancer dans l'invite de commande.
    En revanche, GazeboClassic et GazeboSim semblent ne pas pouvoir coexister.

Nota 2 :
    Au début de son développement, GazeboSim s'appelait Ignition jusqu'à la distribution Fortress. C'est à partir de la distribution Garden que le logiciel a pris son nom actuel. En conséquence, les commandes ont changé, il faut donc prendre garde aux risques d'incompatibilité.
    Exemple : lancer une simulation : 
        - sur Fortress : 'ign gazebo empty.sdf'
        - sur Gazebo : 'gz sim empty.sdf' 

Nota 3 : 
    Noter que le package 'ros_ign_bridge' existe aussi, mais 'ros_gz_bridge' fonctionne bien avec Fortress. En fait, il s'agit du même package sur le github, mais il change de nom selon les versions sélectionnées. Les topic de compatibilité sont simplement à prendre dans le bon README : dans notre cas, [ici](https://github.com/gazebosim/ros_gz/blob/foxy/ros_ign_bridge/README.md). Ce README explique aussi comment utiliser ce package; ici, on se simplifie la vie en utilisant un fichier 'ros_gz_bridge.yaml' stocké dans le dossier 'config' et appelé dans les 'launch' (dossier 'launch')

Nota 4 :
    Lien des documentations : 
    - [ROS2](https://docs.ros.org/en/humble/index.html)
    - [GazeboSim(Ignition)](https://gazebosim.org/docs/fortress/getstarted)
    - [ros_gz_bridge](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
    - [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) : package implémentant le filtre de Kalman
    - [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) : package permettant l'utilisation de l'algorithme SLAM (Simultaneous Localization And Mapping) en utilisant un lidar, entre autres (pour un SLAM utilisant une caméra, voir ['ORB_SLAM3'](https://github.com/UZ-SLAMLab/ORB_SLAM3))
    - [nav2](https://navigation.ros.org/getting_started/index.html) : package proposant des outils pour la navigation (d'où le tutoriel est issu)

### Mise en route

Avant tout, sourcer les commandes ROS2 : 'source /opt/ros/humble/setup.bash'
Cette commande est à lancer à chaque ouverture de terminal où l'on souhaite utiliser ROS2
Pour inscrire cette commande dans le shell startup script et ne plus avoir à la lancer systématiquement : 
    'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'
Référence [ici](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

Si besoin, créer un nouvel espace de travail ROS2 (workspace) et un fichier src à l'intérieur; s'y rendre:
    'mkdir -p ~/ros2_ws/src'
    'cd ~/ros2_ws/src'

Télécharger le package et l'installer dans le fichier src. On peut aussi le cloner avec git:
    'git clone <lien> -b humble'

Retourner dans le dossier workspace et lancer la compilation
    'cd ..'
    'colcon build'

Une fois la compliation terminée, sourcer le contenu du workspace (comme le précédent, à lancer pour chaque nouveau terminal ouvert):
    'source install/setup.bash'

Référence [ici](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Se rendre dans le fichier 'launch' : 
    'cd src/rover_base_test1/launch'

Sélectionner un fichier launch à lancer. Exemple:
    'ros2 launch rover_base_gazebo_slam_launch.py'

Nota : 
    Ce package est configuré pour accueillir des fichiers source c++ (on n'a pas eu besoin ici de coder des noeuds ROS pour cette exemple). Il est aussi possible d'en configurer pour coder en python (Les fichiers launch ne comptent pas : ils peuvent être écrits indifféremment en python, XML ou YAML). Lorsqu'un package est en Python (comme le package teleop qui accompagne ce package), la compilation a tendance à ne pas copier certains launch dans le bon dossier, causant des erreurs au lancement : personnellement, je les mets alors à leur place à la main.

Nota2 : 
    Les fichiers launch permettent de lancer différents noeuds de différents package et d'effectuer d'autres manipulations comme des commandes bash, etc. Ils permettent donc de gagner pas mal de temps.
    J'y inclue la génération d'un URDF à partir du Xacro qui décrit mon robot, puis la génération d'un SDF à partir de cet URDF auquel j'ajoute ensuite des capteurs (lidar et caméra) par édition automatique. Je lance ensuite GazeboSim avec le fichier world.sdf qui inclue le SDF créé.
    Pourquoi ne pas inclure les capteurs dans une balise <gazebo> dès le Xacro? Parce que bizarrement, la conversion URDF-SDF les fait disparaître.

    Documentation :  [launch](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

Nota3 : 
    Pour rappel : 
        - un URDF est un format de fichier XML qui décrit un robot dans un langage compréhensible par ROS2
        - un Xacro est un URDF sur lequel on applique des macros python, ce qui permet d'avoir un fichier souvent beaucoup moins long et d'utiliser des variables, des conditions... pour éditer plus facilement le robot
        - Un SDF est un format de fichier XML compréhensible par GazeboSim (et GazeboClassic) qui peut décrire un robot, un monde avec des robots, des paramètres de simulation... Il est donc plus complet qu'un URDF. De plus, si la logique de description de robot est la même, ily a des éléments et des différences syntaxiques qui rendent nécessaire une traduction; ROS2 n'est pas capable de lire un SDF, mais GazeboSim est capable de lire un URDF décrivant un robot pour l'inclure dans un monde

    Documentations des différents formats : 
    - URDF : [tutoriel](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
             [référence](http://wiki.ros.org/urdf/XML)
    - Xacro : [tutoriel](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)
              [wiki](https://github.com/ros/xacro/wiki)
    - SDF : [tutoriel](https://gazebosim.org/docs/fortress/building_robot)
            [référence](http://sdformat.org/spec)

Nota4:
    La stratégie utilisée me permet de maintenir un seul fichier Xacro, à partir duquel je génère les autres. Il est aussi possible de faire des conversions SDF->URDF (voir [ici](https://gazebosim.org/docs/harmonic/ros2_interop)), mais le SDF ne supporte pas les macros (qui sont très pratiques) et n'est pas généré directement à partir du Xacro.

### Explorer

PLusieurs fichiers launch sont disponibles et permettent de mettre en place une complexité croissante du système.

#### rover_base_base_launch.py

Ce launch lance 4 noeuds pour afficher un robot décrit en URDF

-robot_state_publisher : ce noeud prend l'URDF en entrée et publie deux topic : /tf et /tf_static qui décrivent les caractéristiques de chaque link composant le robot et celles des joints qui relient ces link entre eux (voir les tutoriels URDF et SDF pour mieux appréhender les notions link et joint). Il publie aussi /robot_description
-joint_state_publisher : ce noeud publie des modifications affectant les joint à travers le topic /joint_state, qui est lu par le noeud robot_state_publisher
-joint_state_publisher_gui : comme son nom l'indique, ce noeud lance une petite fenêtre qui permet à l'utilisateur de modifier l'état des joint; sa fonction est la même que joint_state_publisher
-rviz : ce noeud lance une fenêtre de visualisation qui affiche le robot, dont il récupère les caractéristiques en s'abonnant aux topic /tf et /tf_static

Voici le graphe des noeuds résumant cette organisation, et obtenu avec la commande 'rqt_graph' : 

![rqt_graph1](/img/Graphe_base_launch.png?raw=true "Graphe")

Nota
    tf est un package dans ROS2 très utilisé pour établir les transformations statiques ou dynamiques qui existent entre les différents composants (link) d'un robot. Il permet d'établir des arbres de transformation, comme celui-ci, qui est celui de notre robot:

![arbre_tf](/img/Arbre_tf_robot.png?raw=true "Arbre tf robot")

    Pour obtenir ce graphe à partir d'un topic /tf : lancer 'ros2 run tf2_tools view_frames'.

    Pour plus d'informations, voir [ce lien](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

#### rover_base_xacro_launch.py

Ce launch exécute la même opération que le précédent, mais il génère l'URDF utilisé à partir d'un Xacro.

#### rover_base_gazebo_launch2.py

Ce launch introduit le simulateur GazeboSim dans la boucle, avec trois nouveaux noeuds.
De plus, le fichier SDF est généré à partir de l'URDF, et comporte des capteurs : imu (Inertial Measurement Unit, ou Centrale inertielle en français), lidar et caméra de profondeur.

Les nouveaux noeuds sont : 
- bridge : il utilise le package ros_gz_bridge pour transformer des topic Gazebo en topic ROS2, et inversement. La liste des topic à convertir dans l'un ou l'autre sens est écrite dans le fichier ros_gz_bridge.yaml (dossier config).
- gz_sim : plutôt qu'un noeud, il s'agit de lancer la commande de mise en route de GazeboSim avec le fichier world.sdf voulu
- teleop : ce noeud est issu du package teleop de ce repository; il reçoit les commandes clavier communiquées dans Gazebo et les convertit en commandes vitesse. Ce package est une adaptation pour un clavier français d'un package déjà existant. Pour comprendre quelle touche fait quoi, se référer au README du package. La traduction des commandes clavier Gazebo->ROS2 et celle des commandes de vitesse ROS2->Gazebo est assurée par le noeud bridge

Gazebo publie des topic /tf et /tf_static qui rendent compte de l'évolution du robot dans la simulation Gazebo. On peut donc en théorie éliminer les noeuds joint_state_publisher et robot_state_publisher qui n'ont plus d'utilité. 
En fait, éliminer joint_state_publisher ne pose pas de problème, mais robot_state_publisher publie aussi un topic /robot_description qui est en fait l'URDF donné et que Rviz utilise pour afficher le visuel. Sans ce noeud, on voit juste des repères de transformation bouger dans le visualisateur. Il vaut donc mieux garder /robot_state_publisher, dont les topic /tf et /tf_static se font "overwrite" par ceux de Gazebo.
Ne plus avoir joint_state_publisher m'a aussi permis d'éviter des problèmes de noms de joint et de tf, c'est surtout pour cette raison que je l'ai enlevé. Néanmoins, lorsque la simulation Gazebo est lancée, elle est par défaut sur pause et les publications /tf et /tf_static n'arrivent pas. Résultat, Rviz se fie aux topic de robot_state_publisher qui n'a pas de joint_state : les transformations des link les uns par rapport aux autres ne sont pas données, et le robot semble alors démonté. Pour arranger ça, il suffit de mettre la simu Gazebo sur Play et d'attendre que Rviz s'actualise.

A noter que dans Rviz, la visualisation prend pour référence le repère du robot : c'est donc son environnement qui bouge autour de lui lorsqu'il se déplace. Je n'ai pas encore réussi à changer de repère

Pour activer la capture des commandes clavier dans Gazebo, il faut activer le bon plugin : dans la fenêtre Gazebo en haut à droite, entrer dans la barre de recherche "key publisher" et cliquer dessus. Pour que les commandes clavier soient prises, il faut bien sûr que la fenêtre Gazebo soit sélectionnée.

Pour visualiser le lidar, même chose : il faut chercher "Visualize Lidar", puis rafraîchir dans la barre de droite le topic pris pour référence. Il y a aussi un plugin pour visualiser la vidéo de la caméra

![plugins_gazebo](/img/plugins_gazebo.png?raw=true "Plugins Gazebo")

Pour visualiser les données des capteurs dans Rviz, appuyer sur Add dans la barre de gauche, sélectionner l'onglet "By topic" pour avoir la liste des topic disponibles, puis en sélectionner un. Les points du lidar sont petits : augmenter leur taille au besoin.

#### rover_base_loc_launch.py

Ce launch ajoute un noeud au précédent:
-ekf_node : issu du package robot_localization, il utilise les données de l'IMU et du lidar pour estimer sa position : il opère une fusion de données en utilisant un algorithme implémentant un "Filtre de Kalman". La configuration du filtre est faite dans le fichier ekf.yaml (dossier config)

Voir la page [wikipedia](https://fr.wikipedia.org/wiki/Filtre_de_Kalman) du filtre de Kalman

#### rover_base_gazebo_slam_launch.py

Ce launch ajoute deux nouveaux noeuds :
- slam_node : issu du package slam_toolbox, il utilise les données du lidar pour appliquer le SLAM : un algorithme qui consiste à cartographier une zone tout en se repérant à l'intérieur. La carte est publiée sur le topic /map et visualisable sur Rviz
- nav2_node : on lance ici un launch du package nav2 pour obtenir entre autres des costmap (cartes de coût) qui sont elles aussi visualisables sur Rviz (/global_costmap et /local_costmap)

Voir la page [wikipedia](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) du SLAM

Voir les concepts de costmap (entre autres) [ici](https://navigation.ros.org/concepts/index.html)

## Pour aller plus loin

Un autre package qui semble intéressant mais qui reste à explorer : [ros2_control](https://control.ros.org/master/index.html)

Avoir une carte, c'est bien, mais savoir y planifier un chemin, c'est mieux : voir l'algorithme de Dijkstra et toutes ses variantes (aussi appelés algorithmes de pathfinding) : 
- page [wiki](https://fr.wikipedia.org/wiki/Algorithme_de_Dijkstra)
- page [concept](https://control.ros.org/master/index.html) nav2 (voir section Planner, Controller, Smoother and Recovery Servers)

Pour qu'un robot puisse prendre des décisions, un "behaviour tree" (ou arbre de comportement) semble plus que nécessaire : encore une fois, voir la page [concept]((https://control.ros.org/master/index.html)) de nav2.

Et si on veut faire de la reconnaissance d'images? Quelques bibliothèques python utiles : 
- [OpenCV](https://docs.opencv.org/4.8.0/) (masques, reconnaissance de contours...)
- [Tensorflow](https://www.tensorflow.org/tutorials?hl=fr) (deep learning)
- [Pytorch](https://pytorch.org/tutorials/) (deep learning)



# AutoArchitect+

🧠 Contexte et motivation

Le plan de construction utilisé dans ce projet est généré automatiquement à l’aide d’une intelligence artificielle. L’objectif initial était de confier ce plan à un robot équipé d’un bras mécanique afin de construire une maison en simulation à l’aide de briques.

Cependant, la configuration complète de l’environnement de simulation (robot, bras manipulateur, briques et contraintes physiques) s’est révélée trop complexe à stabiliser dans le temps imparti, rendant impossible le lancement fiable du robot constructeur.

Afin de ne pas compromettre l’objectif principal du projet, une approche alternative a été adoptée. Un TurtleBot mobile, sans bras mécanique, a été utilisé pour évaluer la capacité du robot à lire et interpréter le plan généré par l’IA.

🏗️ Construction symbolique

Dans cette approche, le TurtleBot ne construit pas physiquement la maison. Il reproduit le plan de construction en traçant les murs lors de ses déplacements. Les traces laissées au sol dans RViz représentent symboliquement les briques et les murs, feignant ainsi un processus de construction réel.

Cette construction en deux dimensions permet de valider la compréhension du plan par le robot, tout en conservant une forte cohérence conceptuelle avec l’objectif initial du projet.

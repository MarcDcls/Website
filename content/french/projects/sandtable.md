+++
bg_image = "images/banners/banner_sandtable.webp"
image = "images/projects/sandtable/SandTableNoBackground.png"
title = "Sand Table"
date = "2021-05-20"
description = "Ce projet a pour but la création d'une table basse dont la partie supérieure transparente laisse voir une fine couche de sable sur laquelle une bille trace des courbes géométriques. La conception et la construction du dispositif caché permettant l'entraînement de la bille en acier par un aimant est au centre du travail effectué."

link = "projects/sandtable/"

+++

# Introduction

Le but de ce projet est de construire une table basse contenant un dispositif permettant le tracé de formes géométriques dans le sable à l'aide d'une bille en acier. Cette sand table est inspirée de plusieurs projets similaires tels que <a href="https://www.instructables.com/Sand-Table/" target="_blank"> celui-ci </a> ou en encore <a href="https://sisyphus-industries.com/" target="_blank"> celui-là </a>.

Le centre du travail à fournir est d'une part la conception du dispositif de déplacement de l'aimant permettant l'entraînement de la bille, puis sa construction et son intégration à une table basse construite sur mesure. Il est aussi nécessaire de réaliser l'électronique et l'algorithmique associées.

# Conception du mécanisme

Le bac à sable de la table doit recouvrir la majorité de sa surface et la bille doit pouvoir en atteindre tout point. Pour ces 2 raisons, ainsi que par choix esthétique, une forme circulaire a été adoptée pour la table. Elle est composée de 3 parties distinctes :
- **Le bac à sable :** Il s'agit de la partie supérieure de la table, visible du dessus grace à un plateau en PMMA transparent. Il est constitué d'un bac contenant du sable et une bille en acier. 

- **La partie intermédiaire :** Partie centrale de la table, elle contient le mécanisme d'entraînement de la bille. Elle n'est pas visible de l'utilisateur.

- **La partie inférieure :** Partie qui contient l'eléctronique et les moteurs. Elle est également invisible de l'utilisateur.

Le système d'entraînement (partie intermédiaire) a été conçu à l'aide d'une tige filetée en rotation autour du centre de la table permettant la translation de l'aimant le long de son rayon. Une première version de ce mécanisme est composée de 2 moteurs M1 et M2 (nécessaires au 2 degrés de liberté souhaités) placés conformément au schéma suivant, où A représente l'aimant et R une roue libre.

<center>
<img src="/images/projects/sandtable/schema_axe.webp" alt="Image not found !" width="80%"/>
</center>

&nbsp;

Cette première idée comporte néanmoins un problème majeur : la non gestion de l'entortillement des fils nécessaires à l'alimentation et au contrôle de M2. La solution retenue pour résoudre ce problème est le déplacement de M2 sur la partie inférieure de la table, le rendant ainsi solidaire du chassis. Cette décision implique le déport de M1 de l'axe central de la table afin de pouvoir y placer M2, qui à l'aide d'un système de transmission constitué d'engrenages coniques peut assurer la translation radiale de l'aimant. Le déport de M1 de l'axe central de la table est réalisé grâce à un système pignon/courroie. L'ensemble des pièces nécessaires à la construction de ce mécanisme sont modélisées sur Onshape, comme illustré ci-dessous.

<center>
<img src="/images/projects/sandtable/CAD_partial.png" alt="Image not found !" width="75%"/>
<img src="/images/projects/sandtable/bewel_gear_front.png" alt="Image not found !" width="20%"/>
</center>
&nbsp;

# Fabrication

Le premier composant à être assemblé est le bras de l'aimant, qui est constitué de pièces en PMMA découpées à la découpeuse laser, d'une tige filetée de 8mm de diamètre sur laquelle est monté un écrou en laiton et d'une tige en acier de 8mm de diamètre. Les tiges, l'écrou et les roulements sont récupérés sur une imprimante 3D démontée. Le bras monté est présenté ci-dessous.

<center>
<img src="/images/projects/sandtable/shaft.jpg" alt="Image not found !" width="80%"/>
</center>

&nbsp;

La structure de la table basse est réalisée en MDF conformément aux schémas présentés plus haut. Les parties courbes de la table, à savoir le pourtour de la table et du bac à sable, sont également réalisées avec du MDF en utilisant une technique de kerf cutting. Cette technique consiste à découper des fentes dans le MDF afin de lui permettre de supporter une courbure. Le MDF est ensuite collé à l'aide de colle à bois et maintenu en place par des serre-joints pendant le séchage.

Les parties intermédiaires et inférieures de la table sont fixées l'une à l'autre avec la possibilité d'accès à la partie inférieure par le côté de la table. Pour la fixation de la partie supérieure (bac à sable) à la partie intermédiaire, un système de loquet a été mis en place. Ce système permet de maintenir le bac à sable en place tout en permettant un accès facile à la partie intermédiaire pour l'entretien et les réparations de la mécanique. Les loquets, présentés ci-dessous, s'emboitent en réalisant une rotation de quelques degrés. 

<center>
<img src="/images/projects/sandtable/lock_m.jpg" alt="Image not found !" width="39%" style="margin-right: 2%;"/>

<img src="/images/projects/sandtable/lock_f.jpg" alt="Image not found !" width="39%"/>
</center>
&nbsp;

Les différentes étapes de la contruction de la table jusqu'à son assemblage final sont présentées ci-dessous. Notez qu'un tendeur de courroie a été ajouté pour assurer un bon fonctionnement de la transmission du moteur M1.

<center>
<img src="/images/projects/sandtable/timing_belt.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/inside.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/open_1.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/open_2.jpg" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<center>
<img src="/images/projects/sandtable/finished.jpg" alt="Image not found !" width="80%"/>
</center>

&nbsp;

# Contrôle

Afin de permettre la calibration de la position de l'aimant, un capteur de fin de course a d'abord été mis en place. Cependant, la position de ce capteur sur la partie rotative du mécanisme a posé problème du point de vue de l'entortillement des fils, comme discuté plus haut. Pour cette raison, il a été remplacé par un Interrupteur à Lame Souple (ILS), qui permet de détecter le passage de l'aimant. Ainsi, en plaçant ce capteur sur l'axe central de la table, solidaire du chassis, il est possible de détecter le passage de l'aimant au centre de la table sans que le capteur ne soit sur une partie en mouvement du mécanisme.
 
La carte de contrôle utilisée est une Arduino Uno, qui est reliée à un shield de contrôle de moteurs pas à pas. Ce shield permet de contrôler les deux moteurs pas à pas M1 et M2, qui sont utilisés pour déplacer l'aimant sur le sable. Le code de contrôle est écrit en C++ et utilise la bibliothèque AFMotor pour contrôler les moteurs.

Deux motifs ont été implémentés. La fonction `spiral` permet de dessiner une spirale en faisant tourner les deux moteurs à des vitesses différentes, tandis que la fonction `zigzag` permet de dessiner des lignes ondulées en faisant tourner le moteur M1 à une vitesse constante et en déplaçant le moteur M2 à des intervalles réguliers. Le code de contrôle est présenté ci-dessous.

```
#include <AFMotor.h>
#include <time.h>

#define STEPSPERREVOLUTION 200.0
#define MAXREVARM 150.0
#define CENTERREDUCTION 2.8

// Connect motors to port #1 (M1 and M2)
AF_Stepper stepperArm(STEPSPERREVOLUTION, 2);
AF_Stepper stepperCenter(STEPSPERREVOLUTION, 1);

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper test!");

  stepperArm.setSpeed(100);  // 100 rpm
  stepperCenter.setSpeed(30);  // 30 rpm
}

void spiral(int revNum) {
  double stepsArm = STEPSPERREVOLUTION*MAXREVARM; 
  double stepsCenter = STEPSPERREVOLUTION*CENTERREDUCTION*revNum;
  AF_Stepper stepperMax = stepperArm;
  AF_Stepper stepperMin = stepperCenter;
  double nMax = stepsArm;
  double nMin = (stepsCenter < stepsArm) ? stepsCenter : stepsArm; //nombre de steps min
  if (nMin == stepsArm){
    stepperMax = stepperCenter;
    stepperMin = stepperArm;
    nMax = stepsCenter;
  }
  int stepPerLoop = (int) round(nMax/nMin);
  Serial.println(stepPerLoop);
  Serial.println(nMax);
  Serial.println(nMin);
  for (int i = 0; i < nMin; i ++){
    stepperMin.step(1, FORWARD, SINGLE);
    stepperMax.step(stepPerLoop, FORWARD, SINGLE);
  }
}

void zigzag(int revNum, int spacing, int amplitude){
  double stepsCenter = STEPSPERREVOLUTION*CENTERREDUCTION*revNum;
  bool direction = 0;
  for (int i = 0; i < stepsCenter; i ++){
    stepperCenter.step(1, FORWARD, SINGLE);
    if (i % spacing == 0){
      direction = !direction;
    }
    if (direction){
      stepperArm.step(amplitude, FORWARD, SINGLE);
    }
    else{
      stepperArm.step(amplitude, FORWARD, SINGLE);
    }
  }
}

void loop() {
  spiral(1);
}
```

&nbsp;

Une démonstration du fonctionnement de la table est présentée ci-dessous. La vidéo montre le fonctionnement de la table avec un motif circulaire.

<center>
<video width="80%" controls>
  <source src="/images/projects/sandtable/sandtable.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
</center>

&nbsp;

# Pistes d’amélioraton

Les principaux défauts de la version finale du projet sont le bruit (lié aux vibrations, roulements multiples et mécanismes plastiques), et l’entraînement inconsistant de la bille (lorsque la couche de sable est trop épaisse ou que les mouvements sont trop brusques). Afin de les supprimer, les pistes d’améliorations sont les suivantes :

**Vibrations et bruit :** Meilleur contrôle des steppers, plus grande réduction centrale, utilisation d’engrenages usinés et non imprimés.

**Entraînement de la bille :** Aimant plus puissant, utilisation d'un sable plus lisse (sable de plage, sans irrégularités)

Il serait également souhaitable de définir plus de modes de dessin (rosaces, formes simples, etc.) ainsi que d’avoir une alimentation dédiée pour l'Arduino et son shield en vu d’en faire un objet autonome et à l’utilisation simplifiée.
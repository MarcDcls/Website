+++
bg_image = "images/banners/banner_sandtable.webp"
image = "images/projects/sandtable/SandTableNoBackground.png"
title = "Sand Table"
date = "2021-05-20"
description = "Ce projet a pour but la création d'une table basse dont la partie supérieure transparente laisse voir une fine couche de sable sur laquelle une bille trace des courbes géométriques. La conception et la construction du dispositif caché permettant l'entrainement de la bille en acier par un aimant est au centre du travail effectué."
+++



# Introduction


 Sand Table Groupe 1

    De Marc Duclusaud dans la catégorie Maker2021	

Le but de ce projet est de construire une « sand drawing table ». Nous prendrons de l’inspiration sur le travail de l’entreprise Sisyphus qui vend ce genre de table.

 

Lien vers une vidéo de la table finale : https://drive.google.com/file/d/1f2rt9aPrNuJFtPfoBBeAOd0koTvqRyuu/view?usp=sharing

Le principe est simple : une bille guidée par un aimant se déplace sur le sable créant des dessins.

Pour plus d’information sur la presentation du projet : https://docs.google.com/presentation/d/1pck66LWqkjd7cfp0bzTarUT_7bWldCSIgwggRyMb7JY/edit?usp=sharing

Point d’avancement à mi-parcours : https://docs.google.com/presentation/d/1Eqf4eWZo3C32ohfp6-pZT9416RJLI0T7MQQeR5BHg9E/edit?usp=sharing

Soutenance : https://docs.google.com/presentation/d/1yPLmFd40Nh6SfzoZcMZA2ZwRSW8NaPOQ-JQ3lznqQBg/edit?usp=sharing

Decou / Duclusaud
Problématiques initiales et solutions
Deplacement de la bille : principe général

Le problème majeur dans ce projet est le déplacement de la bille. Nous avons opté pour un mode de guidée polaire. Le schéma ci dessous illustre ce fonctionnement :
Déplacement de la bille : le bras

Pour que l’aimant puisse se déplacer le long du bras nous utilisons un système de tige filetée. Le schéma ci dessous permet de mieux comprendre ce système.

Nous avons également modélisé une partie du mécanisme sur Onshape afin de réaliser des prototypes à la découpeuse laser :
Entortillement des fils

Puisque le bras peut tourner et qu’un moteur en est solidaire un problème d’entortillement des fils de ce moteur se pose. Pour régler ce problème nous utiliserons un « collecteur tournant ». Un systeme prévu justement pour ce type de problème et qui permet une continuité des fils entre une partie tournante et une partie fixe.
Evolution de la solution

Après réflexions plus poussées quant à la gestion de l’entortillement du fil, nous avons exploré plusieurs solutions telles que l’utilisation d’un moteur pas à pas à arbre creux par exemple. La solution retenue s’attaque à la source du problème et sera d’une part l’emploi d’un système de « bewel gear » permettant de transmettre la rotation du moteur M2 d’un axe vertical à un axe horizontal, ainsi que l’emploi d’une courroie afin de décaler la position du moteur M1 de l’axe principal. Ainsi, nous obtenons une configuration permettant à nos deux moteurs d’être sur la partie fixe de notre table, et donc qui supprime la question d’un entortillement des fils.
Problèmes rencontrés après la réalisation des premiers prototypes 
Problème	Solution
Courroie mal tendue	Tendeur de courroie
Switch de fin de course en mouvement (entortillement)	Remplacé par ILS fixe
Support du plateau de sable s’affesse au milieu	Support central avec bille folle en dessous
Maintien du plateau de sable peut s’enlever trop facilement	Mise en place de loquets
Mouvement circulaire du bras entraine radialement l’aimant	Correction software (pas implémenté au rendu)
Illustration des solutions mises en place

Tendeur de courroie :


ILS :
Permet de détecter lorsque l’aimant est au centre de la table sans que le capteur ne soit sur une partie en mouvement du mécanisme (l’ILS est placé dans le socle du bac de sable)


Support central :


Loquets :
Emboîtement du bac à sable (bloc supérieur) et de la partie mécanique/électronique (bloc inférieur) par rotation du bloc supérieur par rapport au bloc inférieur
 
Réalisation de la table

Modélisation de l’ensemble de la table sur Onshape :


Construction :

Pistes d’amélioraton

Les principaux défauts de la version finale de notre projet sont le bruit (lié aux vibrations, roulements multiples et mécanismes plastiques), et l’entraînement inconsistant de la bille (lorsque la couche de sable est trop épaisse ou que les mouvements sont trop brusques). Afin de les supprimer, nos pistes d’améliorations sont les suivantes :

Vibrations et bruit:

    Un meilleur contrôle des steppers
    Plus grande réduction centrale
    Utilisation d’engrenages usinés


Entraînement de la bille :

    Aimant plus puissant
    Sable plus lisse (de plage, sans irrégularités)


Nous pensons également qu’il serait souhaitable de définir plus de modes de dessin (rosaces, formes simples, etc.) ainsi que d’avoir une alimentation dédiée pour notre Arduino et son shield en vu d’en faire un objet autonome et à l’utilisation simplifiée.
Software

Le soft tel qu’il était à la fin du projet permet de dessiner des spirales avec un nombre de tours paramétrables et des cercles ondulés sur lesquels la fréquence et la hauteur des dents est paramétrable. Il était exécuté par une Arduino UNO et ne gérait pas l’ILS (et donc le calibrage) par manque de temps, bien que l’électronique nécessaire soit présente et reliée sur notre version finale.fichier de controle sandtable

```
#include <AFMotor.h>
#include <time.h>

#define STEPSPERREVOLUTION 200.0
#define MAXREVARM 150.0
#define CENTERREDUCTION 2.8

// connect motor to port #1 (M1 and M2)
AF_Stepper stepperArm(STEPSPERREVOLUTION, 2);
AF_Stepper stepperCenter(STEPSPERREVOLUTION, 1);

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper test!");

  stepperArm.setSpeed(100);  // 100 rpm
  stepperCenter.setSpeed(30);  // 30 rpm
}

void spirale(int revNum) {
  double stepsArm = STEPSPERREVOLUTION*MAXREVARM; //nb step pour aller de 0 au bout du bras
  double stepsCenter = STEPSPERREVOLUTION*CENTERREDUCTION*revNum; //nb step pour faire revNum tours
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
  double stepsCenter = STEPSPERREVOLUTION*CENTERREDUCTION*revNum; //nb step pour faire revNum tours
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
  //spirale(1);
  stepperArm.step(10, FORWARD, SINGLE);
  stepperCenter.step(2, FORWARD, SINGLE);
}
```

Matériel utilisé
Nom	Commentaire
2 moteurs pas à pas	Récupérés sur imprimante 3D
Controleur	Arduino Uno
Sable	Fin
Bille	Bille de flipper
Plexiglass	PMMA 15mm
Bois	MDF (3-5-6-10mm)
Axe filetée	Récupéré sur imprimante 3D
Billes folles	Récupérées à eirlab
Aimant	Trouvable à eirlab
ILS	Trouvable à eirlab
Système de pignon/courroie	Pignon de 30 dents sur le moteur, 84 en bout de courroie, courroie de 420mm
Avancement
20 mars 2021

    Imprimante 3D démontée, pièce récupérées.
    Système de déplacement sur le bras monté et fonctionnel (pièces découpées à la laser).
    Nouveaux plans pour associé moteur central et bras
    Carte flashé avec un premier prototype logiciel : plateforme du bras se replace au debut (utilisation d’un microrupteur pour detecter quand s’arreter), les deux moteurs (central et bras) tournent en même temps à une vitesse constante => dessin d’une spirale.
    Premier tests sur écran LCD (knob de control semble ne pas fonctionner)
    Découpe de nouveaux prototypes corrigés à la laser

22 mars 2021

    Questionnement sur la mise en place du collecteur tournant amenant à de nouveaux plans
    Modélisation des bewel gears sur Onshape
    Recherche d’équations pour dessins, roses semble une bonne idée : https://mathworld.wolfram.com/Rose.html et https://en.wikipedia.org/wiki/Rose_(mathematics)

23 mars 2021

    Impression à la 3D des bewel gears
    Mise a jours des plans de l’axe fileté pour intégrer ce mécanisme
    Commande de courroie/pignon
    Usinage des axes liés aux bewel gears

24 mars 2021

    Impression des pièces 3D de la nouvelle structure
    Montage d’une partie du nouveau prototype
    Réflexion sur la rotation non voulue de la tige fileté induite par la rotation de M1
    Réflexion sur les paterns programmables et leur implémentation

31 Mars 2021

    Modélisation du socle et découpe des premiers éléments à la laser

1 Avril 2021

    Réalisation du socle supportant la tige fileté (découpe laser + montage)
    Mise à jour du modèle Onshape

8 Avril 2021

    Peaufinage du diapo de mi-parcours
    Modélisation + Impression de la poulie dentée de l’axe central
    Présentation de l’avancement à mi-parcours : https://docs.google.com/presentation/d/1Eqf4eWZo3C32ohfp6-pZT9416RJLI0T7MQQeR5BHg9E/edit?usp=sharing

Avril 2021 – début Mai 2021

    Découpe, impression et construction des différents étages de la tables
    Tests et calibration des différentes parties mécaniques (tension courroie, axes de rotations sans jeu, etc.)
    Mise en place de l’électronique (Arduino UNO, ILS, steppers)

19 Mai 2021

    Dernieres modifications, en particulier sur le software
    Meca fonctionnelle
    Création diapo de soutenance finale

20 Mai 2021

    Soutenance finale
    Démonstrations aux professeurs + aux autres groupes



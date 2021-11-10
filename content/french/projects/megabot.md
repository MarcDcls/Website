+++
bg_image = "images/banners/banner_megabot.webp"
image = "images/projects/megabot/MegabotNoBackground.png"
title = "Megabot"
date = "2021-10-17"
description = "Contrôle, planification et simulation d'un robot quadrupède à vérins électriques de grande envergure"

+++



# Introduction

## Qu'est-ce que le Megabot ?

Le Megabot est un robot quadrupède de grande envergure pouvant transporter un passager et dont la vocation première est d'être présenté lors d'évènements liés à la robotique. Il pèse plus ou moins 250 kg, mesure environ 2,50 m de largeur et est actionné via des vérins électriques. Sa conception ainsi que sa construction ont été réalisées par Julien Allali, maître de conférence à l'ENSEIRB-MATMECA, et il est actuellement conservé au Fablab de Bordeaux INP.


<center>
<img src="/images/projects/megabot/photo_megabot.webp" alt="Image not found !" width="80%"/>
</center>

&nbsp;

## Mission

L'objectif de ce projet est dans un premier temps de revoir le contrôle du robot en améliorant son modèle cinématique, c'est à dire revoir l'ensemble des algorithmes effectuant la traduction d'un ordre de position en une commande d'élongation de vérins. Après cela, il s'agit d'implémenter un algorithme de marche afin que le robot puisse suivre un déplacement courbe. Enfin, afin de vérifier la validité des algorithmes implémentés indépendemment des contraintes réelles telles que la déformation de la structure sous son poids, il faudra modéliser le Megabot et l'intégrer au simulateur PyBullet.
&nbsp;



# Contrôle

Le contrôle a pour objectif l'établissement de la cinématique inverse du Megabot, c'est à dire l'implémentation d'un ensemble de méthodes permettant de déterminer les élongations des 12 vérins nécessaires à l’obtention d’une position donnée. En général la cinématique inverse peut être calculée par des bibliothèques dédiées en leur fournissant un modèle URDF du robot (par exemple pinocchio en python). Néanmmoins, cette option n'a pas pu être mise en place dans le cas du Megabot pour 2 raisons : 

* Le format URDF construit un arbre dont les sommets sont les différentes pièces du robot et dont les arcs sont les liaisons entre ces pièces (glissière, pivot, etc). La structure du Megabot incorporant des fermetures géométriques, cela impliquait la formation de cycles au sein de l'arbre, ce qui est impossible. 

* Dans l'hypothèse ou nous modélisions le Megabot par un arbre en supprimant ses fermetures géométriques et en simulant un robot actionné par des moteurs aux articulations, nous aurions alors une minimisation de l'erreur dans notre cinématique inverse qui porterait sur les angles de nos articulations et non sur nos élongations de vérins.

Nous avons donc décidé de redéfinir clairement la cinématique du Megabot.



## Cinématique du Megabot

### Présentation de la problématique

Etablir le modèle cinématique direct d'un robot consiste à élaborer un ensemble de méthodes de calcul permettant de déterminer la position du robot à partir de valeurs articulaires d'entrée. Dans notre cas, il s'agit de créer une ou un ensemble de fonctions prenant en entrée des élongations de vérin valides et retournant la position du Megabot. Ce problème est solvable de manière analytique assez simplement.

Etablir le modèle cinématique inverse d'un robot en revanche consiste à faire le travail inverse : on veut obtenir les valeurs articulaires à partir d'une position d'entrée. Ce sens est bien plus complexe que le précédent et ne peut pas être résolu de manière analytique dans bien des cas, dont celui du Megabot.

Une solution classique à ce problème est d'avoir recours à la matrice jacobienne associée au modèle cinématique direct : il s'agit de la matrice des dérivées partielles de notre cinématique directe. En effet, elle permet d'approximer linéairement dans le cas de petites variations la relation entre nos valeurs articulaires et nos valeurs de position dans le sens direct. Une fois cette jacobienne \\(J\\) déterminée, il devient possible de déterminer le modèle cinématique inverse en résolvant un problème d'optimisation quadratique de la forme :

<div>
\begin{equation*}
    \begin{array}{cc}
        \underset{\Delta V}{\textit{minimize}} & \dfrac{1}{2} \Delta V^T (J^TJ) \Delta V - (J \Delta X)^T \Delta V \\
        \\
         \textit{subject to} & G \Delta V \leq h \\
          & A \Delta V = b \\
          & lb \leq \Delta V \leq ub
    \end{array}
\end{equation*}
</div>
&nbsp;

Ce problème, où \\(\Delta V\\) est le vecteur des variations des élongations des vérins à déterminer, \\(J\\) la jacobienne de la cinématique directe et \\(\Delta X\\) le vecteur des variations de position, minimise effectivement l'erreur quadratique sur \\(\Delta X\\) tout en permettant de contraindre \\(\Delta V\\) avec les variables \\(G\\), \\(h\\), \\(A\\), \\(b\\), \\(lb\\) et \\(ub\\). Ces variables de contraintes sont utiles pour contraindre les élongations de vérins à ses limites physiques par exemple.

<!-- Démo -->

<details class="details-demo">
<summary class="summary-demo">

<center>

    Démonstration
    
</center>

</summary>

On cherche à minimiser l'erreur quadratique de position. Pour cela, on considère connus les paramètres d'élongations de vérins et de positions à l'instant \\(k\\) et on cherche à minimiser l'erreur à l'instant \\(k+1\\). Notons :

<div>
\begin{equation*}
    \begin{array}{rcl}
        \overline{X_k} & : & \text{position objectif à l'instant } k \\
        X_k & : & \text{position réelle à l'instant } k \\
        \overline{\Delta X_k} = \overline{X_{k+1}} - X_k & : & \text{variation de la position prévue à l'instant } k \\
        \Delta X_k = X_{k+1} - X_k & : & \text{variation de la position réelle à l'instant } k \\
        \Delta V_k & : & \text{variation de l'élongation des vérins correspondant à } \Delta X_k
    \end{array}
\end{equation*}
</div>
&nbsp;

L'erreur quadratique de position vaut alors :

<div>
\begin{equation*}
    \begin{array}{rcl}
         ||\overline{X_{k+1}} - X_{k+1}||^2 & = & ||\overline{X_{k+1}} - X_{k} - \Delta X_k||^2 \\ 
         \\
         & = & ||\overline{X_{k+1}} - X_{k} - J \Delta V_k||^2 \\
         \\
         & = & CST - 2 \left( \overline{\Delta X_{k}}^T J \Delta V_k \right) + \Delta V_k^T J^T J \Delta V_k \\
    \end{array}
\end{equation*}
</div>
&nbsp;

Minimiser l'erreur quadratique revient alors effectivement à minimiser 

<div>
\begin{equation*}
    \boxed{\dfrac{1}{2} \Delta V_k^T (J^TJ) \Delta V_k - (J \overline{\Delta X_k})^T \Delta V_k}
\end{equation*}
</div>
&nbsp;

</details>
&nbsp;

<!-- Fin démo -->

Cette façon de résoudre la cinématique inverse est un exemple d'application de la méthode des moindres carrés. La résolution de ce problème quadratique est réalisable à l'aide d'un solveur QP tel que \\(\verb|qpsolvers|\\) en Pyhton.

### Calcul de la jacobienne de la cinématique directe

Nous allons a présent élaborer la matrice jacobienne associée à la cinématique directe du Megabot. Nous l'élaborerons tout d'abord relativement à une patte dans son plan avant de la généraliser à l'ensemble des pattes dans le référentiel monde.

#### Cinématique directe dans le plan de la patte

Pour déterminer la jacobienne associée à notre cinématique directe dans le plan de la patte, il est nécessaire de formaliser notre problème. Nous allons calculer successivement les écarts de position des points \\(D\\), \\(E\\), \\(F\\), \\(G\\), \\(H\\), \\(I\\) et \\(J\\) représentés sur le schéma suivant en fonction des variations d'élongation \\(\Delta v_1\\) et \\(\Delta v_2\\) des vérins.

<center>
<img src="/images/projects/megabot/schema_leg.webp" alt="Image not found !" width="80%"/>
</center>
&nbsp;

<!-- Calcul de J2 -->

<details class="details-demo">
<summary class="summary-demo">

<center>

    Calcul des écarts
    
</center>

</summary>

Pour 2 points \\(A\\) et \\(B\\) d'éloignement constant, nous noterons la distance les séparant \\(d_{AB}\\). Le graphe des dépendances des articulations de la patte est le suivant :

<center>
<img src="/images/projects/megabot/dependances.webp" alt="Image not found !" width="60%"/>
</center>
&nbsp;

<p class="p-boxed"></p>

Nous cherchons tout d'abord à déterminer l'écart en \\(D\\). Pour cela, nous nous plaçons dans le triangle \\(ADC\\) où l'élongation \\(v_1\\) du segment \\(CD\\) et la position du point \\(D\\) sont variables. Nous obtenons le système \\(S_{1}\\) suivant :

<div>
\begin{equation*}
S_{1} 
\left \{
\begin{array}{l}
    \left\langle \overrightarrow{AD} \middle| \overrightarrow{AD} \right\rangle - d_{AD}^{\hspace{0.1cm}2} = (x_{D} - x_{A})^{2} + (y_{D} - y_{A})^{2} - d_{AD}^{\hspace{0.1cm}2} = 0 \\
    ~\\
    \left\langle \overrightarrow{CD} \middle| \overrightarrow{CD} \right\rangle - v_{1}^{\hspace{0.1cm} 2} = (x_{D} - x_{C})^{2} + (y_{D} - y_{C})^{2} - v_{1}^{\hspace{0.1cm} 2} = 0 \\
\end{array}
\right.
\end{equation*}
</div>
&nbsp;

Après dérivation par rapport à \\(x_{D}\\), \\(y_{D}\\) et \\(v_1\\) et à condition que \\(A\\), \\(D\\) et \\(C\\) ne soient pas alignés (ce qui est toujours vérifié en raison de la structure de la patte), le système \\(S_{1}\\) nous donne la relation matricielle suivante  :

<div>
\begin{equation*}
    \nabla S_{1_{D, v_1}} \cdot \begin{bmatrix}
    \Delta x_{D}\\
    \Delta y_{D}\\ 
    \Delta v_1\\ \end{bmatrix} = \begin{bmatrix}
    2(x_{D} - x_{A}) & 2(y_{D} - y_{A}) & 0 \\
    2(x_{D} - x_{C}) & 2(y_{D} - y_{C}) & -2v_{1}
    \end{bmatrix} \cdot \begin{bmatrix}\Delta x_{D}\\
    \Delta y_{D}\\ 
    \Delta v_1\\ \end{bmatrix} = 
    \begin{bmatrix}0\\0\end{bmatrix}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \Longrightarrow \hspace{0.5cm}
    \Delta D = \begin{bmatrix}\Delta x_{D}\\
    \Delta y_{D} \end{bmatrix} = \begin{bmatrix} 
    2(x_{D} - x_{A}) & 2(y_{D} - y_{A}) \\
    2(x_{D} - x_{C}) & 2(y_{D} - y_{C}) \end{bmatrix}^{-1} \cdot 
    \begin{bmatrix} 
    0 \\
    2v_{1}
    \end{bmatrix} \cdot \Delta v_{1}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \boxed{
    \Delta D = M_D \cdot \Delta v_{1}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    M_D = \begin{bmatrix} 
    2(x_{D} - x_{A}) & 2(y_{D} - y_{A}) \\
    2(x_{D} - x_{C}) & 2(y_{D} - y_{C}) \end{bmatrix}^{-1} \cdot 
    \begin{bmatrix} 
    0 \\
    2v_{1}
    \end{bmatrix}
    }
\end{equation*}
</div>
&nbsp;

<p class="p-boxed"></p>

Du fait que les points \\(A\\), \\(D\\) et \\(E\\) sont alignés, il est possible de déterminer l'écart de position en \\(E\\) à partir de l'écart de position en \\(D\\) en lui appliquant un simple facteur de proportionnalité.

<div>
\begin{equation*}
    \boxed{\Delta E = M_E \cdot \Delta v_1    
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    M_E = \dfrac{d_{AE}}{d_{AD}} \cdot M_D
    }
\end{equation*}
</div>
&nbsp;

<p class="p-boxed"></p>

Nous  calculons maintenant l'écart en 
\\(F\\) en nous plaçant dans le triangle \\(EBF\\) où les positions de \\(E\\) et \\(F\\) sont variables. Nous obtenons le système \\(S_2\\) suivant :

<div>
\begin{equation*}
S_{2} 
\left \{
\begin{array}{l}
    \left\langle \overrightarrow{EF} \middle| \overrightarrow{EF} \right\rangle - d_{EF}^{\hspace{0.1cm}2} = (x_{F} - x_{E})^{2} + (y_{F} - y_{E})^{2} - d_{EF}^{\hspace{0.1cm}2} = 0 \\
    ~\\
    \left\langle \overrightarrow{BF} \middle| \overrightarrow{BF} \right\rangle - d_{BF}^{\hspace{0.1cm}2} = (x_{F} - x_{B})^{2} + (y_{F} - y_{B})^{2} - d_{BF}^{\hspace{0.1cm}2} = 0 \\
\end{array}
\right.
\end{equation*}
</div>
&nbsp;

Nous le dérivons par rapport à \\(x_{F}\\), \\(y_{F}\\), \\(x_{E}\\) et \\(y_{E}\\), et à condition que \\(B\\), \\(E\\) et \\(F\\) ne soient pas alignés (ce qui est toujours vérifié en raison de la structure de la patte), nous obtenons :

<div>
\begin{equation*}
    \nabla S_{2_{F, E}} 
    \cdot 
    \begin{bmatrix}
    \Delta x_{F} \\
    \Delta y_{F} \\
    \Delta x_{E} \\
    \Delta y_{E}
    \end{bmatrix} 
    = 
    \begin{bmatrix}
    2(x_{F} - x_{E}) & 2(y_{F} - y_{E}) & -2(x_{F} - x_{E}) & -2(y_{F} - y_{E}) \\
    2(x_{F} - x_{B}) & 2(y_{F} - y_{B}) & 0 & 0
    \end{bmatrix} 
    \cdot 
    \begin{bmatrix}
    \Delta x_{F} \\
    \Delta y_{F} \\
    \Delta x_{E} \\
    \Delta y_{E}
    \end{bmatrix} 
    = 
    \begin{bmatrix}
    0\\
    0\\
    0\\
    0\\ 
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \Longrightarrow \hspace{0.5cm}
    \Delta F = \begin{bmatrix}
    \Delta x_{F}\\
    \Delta y_{F} \end{bmatrix} = \begin{bmatrix} 
    2(x_{F} - x_{E}) & 2(y_{F} - y_{E}) \\
    2(x_{F} - x_{B}) & 2(y_{F} - y_{B}) \\
    \end{bmatrix}^{-1} \cdot 
    \begin{bmatrix} 
    2(x_{F} - x_{E}) & 2(y_{F} - y_{E}) \\
    0 & 0
    \end{bmatrix} \cdot \Delta E
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \boxed{\Delta F = M_F \cdot \Delta v_1
    \hspace{0.2cm}
    \text{avec} 
    \hspace{0.2cm}
    M_F = \begin{bmatrix} 
    2(x_{F} - x_{E}) & 2(y_{F} - y_{E}) \\
    2(x_{F} - x_{B}) & 2(y_{F} - y_{B}) \\
    \end{bmatrix}^{-1} \cdot 
    \begin{bmatrix} 
    2(x_{F} - x_{E}) & 2(y_{F} - y_{E}) \\
    0 & 0
    \end{bmatrix} \cdot M_E}
\end{equation*}
</div>
&nbsp;

<p class="p-boxed"></p>

Les points \\(E\\), \\(F\\) et \\(G\\) d'une part et \\(B\\), \\(F\\) et \\(H\\) d'autre part sont alignés. On peut donc obtenir les écarts en \\(G\\) et en \\(H\\) à partir de l'écart en \\(F\\) :

<div>
\begin{equation*}
    \boxed{\Delta G = M_{G} \cdot \Delta v_{1}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    M_G = \dfrac{d_{EG}}{d_{EF}} \cdot M_F}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \boxed{\Delta H = M_{H} \cdot \Delta v_{1}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    M_H = \dfrac{d_{BH}}{d_{BF}} \cdot M_F}
\end{equation*}
</div>
&nbsp;

<p class="p-boxed"></p>

Nous pouvons à présent obtenir l'écart en \\(I\\) en nous plaçant dans le triangle \\(GHI\\) où les positions des points \\(G\\) et \\(H\\) ainsi que l'élongation \\(v_2\\) du segment \\(HI\\) sont variables. Nous obtenons le système \\(S_3\\) suivant :

<div>
\begin{equation*}
S_{3} 
\left \{
\begin{array}{l}
    \left\langle \overrightarrow{GI} \middle| \overrightarrow{GI} \right\rangle - d_{GI}^{\hspace{0.1cm}2} = (x_{I} - x_{G})^{2} + (y_{I} - y_{G})^{2} - d_{GI}^{\hspace{0.1cm}2} = 0 \\
    ~\\
    \left\langle \overrightarrow{HI} \middle| \overrightarrow{HI} \right\rangle - v_{2}^{\hspace{0.1cm} 2} = (x_{I} - x_{H})^{2} + (y_{I} - y_{H})^{2} - v_{2}^{\hspace{0.1cm} 2} = 0 \\
\end{array}
\right.
\end{equation*}
</div>
&nbsp;

Nous le dérivons par rapport à \\(x_{I}\\), \\(y_{I}\\), \\(x_{G}\\), \\(y_{G}\\), \\(x_{H}\\), \\(y_{H}\\) et \\(v_{2}\\) pour obtenir la jacobienne suivante :

<div>
\begin{equation*}
    \nabla S_{3_{I, G, H, v_{2}}} = \begin{bmatrix}
    M_{1} & -M_{2} & -M_{3} & -M_{4} 
    \end{bmatrix}
    \hspace{0.2cm} 
    \text{avec}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    M_1 = \begin{bmatrix}
    2(x_{I} - x_{G}) & 2(y_{I} - y_{G}) \\
    2(x_{I} - x_{H}) & 2(y_{I} - y_{H})
    \end{bmatrix}
    \hspace{2cm} 
    M_2 = \begin{bmatrix}
    2(x_{I} - x_{G}) & 2(y_{I} - y_{G}) \\
    0 & 0
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
\hspace{-3.55cm}
    M_3 = \begin{bmatrix}
    0 & 0 \\
    2(x_{I} - x_{H}) & 2(y_{I} - y_{H})
    \end{bmatrix}
    \hspace{2cm} 
    M_4 = \begin{bmatrix}
    0 \\
    2 v_2
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

On a alors la relation suivante permettant de relier l'écart en \\(I\\) à \\(\Delta v_1\\) et \\(\Delta v_2\\) à la condition que \\(I\\), \\(G\\) et \\(H\\) ne soient pas alignés (ce qui est toujours vérifié en raison de la structure de la patte) :

<div>
\begin{equation*}
    \Delta I =
    M_{1}^{-1} \cdot \left( 
    M_{2} \cdot \Delta G +  
    M_{3} \cdot \Delta H +  
    M_{4} \cdot \Delta v_{2} 
    \right) = 
    M_{1}^{-1} \cdot \left( 
    M_{2} \cdot M_{G} + M_{3} \cdot M_{H} \right) \cdot \Delta v1 +  
    M_{1}^{-1} \cdot M_{4} \cdot \Delta v_{2} 
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \boxed{
    \array{
    \Delta I =
    M_I \cdot
    \begin{bmatrix}
    \Delta v_{1}\\
    \Delta v_{2} \end{bmatrix}
    \\[0.2cm]
    \text{avec}
    \\[0.2cm]
    \hspace{0.2cm}
    M_I = \begin{bmatrix}
    V1 & V2 
    \end{bmatrix}
    \hspace{1cm}
    V_1 = M_{1}^{-1} \cdot \left( 
    M_{2} \cdot M_{G} + M_{3} \cdot M_{H} \right)
    \hspace{1cm}
    V_2 = M_{1}^{-1} \cdot M_{4}
    \hspace{0.2cm}}}
\end{equation*}
</div>
&nbsp;

<p class="p-boxed"></p>

Enfin, du fait que \\(G\\), \\(I\\) et \\(J\\) sont alignés, on obtient l'écart de position en \\(J\\) :

<div>
\begin{equation*}
    \boxed{\Delta J =
    M_J \cdot
    \begin{bmatrix}
    \Delta v_{1}\\
    \Delta v_{2} \end{bmatrix}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    M_J = \dfrac{d_{GJ}}{d_{GI}} \cdot M_I}
\end{equation*}
</div>
&nbsp;

</details>
&nbsp;

<!-- Fin du calcul de J2 -->

Une fois les écarts successifs calculés, nous obtenons la matrice \\(M_J\\) telle que :

<div>
\begin{equation*}
    \Delta J =
    M_J \cdot
    \begin{bmatrix}
    \Delta v_{1}\\
    \Delta v_{2} \end{bmatrix}
\end{equation*}
</div>
&nbsp;

On remarque alors que cette matrice est la jacobienne \\(J_2\\) associée à notre modèle cinématique direct dans le plan de la patte :

<div>
\begin{equation*}
    \boxed{\begin{bmatrix}
    \Delta x\\
    \Delta y \end{bmatrix}
     =
    J_2 \cdot
    \begin{bmatrix}
    \Delta v_{1}\\
    \Delta v_{2} \end{bmatrix}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    J_2 = M_J}
\end{equation*}
</div>
&nbsp;

#### Cinématique directe dans l'espace

L'objectif à présent est de ramener notre modèle dans l'espace en ajoutant l'influence du vérin \\(v_3\\) sur notre cinématique directe. Le référentiel cible est le référentiel \\((\overrightarrow{x}, \overrightarrow{y}, \overrightarrow{z})\\) représenté sur la figure suivante. On notera que le vecteur \\(\overrightarrow{Y}\\) utilisés dans la partie précédente est devenu le vecteur \\(\overrightarrow{Z}\\) dans cette modélisation à des fins de convention.

&nbsp;

<center>
<img src="/images/projects/megabot/chgt_ref.webp" alt="Image not found !" width="80%"/>
</center>

&nbsp;

En premier lieu, nous cherchons à exprimer l'angle \\(\alpha\\) de la patte au châssis en fonction de l'élongation du vérin \\(v_3\\). Pour cela, nous nous plaçons dans le triangle \\(KLO\\).

<center>
<img src="/images/projects/megabot/angle.webp" alt="Image not found !" width="70%"/>
</center>
&nbsp;

En utilisant la loi des cosinus, nous obtenons la relation suivante :

<div>
\begin{equation*}
\begin{array}{lc}
    &d_{OK} \cdot d_{OL} \cdot \cos{\gamma} = \dfrac{1}{2} ( d_{OK}^{\hspace{0.1cm} 2} + d_{OL}^{\hspace{0.1cm} 2} - v_3^{\hspace{0.1cm} 2}) \\~\\
    \implies & \dfrac{d_{OK}}{d_{OL}} +
        \dfrac{d_{OL}}{d_{OK}} -
        \dfrac{v_3}{d_{OK} d_{OL}} - 
        2 \cdot \cos(\gamma) = 0
\end{array}
\end{equation*}
</div>
&nbsp;

Nous construisons ensuite la jacobienne associée, ce qui nous permet d'obtenir la relation matricielle suivante :

<div>
\begin{equation*}
    \begin{bmatrix}
        2 \cdot \sin(\gamma) & \dfrac{- 2 \cdot v_3}{d_{OK}d_{OL}}
    \end{bmatrix} \cdot
    \begin{bmatrix}
        \Delta \gamma \\
        \Delta v_3
    \end{bmatrix} = 
    0
\end{equation*}
</div>
&nbsp;

Du fait que \\(\Delta \alpha\\) soit égal à \\(\Delta \gamma\\), nous obtenons enfin la relation suivante :

<div>
\begin{equation*}
\boxed{
    \Delta \alpha = \dfrac{v_3}{\sin{(\alpha - \beta)} \cdot d_{OK} d_{OL}} \cdot \Delta v_3
    }
\end{equation*}
</div>
&nbsp;

L'obtention de la relation entre \\(\Delta \alpha\\) et \\(\Delta v_3\\) nous permet d'obtenir la matrice liant \\(( \Delta X, \Delta Z, \Delta\alpha )\\) à \\((\Delta v_1, \Delta v_2, \Delta v_3)\\). Pour cela, nous réutilisons \\(J_2\\) obtenue lors de la résolution de la cinématique directe dans le plan de la patte :

<div>
\begin{equation*}
\boxed{
    \begin{bmatrix}
        \Delta X \\ \Delta Z \\ \Delta\alpha 
    \end{bmatrix} = A \cdot 
        \begin{bmatrix}
        \Delta v_{1} \\ \Delta v_{2} \\ \Delta v_{3}
    \end{bmatrix}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    A = \begin{bmatrix}
        J_2 & \begin{matrix}0 \\ 0\end{matrix} \\
        \begin{matrix} 0 & 0\end{matrix} & \dfrac{v_3}{\sin(\alpha - \beta) \cdot d_{OK}d_{OL}}
    \end{bmatrix}}
    \label{A}
\end{equation*}
</div>
&nbsp;

La dernière étape permettant l'obtention de notre jacobienne dans le référentiel \\((\overrightarrow{x}, \overrightarrow{y}, \overrightarrow{z})\\) est d'obtenir la matrice reliant \\((\Delta x, \Delta y, \Delta z)\\) à \\((\Delta X, \Delta Z, \Delta \alpha)\\). Pour cela, nous devons étudier la rotation d'angle \\(\alpha\\) séparant \\((\overrightarrow{X}, \overrightarrow{Y}, \overrightarrow{Z})\\) et \\((\overrightarrow{x}, \overrightarrow{y}, \overrightarrow{z})\\).

<center>
<img src="/images/projects/megabot/rota.webp" alt="Image not found !" width="50%"/>
</center>
&nbsp;

Nous notons tout d'abord que l'inégalité \\(0<\alpha<\dfrac{\pi}{2}\\) est toujours vérifiée de par la structure du châssis. De ce fait, nous nous autorisons l'expression de la rotation d'un angle de \\(\dfrac{\pi}{2} - \alpha\\) sans nous questionner sur le signe de l'angle de rotation.

<div>
\begin{equation*}
    \begin{bmatrix}
        x \\
        y \\
        z 
    \end{bmatrix}
    = R_{\frac{\pi}{2} - \alpha} \cdot 
    \begin{bmatrix}
        X \\
        Y \\
         Z
    \end{bmatrix}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    R_{\frac{\pi}{2} - \alpha} = \begin{bmatrix}
        \cos(\frac{\pi}{2}-\alpha) & -\sin(\frac{\pi}{2}-\alpha) & 0\\
        \sin(\frac{\pi}{2}-\alpha) & \cos(\frac{\pi}{2}-\alpha) & 0\\
        0& 0& 1
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

En dérivant cette relation par rapport à \\(\alpha\\), nous obtenons la relation suivante :

<div>
\begin{equation*}
    \begin{bmatrix}
        \Delta x \\ \Delta y \\ \Delta z 
    \end{bmatrix} = 
    \dfrac{\partial R_{\frac{\pi}{2} - \alpha}}{\partial\alpha}
     \cdot
     \begin{bmatrix}
        X \\
        Y \\
         Z
    \end{bmatrix}
    \cdot 
    \Delta \alpha + 
    R_{\frac{\pi}{2} - \alpha}
    \cdot
    \begin{bmatrix}
        \Delta X \\ \Delta Y \\ \Delta Z
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \text{où}
    \hspace{0.5cm}
    \dfrac{\partial R_{\frac{\pi}{2} - \alpha}}{\partial\alpha} =
    \begin{bmatrix}
        \sin(\frac{\pi}{2}-\alpha) & \cos(\frac{\pi}{2}-\alpha) & 0\\
        -\cos(\frac{\pi}{2}-\alpha) & \sin(\frac{\pi}{2}-\alpha) & 0\\
        0& 0& 0
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

Nous savons que \\(Y\\) et \\(\Delta Y\\) sont constant dans notre modèle et valent \\(Y = 0\\) et \\(\Delta Y = 0\\). De ce fait il nous est possible de simplifier la relation précédente en :

<div>
\begin{equation*}
    \begin{bmatrix}
        \Delta x \\ \Delta y \\ \Delta z 
    \end{bmatrix} = 
    \begin{bmatrix}
        X \cdot \sin(\frac{\pi}{2}-\alpha) \\
        -X \cdot \cos(\frac{\pi}{2}-\alpha) \\
        0
    \end{bmatrix}
    \cdot 
    \Delta \alpha + 
    \begin{bmatrix}
        \cos(\frac{\pi}{2}-\alpha) & 0\\
        \sin(\frac{\pi}{2}-\alpha) & 0\\
        0& 1
    \end{bmatrix}
    \begin{bmatrix}
        \Delta X \\ \Delta Z
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

Ou bien, formulé autrement :

<div>
\begin{equation*}
\boxed{
    \begin{bmatrix}
        \Delta x \\ \Delta y \\ \Delta z 
    \end{bmatrix} = B \cdot
    \begin{bmatrix}
        \Delta X \\ \Delta Z \\ \Delta \alpha
    \end{bmatrix}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    B =     
    \begin{bmatrix}
        \cos(\frac{\pi}{2}-\alpha) & 0 & X \cdot \sin(\frac{\pi}{2}-\alpha) \\
        \sin(\frac{\pi}{2}-\alpha) & 0 & -X \cdot \cos(\frac{\pi}{2}-\alpha)\\
        0 & 1 & 0
    \end{bmatrix}}
    \label{B}
\end{equation*}
</div>
&nbsp;

A partir des matrices \\(A\\) et \\(B\\), il devient possible de construire \\(J_3\\), la matrice reliant \\((\Delta x, \Delta y, \Delta z)\\) à \\((\Delta v_1, \Delta v_2, \Delta v_3)\\) :

<div>
\begin{equation*}
    \boxed{\begin{bmatrix}
    \Delta x\\
    \Delta y \\
    \Delta z
    \end{bmatrix}
     =
    J_3 \cdot
    \begin{bmatrix}
    \Delta v_{1}\\
    \Delta v_{2} \\
    \Delta v_3
    \end{bmatrix}
    \hspace{0.2cm}
    \text{avec}
    \hspace{0.2cm}
    J_3 = B \cdot A
    \hspace{0.2cm}}
\end{equation*}
</div>
&nbsp;

#### Généralisation

Nous avons explicité \\(J_3\\), la jacobienne associée au modèle cinématique direct d'une patte du Megabot dans son référentiel. Afin de pouvoir appliquer un contrôle simultané sur l'ensemble des pattes, il est nécessaire de généraliser notre modèle cinématique à l'ensemble de notre robot.

<center>
<img src="/images/projects/megabot/robot_ref.webp" alt="Image not found !" width="80%"/>
</center>
&nbsp;

Nous noterons à partir de maintenant \\(\Delta V\\) les variations d'élongation des 12 vérins et \\(\Delta X\\) les variations de position de l'extrémité des pattes dans le référentiel \\((\overrightarrow{x}, \overrightarrow{y}, \overrightarrow{z})\\) du robot. La nouvelle matrice de passage entre ces vecteurs sera nommée \\(J_{12}\\) :

<div>
\begin{equation*}    
    \Delta X = J_{12} \cdot \Delta V
    \hspace{1cm}
    \text{avec}
    \hspace{1cm}
    \Delta V =     
    \begin{bmatrix}
    \Delta v_1\\
    \Delta v_2\\
    \Delta v_3\\
    \Delta v_4\\
    \Delta v_5\\
    \Delta v_6\\
    \Delta v_7\\
    \Delta v_8\\
    \Delta v_9\\
    \Delta v_{10}\\
    \Delta v_{11}\\
    \Delta v_{12}
    \end{bmatrix}
    \hspace{0.2cm}
    \text{et}
    \hspace{0.4cm}
    \Delta X =     
    \begin{bmatrix}
    \Delta x_A\\
    \Delta y_A\\
    \Delta z_A\\
    \Delta x_B\\
    \Delta y_B\\
    \Delta z_B\\
    \Delta x_C\\
    \Delta y_C\\
    \Delta z_C\\
    \Delta x_D\\
    \Delta y_D\\
    \Delta z_D\\
    \end{bmatrix}
\end{equation*}
</div>
&nbsp;

On peut alors remarquer qu'en utilisant des matrices de rotation il est facile d'exprimer \\(J_{12}\\) à partir de \\(J_3\\) qui a été calculée dans la partie précédente :

<div>
\begin{equation*}
    \boxed{
    J_{12} = \begin{bmatrix}
    J_3 & 0 & 0 & 0\\
    0 & R_{\frac{\pi}{2}} \cdot J_3 & 0 & 0\\
    0 & 0 & R_{- \frac{\pi}{2}} \cdot J_3 & 0\\
    0 & 0 & 0 & R_{\pi} \cdot J_3
    \end{bmatrix}
    }
\end{equation*}
</div>
&nbsp;

Les jacobiennes \\(J_2\\), \\(J_3\\) et \\(J_{12}\\) calculées permettent toutes d'élaborer des algorithmes de contrôle respectivement d'une patte dans son plan, d'une patte dans son référentiel et de l'ensemble des pattes dans le référentiel du Megabot en utilisant un solveur QP. Néanmoins, la problématique importante de gestion du centre de gravité peut également être traitée dès la phase de contrôle, c'est pourquoi nous allons l'intégrer à nos équations.

### Gestion du centre de gravité

En effet, une problématique importante que nous allons chercher à résoudre est le maintien du projeté du centre de gravité du robot dans son polygone de sustentation, c'est à dire le polygone convexe formé par les points de contact entre le sol et le robot. Dans le cas où le centre de gravité du Megabot viendrait à ne plus être au dessus de cette zone alors il basculerait, ce qui modifierait ses pattes au sol et fausserait toute planification de mouvement.

Nous avons implémenté 2 modes de contrôle principaux, chacun ayant des avantages et inconvénients. Le premier ne fait que contraindre le centre de gravité au dessus du polygone de sustentation par l'ajout de contraintes dans le solveur QP. Le second quant à lui permet de fournir en entrée non pas seulement une trajectoire pour les pattes du robot, mais également pour son centre de gravité.

Dans les 2 cas, il est nécessaire d'élaborer la jacobienne associée au déplacement du centre de gravité du Megabot. Connaissant le poids de chacune des pièces du Megabot, il est possible de déterminer les variations \\(\Delta C\\) du centre de gravité à partir de \\(\Delta V\\) et des jacobiennes du modèle cinématique direct. En effet la position du centre de gravité n'est qu'une combinaison linéaire des positions de chacune des articulations du robot. On obtient alord une relation du type :

<div>
\begin{equation*}
    \boxed{
    \Delta C = J_{C} \cdot \Delta V
    }
\end{equation*}
</div>
&nbsp;

#### Contrainte du centre de gravité

Une fois la jacobienne \\(J_C\\) explicitée, il devient possible d'utiliser les variables \\(G\\) et \\(h\\) du solveur QP afin de contraindre le centre de gravité. En posant \\(G = J_C\\) et en choisissant \\(h\\) en accord avec les contraintes à appliquer au centre de gravité, le solveur garantira alors :

<div>
\begin{equation*}
    G \Delta V = \Delta C \hspace{0.2cm} \leq  \hspace{0.2cm} h
\end{equation*}
</div>
&nbsp;

Notons qu'ici chaques lignes de \\(G\\) et \\(h\\) correspondent à une contrainte sur une coordonnées de centre de gravité. Dans les faits, on souhaite contraindre le projeté du centre de gravité dans le polygone de sustentation, il faut donc construire \\(G\\) de sorte à ce que chacune de ses lignes soit une combinaison linéaire des lignes de \\(J_{C}\\), tandis que \\(h\\) contiendrait les valeurs de contrainte associées. Dans le cas où nous aurions besoin de \\(n\\) contraintes sur les coordonnées du centre de gravité pour le maintenir dans les polygone de sustentation, \\(G\\) et \\(h\\) seraient alors constitués de \\(n\\) lignes.

Cette solution apporte une forme de sécurité tout en laissant au centre de gravité une certaine liberté. Elle semble adaptée à un robot possédant une zone de sustentation constante au cours de son déplacement car, le cas échéant, elle permet de contraindre le centre de gravité à cette zone et de ne plus avoir à s'en préoccuper durant la phase de planification. 

Ce n'est malheureusement pas le cas du Megabot, c'est pourquoi nous avons opté pour la seconde solution qui offre un contrôle exact du centre de gravité mais qui en contrepartie nécessite sa prise en compte dans la planification.

#### Contrôle du centre de gravité

Afin de commander la position du centre de gravité, il est nécessaire de l'intégrer aux entrées du solveur QP. On obtient alors la relation suivante en reprenant les notations précédentes:

<div>
\begin{equation*}
    \boxed{\Delta X' = J_{15:12} \cdot \Delta V
    \hspace{0.5 cm}
    \text{avec}
    \hspace{0.5 cm}
    \Delta X' =     
    \begin{bmatrix}
    \Delta X \\
    \Delta C\\
    \end{bmatrix}
    \hspace{0.5 cm}
    \text{et}
    \hspace{0.5 cm}
    J_{15:12} =    
    \begin{bmatrix}
    J_{12} & J_C 
    \end{bmatrix}}
\end{equation*}
</div>
&nbsp;

La nouvelle jacobienne \\(J_{15:12}\\) ainsi construite permet bien de prendre en compte la position du centre de gravité dans notre cinématique directe, néanmoins la matrice \\(J_{15:12}\\) ne peut pas être exploitée telle quelle par le solveur QP car la matrice \\(J_{15:12}\hspace{0.15cm}^TJ_{15:12}\\) est singulière (cf. démonstration effectuée dans la présentation de la problématique). Une solution pour contourner la singularité de cette matrice, est d'ajouter une d'ajouter un terme de régularisation à notre minimisation.

<!-- Régularisation -->

<details class="details-demo">
<summary class="summary-demo">

<center>

    Ajout d'un terme de régularisation
    
</center>

</summary>

de Tikhonov coefficient \\(r\\) à notre optimisation quadratique

<div>
\begin{equation*}
    \begin{array}{cc}
        \underset{\Delta V_{k+1}}{\textit{minimize}} & ||\overline{X'_{k+1}} - X'_{k+1}||^2 + r ||\Delta V_{k+1}||^2 
    \end{array}
\end{equation*}
</div>
&nbsp;

On l'on note :

<div>
\begin{equation*}
    \begin{array}{rcl}
        \overline{X_k} & : & \text{position objectif à l'instant } k \\
        X_k & : & \text{position réelle à l'instant } k \\
        \overline{\Delta X_k} = \overline{X_{k+1}} - X_k & : & \text{variation de la position prévue à l'instant } k \\
        \Delta X_k = X_{k+1} - X_k & : & \text{variation de la position réelle à l'instant } k \\
        \Delta V_k & : & \text{variation de l'élongation des vérins correspondant à } \Delta X_k
    \end{array}
\end{equation*}
</div>
&nbsp;

L'objectif à minimiser devient alors :

<div>
\begin{equation*}
    \hspace{-0.5cm}
    \begin{array}{rcl}
         ||\overline{X_{k+1}} - X_{k+1}||^2 + r ||\Delta V_{k+1}||^2 & = & \Delta V_k^T (J_{15:12}\hspace{0.15cm}^TJ_{15:12} + r Id_{12})\Delta V_k - 2 \left( \overline{\Delta X_{k}}^T J_{15:12} \Delta V_k \right) + CST
    \end{array}
\end{equation*}
</div>
&nbsp;

<div>
\begin{equation*}
    \Longrightarrow
    \hspace{0.3cm}
    \boxed{
    \begin{array}{cc}
        \underset{\Delta V_{k+1}}{\textit{minimize}} & V_k^T (J_{15:12}\hspace{0.15cm}^TJ_{15:12} + r Id_{12})\Delta V_k - 2 \left( \overline{\Delta X_{k}}^T J_{15:12} \Delta V_k \right)
    \end{array}}
\end{equation*}
</div>
&nbsp;

Ainsi on transforme la matrice singulière $J_{15:12}\hspace{0.15cm}^TJ_{15:12}$ de notre précédente optimisation quadratique en la matrice $J_{15:12}\hspace{0.15cm}^TJ_{15:12} + r Id_{12}$. Cela permet d'en assurer la non-singularité, car il n'y a plus de dépendance entre les lignes de la matrice. Cette régularisation implique également l'homogénisation des variations de vérins, l'homogénisation étant d'autant plus forte que la valeur choisie pour $r$ est élevée.

</details>

<!-- Fin régularisation -->


### Commande sur la position des pattes

Le premier mode de contrôle prend en entrée les nouvelles positions absolues des extrémités des 4 pattes du Megabot et renvoie les variations d'élongation des vérins, de la position du centre du robot $O$ et de son orientation $\Omega$. Il contraint les élongations des vérins dans leurs limites physiques, le centre de gravité au dessus du triangle de sustentation formé par les 3 pattes au sol et permet de contraindre les déplacements et l'inclinaison du centre du robot, ce qui a pour objectif d'améliorer le confort du passager.

L'avantage de ce modèle est son exactitude. Pour une entrée réalisable physiquement, le modèle retourne des élongations de vérins conformes à celles attendues, c'est-à-dire que l'application de la cinématique directe à ces élongations aboutit à notre entrée de position. Cependant, il possède un défaut : contraindre le centre de gravité du robot n'est pas suffisant.

Dans l'optique de la planification qui va suivre la phase de contrôle, si le centre de gravité est laissé libre dans son triangle de sustentation, nous risquons lors du changement de patte en déplacement de nous retrouver dans le cas où le centre de gravité est à l'extérieur du triangle de sustentation dès le début du mouvement, ce qui se traduira par un soulèvement de la patte opposée à celle souhaitée et à une impossibilité pour le solveur de trouver une solution à notre problème d'optimisation. C'est pourquoi un second mode de contrôle a été développé.

### Commande sur les positions de la patte flottante et le centre de gravité

Cette fois-ci les entrées ont été réduites à uniquement la nouvelle position de la patte en déplacement (les 3 autres restant au sol, elles ne doivent pas bouger) et celle du projeté du centre de gravité, tandis que les sorties sont restées inchangées.

Avec ce nouveau modèle, il est devenu possible d'effectuer une commande de position à la fois sur la patte et le centre de gravité, ce qui assure un contrôle du non basculement du robot. Néanmoins, un de ses inconvénients est que dans le cas de déplacements du la patte et du centre de gravité incompatibles, la sortie peut s'écarter de celle attendue.

########################################################################################################



# Planification

La phase de planification consiste en l'élaboration du mouvement indépendemment des réflexions liées au contrôle. Il s'agit de savoir ce que va faire le robot et non comment il va le réaliser et se traduit par l'implémentation d'algorithmes fournissant des positions successives pour les pattes et le centre de gravité du Megabot.

## Elaboration des trajectoires

Nous avions pour objectif de permettre au Megabot un déplacement courbe. De ce fait nous avons tout d'abord cherché à traduire une entrée de direction (2 dimensions) et de rotation (1 dimension) en une trajectoire pour le Megabot. Cette modélisation a pour objectif de pouvoir commander le robot à l'aide d'une manette à 2 joysticks.

La direction permet de définir le vecteur \\(\overrightarrow{d}\\) représenté sur le schéma suivant tandis que la valeur de rotation permet de définir la position du centre de rotation \\(R\\) sur l'axe perpendiculaire à la direction : plus la valeur est proche de 0, moins la courbe est importante et plus \\(R\\) est éloigné, à droite pour une rotation positive et à gauche pour une rotation négative. Les trajectoires des 4 pattes sont ensuite déterminées en traçant les cercles de centre $R$ passant par leurs extrémités.

<center>
<img src="/images/projects/megabot/curve.webp" alt="Image not found !" width="90%"/>
</center>
&nbsp;

Dans le cas d'une direction nulle et d'une rotation, \\(R\\) est placé au centre du Megabot. Dans le cas d'une rotation nulle et d'une direction, les trajectoires des pattes sont simplement des droites parallèles à l'axe de direction.

Une fois les trajectoires des 4 pattes déterminées, nous nous sommes questionnés sur la gestion du centre de gravité du robot. La solution retenue permettant de garantir que le centre de gravité soit constamment au dessus du triangle de sustentation a été de lui faire suivre une trajectoire circulaire autour de l'intersection des diagonales du polygone formé par les extrémités des 4 pattes \\(P_0\\), \\(P_1\\), \\(P_2\\) et \\(P_3\\) durant leur déplacement. Nous noterons ce point \\(C\\) dans la suite des explications. Ce choix nous amène à un découpage de notre marche en 4 phases, précédées d'une phase d'initialisation :

<center>
<img src="/images/projects/megabot/rota_com.webp" alt="Image not found !" width="90%"/>
</center>
&nbsp;

## Algorithme de marche

Une fois les trajectoires des extrémités des 4 pattes et du centre de gravité établies, nous avons pour chacune des phases détaillées sur la figure précédente procédé à une discrétisation du parcours des pattes et du centre de gravité. Le point \\(C\\) est mis à jour à chaque étape de cette discrétisation, ce qui assure un maintien du centre de gravité au dessus du polygone de sustentation.

Durant la phase d'initialisation, les 4 pattes sont fixes au sol et le centre de gravité est déplacé vers l'arrière du Megabot. Ensuite, lors de chacune des phases suivantes, une patte est levée, déplacée le long de sa trajectoire puis abaissée en même temps que le centre de gravité effectue un quart de tour autour de \\(C\\). Cette stratégie implique que les pattes sont déplacées une à une dans le sens anti-horaire. L'algorithme de contrôle implémenté dans la partie Contôle permet de traduire cette commande en élongations de vérins et d'effectuer la marche.
    


# Simulation

Afin de vérifier notre algorithme de marche il est nécessaire de procéder à une simulation du Megabot. En effet, bien que nous ayons effectué des tests unitaires et d'intégration de nos fonctions au cours du développement, tester directement notre marche sur le Megabot réel serait trop précipité, car dans le cas où nous constaterions des écarts entre le résultat et le comportement attendu, rien ne nous garantirait que l'erreur proviendrait bien de notre partie algorithmique et non de notre électronique ou de notre mécanique.

Afin de simuler la physique lors des tests de marche du Megabot, nous avons choisis d'utiliser PyBullet. Ce choix implique tout d'abord la réalisation d'un fichier URDF contenant une modélisation du Megabot.

Le format URDF représente les robots sous forme d'un arbre de pièces reliées par des liaisons (pivot, glissière, etc). Cette modélisation très commune est adaptée actionné par des moteurs aux articulations, mais empêche la formation de cycle et donc toute fermeture géométrique au sein de la structure du robot. Comme l'ensemble des mouvements du Megabot repose sur des boucles cinématiques au niveau des vérins, il a été nécessaire de trouver un moyen de contourner ce problème. La solution trouvée a été de réaliser un modèle cinématique incomplet et de rajouter des contraintes refermant les boucles a posteriori dans PyBullet.

Une première étape a été la modélisation du Megabot sur OnShape, en incluant le maximum de détail possible et en renseignant le poids et les matrices inertielles de chaque pièce. L'ensemble des liaisons entre les pièces ont été construites dans un assemblage, excepté pour celles amenant des fermetures géométriques, puis le modèle OnShape a été converti en URDF en utilisant l'outil \\(\verb|onshape-to-robot|\\).

<center>
<img src="/images/projects/megabot/megabot_onshape.webp" alt="Image not found !" width="90%"/>
</center>
&nbsp;

Une fois l'URDF généré, les contraintes de fermetures géométriques ont été construites via PyBullet, et la simulation de la marche a pu être réalisée.



<!-- 
La comportement du Megabot dans le simulateur est proche de celui attendu, néanmoins il est possible de constater de légers écarts avec le résultat attendu. Cette différence s'explique de par une répartition de masse différente dans le modèle du robot de la partie algorithmique et dans celui simulé, en particulier au niveau des vérins qui sont considérés comme des segments de longueur variable au poids uniformément réparti dans l'algorithmique et comme deux pièces de poids fixes en translation dans la simulation. Une autre piste pouvant expliquer les écarts est également la prise en compte de la dynamique dans le simulateur, là où la cinématique du Megabot le considère comme statique dans l'algorithmique du fait de sa faible vitesse de déplacement. 
-->

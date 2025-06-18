//Définition des entrées et sorties(qui nous avons bien évidemment pris le temps de vérifier)
int moteur_droit_marche_avant = A0;
int moteur_droit_marche_arriere = A1;
int moteur_gauche_marche_arriere = A2;
int moteur_gauche_marche_avant = A3;

int vitesse_moteur_droite = 5;
int vitesse_moteur_gauche = 6;
//Effectivement nous allons pas utiliser les codeurs B sur les roues puisque cela n'est pas nécessaire (comme expliqué dans le compte rendu)
int codeur_moteur_droite_A = 2;
int codeur_moteur_droite_B = 7;
int codeur_moteur_gauche_A = 3;
int codeur_moteur_gauche_B = 8;

//On définit le nombres d'impulsions, pour chaque roue, afin de calculer la vitesse à chaque instant donné
volatile long nbImpulsions_codeur_droite = 0;
volatile long nbImpulsions_codeur_gauche = 0;

//On définit la vitesse PWM initiale à 0
int vitesse_droite_PWM = 0;
int vitesse_gauche_PWM = 0;

//On définit les 2 instants qui vont nous calculer la variation de temps afin de calculer la vitesse
volatile long tf;
volatile long ti;

//On définit le temps qui va nous donner les différentes trajectoires à suivre
volatile long temps_total;

// Variables importantes du robots et sur les paramètres
float diametre_roue_droite = 69;
float diametre_roue_gauche = 65.1;
float nbTops_moteur = 16; //il y a 16 changements d'impulsion sur un tour moteur
float rapport_reduc = 118.58;
float nbsTops_par_trderoue = rapport_reduc * nbTops_moteur; //nombre de tops par tour de roue
float distance_par_top_roue_droite = (3.14*diametre_roue_droite)/ nbsTops_par_trderoue; //distance par top(mm/top), env égale à 0.1076
float distance_par_top_roue_gauche = (3.14*diametre_roue_gauche)/ nbsTops_par_trderoue;

//Vitesse du sujet (imposé) et/ou à modifier pour l'utilisateur
int V1 = 150;
int V2 = 280;
//écartement entre les roues par rapport au centre (en mm)
float ecartement_droite = 70;
float ecartement_gauche = 76; 

//Rayons des trajectoires en mm
int R2 = 150; //zone 2
int R4 = -200; //zone 4
int R7 = 300; //zone 5

//Distance au centre des différentes zones en mm 
//On définit nos variables en entier pour avoir des distances entières
int distance_Z1 = 400;
int distance_Z2 = 150 * 3.14; //un demi périmètre soit un virage de 180°
int distance_Z3 = 93; //on a arrondi notre calcul, de base la distance est de 92.9 mm
int distance_Z4 = (200 * 3.14)/4; //virage à 45°
int distance_Z5 = 100;
int distance_Z6 = 117;
int distance_Z7 = 300*3.14*1.25;  //virage de 225°
int distance_Z8 = 200;

//Temps du parcours découpé en zone en ms 
float temps_zone_1 = distance_Z1/(V1*0.001);
float temps_zone_2 = distance_Z2/(V1*0.001);
float temps_zone_3 = distance_Z3/(V1*0.001);
float temps_zone_4 = distance_Z4/(V1*0.001);
float temps_zone_5 = distance_Z5/(V1*0.001);
float temps_zone_6 = distance_Z6/(V2*0.001);
float temps_zone_7 = distance_Z7/(V2*0.001);
float temps_zone_8 = distance_Z8/(V2*0.001);

//Calculons les vitesses pour chaque zones en mm/s
float vitesse_Z1_roue_droite = V1;
float vitesse_Z1_roue_gauche = V1;
float vitesse_Z2_roue_droite = V1 * ((R2-(ecartement_droite))/R2); 
float vitesse_Z2_roue_gauche = V1 * ((R2+(ecartement_gauche))/R2);
float vitesse_Z3_roue_droite = V1;
float vitesse_Z3_roue_gauche = V1;
float vitesse_Z4_roue_droite = V1 * ((R4-(ecartement_droite))/R4);
float vitesse_Z4_roue_gauche = V1 * ((R4+(ecartement_gauche))/R4);
float vitesse_Z5_roue_droite = V1;
float vitesse_Z5_roue_gauche = V1;
float vitesse_Z6_roue_droite = V2;
float vitesse_Z6_roue_gauche = V2;
float vitesse_Z7_roue_droite = V2 * ((R7-(ecartement_droite))/R7);
float vitesse_Z7_roue_gauche = V2 * ((R7+(ecartement_gauche))/R7);
float vitesse_Z8_roue_droite = V2;
float vitesse_Z8_roue_gauche = V2;

//On définit le temps additionné sur chaque zone, cela nous permettra d'éffectuer la trajectoire avec une variable uniquement
float temps_total_zone_2 = temps_zone_1 +temps_zone_2;
float temps_total_zone_3 = temps_zone_1 + temps_zone_2 + temps_zone_3;
float temps_total_zone_4 = temps_zone_1 + temps_zone_2 + temps_zone_3 + temps_zone_4;
float temps_total_zone_5 = temps_zone_1 + temps_zone_2 + temps_zone_3 + temps_zone_4 + temps_zone_5;
float temps_total_zone_6 = temps_zone_1 + temps_zone_2 + temps_zone_3 + temps_zone_4 + temps_zone_5 + temps_zone_6;
float temps_total_zone_7 = temps_zone_1 + temps_zone_2 + temps_zone_3 + temps_zone_4 + temps_zone_5 + temps_zone_6 + temps_zone_7;
float temps_total_zone_8 = temps_zone_1 + temps_zone_2 + temps_zone_3 + temps_zone_4 + temps_zone_5 + temps_zone_6 + temps_zone_7 + temps_zone_8;

//On définit deux variables auquelles on va calculer la vitesse à chaque instants
float vitesse_roue_droite;
float vitesse_roue_gauche;

//Valeur de nos gains pour l'asservissement PI (ce sont les mêmes valeurs pour les deux roues)
volatile long sommeErreurD = 0;
volatile long sommeErreurG = 0;
float Ki = 0.1;
float Kp = 18;


void setup(){
  //Vitesse de sortie au maximum afin d'avoir une meilleur précision sur le traceur série
  Serial.begin(2000000); 

  //On définit toute les entrées et sorties nécessaires à l'asservissement
  pinMode(vitesse_moteur_droite, INPUT);
  pinMode(moteur_droit_marche_avant, OUTPUT);
  pinMode(moteur_droit_marche_arriere, OUTPUT);

  pinMode(vitesse_moteur_gauche, INPUT);
  pinMode(moteur_gauche_marche_avant, OUTPUT);
  pinMode(moteur_gauche_marche_arriere, OUTPUT);

  pinMode(codeur_moteur_droite_A, INPUT);
  pinMode(codeur_moteur_gauche_A, INPUT);

  //Permet de faire appel à une fonction lorsque la sortie detecte un changement d'état afin de compter le nombres d'impulsions totales
  attachInterrupt(digitalPinToInterrupt(codeur_moteur_droite_A), codeurdroiteA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(codeur_moteur_gauche_A), codeurgaucheA, CHANGE);

  //On commence à l'instant 0 avec un PWM nulle et on définit les sorties des moteurs 
  digitalWrite(moteur_droit_marche_avant, HIGH);
  analogWrite(vitesse_moteur_droite,vitesse_droite_PWM);

  digitalWrite(moteur_gauche_marche_avant, HIGH);
  analogWrite(vitesse_moteur_droite,vitesse_gauche_PWM);


  //On commence par calculer deux instants qui vont nous permettre dans la suite de calculer une variation de temps
  ti = millis();
  tf = millis();
}

void loop () {
  temps_total = millis();

  tf = millis(); //on recalcule un instants final

  //Il s'agit ici d'avoir une variation de temps suffisament grande pour que arduino calcule une vitesse cohérente en étant précis aussi
  //Dans notre cas
  if ((tf - ti) >= 5) {
    float dt = (tf - ti);
    circuit(dt); //On fait appelle à la fonction circuit qui trace la trajectoire
    ti = tf; //dès qu'on rentre dans la boucle, on pose l'instant initial comme le dernier instant final
    afficheur(); //appelle à une fonction qui permet de voir nos valeurs directs
  }
}

void circuit (float dt) {
  //La fonction prend en paramètre la variable dt pour l'envoyer dans les fonctions respectives d'asservissement afin de calculer la vitesse

  //Tracé notre trajectoire
  //La fonction abs sur arduino prend la valeur absolue, permet de facilier la comparaison entre le temps total de la trajectoire et le temps de consigne afin de changer de zone
  if (temps_total <= abs(temps_zone_1)) {
    asservissement_droite(vitesse_Z1_roue_droite, dt);
    asservissement_gauche(vitesse_Z1_roue_gauche, dt);
  }
  else if (temps_total <= abs(temps_total_zone_2)) {
    asservissement_droite(vitesse_Z2_roue_droite, dt);
    asservissement_gauche(vitesse_Z2_roue_gauche, dt);
  }
  else if (temps_total <= abs(temps_total_zone_3)) {
    asservissement_droite(vitesse_Z3_roue_droite, dt);
    asservissement_gauche(vitesse_Z3_roue_gauche, dt);
  }
  else if (temps_total <= abs(temps_total_zone_4)) {
    asservissement_droite(vitesse_Z4_roue_droite, dt);
    asservissement_gauche(vitesse_Z4_roue_gauche, dt);
  }
  else if (temps_total <= abs(temps_total_zone_5)) {
    asservissement_droite(vitesse_Z5_roue_droite, dt);
    asservissement_gauche(vitesse_Z5_roue_gauche, dt);
  }
  else if (temps_total <= abs(temps_total_zone_6)) {
    asservissement_droite(vitesse_Z6_roue_droite, dt);
    asservissement_gauche(vitesse_Z6_roue_gauche, dt);
  }
  else if (temps_total <= abs(temps_total_zone_7)) {
    asservissement_droite(vitesse_Z7_roue_droite, dt);
    asservissement_gauche(vitesse_Z7_roue_gauche, dt);
  }
  else if (temps_total <= abs(temps_total_zone_8)) {
    asservissement_droite(vitesse_Z8_roue_droite, dt);
    asservissement_gauche(vitesse_Z8_roue_gauche, dt);
  }
  else {
    stop();
  }

}

void asservissement_droite (float consigne_droite, float dt) {
  //On calcule la vitesse (en mm/s), le calcule est expliquée sur le comptre rendu
  vitesse_roue_droite = (nbImpulsions_codeur_droite*1000/dt)*distance_par_top_roue_droite;

  //On calcule l'erreur entre la vitesse consigne et la vitesse sortie
  float ErreurD = consigne_droite - vitesse_roue_droite;
  //On calcule notre commande de signal PWM, c'est un asservissement PI
  float commandeD = (Kp*ErreurD) + (Ki*sommeErreurD);
  //On ajoute l'erreur à notre variable, c'est pour la partie Intégrateur de PI
  sommeErreurD+=ErreurD;

  //la commande constrain permet de borner notre entrée, si la commande calculer est supérieur à 255, on renvoie un signal de 255, de même si la commande est inférieur à 0, on envoie un signal PWM de 0
  commandeD = constrain(commandeD,0,255);
  //On envoie notre nouvelle vitesse consigne PWM dans les moteur
  analogWrite(vitesse_moteur_droite,commandeD);
  //On reset le nombres d'impulsions pour pouvoir continuellement calculer notre vitesse grâce aux nombres d'impulsions perçues dans un temps donné
  nbImpulsions_codeur_droite = 0;
}

void asservissement_gauche(float consigne_gauche, float dt) {
  //On calcule la vitesse (en mm/s), le calcule est expliquée sur le comptre rendu
  vitesse_roue_gauche = (nbImpulsions_codeur_gauche*1000/dt)*distance_par_top_roue_gauche;
  //On calcule l'erreur entre la vitesse consigne et la vitesse sortie
  float erreurG = consigne_gauche - vitesse_roue_gauche;
  //On calcule notre commande de signal PWM, c'est un asservissement PI
  float commandeG = (Kp * erreurG)+(Ki*sommeErreurG);
  //On ajoute l'erreur à notre variable, c'est pour la partie Intégrateur de PI
  sommeErreurG += erreurG;

  //la commande constrain permet de borner notre entrée, si la commande calculer est supérieur à 255, on renvoie un signal de 255, de même si la commande est inférieur à 0, on envoie un signal PWM de 0
  commandeG = constrain(commandeG,0,255);
  //On envoie notre nouvelle vitesse consigne PWM dans les moteur
  analogWrite(vitesse_moteur_gauche,commandeG);
  //On reset le nombres d'impulsions pour pouvoir continuellement calculer notre vitesse grâce aux nombres d'impulsions perçues dans un temps donné
  nbImpulsions_codeur_gauche = 0;
}

//Fonction qui affcihe la vitesse des deux roues par rapport au temps
void afficheur () {
  Serial.print(temps_total);
  Serial.print(",");
  Serial.print(vitesse_roue_droite);
  Serial.print(",");
  Serial.println(vitesse_roue_gauche);
}

//Fonctions qui incrémente dès qu'on detecte un changement d'état sur le codeur, à la fois sur la roue droite et la roue gauche, nous l'expliquons sur le compte rendu
void codeurdroiteA () {
  nbImpulsions_codeur_droite++;
}
void codeurgaucheA () {
  nbImpulsions_codeur_gauche++;
}

//Fonction qui arrête le robot, on envoie un PWM de 0 sur les deux roues
void stop () {
  analogWrite(vitesse_moteur_gauche, 0);
  analogWrite(vitesse_moteur_droite, 0);
}
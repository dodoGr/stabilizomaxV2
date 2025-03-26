# Suivi de l'avancement du projet Stabilizomax
    ctrl shift v pour afficher le fichier en même temps 
## Récupération des données
### Entrées

On a besoin de :

#### Plateau
[Code Plateau](plateau.hpp)

- [x] Initialisation des broches Xri, Xle, Yup, Ylo
- [x] récupération des positions
- [x] Conversion des valeurs
+ [ ] faire un filtrage ?

#### Potentiomètres
[Code Potentiometre](potentiometre.hpp)

- [x] Initialisation des broches BDpot, ACpot
- [x] récupération des positions
- [x] Conversion des valeurs
+ [ ] faire un filtrage ?

---

## Fichier "base" qui regroupe tout
[base.hpp](base.hpp)
- [x] Initialisation des broches
- [x] bibliothèques 
- [x] Définition des valeurs fixes

---

## Calcul

- [ ] Vitesse et accélération
- [ ] Calcul des dutyCycles à envoyer aux bobines
- [ ] Réalisation du PID

---

## Envoi des données
### Sorties

Signal PWM 
[Code signalPWM](signalPWM.hpp)
- BobineA, BobineB, BobineC, BobineD

---

## Interface

Site pour afficher, contrôler les trucs (meo)

---

## Échange d'info entre différentes tâches

---
# cdfr2024-programme-pamis

---

# PAMI - Petit Automate Mobile Indépendant

## Description

Ce projet consiste en la programmation des PAMIs pour la CDFR 2024. Il a été réalisé dans le cadre d'un projet FabLab d'un groupe de 5 étudiants SEOC de 2e année. Les PAMIs sont mûs par des moteurs pas à pas NEMA 17, pilotés via des drivers A4988 et la bibliothèque AccelStepper. L'évitement est réalisé avec un capteur Time of Flight (ToF) de STMicroelectronics. 

Le projet a été développé en utilisant l'extension PlatformIO de VSCode, avec le framework Arduino.
## Table des matières

- [Description](#description)
- [Installation](#installation)
- [Utilisation](#utilisation)
- [Fonctionnalités](#fonctionnalités)
- [Licences](#licences)
- [Auteurs](#auteurs)
- [Remerciements](#remerciements)

## Installation

Pour installer le projet , vous devez:

1. **Disposer de VSCode et de l'extension PlatfromIO :**

2. **Cloner ce répertoire :**
  ```bash
  $ git clone https://github.com/robotronik/cdfr2024-programme-pamis.git
  ```

3. **Cloner également le répertoire de la bibliothèque AccelStepper et le placer dasn le réperoitre du projet approprié :**
  ```bash
 $ cd cdfr2024-programme-pamis/lib
 $ git clone https://github.com/waspinator/AccelStepper.git
  ```

## Utilisation

Pour utiliser un pami PAMI, suivez les instructions ci-dessous :

1. **Alimentez le PAMI via le connecteur batterie (batterie 9V ou alimentation DC 9V 2A).**
2. **Téléversez le programme via le port USB. Attention a bien appuyer sur le bouton boot lors du téléversement :**
   ```bash
   Connecting to COM3 ....
   ```

## Fonctionnalités

- **Stratégie :** Le PAMI peut soit réaliser des mouvments de test (rotation ou translation) soit "jouer un match".
- **Evitement :** le PAMI peut soit activement éviter les obstacles qu'il détecte (rotation puis translation) ou attendre qu'il n'y ait plus d'obstacle.
- **Déplacement:** il est possible de modifier les variabales globales liées aux moteurs et à la géométrie du PAMI via le fichier "pami.h"

Le choix des fonctionalités utilisées est définie dans le fichier "define.h"

## Licences

Ce projet est sous licence MIT. Voir le fichier [LICENSE](LICENSE) pour plus de détails.

## Auteurs

- **Daniel BANNISTER** - *Développeur principal* - (https://github.com/Dany-CMB1)
- **Arthur DOUTRELEAU** - *Fonctionalités WiFi et top départ*
- **Jihane ZEMMOURA** - *Zones et stratége générale*
- **Simon BICHEBOIS** - *Evitement*

## Remerciements

- **Robtronik Phelma.**
- **Guillaume DALLE.**

---

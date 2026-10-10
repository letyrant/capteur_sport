# Capteur pour le sport

Capteur d'activité sportive mesurant la **position et l'accélération d'un segment en
mouvement** : jambe d'un joueur de football, dos d'un joueur de rugby — et, à terme,
tout élément rigide en rotation (manivelle de pédalier, rame).

Matériel : Raspberry Pi Pico 2 W, centrale inertielle BNO055, GPS u-blox NEO-6M.
Logiciel : MicroPython.

---

## Les deux modes

|  | Mode foot | Mode rugby |
|---|---|---|
| **Emplacement du capteur** | Mollet / tibia, solidaire de l'os | Dos du T-shirt, au niveau du col |
| **Mesures** | Tirs, Tir max, + tous les indicateurs de course | Indicateurs de course |
| **Cycle détecté** | Foulée (2 pas) : pic de vitesse angulaire de la jambe | Pas : pic d'accélération verticale du tronc |
| **Modèle de distance** | v = K × ω_pic | L = K × (a_max − a_min)^(1/4) — Weinberg |
| **Fréquence** | 125 Hz | 100 Hz |
| **Étalonnage** | `modele_foot.json` | `modele_rugby.json` |

Le GPS est optionnel dans les deux modes et sert de secours à la centrale inertielle.
Il ne peut pas assister la mesure d'une frappe, trop brève et trop locale pour lui.

## Indicateurs produits

Tirs, Tir max, Distance, Activité, Sprint max, Course, Sprints, Accélérations,
Décélérations, vitesse instantanée et position. Les définitions chiffrées
(seuils de course, de sprint, d'accélération) sont figées dans
[`docs/cahier_des_charges.txt`](docs/cahier_des_charges.txt).

---

## Mise en route

### 1. Firmware

Dans Thonny : **Outils → Options → Interpréteur → MicroPython (Raspberry Pi Pico)**,
puis **Installer ou mettre à jour MicroPython**, variante **Pico 2 W**
(Pico branché en maintenant BOOTSEL).

### 2. Pilote BNO055 (dépendance externe)

Les deux fichiers du pilote de Peter Hinch ne sont pas inclus dans ce dépôt.
Les récupérer depuis [micropython-IMU/micropython-bno055](https://github.com/micropython-IMU/micropython-bno055)
(licence MIT) et les placer dans `pico/` :

```
pico/bno055_base.py
pico/bno055.py
```

### 3. Copie sur le Pico

Tous les fichiers de `pico/` vont **à la racine** du Pico (Thonny : panneau
Fichiers, clic droit → *Téléverser vers /*). Ou depuis un terminal :

```sh
mpremote cp pico/*.py :
```

### 4. Étalonnage, une fois par joueur et par mode

Les coefficients K par défaut sont des ordres de grandeur : **sans étalonnage,
les vitesses et distances sont fausses.**

```python
import mode_rugby
mode_rugby.demo_etalonnage("marche", 50)    # 50 m mesurés au décamètre
mode_rugby.demo_etalonnage("course", 100)
mode_rugby.demo_etalonnage("sprint", 40)    # départ lancé
```

Parcourir la distance en ligne droite à l'allure voulue, puis rester immobile 3 s :
la mesure s'arrête seule. Même chose avec `mode_foot` pour le mode foot.

### 5. Une séance

```python
import mode_rugby
mode_rugby.demo_rugby(600)                  # 10 min, sans GPS
mode_rugby.demo_rugby(600, avec_gps=True)   # GPS en secours

import mode_foot
mode_foot.demo_foot(600, longueur_levier_m=0.50)
```

Sur batterie, sans PC : régler les constantes en tête de `pico/main.py`, le
téléverser, et suivre la LED du Pico (clignotement lent = s'équiper, fixe =
ne plus bouger, rapide = mesure en cours, éteinte = terminé).

---

## Câblage

| Module | Broche | Pico 2 W | Broche physique |
|---|---|---|---|
| BNO055 | VIN | 3V3(OUT) | 36 |
| | GND | GND | 38 |
| | SDA | GP4 (I2C0 SDA) | 6 |
| | SCL | GP5 (I2C0 SCL) | 7 |
| GPS | VCC | 3V3(OUT) | 36 |
| | GND | GND | 38 |
| | TX | GP1 (UART0 RX) | 2 |
| | RX | GP0 (UART0 TX) | 1 |

L'entrée ADR du BNO055 reste non connectée (adresse 0x28).

---

## Organisation du dépôt

```
pico/        code embarqué, à copier à la racine du Pico
  capteur_bno055.py   accès au BNO055 : réglages, lectures, acquisition cadencée
  capteur_gps.py      accès au GPS : NMEA générique, qualité, position locale
  deplacement.py      tronc commun : verticale, cycles, vitesse, indicateurs, GPS
  journal.py          horodatage, noms uniques, journaux d'état et d'événements
  mode_foot.py        frappes + course, capteur au mollet
  mode_rugby.py       course, capteur dans le dos
  main.py             programme lancé seul à la mise sous tension (batterie)
docs/        cahier des charges, documentation, plan de tests
outils/      scripts exécutés sur le PC, pas sur le Pico
```

## Fichiers produits par le capteur

Ils restent dans la mémoire du Pico et ne sont pas suivis par git
(voir `.gitignore`). `NNNN` s'incrémente à chaque séance : aucune séance
n'écrase la précédente.

| Fichier | Contenu | Volume |
|---|---|---|
| `seance_<mode>_NNNN.json` | Indicateurs, réécrits toutes les 30 s | < 1 ko |
| `etat_<mode>_NNNN.csv` | Journal d'état, 1 ligne par seconde | ≈ 2,4 ko/min |
| `frappes_NNNN.bin` | Brut autour de chaque frappe (mode foot) | 6 ko par frappe |
| `modele_<mode>.json` | Coefficients K étalonnés | < 1 ko |

Relecture d'un journal de frappes sur le PC :

```sh
python outils/convertir_journal.py frappes_0007.bin
```

---

## Documentation

- [`docs/Documentation_capteur.pdf`](docs/Documentation_capteur.pdf) — architecture,
  principes de mesure, référence des fonctions, paramètres, limites, dépannage.
- [`docs/Plan_de_tests_capteur.pdf`](docs/Plan_de_tests_capteur.pdf) — 9 niveaux de
  tests, du câblage à la validation du cahier des charges.
- [`docs/cahier_des_charges.txt`](docs/cahier_des_charges.txt) — analyse
  fonctionnelle et définitions chiffrées des indicateurs.

## Limites connues

- **Gyroscope plafonné à 2000 °/s** : la vitesse du pied n'est mesurable que
  jusqu'à environ 63 km/h pour un levier de 0,50 m. Au-delà, la valeur est une
  borne inférieure, signalée par `vitesse_minoree`.
- **`v_pied = ω × L` suppose le genou immobile** : en course, la vitesse de
  frappe est sous-estimée de 10 à 25 %.
- **Lecture I2C lente du BNO055** : environ 210 lectures complètes par seconde,
  ce qui limite le mode foot à 125 Hz.
- **Sans GPS, la position dérive** ; les pas chassés et la mêlée sont comptés
  comme des pas vers l'avant.
- **Base fixe C0 et liaison radio non réalisées** : les fonctions FP3, FP4 et
  FP5 du cahier des charges ne sont pas traitées par ce code.

## État des tests

Validés sur matériel : niveaux 0 (préparation), 1 (câblage, hors GPS),
2 (BNO055) et 4 (calculs hors ligne). Les niveaux 3 (GPS), 5, 6, 7 et 8
demandent du terrain et restent à faire.

## Crédits

Pilote BNO055 : [micropython-IMU/micropython-bno055](https://github.com/micropython-IMU/micropython-bno055),
Peter Hinch, licence MIT.

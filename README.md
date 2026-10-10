# Capteur pour le sport

Capteur d'activité sportive mesurant la **position et l'accélération d'un segment en
mouvement** : jambe d'un joueur de football, dos d'un joueur de rugby — et, à terme,
tout élément rigide en rotation (manivelle de pédalier, rame).

Matériel : Raspberry Pi Pico 2 W, centrale inertielle BNO055, GPS u-blox NEO-6M.
Logiciel : MicroPython.

Le système comporte **deux Pico 2 W** :

| | Rôle | Dossier | État |
|---|---|---|---|
| **C1 — capteur** | porté par le joueur : mesure les frappes et le déplacement | [`c1_capteur/`](c1_capteur/) | fonctionnel |
| **C0 — base fixe** | bord de terrain : repère GPS fixe (GPS relatif), affichage OLED, enregistrement | [`c0_base/`](c0_base/) | écran et GPS faits, **liaison C1 → C0 à faire** |

Le module GPS est commun aux deux Pico : [`commun/capteur_gps.py`](commun/capteur_gps.py).

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
(licence MIT) et les placer dans `c1_capteur/` :

```
c1_capteur/bno055_base.py
c1_capteur/bno055.py
```

### 3. Copie sur le Pico C1

Les fichiers de `c1_capteur/` **et** de `commun/` vont **à la racine** du Pico
(Thonny : panneau Fichiers, clic droit → *Téléverser vers /*). Ou depuis un terminal :

```sh
mpremote cp c1_capteur/*.py commun/*.py :
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

Sur batterie, sans PC : régler les constantes en tête de `c1_capteur/main.py`, le
téléverser, et suivre la LED du Pico (clignotement lent = s'équiper, fixe =
ne plus bouger, rapide = mesure en cours, éteinte = terminé).

---

## Base C0

### Mise en route

1. Pilote de l'écran : `ssd1306.py` de micropython-lib (Thonny : **Outils → Gérer
   les paquets → « ssd1306 » → Installer**, ou copier
   `micropython-lib/micropython/drivers/display/ssd1306/ssd1306.py` dans `c0_base/`).
2. Copie à la racine du Pico de la base :

   ```sh
   mpremote cp c0_base/*.py commun/*.py :
   ```

3. Mise sous tension : `main.py` démarre seul. Le GPS de la base règle aussi
   l'horloge du Pico.

Sans C1 ni liaison, `SIMULATION_C1 = True` en tête de `c0_base/main.py` envoie de
faux états à l'écran (et un faux joueur autour de la base dès que le GPS a un fix) :
de quoi tester les pages, la carte et le calibrage.

### Pages de l'écran

| Page | Contenu |
|---|---|
| LIVE | vitesse en grand, distance, tirs (foot) ou pas (rugby) |
| CARTE | terrain vu de la base, position et trace du joueur |
| STATS | état complet reçu de C1 |
| SYSTEME | GPS de la base, liaison, calibrage, enregistrement |
| RESUME | résumé de fin de séance, affiché dès sa réception |

Bouton : appui court = page suivante ; appui long (2 s) sur CARTE = calibrage du
repère terrain (joueur immobile au drapeau de coin, à droite de la base vue du
terrain). Sans bouton (`BOUTON_GPIO = None`), les pages défilent seules.

### Liaison C1 → C0 : emplacement réservé

Le transport n'est pas écrit. Tout est concentré dans `c0_base/reception.py`,
aux endroits marqués **`A FAIRE (liaison)`** :

- `demarrer()` : ouvrir le lien (Wi-Fi point d'accès + UDP, BLE, LoRa…) ;
- `lire()` : lire les messages de C1 sans attendre, puis appeler
  `recevoir_etat(...)` ou `recevoir_resume(...)`, déjà écrits ;
- `LIAISON_REALISEE = True` une fois fait.

Le traitement en aval (GPS relatif, carte, CSV, écran) est prêt. Côté C1, il
faudra envoyer `seance.etat()` et la position GPS brute (`lat_unites`,
`lon_unites`, `heure_utc`) quelques fois par seconde, sans bloquer la mesure.

---

## Câblage

### C1 — capteur

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

### C0 — base

| Module | Broche | Pico 2 W | Broche physique |
|---|---|---|---|
| Écran OLED SSD1306 (Keyestudio KS0271) | VCC | 3V3(OUT) | 36 |
| | GND | GND | 38 |
| | SDA | GP4 (I2C0 SDA) | 6 |
| | SCL | GP5 (I2C0 SCL) | 7 |
| GPS | VCC | 3V3(OUT) | 36 |
| | GND | GND | 38 |
| | TX | GP1 (UART0 RX) | 2 |
| | RX | GP0 (UART0 TX) | 1 |
| Bouton (facultatif) | — | GP15 | 20 |
| | — | GND | 18 |

Écran et GPS partagent la broche 36 (plaque d'essai ou domino).

---

## Organisation du dépôt

```
c1_capteur/  Pico C1, porté par le joueur (à copier avec commun/)
  capteur_bno055.py   accès au BNO055 : réglages, lectures, acquisition cadencée
  deplacement.py      tronc commun : verticale, cycles, vitesse, indicateurs, GPS
  journal.py          horodatage, noms uniques, journaux d'état et d'événements
  mode_foot.py        frappes + course, capteur au mollet
  mode_rugby.py       course, capteur dans le dos
  main.py             programme lancé seul à la mise sous tension (batterie)
c0_base/     Pico C0, base fixe en bord de terrain (à copier avec commun/)
  main.py             boucle principale : GPS, écran, bouton, réception
  ecran.py            écran OLED : pages LIVE, CARTE, STATS, SYSTEME, RESUME
  gps_relatif.py      GPS de la base, position relative, repère terrain
  journal_base.py     enregistrement CSV des états reçus
  reception.py        données reçues de C1 — liaison À FAIRE (emplacement réservé)
commun/      modules utilisés par les deux Pico
  capteur_gps.py      accès au GPS : NMEA générique, qualité, position locale
docs/        cahier des charges, documentation, plan de tests
outils/      scripts exécutés sur le PC, pas sur le Pico
```

## Fichiers produits par le capteur C1

Ils restent dans la mémoire du Pico et ne sont pas suivis par git
(voir `.gitignore`). `NNNN` s'incrémente à chaque séance : aucune séance
n'écrase la précédente.

| Fichier | Contenu | Volume |
|---|---|---|
| `seance_<mode>_NNNN.json` | Indicateurs, réécrits toutes les 30 s | < 1 ko |
| `etat_<mode>_NNNN.csv` | Journal d'état, 1 ligne par seconde | ≈ 2,4 ko/min |
| `frappes_NNNN.bin` | Brut autour de chaque frappe (mode foot) | 6 ko par frappe |
| `modele_<mode>.json` | Coefficients K étalonnés | < 1 ko |

La base C0 écrit `reception_NNN.csv` (un état reçu par ligne), créé au premier
état reçu seulement.

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
- **Liaison C1 → C0 non réalisée** : la base affiche son propre GPS et les
  pages de l'écran, mais ne reçoit encore rien de C1 (voir *Base C0*). Les
  documents PDF de `docs/` ne couvrent que C1 (la base y est notée hors périmètre).

## État des tests

Validés sur matériel : niveaux 0 (préparation), 1 (câblage, hors GPS),
2 (BNO055) et 4 (calculs hors ligne). Les niveaux 3 (GPS), 5, 6, 7 et 8
demandent du terrain et restent à faire.

## Crédits

Pilote BNO055 : [micropython-IMU/micropython-bno055](https://github.com/micropython-IMU/micropython-bno055),
Peter Hinch, licence MIT.

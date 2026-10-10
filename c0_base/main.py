"""
main.py — BASE FIXE C0 (Raspberry Pi Pico 2 W), lancé seul à la mise sous tension
===============================================================================

Rôle de la base (cahier des charges)
    FP3  repère fixe en bord de terrain : GPS de la base -> GPS relatif
    FP4  restitution : états reçus enregistrés (reception_001.csv...)
    FP5  affichage en temps réel sur l'écran OLED

État : la LIAISON C1 -> C0 N'EST PAS ENCORE RÉALISÉE (reception.py, zones
« A FAIRE (liaison) »). En attendant, la base affiche son propre GPS, et
SIMULATION_C1 = True permet de tester l'écran avec de faux états de C1.

Fichiers à copier à la racine du Pico de la base
    c0_base/*.py, commun/*.py et ssd1306.py (voir README)

Câblage
-------
    Écran OLED (SSD1306, I2C0)   VCC -> 3V3(OUT) broche 36   GND -> GND broche 38
                                 SDA -> GP4      broche 6    SCL -> GP5 broche 7
    GPS GY-NEO6MV2 (UART0)       comme sur C1 (commun/capteur_gps.py) :
                                 TX GPS -> GP1 broche 2, RX GPS -> GP0 broche 1,
                                 alimenté aussi sur 3V3(OUT) (partager la broche 36)
    Bouton (facultatif)          entre GP15 (broche 20) et GND (broche 18)
        appui court          page suivante
        appui long (2 s)     sur la page CARTE : calibrage du repère terrain
                             (joueur immobile au drapeau de coin, à droite de la base)
    Sans bouton : BOUTON_GPIO = None, les pages défilent seules.

Sortie : Stop dans Thonny, ou supprimer/renommer main.py sur le Pico.
En cas d'erreur : message à l'écran et détail dans erreur.txt.
"""

import sys
import time

from machine import I2C, Pin

# ----------------------------------------------------------------------------
# Réglages
# ----------------------------------------------------------------------------
SDA_GPIO = 4
SCL_GPIO = 5
BOUTON_GPIO = 15             # None si pas de bouton
DEFILEMENT_AUTO_S = 6        # sans bouton : durée d'affichage d'une page
PERIODE_ECRAN_S = 0.25       # rafraîchissement de l'écran
FREQUENCE_GPS_HZ = 5         # même fréquence que le GPS de C1
CONFIGURATION_UBLOX = True   # False si le GPS n'est pas un u-blox
BASE_X_M = 52.5              # position de la base dans le repère terrain :
BASE_Y_M = -2.0              # milieu de touche, 2 m à l'extérieur
DISTANCE_MIN_CALIBRAGE_M = 20.0
AGE_MAX_C1_S = 3.0           # au-delà sans état reçu : liaison perdue
LONGUEUR_TRACE = 40          # positions gardées pour la trace sur la carte
ENREGISTRER = True           # enregistrement CSV des états reçus
SIMULATION_C1 = False        # True : faux états de C1, pour tester l'écran
MODE_SIMULATION = "foot"     # "foot" ou "rugby"
# ----------------------------------------------------------------------------


def programme():
    from capteur_gps import RecepteurGPS, creer_uart_gps, configurer_ublox
    from ecran import Affichage, Bouton, Ecran, trouver_adresse_oled
    from gps_relatif import GPSRelatif, RepereTerrain
    from journal_base import JournalCSV
    from reception import Reception, SimulateurC1

    i2c = I2C(0, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=400000)
    ecran = Ecran(i2c, trouver_adresse_oled(i2c))
    ecran.message(["BASE C0", "", "Config. GPS..."])

    uart = creer_uart_gps()
    if CONFIGURATION_UBLOX:
        print("Configuration GPS :", configurer_ublox(uart, FREQUENCE_GPS_HZ))
    gps_rel = GPSRelatif(RecepteurGPS(uart))
    repere = RepereTerrain(BASE_X_M, BASE_Y_M, DISTANCE_MIN_CALIBRAGE_M)
    journal = JournalCSV(ENREGISTRER)
    rec = Reception(gps_rel, repere, journal, AGE_MAX_C1_S, LONGUEUR_TRACE)

    etat_liaison = rec.demarrer()          # A FAIRE (liaison) : voir reception.py
    simulateur = SimulateurC1(rec, MODE_SIMULATION) if SIMULATION_C1 else None
    ecran.message(["BASE C0", "", etat_liaison,
                   "SIMULATION C1" if simulateur else "", "", "GPS: attente fix"])
    time.sleep_ms(1500)

    bouton = Bouton(BOUTON_GPIO) if BOUTON_GPIO is not None else None
    affichage = Affichage(ecran, rec, gps_rel, repere, journal, DEFILEMENT_AUTO_S)
    print("Base prete ({})".format(etat_liaison))

    periode_ecran_ms = int(PERIODE_ECRAN_S * 1000)
    t_ecran = time.ticks_ms()
    while True:
        gps_rel.mettre_a_jour()
        if rec.lire() == "resume":         # A FAIRE (liaison) : lire() renvoie None
            affichage.montrer("RESUME")
        if simulateur is not None:
            simulateur.mettre_a_jour()
        if bouton is not None:
            affichage.action(bouton.lire())
        else:
            affichage.defilement_auto()
        if time.ticks_diff(time.ticks_ms(), t_ecran) >= periode_ecran_ms:
            t_ecran = time.ticks_ms()
            affichage.dessiner()
        time.sleep_ms(5)


try:
    programme()
except Exception as e:
    with open("erreur.txt", "w") as f:
        sys.print_exception(e, f)
    sys.print_exception(e)
    try:
        from ecran import Ecran, trouver_adresse_oled
        i2c = I2C(0, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=400000)
        Ecran(i2c, trouver_adresse_oled(i2c)).message(
            ["ERREUR", type(e).__name__, str(e)[:16], str(e)[16:32], "", "voir erreur.txt"])
    except Exception:
        pass

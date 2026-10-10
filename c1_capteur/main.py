"""
main.py — programme lancé automatiquement à la mise sous tension du Pico
(sur batterie, sans PC). Choisir le programme dans les réglages ci-dessous,
puis téléverser ce fichier sur le Pico.

Signaux de la LED du Pico
-------------------------
    clignotement lent (1 s)    temps pour s'équiper (ATTENTE_S secondes) :
                               rugby -> capteur dans le dos du T-shirt, au
                               niveau du col (haut de la colonne) ;
                               foot -> capteur sur le tibia / mollet
    allumée fixe               NE PLUS BOUGER : démarrage et mesure du biais
    clignotement rapide        mesure en cours : partir / jouer
    éteinte                    terminé, résultats enregistrés
    clignotement très rapide   erreur : le détail est dans erreur.txt

Résultats écrits sur le Pico (à récupérer avec Thonny, panneau Fichiers)
    seance_<mode>_NNNN.json   indicateurs de la séance, réécrits toutes les
                              30 s : une coupure ne fait perdre au plus que
                              la dernière demi-minute
    etat_<mode>_NNNN.csv      journal d'état, 1 ligne par seconde
    frappes_NNNN.bin          mode foot : brut autour de chaque frappe
                              (relecture : journal.convertir_csv)
    modele_<mode>.json        étalonnage : K mis à jour
    erreur.txt                trace de la dernière erreur

NNNN est un numéro de séance qui s'incrémente tout seul : aucune séance
n'écrase la précédente. Le Pico n'ayant pas de pile d'horloge, la date du
résumé n'est juste que si le Pico a été relié à Thonny depuis sa dernière
mise sous tension ; le champ "horloge_valide" le signale.

Retour au bureau : avec ce fichier présent, le Pico relance la mesure à
chaque branchement. Cliquer sur Stop dans Thonny pour l'interrompre, ou
supprimer/renommer main.py sur le Pico.
"""

import sys
import time
from machine import Pin, Timer

# ----------------------------------------------------------------------------
# Réglages
# ----------------------------------------------------------------------------
PROGRAMME = "rugby"       # "rugby", "foot", "etalonnage_rugby" ou "etalonnage_foot"
DUREE_S = 600             # durée d'une séance rugby / foot (s)
AVEC_GPS = False          # True si le GPS est branché (séances uniquement)
ALLURE = "course"         # étalonnage : "marche", "course" ou "sprint"
DISTANCE_M = 100          # étalonnage : distance mesurée au décamètre (m)
LONGUEUR_LEVIER_M = 0.50  # MODE FOOT UNIQUEMENT (ignoré en rugby) : distance
                          # genou -> point de contact du pied, pour la vitesse de frappe (m)
ATTENTE_S = 15            # temps pour s'équiper avant la mesure du biais (s)
JOURNAL = True            # journal d'état (1 ligne/s) pendant la séance
JOURNAL_FRAPPES = True    # mode foot : brut enregistré autour de chaque frappe

# ----------------------------------------------------------------------------
led = Pin("LED", Pin.OUT)
minuteur = Timer()


def clignoter(periode_ms):
    """Fait clignoter la LED en tâche de fond."""
    minuteur.init(period=periode_ms, mode=Timer.PERIODIC, callback=lambda t: led.toggle())


def led_fixe(etat):
    """Arrête le clignotement ; LED allumée (1) ou éteinte (0)."""
    minuteur.deinit()
    led.value(etat)


def programme():
    from capteur_bno055 import CapteurBNO055, creer_bus_i2c
    from deplacement import creer_gps, etalonner_sur_distance, executer_seance
    from journal import JournalEtat, JournalEvenements, nom_unique, prochain_numero

    if PROGRAMME not in ("rugby", "foot", "etalonnage_rugby", "etalonnage_foot"):
        raise ValueError("PROGRAMME inconnu : {}".format(PROGRAMME))
    foot = PROGRAMME.endswith("foot")
    etalonnage = PROGRAMME.startswith("etalonnage")
    gps = creer_gps() if (AVEC_GPS and not etalonnage) else None

    # 1. Temps pour s'équiper
    clignoter(1000)
    time.sleep(ATTENTE_S)

    # 2. Ne plus bouger : initialisation et biais du gyroscope
    led_fixe(1)
    capteur = CapteurBNO055(creer_bus_i2c())
    if foot:
        import mode_foot as mode
        capteur.passer_en_mode_foot()
        frequence = mode.FREQUENCE_FOOT_HZ
        modele = mode.ModeleFoulee()
        fichier_modele = mode.FICHIER_MODELE_FOOT
    else:
        import mode_rugby as mode
        capteur.passer_en_mode_rugby()
        frequence = mode.FREQUENCE_RUGBY_HZ
        modele = mode.ModeleWeinberg()
        fichier_modele = mode.FICHIER_MODELE_RUGBY
    capteur.estimer_biais_gyro()
    modele.charger(fichier_modele)
    if foot:
        seance = mode.SeanceFoot(
            modele=modele, gps=gps,
            detecteur_frappe=mode.DetecteurFrappe(longueur_levier_m=LONGUEUR_LEVIER_M))
    else:
        seance = mode.SeanceRugby(modele=modele, gps=gps)

    # 3. Mesure
    clignoter(150)
    if etalonnage:
        etalonner_sur_distance(capteur, seance, frequence, DISTANCE_M, ALLURE, fichier_modele)
    else:
        nom = "foot" if foot else "rugby"
        numero = prochain_numero(nom)
        jetat = JournalEtat(nom_unique("etat_" + nom, "csv", numero)) if JOURNAL else None
        jfrappes = None
        if foot and JOURNAL_FRAPPES:
            jfrappes = JournalEvenements(nom_unique("frappes", "bin", numero),
                                         frequence_hz=frequence)
        executer_seance(capteur, seance, frequence, DUREE_S,
                        nom_resultats=nom_unique("seance_" + nom, "json", numero),
                        journal_etat=jetat, journal_evenements=jfrappes,
                        afficher=False)
        if gps is not None:
            modele.sauvegarder(fichier_modele)   # K ajustés par le GPS

    # 4. Terminé
    led_fixe(0)


try:
    programme()
except Exception as e:
    led_fixe(0)
    with open("erreur.txt", "w") as f:
        sys.print_exception(e, f)
    sys.print_exception(e)
    clignoter(50)

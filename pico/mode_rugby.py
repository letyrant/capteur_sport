"""
mode_rugby.py
=============

Mode RUGBY : BNO055 porté dans le dos, sur le maillot, au niveau du col
(haut de la colonne vertébrale). MicroPython, Raspberry Pi Pico 2 W.

Fichiers nécessaires sur le Pico
    bno055_base.py, bno055.py, capteur_bno055.py, deplacement.py,
    mode_rugby.py, et capteur_gps.py si le GPS est utilisé

Mesures (cahier des charges, FP2)
---------------------------------
    Distance, Activité, Sprint max, Course, Sprints, Accélérations,
    Décélérations, vitesse instantanée, position.
    Tirs et Tir max : non mesurables depuis le dos (voir mode_foot.py).

Principe
--------
Chaque appui au sol crée un pic d'accélération verticale du tronc : un
cycle = un PAS. La longueur du pas suit le modèle de Weinberg (podomètres
portés sur le tronc) :

    L = K[allure] × (a_max - a_min)^(1/4)

avec a_max - a_min l'amplitude de l'accélération verticale pendant le pas,
et un K par allure (marche, course, sprint) selon la cadence (pas/s).
Le traitement commun (verticale, vitesse, position, GPS, indicateurs) est
dans deplacement.py.

Étalonnage (une fois par joueur)
--------------------------------
    demo_etalonnage("marche", 50)
    demo_etalonnage("course", 100)
    demo_etalonnage("sprint", 40)    # départ lancé
Les K sont enregistrés dans modele_rugby.json. Avec le GPS, ils sont aussi
ajustés automatiquement pendant les séances.

Limites
-------
    - Les pas chassés, en arrière et les poussées en mêlée sont comptés
      comme des pas vers l'avant ; les contacts peuvent créer de faux pas.
    - Sans GPS, la position dérive avec le temps.

Exemples
    import mode_rugby
    mode_rugby.demo_rugby(600)                    # sans GPS
    mode_rugby.demo_rugby(600, avec_gps=True)     # GPS en secours
"""

from capteur_bno055 import CapteurBNO055, creer_bus_i2c
from deplacement import (
    DetecteurCycles,
    ModeleCycle,
    SeanceDeplacement,
    afficher_resume,
    creer_gps,
    etalonner_sur_distance,
    executer_seance,
)
from journal import JournalEtat, nom_unique, prochain_numero


FREQUENCE_RUGBY_HZ = 100
FICHIER_MODELE_RUGBY = "modele_rugby.json"


# =============================================================================
# Modèle de longueur de pas
# =============================================================================

class ModeleWeinberg(ModeleCycle):
    """
    L = K[allure] × amplitude^(1/4), amplitude en m/s².

    Allures (cadence en pas/s) : marche < 2,3 <= course < 3,4 <= sprint.
    Pas de départ : course si amplitude >= amplitude_course_ms2.
    Les K par défaut sont des ordres de grandeur : à étalonner.
    """

    TYPE = "rugby_weinberg"

    def __init__(self, k_marche=0.45, k_course=0.60, k_sprint=0.80,
                 cadence_course_hz=2.3, cadence_sprint_hz=3.4,
                 amplitude_course_ms2=15.0, longueur_min_m=0.2,
                 longueur_max_m=2.8, k_min=0.1, k_max=2.0):
        super().__init__({"marche": k_marche, "course": k_course, "sprint": k_sprint},
                         cadence_course_hz, cadence_sprint_hz,
                         longueur_min_m, longueur_max_m, k_min, k_max)
        self.amplitude_course_ms2 = amplitude_course_ms2

    def _allure_depart(self, amplitude_ms2):
        return "course" if amplitude_ms2 >= self.amplitude_course_ms2 else "marche"

    def _distance_brute(self, amplitude_ms2, intervalle_s, allure):
        if amplitude_ms2 <= 0:
            return 0.0
        return self.k[allure] * amplitude_ms2 ** 0.25


# =============================================================================
# Séance rugby
# =============================================================================

def creer_detecteur_pas():
    """
    Détecteur de pas sur l'accélération verticale du tronc :
    passe-bas 8 Hz, seuil >= 1,5 m/s² (30 % de l'enveloppe des pics),
    0,18 s à 1,2 s entre deux pas (cadence max ≈ 5,5 pas/s).
    """
    return DetecteurCycles(frequence_coupure_hz=8.0, seuil_min=1.5,
                           fraction_seuil=0.3, intervalle_min_s=0.18,
                           intervalle_max_s=1.2, caracteristique="amplitude")


class SeanceRugby(SeanceDeplacement):
    """
    Séance rugby. Utilisation (capteur en mode rugby, biais gyro estimé) :

        seance = SeanceRugby()                 # sans GPS
        seance = SeanceRugby(gps=gps)          # GPS en secours
        for ech in capteur.echantillons(100, duree_s):
            seance.traiter(*ech)
        seance.finaliser()
        print(seance.resume())

    Paramètres : modele (ModeleWeinberg), gps, et toutes les options de
    deplacement.SeanceDeplacement (priorite, critères GPS...).
    """

    NOM_CYCLES = "nb_pas"

    def __init__(self, modele=None, gps=None, **options):
        super().__init__(creer_detecteur_pas(),
                         modele if modele is not None else ModeleWeinberg(),
                         gps=gps, **options)

    def _signal(self, a_vert, gx, gy, gz):
        return a_vert


# =============================================================================
# Programmes
# =============================================================================

def _preparer(avec_gps, frequence_gps_hz, configuration_ublox):
    """GPS éventuel, capteur en mode rugby, biais gyro, modèle chargé."""
    gps = creer_gps(frequence_gps_hz, configuration_ublox) if avec_gps else None
    capteur = CapteurBNO055(creer_bus_i2c())
    capteur.passer_en_mode_rugby()
    print("Joueur immobile, debout : mesure du biais gyro (1 s)...")
    capteur.estimer_biais_gyro()
    modele = ModeleWeinberg()
    if modele.charger(FICHIER_MODELE_RUGBY):
        print("Modele de pas charge :", modele.vers_dict())
    else:
        print("ATTENTION : modele de pas non etalonne (demo_etalonnage)")
    return capteur, modele, gps


def demo_rugby(duree_s=600, avec_gps=False, priorite="imu", frequence_gps_hz=5,
               configuration_ublox=True, journal=True, afficher=True):
    """
    Séance rugby.

    Les fichiers produits sont numérotés (seance_rugby_0007.json,
    etat_rugby_0007.csv) : une séance n'écrase jamais la précédente. Le
    résumé est réécrit toutes les 30 s pendant la séance.

    Paramètres
        journal    True pour écrire aussi le journal d'état (1 ligne/s)
        afficher   False pour une séance silencieuse (sur batterie)

    Retour : le résumé de séance.
    """
    capteur, modele, gps = _preparer(avec_gps, frequence_gps_hz, configuration_ublox)
    seance = SeanceRugby(modele=modele, gps=gps, priorite=priorite)
    numero = prochain_numero("rugby")
    nom_resultats = nom_unique("seance_rugby", "json", numero)
    jetat = JournalEtat(nom_unique("etat_rugby", "csv", numero)) if journal else None
    resume = executer_seance(capteur, seance, FREQUENCE_RUGBY_HZ, duree_s,
                             nom_resultats=nom_resultats, journal_etat=jetat,
                             afficher=afficher)
    if afficher:
        afficher_resume(resume)
        print("Resultats : {}".format(nom_resultats))
    if avec_gps:
        modele.sauvegarder(FICHIER_MODELE_RUGBY)
    return resume


def demo_etalonnage(allure, distance_m):
    """Étalonne K[allure] sur distance_m mètres (sans GPS)."""
    capteur, modele, _ = _preparer(False, 0, False)
    seance = SeanceRugby(modele=modele)
    etalonner_sur_distance(capteur, seance, FREQUENCE_RUGBY_HZ, distance_m,
                           allure, FICHIER_MODELE_RUGBY)


if __name__ == "__main__":
    demo_rugby()

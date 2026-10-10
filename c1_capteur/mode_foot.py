"""
mode_foot.py
============

Mode FOOT : BNO055 fixé sur la jambe (mollet / tibia). MicroPython,
Raspberry Pi Pico 2 W. Un seul capteur mesure à la fois :
    - les FRAPPES (FP1)    : nombre de tirs, vitesse du pied, intensité ;
    - la COURSE (FP2)      : distance, vitesse, indicateurs, position.

Fichiers nécessaires sur le Pico
    bno055_base.py, bno055.py, capteur_bno055.py, deplacement.py,
    mode_foot.py, et capteur_gps.py si le GPS est utilisé

Acquisition : 125 Hz. Le BNO055 ne permet qu'environ 210 lectures
complètes par seconde (voir capteur_bno055.py) ; 125 Hz laisse le temps
aux calculs. Une frappe (~0,1 s) est ainsi décrite par une douzaine de
points, assez pour saisir son pic de vitesse angulaire, qui est arrondi.

Placement et fixation
---------------------
La jambe (genou -> cheville) est un solide en rotation autour du genou : la
vitesse angulaire ω est LA MÊME en tout point du tibia. La position du
capteur sur la jambe ne change donc pas ω.
    - Fixer le boîtier solidaire de l'os : face avant/interne du tibia ou
      serrage ferme (sous le protège-tibia). Le muscle du mollet oscille à
      l'impact et ajoute des vibrations parasites.
    - Garder toujours la même position : l'accélération mesurée
      (≈ ω² × distance genou-capteur) en dépend.

1. Frappes : vitesse du pied
----------------------------
    v_pied = ω_max × L     L = distance genou -> point de contact pied/ballon
                           (à mesurer sur le joueur, ≈ 0,45 à 0,55 m adulte)
La distance genou -> capteur n'intervient pas. Le modèle suppose la cuisse
quasi immobile à l'impact : à valider avec une mesure de référence (radar).
LIMITE : le gyroscope sature à 2000 °/s, soit ≈ 63 km/h pour L = 0,50 m
(et ≈ 57 km/h pour L = 0,45 m). Au-delà, la vitesse donnée est une borne
inférieure (drapeau "vitesse_minoree"). Pour dépasser 60 km/h de façon
fiable, il faut un gyroscope ±4000 °/s.
Le GPS ne peut pas assister cette mesure (geste trop bref et trop local).

2. Course : foulées
-------------------
À chaque foulée (2 pas), la jambe balance vers l'avant : pic de vitesse
angulaire. Un cycle = une FOULÉE. Modèle :
    v = K[allure] × ω_pic (rad/s)     distance de la foulée = v × durée
avec un K par allure (marche, course, sprint) selon la cadence (foulées/s).
Le traitement commun (verticale, vitesse, position, GPS, indicateurs) est
dans deplacement.py. Une foulée qui coïncide avec une frappe validée est
écartée (le balancement de la frappe n'est pas une foulée).

Étalonnage de la course (une fois par joueur)
---------------------------------------------
    demo_etalonnage("marche", 50)
    demo_etalonnage("course", 100)
    demo_etalonnage("sprint", 40)    # départ lancé
Les K sont enregistrés dans modele_foot.json. Avec le GPS, ils sont aussi
ajustés automatiquement pendant les séances.

Réglage des seuils de frappe
----------------------------
Les seuils de DetecteurFrappe sont des points de départ. Une foulée de
sprint peut dépasser 1000 °/s et être prise pour une frappe : enregistrer
de vraies frappes ET de vrais sprints (capteur_bno055.demo_enregistrement)
pour régler omega_min_dps et acc_min_g.

Exemples
    import mode_foot
    mode_foot.demo_foot(600)                    # sans GPS
    mode_foot.demo_foot(600, avec_gps=True)     # GPS en secours
"""

from capteur_bno055 import (
    CapteurBNO055,
    creer_bus_i2c,
    norme3,
    dps_vers_rads,
    ms_vers_kmh,
    G,
    ACC_PLAGE_G,
    GYRO_PLAGE_DPS,
)
from deplacement import (
    DetecteurCycles,
    ModeleCycle,
    SeanceDeplacement,
    afficher_resume,
    creer_gps,
    etalonner_sur_distance,
    executer_seance,
)
from journal import JournalEtat, JournalEvenements, nom_unique, prochain_numero


FREQUENCE_FOOT_HZ = 125
DECIMATION_COURSE = 1          # course traitée à chaque échantillon
FICHIER_MODELE_FOOT = "modele_foot.json"

_FRACTION_SATURATION = 0.98


def estimer_vitesse_ballon_kmh(vitesse_pied_kmh, rapport=1.2):
    """
    Estime la vitesse du ballon à partir de la vitesse du pied.
    Le ballon part plus vite que le pied (rapport > 1). La valeur par défaut
    1,2 est un ordre de grandeur : à calibrer avec un radar ou une vidéo.
    """
    return vitesse_pied_kmh * rapport


# =============================================================================
# 1. Détection des frappes
# =============================================================================

class DetecteurFrappe:
    """
    Détecte les frappes et estime la vitesse du pied.

    Principe
        1. Début d'événement : |ω| dépasse seuil_debut_dps.
        2. Pendant l'événement : |ω| max, |a| max, saturations.
        3. Fin : |ω| repasse sous seuil_fin_dps.
        4. Frappe validée si duree_min_ms <= durée <= duree_max_ms,
           |ω|max >= omega_min_dps et |a|max >= acc_min_g.
        5. Vitesse du pied : v = |ω|max (rad/s) × longueur_levier_m.
        6. Après une frappe validée, aucune nouvelle frappe pendant
           periode_refractaire_ms.

    Attributs publics (statistiques)
        nb_frappes            nombre de frappes validées
        vitesse_pied_max_kmh  vitesse de la frappe la plus rapide
        vitesse_max_minoree   True si cette vitesse max est une borne
                              inférieure (gyroscope saturé)
    """

    def __init__(self, longueur_levier_m=0.50,
                 seuil_debut_dps=400.0, seuil_fin_dps=200.0,
                 omega_min_dps=1000.0, acc_min_g=0.0,
                 duree_min_ms=30, duree_max_ms=500,
                 periode_refractaire_ms=400):
        """
        Paramètres
            longueur_levier_m       distance genou -> point de contact
                                    pied/ballon (m), mesurée sur le joueur ;
                                    PAS la distance genou -> capteur
            seuil_debut_dps         |ω| qui déclenche un événement
            seuil_fin_dps           |ω| qui termine un événement
            omega_min_dps           |ω| max minimal pour valider une frappe
            acc_min_g               |a| max minimal (0 = critère désactivé)
            duree_min_ms            durée minimale d'un événement
            duree_max_ms            durée maximale d'un événement
            periode_refractaire_ms  temps mort après une frappe validée

        Contraintes : 0 < seuil_fin < seuil_debut <= omega_min,
                      0 < duree_min < duree_max, longueur_levier > 0.
        """
        if not (0 < seuil_fin_dps < seuil_debut_dps <= omega_min_dps):
            raise ValueError("Il faut 0 < seuil_fin_dps < seuil_debut_dps <= omega_min_dps")
        if not (0 < duree_min_ms < duree_max_ms):
            raise ValueError("Il faut 0 < duree_min_ms < duree_max_ms")
        if longueur_levier_m <= 0:
            raise ValueError("longueur_levier_m doit etre > 0")
        if acc_min_g < 0 or periode_refractaire_ms < 0:
            raise ValueError("acc_min_g et periode_refractaire_ms doivent etre >= 0")

        self.longueur_levier_m = longueur_levier_m
        self.seuil_debut_dps = seuil_debut_dps
        self.seuil_fin_dps = seuil_fin_dps
        self.omega_min_dps = omega_min_dps
        self.acc_min_g = acc_min_g
        self._duree_min_us = int(duree_min_ms * 1000)
        self._duree_max_us = int(duree_max_ms * 1000)
        self._refractaire_us = int(periode_refractaire_ms * 1000)
        self._seuil_sat_acc = _FRACTION_SATURATION * ACC_PLAGE_G * G
        self._seuil_sat_gyro = _FRACTION_SATURATION * GYRO_PLAGE_DPS
        self.reinitialiser()

    def reinitialiser(self):
        """Remet à zéro les statistiques et l'état de détection."""
        self.nb_frappes = 0
        self.vitesse_pied_max_kmh = 0.0
        self.vitesse_max_minoree = False
        self._en_cours = False
        self._attente_repos = False
        self._t_derniere_frappe = None
        self._t_debut = 0
        self._omega_max = 0.0
        self._t_omega_max = 0
        self._acc_max_g = 0.0
        self._gyro_sature = False
        self._acc_sature = False

    def en_cours(self):
        """True pendant un événement (frappe possible pas encore tranchée)."""
        return self._en_cours

    def vitesse_max_mesurable_kmh(self):
        """Vitesse du pied au-delà de laquelle le gyroscope sature (km/h)."""
        return ms_vers_kmh(dps_vers_rads(GYRO_PLAGE_DPS) * self.longueur_levier_m)

    def _demarrer(self, t_us):
        self._en_cours = True
        self._t_debut = t_us
        self._omega_max = 0.0
        self._t_omega_max = t_us
        self._acc_max_g = 0.0
        self._gyro_sature = False
        self._acc_sature = False

    def _suivre(self, t_us, omega, ax, ay, az, gx, gy, gz):
        """Met à jour les extremums de l'événement en cours."""
        if omega > self._omega_max:
            self._omega_max = omega
            self._t_omega_max = t_us
        a_g = norme3(ax, ay, az) / G
        if a_g > self._acc_max_g:
            self._acc_max_g = a_g
        if max(abs(gx), abs(gy), abs(gz)) >= self._seuil_sat_gyro:
            self._gyro_sature = True
        if max(abs(ax), abs(ay), abs(az)) >= self._seuil_sat_acc:
            self._acc_sature = True

    def mettre_a_jour(self, t_us, ax, ay, az, gx, gy, gz):
        """
        Traite un échantillon (µs, m/s², °/s biais retranché).

        Retour : None, ou un dict pour chaque frappe validée :
            "numero", "t_us" (instant du pic), "duree_ms", "omega_max_dps",
            "acc_max_g" (intensité), "vitesse_pied_kmh",
            "vitesse_minoree" (gyroscope saturé : vitesse réelle supérieure),
            "acc_saturee" (accéléromètre saturé, >= ~16 g)
        """
        omega = norme3(gx, gy, gz)

        if not self._en_cours:
            if self._attente_repos:
                if omega < self.seuil_fin_dps:
                    self._attente_repos = False
                return None
            if self._t_derniere_frappe is not None and \
                    t_us - self._t_derniere_frappe < self._refractaire_us:
                return None
            if omega >= self.seuil_debut_dps:
                self._demarrer(t_us)
                self._suivre(t_us, omega, ax, ay, az, gx, gy, gz)
            return None

        self._suivre(t_us, omega, ax, ay, az, gx, gy, gz)
        duree_us = t_us - self._t_debut

        if duree_us > self._duree_max_us:
            # Mouvement trop long pour une frappe : abandon
            self._en_cours = False
            self._attente_repos = True
            return None

        if omega >= self.seuil_fin_dps:
            return None

        # Fin de l'événement : validation
        self._en_cours = False
        if duree_us < self._duree_min_us or \
                self._omega_max < self.omega_min_dps or \
                self._acc_max_g < self.acc_min_g:
            return None
        return self._enregistrer_frappe(duree_us)

    def _enregistrer_frappe(self, duree_us):
        """Calcule la vitesse du pied et met à jour les statistiques."""
        v_kmh = ms_vers_kmh(dps_vers_rads(self._omega_max) * self.longueur_levier_m)
        self.nb_frappes += 1
        self._t_derniere_frappe = self._t_omega_max
        if v_kmh > self.vitesse_pied_max_kmh:
            self.vitesse_pied_max_kmh = v_kmh
            self.vitesse_max_minoree = self._gyro_sature
        return {
            "numero": self.nb_frappes,
            "t_us": self._t_omega_max,
            "duree_ms": duree_us / 1000.0,
            "omega_max_dps": self._omega_max,
            "acc_max_g": self._acc_max_g,
            "vitesse_pied_kmh": v_kmh,
            "vitesse_minoree": self._gyro_sature,
            "acc_saturee": self._acc_sature,
        }


# =============================================================================
# 2. Modèle de foulée
# =============================================================================

class ModeleFoulee(ModeleCycle):
    """
    v = K[allure] × ω_pic (rad/s) ; distance de la foulée = v × durée.

    Allures (cadence en foulées/s = moitié de la cadence en pas/s) :
        marche < 1,15 <= course < 1,7 <= sprint
    Foulée de départ (durée inconnue) : allure course si ω_pic >=
    omega_course_dps, durée prise dans periode_depart_s.
    Les K par défaut sont des ordres de grandeur : à étalonner.
    """

    TYPE = "foot_foulee"

    def __init__(self, k_marche=0.23, k_course=0.31, k_sprint=0.36,
                 cadence_course_hz=1.15, cadence_sprint_hz=1.7,
                 omega_course_dps=450.0, longueur_min_m=0.4,
                 longueur_max_m=5.5, k_min=0.05, k_max=1.0):
        super().__init__({"marche": k_marche, "course": k_course, "sprint": k_sprint},
                         cadence_course_hz, cadence_sprint_hz,
                         longueur_min_m, longueur_max_m, k_min, k_max)
        self.omega_course_dps = omega_course_dps
        self.periode_depart_s = {"marche": 1.1, "course": 0.72, "sprint": 0.5}

    def _allure_depart(self, omega_dps):
        return "course" if omega_dps >= self.omega_course_dps else "marche"

    def _distance_brute(self, omega_dps, intervalle_s, allure):
        duree = intervalle_s if intervalle_s is not None else self.periode_depart_s[allure]
        return self.k[allure] * dps_vers_rads(omega_dps) * duree


# =============================================================================
# 3. Séance foot : frappes + course
# =============================================================================

def creer_detecteur_foulees():
    """
    Détecteur de foulées sur |ω| de la jambe (traité à 125 Hz) :
    passe-bas 6 Hz, seuil >= 150 °/s (50 % de l'enveloppe des pics),
    0,35 s à 2,0 s entre deux foulées.
    """
    return DetecteurCycles(frequence_coupure_hz=6.0, seuil_min=150.0,
                           fraction_seuil=0.5, intervalle_min_s=0.35,
                           intervalle_max_s=2.0, caracteristique="pic")


class SeanceFoot(SeanceDeplacement):
    """
    Séance foot. Utilisation (capteur en mode foot, biais gyro estimé) :

        seance = SeanceFoot()                  # sans GPS
        seance = SeanceFoot(gps=gps)           # GPS en secours (course)
        for ech in capteur.echantillons(125, duree_s):
            frappe = seance.traiter(*ech)      # dict si frappe validée
        seance.finaliser()
        print(seance.resume())

    Paramètres
        modele             ModeleFoulee
        gps                récepteur GPS (assiste uniquement la course)
        detecteur_frappe   DetecteurFrappe (ex. longueur_levier_m du joueur)
        decimation         1 : course traitée à chaque échantillon
        fenetre_frappe_s   une foulée à moins de cette durée d'une frappe
                           validée est écartée
        **options          options de deplacement.SeanceDeplacement
    """

    NOM_CYCLES = "nb_foulees"

    def __init__(self, modele=None, gps=None, detecteur_frappe=None,
                 decimation=DECIMATION_COURSE, fenetre_frappe_s=0.3, **options):
        super().__init__(creer_detecteur_foulees(),
                         modele if modele is not None else ModeleFoulee(),
                         gps=gps, decimation=decimation, **options)
        self.detecteur_frappe = detecteur_frappe if detecteur_frappe is not None \
            else DetecteurFrappe()
        self.derniere_frappe = None
        self.nb_foulees_ecartees = 0
        self._fenetre_frappe_us = int(fenetre_frappe_s * 1000000)
        self._en_attente = None
        self._t_derniere_frappe_us = None

    def traiter(self, t_us, ax, ay, az, gx, gy, gz):
        """Traite un échantillon ; renvoie le dict d'une frappe validée, sinon None."""
        frappe = self.detecteur_frappe.mettre_a_jour(t_us, ax, ay, az, gx, gy, gz)
        if frappe is not None:
            self.derniere_frappe = frappe
            self._t_derniere_frappe_us = frappe["t_us"]
        super().traiter(t_us, ax, ay, az, gx, gy, gz)
        return frappe

    def _signal(self, a_vert, gx, gy, gz):
        return norme3(gx, gy, gz)

    def _hors_frappe(self, cycle):
        """True si la foulée n'est pas le balancement d'une frappe validée."""
        return self._t_derniere_frappe_us is None or \
            abs(cycle[0] - self._t_derniere_frappe_us) > self._fenetre_frappe_us

    def _cycles_prets(self, t_us, cycle):
        """
        Une foulée détectée est mise en attente tant que le détecteur de
        frappe n'a pas tranché (événement en cours), puis acceptée ou
        écartée si elle coïncide avec une frappe validée.
        """
        prets = []
        if self._en_attente is not None and \
                (cycle is not None or not self.detecteur_frappe.en_cours()):
            c = self._en_attente
            self._en_attente = None
            if self._hors_frappe(c):
                prets.append(c)
            else:
                self.nb_foulees_ecartees += 1
        if cycle is not None:
            self._en_attente = cycle
        return prets

    def finaliser(self):
        """Traite la foulée éventuellement en attente en fin de séance."""
        if self._en_attente is None:
            return
        c = self._en_attente
        self._en_attente = None
        if self._hors_frappe(c):
            d = self._accepter_cycle(c)
            if self.source_vitesse == "imu":
                self.indicateurs.distance_m += d
        else:
            self.nb_foulees_ecartees += 1

    def etat(self):
        e = super().etat()
        e["tirs"] = self.detecteur_frappe.nb_frappes
        return e

    def resume(self):
        """Indicateurs de déplacement + indicateurs de tir."""
        r = super().resume()
        df = self.detecteur_frappe
        r["tirs"] = df.nb_frappes
        r["tir_max_pied_kmh"] = round(df.vitesse_pied_max_kmh, 1)
        r["tir_max_minore"] = df.vitesse_max_minoree
        r["tir_max_ballon_estime_kmh"] = round(estimer_vitesse_ballon_kmh(df.vitesse_pied_max_kmh), 1)
        r["nb_foulees_ecartees"] = self.nb_foulees_ecartees
        return r


# =============================================================================
# 4. Programmes
# =============================================================================

def _preparer(avec_gps, frequence_gps_hz, configuration_ublox):
    """GPS éventuel, capteur en mode foot, biais gyro, modèle chargé."""
    gps = creer_gps(frequence_gps_hz, configuration_ublox) if avec_gps else None
    capteur = CapteurBNO055(creer_bus_i2c())
    capteur.passer_en_mode_foot()
    print("Frequence de lecture max : {:.0f} Hz".format(capteur.mesurer_frequence_max()))
    print("Jambe immobile : mesure du biais gyro (1 s)...")
    capteur.estimer_biais_gyro()
    modele = ModeleFoulee()
    if modele.charger(FICHIER_MODELE_FOOT):
        print("Modele de foulee charge :", modele.vers_dict())
    else:
        print("ATTENTION : modele de foulee non etalonne (demo_etalonnage)")
    return capteur, modele, gps


def afficher_frappe(frappe):
    """Affiche une frappe validée."""
    print("Frappe {} : pied {}{:.1f} km/h | omega {:.0f} deg/s | {:.1f} g{} | {:.0f} ms".format(
        frappe["numero"],
        ">= " if frappe["vitesse_minoree"] else "",
        frappe["vitesse_pied_kmh"],
        frappe["omega_max_dps"],
        frappe["acc_max_g"],
        " (sature)" if frappe["acc_saturee"] else "",
        frappe["duree_ms"]))


def demo_foot(duree_s=600, avec_gps=False, priorite="imu", longueur_levier_m=0.50,
              frequence_gps_hz=5, configuration_ublox=True, journal=True,
              journal_frappes=True, afficher=True):
    """
    Séance foot : frappes affichées en direct, état chaque seconde, résumé.

    Les fichiers produits sont numérotés (seance_foot_0007.json,
    etat_foot_0007.csv, frappes_0007.bin) : une séance n'écrase jamais la
    précédente. Le résumé est réécrit toutes les 30 s pendant la séance.

    Paramètres
        journal          True pour écrire le journal d'état (1 ligne/s)
        journal_frappes  True pour enregistrer le brut autour de chaque
                         frappe (2 s avant, 1 s après), afin de régler les
                         seuils après coup sans retourner sur le terrain
        afficher         False pour une séance silencieuse (sur batterie)

    Retour : le résumé de séance.
    """
    capteur, modele, gps = _preparer(avec_gps, frequence_gps_hz, configuration_ublox)
    detecteur_frappe = DetecteurFrappe(longueur_levier_m=longueur_levier_m)
    if afficher:
        print("Vitesse pied mesurable jusqu'a {:.1f} km/h".format(
            detecteur_frappe.vitesse_max_mesurable_kmh()))
    seance = SeanceFoot(modele=modele, gps=gps, detecteur_frappe=detecteur_frappe,
                        priorite=priorite)

    numero = prochain_numero("foot")
    nom_resultats = nom_unique("seance_foot", "json", numero)
    jetat = JournalEtat(nom_unique("etat_foot", "csv", numero)) if journal else None
    jfrappes = JournalEvenements(nom_unique("frappes", "bin", numero),
                                 frequence_hz=FREQUENCE_FOOT_HZ) if journal_frappes else None

    resume = executer_seance(capteur, seance, FREQUENCE_FOOT_HZ, duree_s,
                             nom_resultats=nom_resultats, journal_etat=jetat,
                             journal_evenements=jfrappes, afficher=afficher,
                             sur_evenement=afficher_frappe if afficher else None)
    if afficher:
        afficher_resume(resume)
        print("----- Tirs -----")
        print("Tirs           : {}".format(resume["tirs"]))
        print("Tir max (pied) : {}{:.1f} km/h".format(
            ">= " if resume["tir_max_minore"] else "", resume["tir_max_pied_kmh"]))
        print("Tir max ballon : ~{:.1f} km/h (estimation)".format(
            resume["tir_max_ballon_estime_kmh"]))
        print("Resultats : {}".format(nom_resultats))
    if avec_gps:
        modele.sauvegarder(FICHIER_MODELE_FOOT)
    return resume


def demo_etalonnage(allure, distance_m):
    """Étalonne K[allure] de la course sur distance_m mètres (sans GPS)."""
    capteur, modele, _ = _preparer(False, 0, False)
    seance = SeanceFoot(modele=modele)
    etalonner_sur_distance(capteur, seance, FREQUENCE_FOOT_HZ, distance_m,
                           allure, FICHIER_MODELE_FOOT)


if __name__ == "__main__":
    demo_foot()

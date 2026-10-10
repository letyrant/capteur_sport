"""
deplacement.py
==============

Tronc commun des modes foot et rugby : mesure du déplacement du joueur
(vitesse, distance, position, indicateurs du cahier des charges) à partir du
BNO055, assistée en option par le GPS. MicroPython, Raspberry Pi Pico 2 W.

Utilisé par
    mode_foot.py   capteur sur le mollet : un cycle = une FOULÉE (2 pas),
                   détectée sur la vitesse angulaire de la jambe
    mode_rugby.py  capteur dans le dos (col du maillot) : un cycle = un PAS,
                   détecté sur l'accélération verticale du tronc

Chaîne de traitement (à chaque échantillon)
-------------------------------------------
    1. EstimateurVertical : direction de la verticale dans le repère du
       capteur (gyroscope + accéléromètre) -> accélération verticale et
       rotation autour de la verticale. Le boîtier peut être fixé dans
       n'importe quelle orientation.
    2. NavigationPDR : cap (rotation intégrée) et position à l'estime.
    3. DetecteurCycles : détection des pas ou des foulées.
    4. Modèle de cycle (propre à chaque mode) : distance parcourue par
       cycle, avec un coefficient K par allure (marche, course, sprint)
       choisie selon la cadence. K s'étalonne sur une distance connue
       (etalonner_sur_distance) ou automatiquement par le GPS.
    5. Vitesse = distance des 2 derniers cycles / durée des 2 derniers cycles.
    6. IndicateursSeance : indicateurs du cahier des charges.

Pourquoi ne pas intégrer l'accélération ?
-----------------------------------------
Un biais de 0,05 m/s² intégré donne 3 m/s d'erreur de vitesse en une
minute : l'intégration directe est inutilisable. La méthode par cycles
(podomètre) ne dérive pas.

Assistance GPS (optionnelle, dans les deux modes)
-------------------------------------------------
    priorite="imu" (par défaut) : l'accéléromètre/gyroscope fournit la
        vitesse ; le GPS sert de SECOURS quand la centrale ne détecte plus de
        cycles alors que le GPS mesure un déplacement (pas chassés,
        déplacement non reconnu...). En course régulière, le GPS étalonne
        aussi K en continu et corrige le cap.
    priorite="gps" : vitesse du GPS dès que son signal est bon, centrale en
        relais pendant les pertes de signal. Plus précis en plein air
        (vitesse Doppler à ~0,1 m/s près), mais limité à 1-5 mesures/s.
    Dans les deux cas, la POSITION est recalée sur le GPS à chaque mesure
    (le GPS est bien plus précis que la position à l'estime).
    Le GPS ne peut PAS assister la vitesse de frappe (geste de 0,1 s, que le
    GPS ne voit pas : il mesure le déplacement du corps).
"""

import json
import math

from capteur_bno055 import norme3, G


_DEG_VERS_RAD = math.pi / 180.0
_RAD_VERS_DEG = 180.0 / math.pi
_DEUX_PI = 2.0 * math.pi

ALLURES = ("marche", "course", "sprint")


# =============================================================================
# Fonctions utilitaires
# =============================================================================

def angle_pi(a):
    """Ramène un angle (rad) dans l'intervalle ]-pi, pi]."""
    while a > math.pi:
        a -= _DEUX_PI
    while a <= -math.pi:
        a += _DEUX_PI
    return a


def format_min_s(secondes):
    """Formate une durée en "min:ss" (ex. 754 s -> "12:34")."""
    s = int(secondes + 0.5)
    return "{}:{:02d}".format(s // 60, s % 60)


def sauvegarder_json(nom_fichier, donnees):
    """Écrit un dict dans un fichier JSON sur la flash du Pico (FP4)."""
    with open(nom_fichier, "w") as f:
        f.write(json.dumps(donnees))


def charger_json(nom_fichier):
    """Lit un fichier JSON ; renvoie None s'il n'existe pas."""
    try:
        with open(nom_fichier, "r") as f:
            return json.loads(f.read())
    except OSError:
        return None


# =============================================================================
# 1. Verticale et rotation autour de la verticale
# =============================================================================

class EstimateurVertical:
    """
    Suit la direction verticale (vecteur unitaire u, vers le haut) exprimée
    dans le repère du capteur.

    Filtre complémentaire :
      - prédiction gyroscope : un vecteur fixe dans le repère terrestre
        tourne dans le repère capteur selon du/dt = u × ω ;
      - correction accéléromètre : en moyenne, l'accéléromètre mesure +g
        vers le haut ; u est rapproché de a/|a| avec une constante de temps
        constante_temps_s, seulement si 0,3 g < |a| < 3 g (rejet des phases
        de vol et des chocs).

    mettre_a_jour() renvoie
      a_vert   accélération verticale sans la gravité (m/s², + vers le haut)
      lacet    vitesse de rotation autour de la verticale (rad/s,
               + = sens trigonométrique vu de dessus)
    """

    def __init__(self, constante_temps_s=2.0):
        if constante_temps_s <= 0:
            raise ValueError("constante_temps_s doit etre > 0")
        self.constante_temps_s = constante_temps_s
        self.ux = 0.0
        self.uy = 0.0
        self.uz = 1.0
        self.initialise = False

    def initialiser(self, ax, ay, az):
        """Prend la direction de l'accélération mesurée comme verticale (joueur immobile)."""
        n = norme3(ax, ay, az)
        if n < 0.1 * G:
            raise ValueError("Acceleration trop faible pour trouver la verticale")
        self.ux = ax / n
        self.uy = ay / n
        self.uz = az / n
        self.initialise = True

    def mettre_a_jour(self, dt, ax, ay, az, gx, gy, gz):
        """
        Paramètres : dt (s), accélération (m/s², gravité incluse),
        vitesse angulaire (°/s, biais retranché).
        Retour : (a_vert en m/s², lacet en rad/s).
        """
        if not self.initialise:
            self.initialiser(ax, ay, az)
        wx = gx * _DEG_VERS_RAD
        wy = gy * _DEG_VERS_RAD
        wz = gz * _DEG_VERS_RAD
        ux = self.ux
        uy = self.uy
        uz = self.uz

        # Prédiction par le gyroscope : du/dt = u × ω
        nx = ux + (uy * wz - uz * wy) * dt
        ny = uy + (uz * wx - ux * wz) * dt
        nz = uz + (ux * wy - uy * wx) * dt

        # Correction par l'accéléromètre
        na = norme3(ax, ay, az)
        if 0.3 * G < na < 3.0 * G:
            beta = dt / (self.constante_temps_s + dt)
            nx += beta * (ax / na - nx)
            ny += beta * (ay / na - ny)
            nz += beta * (az / na - nz)

        n = norme3(nx, ny, nz)
        self.ux = nx / n
        self.uy = ny / n
        self.uz = nz / n

        a_vert = ax * self.ux + ay * self.uy + az * self.uz - G
        lacet = wx * self.ux + wy * self.uy + wz * self.uz
        return a_vert, lacet


# =============================================================================
# 2. Détection des cycles (pas ou foulées)
# =============================================================================

class DetecteurCycles:
    """
    Détecteur de pics périodiques, utilisé pour les pas (rugby : accélération
    verticale) et les foulées (foot : vitesse angulaire de la jambe).

      1. Filtre passe-bas du 1er ordre (frequence_coupure_hz).
      2. Seuil adaptatif : max(seuil_min, fraction_seuil × enveloppe des pics
         récents) ; l'enveloppe décroît avec une constante de temps
         constante_enveloppe_s, ce qui suit les changements d'allure.
      3. Un cycle = une zone au-dessus du seuil ; son instant est celui du
         maximum. Fin de zone quand le signal repasse sous la moitié du seuil.
      4. Deux pics séparés de moins de intervalle_min_s sont fusionnés.
      5. Après plus de intervalle_max_s sans cycle, le cycle suivant est un
         cycle de "départ" (intervalle inconnu).

    caracteristique : grandeur transmise au modèle de cycle
        "amplitude"  pic - creux depuis le cycle précédent (pas, rugby)
        "pic"        valeur du pic (foulées, foot)

    mettre_a_jour() renvoie None, ou (t_us, caracteristique, intervalle_s),
    intervalle_s valant None pour un cycle de départ.
    """

    def __init__(self, frequence_coupure_hz, seuil_min, fraction_seuil,
                 intervalle_min_s, intervalle_max_s, caracteristique="amplitude",
                 constante_enveloppe_s=1.0):
        if frequence_coupure_hz <= 0 or seuil_min <= 0 or constante_enveloppe_s <= 0:
            raise ValueError("frequence_coupure_hz, seuil_min et constante_enveloppe_s doivent etre > 0")
        if not (0 <= fraction_seuil < 1):
            raise ValueError("fraction_seuil doit etre dans [0, 1[")
        if not (0 < intervalle_min_s < intervalle_max_s):
            raise ValueError("Il faut 0 < intervalle_min_s < intervalle_max_s")
        if caracteristique not in ("amplitude", "pic"):
            raise ValueError('caracteristique doit valoir "amplitude" ou "pic"')
        self.seuil_min = seuil_min
        self.fraction_seuil = fraction_seuil
        self.intervalle_min_s = intervalle_min_s
        self.intervalle_max_s = intervalle_max_s
        self.caracteristique = caracteristique
        self.constante_enveloppe_s = constante_enveloppe_s
        self._rc = 1.0 / (2.0 * math.pi * frequence_coupure_hz)
        self._intervalle_min_us = int(intervalle_min_s * 1000000)
        self._intervalle_max_us = int(intervalle_max_s * 1000000)
        self.reinitialiser()

    def reinitialiser(self):
        """Remet le détecteur à zéro."""
        self.signal_filtre = 0.0
        self._au_dessus = False
        self._pic = 0.0
        self._t_pic = 0
        self._creux = 0.0
        self._enveloppe = 0.0
        self._t_dernier = None

    def seuil(self):
        """Seuil de détection actuel (unité du signal)."""
        return max(self.seuil_min, self.fraction_seuil * self._enveloppe)

    def mettre_a_jour(self, t_us, dt, signal):
        """t_us : temps (µs) ; dt : pas de temps (s) ; signal : grandeur surveillée."""
        alpha = dt / (self._rc + dt)
        s = self.signal_filtre + alpha * (signal - self.signal_filtre)
        self.signal_filtre = s

        self._enveloppe -= self._enveloppe * dt / self.constante_enveloppe_s
        if s < self._creux:
            self._creux = s

        seuil = self.seuil()
        if not self._au_dessus:
            if s > seuil:
                self._au_dessus = True
                self._pic = s
                self._t_pic = t_us
            return None

        if s > self._pic:
            self._pic = s
            self._t_pic = t_us
        if s > 0.5 * seuil:
            return None

        # Fin de la zone au-dessus du seuil
        self._au_dessus = False
        if self._t_dernier is not None and \
                self._t_pic - self._t_dernier < self._intervalle_min_us:
            return None  # rebond du même cycle

        if self.caracteristique == "amplitude":
            valeur = self._pic - self._creux
        else:
            valeur = self._pic
        if self._t_dernier is None or \
                self._t_pic - self._t_dernier > self._intervalle_max_us:
            intervalle = None
        else:
            intervalle = (self._t_pic - self._t_dernier) / 1000000.0
        self._t_dernier = self._t_pic
        if self._pic > self._enveloppe:
            self._enveloppe = self._pic
        self._creux = s
        return (self._t_pic, valeur, intervalle)


# =============================================================================
# 3. Modèle de cycle (base commune, une sous-classe par mode)
# =============================================================================

class ModeleCycle:
    """
    Distance parcourue pendant un cycle, avec un coefficient K par allure.

    Allure choisie selon la cadence (cycles par seconde = 1 / intervalle) :
        marche   cadence <  cadence_course_hz
        course   cadence_course_hz <= cadence < cadence_sprint_hz
        sprint   cadence >= cadence_sprint_hz
    Cycle de départ (intervalle inconnu) : _allure_depart(caracteristique).

    Les sous-classes définissent TYPE, _allure_depart() et _distance_brute().
    Un modèle étalonné s'enregistre sur la flash (sauvegarder / charger) ;
    TYPE empêche de charger les K d'un mode dans l'autre.
    """

    TYPE = "generique"

    def __init__(self, k, cadence_course_hz, cadence_sprint_hz,
                 distance_min_m, distance_max_m, k_min, k_max):
        if not (0 < cadence_course_hz < cadence_sprint_hz):
            raise ValueError("Il faut 0 < cadence_course_hz < cadence_sprint_hz")
        if not (0 < distance_min_m < distance_max_m):
            raise ValueError("Il faut 0 < distance_min_m < distance_max_m")
        if not (0 < k_min < k_max):
            raise ValueError("Il faut 0 < k_min < k_max")
        self.k = {}
        for allure in ALLURES:
            if not (k_min <= k[allure] <= k_max):
                raise ValueError("K[{}] hors de [{}, {}]".format(allure, k_min, k_max))
            self.k[allure] = k[allure]
        self.cadence_course_hz = cadence_course_hz
        self.cadence_sprint_hz = cadence_sprint_hz
        self.distance_min_m = distance_min_m
        self.distance_max_m = distance_max_m
        self.k_min = k_min
        self.k_max = k_max

    def _allure_depart(self, caracteristique):
        raise NotImplementedError

    def _distance_brute(self, caracteristique, intervalle_s, allure):
        raise NotImplementedError

    def allure(self, caracteristique, intervalle_s):
        """Renvoie "marche", "course" ou "sprint"."""
        if intervalle_s is not None and intervalle_s > 0:
            cadence = 1.0 / intervalle_s
            if cadence >= self.cadence_sprint_hz:
                return "sprint"
            if cadence >= self.cadence_course_hz:
                return "course"
            return "marche"
        return self._allure_depart(caracteristique)

    def distance_cycle(self, caracteristique, intervalle_s):
        """Distance du cycle (m), bornée à [distance_min_m, distance_max_m]."""
        allure = self.allure(caracteristique, intervalle_s)
        d = self._distance_brute(caracteristique, intervalle_s, allure)
        if d < self.distance_min_m:
            return self.distance_min_m
        if d > self.distance_max_m:
            return self.distance_max_m
        return d

    def _borner_k(self, k):
        return min(self.k_max, max(self.k_min, k))

    def etalonner(self, distance_reelle_m, distances_par_allure, allure="course"):
        """
        Corrige K[allure] après un parcours de distance connue.

        distances_par_allure : distances estimées pour chaque allure pendant
        le parcours (SeanceDeplacement.distance_par_allure). La distance des
        autres allures est considérée comme juste : étalonner dans l'ordre
        marche -> course -> sprint.

        Retour : nouveau K. Lève ValueError si l'étalonnage est impossible.
        """
        if allure not in self.k:
            raise ValueError("allure inconnue : {}".format(allure))
        d_allure = distances_par_allure.get(allure, 0.0)
        d_autres = 0.0
        for nom in distances_par_allure:
            if nom != allure:
                d_autres += distances_par_allure[nom]
        cible = distance_reelle_m - d_autres
        if d_allure <= 0 or cible <= 0:
            raise ValueError("Pas assez de cycles a l'allure '{}' pour etalonner".format(allure))
        self.k[allure] = self._borner_k(self.k[allure] * cible / d_allure)
        return self.k[allure]

    def ajuster(self, allure, rapport, gain):
        """
        Ajustement progressif de K[allure] par une vitesse de référence (GPS) :
        K <- K × (1 + gain × (rapport - 1)), rapport = v_reference / v_cycles.
        """
        if allure in self.k:
            self.k[allure] = self._borner_k(self.k[allure] * (1.0 + gain * (rapport - 1.0)))

    def vers_dict(self):
        """Paramètres du modèle (sauvegarde, résumé de séance)."""
        return {
            "type": self.TYPE,
            "k": {"marche": self.k["marche"], "course": self.k["course"],
                  "sprint": self.k["sprint"]},
            "cadence_course_hz": self.cadence_course_hz,
            "cadence_sprint_hz": self.cadence_sprint_hz,
        }

    def sauvegarder(self, nom_fichier):
        """Enregistre le modèle étalonné sur la flash du Pico."""
        sauvegarder_json(nom_fichier, self.vers_dict())

    def charger(self, nom_fichier):
        """
        Recharge un modèle étalonné.
        Retour : True si chargé, False si absent ou d'un autre type.
        """
        d = charger_json(nom_fichier)
        if d is None or d.get("type") != self.TYPE:
            return False
        k = d.get("k", {})
        for allure in ALLURES:
            if allure in k:
                self.k[allure] = self._borner_k(k[allure])
        self.cadence_course_hz = d.get("cadence_course_hz", self.cadence_course_hz)
        self.cadence_sprint_hz = d.get("cadence_sprint_hz", self.cadence_sprint_hz)
        return True


# =============================================================================
# 4. Navigation à l'estime (position)
# =============================================================================

class NavigationPDR:
    """
    Position (x, y) en mètres dans le plan horizontal.

    cap_rad           cap intégré au gyroscope (0 = orientation du joueur au
                      démarrage, sens trigonométrique)
    decalage_cap_rad  correction ajoutée au cap (réglée par le GPS pour
                      passer dans le repère est/nord)
    Sans GPS : x vers l'avant du joueur au démarrage, y vers sa gauche.
    Avec GPS : x vers l'est, y vers le nord, origine à la 1re position GPS.
    """

    def __init__(self):
        self.reinitialiser()

    def reinitialiser(self):
        """Remet la position et le cap à zéro."""
        self.cap_rad = 0.0
        self.decalage_cap_rad = 0.0
        self.x = 0.0
        self.y = 0.0

    def tourner(self, lacet_rad_s, dt):
        """Intègre la rotation autour de la verticale."""
        self.cap_rad = angle_pi(self.cap_rad + lacet_rad_s * dt)

    def avancer(self, distance_m):
        """Avance de distance_m dans la direction du cap corrigé."""
        angle = self.cap_rad + self.decalage_cap_rad
        self.x += distance_m * math.cos(angle)
        self.y += distance_m * math.sin(angle)

    def cap_deg(self):
        """Cap corrigé en degrés, dans ]-180, 180]."""
        return angle_pi(self.cap_rad + self.decalage_cap_rad) * _RAD_VERS_DEG


# =============================================================================
# 5. Indicateurs du cahier des charges (indépendants de la source de vitesse)
# =============================================================================

class IndicateursSeance:
    """
    Indicateurs de séance calculés à partir d'une vitesse échantillonnée,
    quelle que soit sa source (cycles, GPS).

    Définitions (paramétrables)
        en course      vitesse >= seuil_course_kmh
        Activité       durée en course / durée totale × 100
        une "course"   période continue en course d'au moins duree_min_course_s
        Sprints        courses dont la vitesse max >= facteur_sprint ×
                       vitesse moyenne de référence.

                       DÉFINITION RETENUE : reference_sprint = "course",
                       c'est-à-dire la vitesse moyenne pendant les phases de
                       course. Le cahier des charges dit « 30 % de plus que
                       ta vitesse moyenne » sans préciser laquelle, et les
                       deux lectures ne donnent pas du tout le même seuil.
                       Exemple mesuré (5 min d'arrêt, 2 min de trot à
                       10,8 km/h, 6 s de sprint) :
                           "course" -> seuil 14,9 km/h  (un vrai sprint)
                           "seance" -> seuil  3,9 km/h  (la marche compte)
                       La moyenne de séance inclut les arrêts, très nombreux
                       en sport collectif : elle abaisse le seuil au point de
                       compter des trots comme des sprints. "course" est donc
                       la seule lecture qui donne un indicateur utile, et
                       c'est celle inscrite au cahier des charges.
                       "seance" reste disponible pour comparer.
        Accélérations  dérivée de la vitesse sur fenetre_acceleration_s
                       >= +seuil_acceleration_ms2 (un événement par
                       franchissement : il faut repasser sous la moitié du
                       seuil avant d'en compter un autre)
        Décélérations  idem avec <= -seuil_acceleration_ms2
    """

    def __init__(self, seuil_course_kmh=7.0, seuil_acceleration_ms2=2.0,
                 fenetre_acceleration_s=0.5, facteur_sprint=1.3,
                 duree_min_course_s=1.0, reference_sprint="course"):
        if reference_sprint not in ("course", "seance"):
            raise ValueError('reference_sprint doit valoir "course" ou "seance"')
        if seuil_course_kmh <= 0 or seuil_acceleration_ms2 <= 0 or \
                fenetre_acceleration_s <= 0 or facteur_sprint <= 1 or \
                duree_min_course_s < 0:
            raise ValueError("Parametre d'indicateur invalide")
        self.seuil_course_ms = seuil_course_kmh / 3.6
        self.seuil_acceleration_ms2 = seuil_acceleration_ms2
        self.fenetre_acceleration_s = fenetre_acceleration_s
        self.facteur_sprint = facteur_sprint
        self.duree_min_course_s = duree_min_course_s
        self.reference_sprint = reference_sprint
        self.reinitialiser()

    def reinitialiser(self):
        """Remet tous les indicateurs à zéro."""
        self.duree_totale_s = 0.0
        self.duree_course_s = 0.0
        self.distance_m = 0.0
        self.vitesse_ms = 0.0
        self.vitesse_max_ms = 0.0
        self.acceleration_ms2 = 0.0
        self.nb_accelerations = 0
        self.nb_decelerations = 0
        self.maxima_courses = []
        self._integrale_v_course = 0.0
        self._en_course = False
        self._debut_course_s = 0.0
        self._max_course = 0.0
        self._t_prec_us = None
        self._t_ref_us = None
        self._v_ref = 0.0
        self._etat_acc = 0

    def mettre_a_jour(self, t_us, vitesse_ms, distance_m=0.0):
        """
        t_us : temps (µs, croissant) ; vitesse_ms : vitesse actuelle (m/s) ;
        distance_m : distance parcourue depuis l'appel précédent (m).
        """
        self.distance_m += distance_m
        self.vitesse_ms = vitesse_ms
        if self._t_prec_us is None:
            self._t_prec_us = t_us
            self._t_ref_us = t_us
            self._v_ref = vitesse_ms
            return
        dt = (t_us - self._t_prec_us) / 1000000.0
        self._t_prec_us = t_us
        if dt <= 0:
            return
        self.duree_totale_s += dt
        if vitesse_ms > self.vitesse_max_ms:
            self.vitesse_max_ms = vitesse_ms

        # Temps en course et découpage en "courses"
        if vitesse_ms >= self.seuil_course_ms:
            self.duree_course_s += dt
            self._integrale_v_course += vitesse_ms * dt
            if not self._en_course:
                self._en_course = True
                self._debut_course_s = self.duree_totale_s - dt
                self._max_course = vitesse_ms
            elif vitesse_ms > self._max_course:
                self._max_course = vitesse_ms
        elif self._en_course:
            self._en_course = False
            if self.duree_totale_s - dt - self._debut_course_s >= self.duree_min_course_s:
                self.maxima_courses.append(self._max_course)

        # Accélérations / décélérations
        dta = (t_us - self._t_ref_us) / 1000000.0
        if dta >= self.fenetre_acceleration_s:
            a = (vitesse_ms - self._v_ref) / dta
            self._t_ref_us = t_us
            self._v_ref = vitesse_ms
            self.acceleration_ms2 = a
            seuil = self.seuil_acceleration_ms2
            if a >= seuil:
                if self._etat_acc != 1:
                    self.nb_accelerations += 1
                    self._etat_acc = 1
            elif a <= -seuil:
                if self._etat_acc != -1:
                    self.nb_decelerations += 1
                    self._etat_acc = -1
            elif abs(a) < 0.5 * seuil:
                self._etat_acc = 0

    def vitesse_moyenne_course_ms(self):
        """Vitesse moyenne pendant les phases de course (m/s)."""
        if self.duree_course_s <= 0:
            return 0.0
        return self._integrale_v_course / self.duree_course_s

    def resume(self):
        """Indicateurs du cahier des charges (dict prêt à afficher ou à envoyer)."""
        maxima = list(self.maxima_courses)
        if self._en_course and \
                self.duree_totale_s - self._debut_course_s >= self.duree_min_course_s:
            maxima.append(self._max_course)  # course en cours à la fin

        v_moy_course = self.vitesse_moyenne_course_ms()
        v_moy_seance = self.distance_m / self.duree_totale_s if self.duree_totale_s > 0 else 0.0
        reference = v_moy_course if self.reference_sprint == "course" else v_moy_seance
        seuil_sprint = self.facteur_sprint * reference
        nb_sprints = 0
        if reference > 0:
            for v_max in maxima:
                if v_max >= seuil_sprint:
                    nb_sprints += 1
        activite = 100.0 * self.duree_course_s / self.duree_totale_s \
            if self.duree_totale_s > 0 else 0.0

        return {
            "distance_km": round(self.distance_m / 1000.0, 3),
            "activite_pct": round(activite, 1),
            "sprint_max_kmh": round(self.vitesse_max_ms * 3.6, 1),
            "course": format_min_s(self.duree_course_s),
            "course_s": round(self.duree_course_s),
            "sprints": nb_sprints,
            "accelerations": self.nb_accelerations,
            "decelerations": self.nb_decelerations,
            "duree_s": round(self.duree_totale_s),
            "vitesse_moy_course_kmh": round(v_moy_course * 3.6, 1),
            "seuil_sprint_kmh": round(seuil_sprint * 3.6, 1),
            "reference_sprint": self.reference_sprint,
        }


# =============================================================================
# 6. Séance de déplacement (base des modes foot et rugby), GPS optionnel
# =============================================================================

class SeanceDeplacement:
    """
    Assemble verticale, navigation, détection de cycles, modèle, GPS et
    indicateurs. Les modes définissent _signal() (et éventuellement
    _cycles_prets() et finaliser()).

    Paramètres
        detecteur        DetecteurCycles configuré pour le mode
        modele           ModeleCycle du mode
        indicateurs      IndicateursSeance (défaut : réglages standard)
        gps              None (sans GPS) ou objet respectant l'interface de
                         capteur_gps.py (RecepteurGPS ou équivalent)
        priorite         "imu" : centrale d'abord, GPS en secours (défaut)
                         "gps" : GPS d'abord, centrale en relais
        decimation       le déplacement n'est traité qu'un échantillon sur
                         `decimation` (ex. 5 : 500 Hz -> 100 Hz) ; la
                         verticale et le cap restent suivis à chaque échantillon
        hdop_max, nb_sat_min, age_max_ms
                         critères de qualité du GPS
        vitesse_min_gps_ms
                         en dessous, la vitesse GPS est considérée nulle
                         (bruit Doppler à l'arrêt)
        vitesse_secours_ms
                         (priorite "imu") vitesse GPS à partir de laquelle le
                         GPS prend le relais quand la centrale ne voit plus de
                         cycles
        vitesse_min_recalage_ms, gain_recalage
                         étalonnage automatique de K et du cap par le GPS,
                         seulement en course régulière au-dessus de cette
                         vitesse

    Attributs publics
        nb_cycles, distance_par_allure, vitesse_imu_ms, cadence_hz,
        source_vitesse ("imu", "gps" ou "gps (secours)"), nb_positions_gps,
        cap_aligne, navigation, indicateurs, modele, detecteur, vertical
    """

    NOM_CYCLES = "nb_cycles"   # clé du résumé, redéfinie par les modes

    def __init__(self, detecteur, modele, indicateurs=None, gps=None,
                 priorite="imu", decimation=1, constante_temps_vertical_s=2.0,
                 hdop_max=2.5, nb_sat_min=5, age_max_ms=1500,
                 vitesse_min_gps_ms=0.3, vitesse_secours_ms=1.5,
                 vitesse_min_recalage_ms=2.5, gain_recalage=0.05):
        if priorite not in ("imu", "gps"):
            raise ValueError('priorite doit valoir "imu" ou "gps"')
        if not isinstance(decimation, int) or decimation < 1:
            raise ValueError("decimation doit etre un entier >= 1")
        if not (0 < gain_recalage <= 1):
            raise ValueError("gain_recalage doit etre dans ]0, 1]")
        self.detecteur = detecteur
        self.modele = modele
        self.indicateurs = indicateurs if indicateurs is not None else IndicateursSeance()
        self.gps = gps
        self.priorite = priorite
        self.decimation = decimation
        self.hdop_max = hdop_max
        self.nb_sat_min = nb_sat_min
        self.age_max_ms = age_max_ms
        self.vitesse_min_gps_ms = vitesse_min_gps_ms
        self.vitesse_secours_ms = vitesse_secours_ms
        self.vitesse_min_recalage_ms = vitesse_min_recalage_ms
        self.gain_recalage = gain_recalage

        self.vertical = EstimateurVertical(constante_temps_vertical_s)
        self.navigation = NavigationPDR()
        self.nb_cycles = 0
        self.distance_par_allure = {}
        for allure in ALLURES:
            self.distance_par_allure[allure] = 0.0
        self.vitesse_imu_ms = 0.0
        self.cadence_hz = 0.0
        self.source_vitesse = "imu"
        self.nb_positions_gps = 0
        self.cap_aligne = False

        self._t_prec_us = None
        self._t_dep_prec_us = None
        self._compteur = 0
        self._t_dernier_cycle_us = None
        self._dernier_cycle = None        # (distance_m, intervalle_s ou None)
        self._allure_dernier_cycle = None
        self._gps_ok = False
        self._sin_moy = 0.0
        self._cos_moy = 0.0

    # ------------------------------------------------------------------
    # À définir par les modes
    # ------------------------------------------------------------------

    def _signal(self, a_vert, gx, gy, gz):
        """Grandeur surveillée par le détecteur de cycles."""
        raise NotImplementedError

    def _cycles_prets(self, t_us, cycle):
        """Cycles à comptabiliser maintenant (un mode peut en retarder ou en rejeter)."""
        return () if cycle is None else (cycle,)

    def finaliser(self):
        """À appeler en fin de séance, avant resume() (cycles en attente)."""
        pass

    # ------------------------------------------------------------------
    # Traitement
    # ------------------------------------------------------------------

    def traiter(self, t_us, ax, ay, az, gx, gy, gz):
        """
        Traite un échantillon de CapteurBNO055.echantillons() :
        t_us (µs), accélération (m/s²), vitesse angulaire (°/s, biais retranché).
        """
        if self._t_prec_us is None:
            self._t_prec_us = t_us
            self._t_dep_prec_us = t_us
            self.vertical.initialiser(ax, ay, az)
            self.indicateurs.mettre_a_jour(t_us, 0.0, 0.0)
            return
        dt = (t_us - self._t_prec_us) / 1000000.0
        self._t_prec_us = t_us
        if dt <= 0:
            return

        a_vert, lacet = self.vertical.mettre_a_jour(dt, ax, ay, az, gx, gy, gz)
        self.navigation.tourner(lacet, dt)

        self._compteur += 1
        if self._compteur < self.decimation:
            return
        self._compteur = 0
        dt_dep = (t_us - self._t_dep_prec_us) / 1000000.0
        self._t_dep_prec_us = t_us

        cycle = self.detecteur.mettre_a_jour(t_us, dt_dep, self._signal(a_vert, gx, gy, gz))
        distance_imu = 0.0
        for c in self._cycles_prets(t_us, cycle):
            distance_imu += self._accepter_cycle(c)
        self._verifier_arret(t_us)
        vitesse, distance = self._choisir_vitesse(dt_dep, distance_imu)
        self.indicateurs.mettre_a_jour(t_us, vitesse, distance)

    def _accepter_cycle(self, cycle):
        """Distance, vitesse et position pour un cycle validé. Retour : distance (m)."""
        t_cycle, caracteristique, intervalle = cycle
        allure = self.modele.allure(caracteristique, intervalle)
        distance = self.modele.distance_cycle(caracteristique, intervalle)
        self.nb_cycles += 1
        self.distance_par_allure[allure] += distance
        self._allure_dernier_cycle = allure
        self.navigation.avancer(distance)

        if intervalle is None:
            # Cycle de départ : durée inconnue, vitesse connue au cycle suivant
            self._dernier_cycle = (distance, None)
        else:
            self.cadence_hz = 1.0 / intervalle
            if self._dernier_cycle is not None and self._dernier_cycle[1] is not None:
                self.vitesse_imu_ms = (distance + self._dernier_cycle[0]) / \
                    (intervalle + self._dernier_cycle[1])
            else:
                self.vitesse_imu_ms = distance / intervalle
            self._dernier_cycle = (distance, intervalle)
        self._t_dernier_cycle_us = t_cycle
        return distance

    def _verifier_arret(self, t_us):
        """Vitesse mise à 0 sans cycle depuis 2 intervalles (ou intervalle_max_s)."""
        if self._t_dernier_cycle_us is None:
            return
        limite = self.detecteur.intervalle_max_s
        if self._dernier_cycle is not None and self._dernier_cycle[1] is not None:
            limite = min(limite, 2.0 * self._dernier_cycle[1])
        if (t_us - self._t_dernier_cycle_us) / 1000000.0 > limite:
            self.vitesse_imu_ms = 0.0
            self.cadence_hz = 0.0
            self._dernier_cycle = None

    # ------------------------------------------------------------------
    # Assistance GPS
    # ------------------------------------------------------------------

    def _gps_exploitable(self):
        return self.gps.est_exploitable(self.hdop_max, self.nb_sat_min, self.age_max_ms)

    def _choisir_vitesse(self, dt, distance_imu):
        """Choisit la source de vitesse et de distance selon la priorité."""
        v_imu = self.vitesse_imu_ms
        if self.gps is None:
            self.source_vitesse = "imu"
            return v_imu, distance_imu

        nouvelle = self.gps.mettre_a_jour()
        self._gps_ok = self._gps_exploitable()
        if nouvelle and self._gps_ok:
            self._nouvelle_position_gps()

        if self._gps_ok:
            v_gps = self.gps.vitesse_ms
            if v_gps < self.vitesse_min_gps_ms:
                v_gps = 0.0
            if self.priorite == "gps":
                if v_imu == 0.0 and v_gps < 1.0:
                    v_gps = 0.0  # la centrale confirme l'arrêt : bruit Doppler
                self.source_vitesse = "gps"
                return v_gps, v_gps * dt
            if v_imu == 0.0 and v_gps >= self.vitesse_secours_ms:
                self.source_vitesse = "gps (secours)"
                return v_gps, v_gps * dt

        self.source_vitesse = "imu"
        return v_imu, distance_imu

    def _nouvelle_position_gps(self):
        """Recalage de la position ; en course régulière, étalonnage du cap et de K."""
        gps = self.gps
        if not gps.origine_definie():
            gps.definir_origine()
        self.nb_positions_gps += 1
        v_gps = gps.vitesse_ms
        regulier = abs(self.indicateurs.acceleration_ms2) < 0.5

        if v_gps >= self.vitesse_min_recalage_ms and regulier:
            # Cap GPS (degrés depuis le nord, sens horaire) -> angle
            # trigonométrique depuis l'est ; moyenne circulaire de l'écart
            if gps.cap_deg is not None:
                phi = (90.0 - gps.cap_deg) * _DEG_VERS_RAD
                ecart = phi - self.navigation.cap_rad
                g = self.gain_recalage
                self._sin_moy += g * (math.sin(ecart) - self._sin_moy)
                self._cos_moy += g * (math.cos(ecart) - self._cos_moy)
                self.navigation.decalage_cap_rad = math.atan2(self._sin_moy, self._cos_moy)
                self.cap_aligne = True
            # K : rapport vitesse GPS / vitesse des cycles
            if self.vitesse_imu_ms > 0.5 and self._allure_dernier_cycle is not None:
                rapport = v_gps / self.vitesse_imu_ms
                if 0.5 < rapport < 2.0:
                    self.modele.ajuster(self._allure_dernier_cycle, rapport, self.gain_recalage)

        position = gps.position_locale()
        if position is not None:
            self.navigation.x = position[0]
            self.navigation.y = position[1]

    # ------------------------------------------------------------------
    # Résultats
    # ------------------------------------------------------------------

    def temps_depuis_dernier_cycle_s(self, t_us):
        """Secondes écoulées depuis le dernier cycle (None si aucun)."""
        if self._t_dernier_cycle_us is None:
            return None
        return (t_us - self._t_dernier_cycle_us) / 1000000.0

    def etat(self):
        """État instantané (affichage temps réel sur la base C0, FP5)."""
        ind = self.indicateurs
        e = {
            "vitesse_kmh": round(ind.vitesse_ms * 3.6, 1),
            "acceleration_ms2": round(ind.acceleration_ms2, 2),
            "distance_km": round(ind.distance_m / 1000.0, 3),
            "x_m": round(self.navigation.x, 1),
            "y_m": round(self.navigation.y, 1),
            "cap_deg": round(self.navigation.cap_deg()),
            self.NOM_CYCLES: self.nb_cycles,
            "cadence_hz": round(self.cadence_hz, 2),
            "source": self.source_vitesse,
        }
        if self.gps is not None:
            e["gps_ok"] = self._gps_ok
            e["nb_satellites"] = getattr(self.gps, "nb_satellites", None)
            e["hdop"] = getattr(self.gps, "hdop", None)
        return e

    def resume(self):
        """Indicateurs de fin de séance (FP4)."""
        r = self.indicateurs.resume()
        r[self.NOM_CYCLES] = self.nb_cycles
        r["modele"] = self.modele.vers_dict()
        if self.gps is not None:
            r["nb_positions_gps"] = self.nb_positions_gps
        return r


# =============================================================================
# 7. Programmes communs
# =============================================================================

def creer_gps(frequence_gps_hz=5, configuration_ublox=True):
    """
    Crée le récepteur GPS (capteur_gps.py) ; configuration_ublox=False pour
    un récepteur qui n'est pas de marque u-blox (il reste alors à 1 Hz ou à
    sa fréquence par défaut).
    """
    from capteur_gps import RecepteurGPS, creer_uart_gps, configurer_ublox
    uart = creer_uart_gps()
    if configuration_ublox:
        print("Configuration GPS :", configurer_ublox(uart, frequence_gps_hz))
    return RecepteurGPS(uart)


def executer_seance(capteur, seance, frequence_hz, duree_s, periode_affichage_s=1.0,
                    nom_resultats=None, periode_sauvegarde_s=30.0,
                    journal_etat=None, journal_evenements=None, afficher=True,
                    sur_evenement=None):
    """
    Boucle d'acquisition d'une séance complète.

    Paramètres
        capteur, seance       objets déjà configurés
        frequence_hz          fréquence d'échantillonnage
        duree_s               durée de la séance
        periode_affichage_s   intervalle d'affichage de seance.etat()
        nom_resultats         fichier JSON du résumé ; réécrit toutes les
                              periode_sauvegarde_s secondes ET à la fin, pour
                              qu'une coupure d'alimentation ne fasse perdre
                              au plus que la dernière période. None = pas de
                              sauvegarde automatique.
        periode_sauvegarde_s  intervalle entre deux sauvegardes du résumé
        journal_etat          journal.JournalEtat ou None
        journal_evenements    journal.JournalEvenements ou None ; déclenché
                              à chaque frappe validée (mode foot)
        afficher              False pour une séance silencieuse (sur batterie)
        sur_evenement         fonction appelée avec le dict de chaque frappe
                              validée (mode foot), par exemple pour l'afficher

    Retour : le résumé de séance, enrichi des compteurs de qualité
    (echantillons, manques, erreurs) et de l'état des journaux.
    """
    t_affichage = 0
    t_sauvegarde = 0
    periode_affichage_us = int(periode_affichage_s * 1000000)
    periode_sauvegarde_us = int(periode_sauvegarde_s * 1000000)

    for ech in capteur.echantillons(frequence_hz, duree_s):
        evenement = seance.traiter(*ech)
        if journal_evenements is not None:
            journal_evenements.ajouter(*ech)
        if evenement is not None:
            if journal_evenements is not None:
                journal_evenements.declencher()
            if sur_evenement is not None:
                sur_evenement(evenement)
        # etat() construit un dict : ne l'appeler que si quelqu'un l'utilise
        affichage_du = afficher and ech[0] - t_affichage >= periode_affichage_us
        journal_du = journal_etat is not None and journal_etat.est_du(ech[0])
        if affichage_du or journal_du:
            etat = seance.etat()
            if journal_du:
                journal_etat.mettre_a_jour(ech[0], etat)
            if affichage_du:
                t_affichage = ech[0]
                print(etat)
        if nom_resultats is not None and ech[0] - t_sauvegarde >= periode_sauvegarde_us:
            t_sauvegarde = ech[0]
            _sauvegarder_resume(nom_resultats, capteur, seance,
                                journal_etat, journal_evenements, partiel=True)

    seance.finaliser()
    for j in (journal_etat, journal_evenements):
        if j is not None:
            j.fermer()
    resume = _sauvegarder_resume(nom_resultats, capteur, seance,
                                 journal_etat, journal_evenements, partiel=False)
    if afficher:
        print("Echantillons : {} | manques : {} | erreurs de lecture : {}".format(
            capteur.nb_echantillons, capteur.nb_manques, capteur.nb_erreurs))
    return resume


def _sauvegarder_resume(nom_resultats, capteur, seance, journal_etat,
                        journal_evenements, partiel):
    """
    Construit le résumé (compteurs de qualité et journaux compris) et
    l'écrit si nom_resultats est fourni. Une écriture qui échoue ne doit pas
    interrompre la séance : l'erreur est ignorée ici, la séance continue.
    """
    from journal import horodatage
    resume = seance.resume()
    resume["partiel"] = partiel
    resume["echantillons"] = capteur.nb_echantillons
    resume["echantillons_manques"] = capteur.nb_manques
    resume["erreurs_lecture"] = capteur.nb_erreurs
    resume["horodatage"] = horodatage()
    if journal_etat is not None:
        resume["journal_etat"] = journal_etat.resume()
    if journal_evenements is not None:
        resume["journal_evenements"] = journal_evenements.resume()
    if nom_resultats is not None:
        try:
            sauvegarder_json(nom_resultats, resume)
        except OSError:
            pass
    return resume


def etalonner_sur_distance(capteur, seance, frequence_hz, distance_m, allure,
                           fichier_modele, duree_max_s=180, arret_s=3.0,
                           nb_cycles_min=6):
    """
    Étalonne K[allure] : le joueur parcourt distance_m (mesurée) en ligne
    droite à l'allure voulue, puis reste immobile arret_s secondes.
    Ordre conseillé : "marche", puis "course", puis "sprint" (départ lancé).
    Le modèle est enregistré dans fichier_modele. Retour : (ancien K, nouveau K).
    """
    print("Partez ! {} m a l'allure '{}', puis immobile {} s".format(distance_m, allure, arret_s))
    for ech in capteur.echantillons(frequence_hz, duree_max_s):
        seance.traiter(*ech)
        attente = seance.temps_depuis_dernier_cycle_s(ech[0])
        if seance.nb_cycles >= nb_cycles_min and attente is not None and attente > arret_s:
            break
    seance.finaliser()
    print("Cycles : {} | distances estimees : {}".format(seance.nb_cycles, seance.distance_par_allure))
    ancien = seance.modele.k[allure]
    nouveau = seance.modele.etalonner(distance_m, seance.distance_par_allure, allure)
    seance.modele.sauvegarder(fichier_modele)
    print("K {} : {:.3f} -> {:.3f} (enregistre dans {})".format(allure, ancien, nouveau, fichier_modele))
    return ancien, nouveau


def afficher_resume(r):
    """Affiche les indicateurs de déplacement d'un résumé de séance."""
    print("----- Deplacement -----")
    print("Distance       : {:.3f} km".format(r["distance_km"]))
    print("Activite       : {:.1f} %".format(r["activite_pct"]))
    print("Sprint max     : {:.1f} km/h".format(r["sprint_max_kmh"]))
    print("Course         : {}".format(r["course"]))
    print("Sprints        : {} (seuil {:.1f} km/h)".format(r["sprints"], r["seuil_sprint_kmh"]))
    print("Accelerations  : {}".format(r["accelerations"]))
    print("Decelerations  : {}".format(r["decelerations"]))
    print("Duree          : {}".format(format_min_s(r["duree_s"])))

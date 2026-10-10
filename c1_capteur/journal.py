"""
journal.py
==========

Traçabilité des séances : horodatage, noms de fichiers uniques, journal
d'état et enregistrement brut autour des frappes. MicroPython, Pico 2 W.

Pourquoi ce module
------------------
    - Les seuils de détection (frappes, pas, foulées) ne sont pas encore
      réglés. Sans trace de ce qui a été mesuré, chaque sortie terrain dont
      les seuils se révèlent mauvais est une sortie à refaire.
    - Un résumé écrit seulement à la fin est perdu si la batterie se vide.
    - Un nom de fichier fixe fait écraser la séance précédente.

Trois niveaux de trace, du plus léger au plus lourd
---------------------------------------------------
    résumé JSON          quelques centaines d'octets, réécrit toutes les 30 s
    JournalEtat          CSV du résultat calculé, 1 ligne/s (~40 o/s)
    JournalEvenements    échantillons bruts autour de chaque frappe, en
                         binaire (16 o/échantillon)

Pourquoi pas tout enregistrer en brut
-------------------------------------
À 125 Hz, 16 octets par échantillon font 2 ko/s, soit 10,8 Mo pour un match
de 90 min : bien plus que la mémoire libre du Pico (quelques Mo). Le brut
n'est donc conservé qu'autour des événements intéressants ; pour une
acquisition brute continue et courte, utiliser CapteurBNO055.enregistrer().

Sécurité d'écriture
-------------------
Aucune écriture ne doit faire échouer une séance : toutes les opérations
fichier sont protégées, et un journal qui échoue se désactive en notant la
raison dans son attribut `erreur`. L'espace libre est vérifié avant chaque
bloc ; sous `octets_min_libres`, le journal s'arrête proprement.

Horloge
-------
Le Pico n'a pas de pile d'horloge : après une coupure d'alimentation, la
date repart à l'origine du système. Thonny met l'horloge à jour quand il se
connecte, donc la date est juste tant que le Pico n'a pas été débranché.
`horodatage()` renvoie la date ET un drapeau `horloge_valide` ; le numéro de
séance (compteur persistant) reste fiable dans tous les cas.
"""

import json
import os
import struct
import time


# Format d'un échantillon brut : t_us (int32) + ax ay az gx gy gz (int16)
_FMT_ENREG = "<ihhhhhh"
TAILLE_ENREG = 16

# Échelles de conversion (identiques aux registres du BNO055)
_LSB_PAR_MS2 = 100.0
_LSB_PAR_DPS = 16.0
_LIMITE_INT16 = 32767

_ANNEE_MIN_VALIDE = 2024
FICHIER_COMPTEUR = "compteur_seances.json"


# =============================================================================
# Espace, horloge, numérotation
# =============================================================================

def espace_libre_octets():
    """Octets libres sur la mémoire du Pico, ou None si l'info est indisponible."""
    try:
        s = os.statvfs("/")
        return s[0] * s[3]
    except (OSError, AttributeError):
        return None


def horodatage():
    """
    Date, heure et validité de l'horloge.

    Retour : {"date_heure": "AAAA-MM-JJ hh:mm:ss", "horloge_valide": bool}
    horloge_valide est False si l'année est antérieure à 2024, c'est-à-dire
    si le Pico a été alimenté sans être relié à un PC qui règle son horloge.
    """
    t = time.localtime()
    return {
        "date_heure": "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            t[0], t[1], t[2], t[3], t[4], t[5]),
        "horloge_valide": t[0] >= _ANNEE_MIN_VALIDE,
    }


def prochain_numero(cle="seance"):
    """
    Incrémente et renvoie le compteur persistant associé à `cle`
    (fichier FICHIER_COMPTEUR). Renvoie 0 si le compteur est illisible et
    non réinscriptible, ce qui n'empêche pas la séance de se dérouler.
    """
    compteurs = {}
    try:
        with open(FICHIER_COMPTEUR) as f:
            lu = json.loads(f.read())
        if isinstance(lu, dict):
            compteurs = lu
    except (OSError, ValueError):
        compteurs = {}
    try:
        numero = int(compteurs.get(cle, 0)) + 1
    except (TypeError, ValueError):
        numero = 1
    compteurs[cle] = numero
    try:
        with open(FICHIER_COMPTEUR, "w") as f:
            f.write(json.dumps(compteurs))
    except OSError:
        return 0
    return numero


def nom_unique(prefixe, extension, numero):
    """Nom de fichier numéroté, ex. nom_unique("seance_foot", "json", 7)
    -> "seance_foot_0007.json"."""
    return "{}_{:04d}.{}".format(prefixe, numero, extension)


# =============================================================================
# Journal d'état : CSV basse cadence du résultat calculé
# =============================================================================

class JournalEtat:
    """
    Écrit une ligne CSV par seconde avec l'état calculé de la séance.
    Permet de rejouer la chronologie d'une séance (vitesse, source, position)
    sans conserver le brut.

    Colonnes : t_s, vitesse_kmh, acceleration_ms2, distance_km, x_m, y_m,
               cap_deg, cycles, cadence_hz, source, gps_ok

    Taille : environ 40 octets par ligne, soit ~2,4 ko/min et ~216 ko pour
    90 min. L'écriture se fait par blocs pour limiter les accès à la mémoire
    flash, qui bloquent le processeur quelques dizaines de millisecondes.
    """

    COLONNES = ("t_s", "vitesse_kmh", "acceleration_ms2", "distance_km",
                "x_m", "y_m", "cap_deg", "cycles", "cadence_hz", "source", "gps_ok")

    def __init__(self, nom_fichier, periode_s=1.0, lignes_par_bloc=20,
                 octets_min_libres=100000):
        """
        Paramètres
            nom_fichier        fichier CSV créé (écrasé s'il existe)
            periode_s          intervalle entre deux lignes
            lignes_par_bloc    lignes gardées en RAM avant écriture
            octets_min_libres  sous ce seuil d'espace libre, le journal
                               s'arrête proprement
        """
        if periode_s <= 0 or lignes_par_bloc < 1:
            raise ValueError("periode_s doit etre > 0 et lignes_par_bloc >= 1")
        self.nom_fichier = nom_fichier
        self.periode_us = int(periode_s * 1000000)
        self.lignes_par_bloc = lignes_par_bloc
        self.octets_min_libres = octets_min_libres
        self.nb_lignes = 0
        self.actif = True
        self.erreur = None
        self._tampon = []
        self._t_prochaine = None
        try:
            with open(self.nom_fichier, "w") as f:
                f.write(",".join(self.COLONNES) + "\n")
        except OSError as e:
            self._desactiver("creation impossible : {}".format(e))

    def _desactiver(self, raison):
        self.actif = False
        self.erreur = raison

    def _ecrire_bloc(self):
        """Écrit le tampon ; désactive le journal en cas de problème."""
        if not self._tampon:
            return
        libre = espace_libre_octets()
        if libre is not None and libre < self.octets_min_libres:
            self._desactiver("espace insuffisant ({} octets libres)".format(libre))
            self._tampon = []
            return
        try:
            with open(self.nom_fichier, "a") as f:
                f.write("".join(self._tampon))
        except OSError as e:
            self._desactiver("ecriture impossible : {}".format(e))
        self._tampon = []

    def est_du(self, t_us):
        """
        True si une ligne doit être écrite à cet instant. À tester avant de
        construire l'état : seance.etat() alloue un dict, inutile de le faire
        à chaque échantillon alors qu'on n'en garde qu'un par seconde.
        """
        if not self.actif:
            return False
        if self._t_prochaine is None:
            self._t_prochaine = t_us
        return t_us >= self._t_prochaine

    def mettre_a_jour(self, t_us, etat):
        """
        Ajoute une ligne si la période est écoulée (voir est_du).
        `etat` est le dict renvoyé par SeanceDeplacement.etat().
        """
        if not self.est_du(t_us):
            return
        self._t_prochaine += self.periode_us
        cycles = etat.get("nb_pas", etat.get("nb_foulees", etat.get("nb_cycles", 0)))
        gps_ok = etat.get("gps_ok")
        self._tampon.append("{:.1f},{},{},{},{},{},{},{},{},{},{}\n".format(
            t_us / 1000000.0,
            etat.get("vitesse_kmh", 0), etat.get("acceleration_ms2", 0),
            etat.get("distance_km", 0), etat.get("x_m", 0), etat.get("y_m", 0),
            etat.get("cap_deg", 0), cycles, etat.get("cadence_hz", 0),
            etat.get("source", ""), "" if gps_ok is None else int(gps_ok)))
        self.nb_lignes += 1
        if len(self._tampon) >= self.lignes_par_bloc:
            self._ecrire_bloc()

    def fermer(self):
        """Écrit les lignes restantes."""
        if self.actif:
            self._ecrire_bloc()

    def resume(self):
        """État du journal, à joindre au résumé de séance."""
        return {"fichier": self.nom_fichier, "lignes": self.nb_lignes,
                "actif": self.actif, "erreur": self.erreur}


# =============================================================================
# Journal d'événements : brut autour de chaque frappe
# =============================================================================

class JournalEvenements:
    """
    Tampon circulaire d'échantillons bruts en RAM, vidé dans un fichier
    binaire autour de chaque événement déclenché (une frappe validée).

    Permet de régler après coup les seuils de DetecteurFrappe sur de vraies
    frappes, sans retourner sur le terrain.

    Taille : (nb_avant + nb_apres) × 16 octets en RAM, et autant par
    événement dans le fichier. Par défaut à 125 Hz : 2 s avant et 1 s après,
    soit 375 échantillons = 6 ko par frappe.

    Les valeurs sont stockées aux mêmes échelles que les registres du
    BNO055 (accélération ×100, gyroscope ×16), avec le biais du gyroscope
    déjà retranché. Relecture : convertir_csv().
    """

    def __init__(self, nom_fichier, frequence_hz=125, secondes_avant=2.0,
                 secondes_apres=1.0, nb_evenements_max=40,
                 octets_min_libres=100000):
        """
        Paramètres
            nom_fichier         fichier binaire créé (écrasé s'il existe)
            frequence_hz        fréquence d'échantillonnage de la séance
            secondes_avant      durée conservée avant l'événement
            secondes_apres      durée conservée après l'événement
            nb_evenements_max   au-delà, les événements ne sont plus écrits
                                (borne la taille du fichier)
            octets_min_libres   seuil d'arrêt sur l'espace libre
        """
        if frequence_hz <= 0 or secondes_avant < 0 or secondes_apres <= 0:
            raise ValueError("frequence_hz > 0, secondes_avant >= 0, secondes_apres > 0")
        self.nom_fichier = nom_fichier
        self.nb_avant = int(secondes_avant * frequence_hz)
        self.nb_apres = int(secondes_apres * frequence_hz)
        self.taille_fenetre = self.nb_avant + self.nb_apres
        self.nb_evenements_max = nb_evenements_max
        self.octets_min_libres = octets_min_libres
        self.nb_evenements = 0
        self.actif = True
        self.erreur = None
        self._anneau = bytearray(self.taille_fenetre * TAILLE_ENREG)
        self._position = 0        # prochain emplacement d'écriture
        self._remplissage = 0     # échantillons valides dans l'anneau
        self._restant = None      # échantillons restants avant de vider
        try:
            with open(self.nom_fichier, "wb"):
                pass
        except OSError as e:
            self._desactiver("creation impossible : {}".format(e))

    def _desactiver(self, raison):
        self.actif = False
        self.erreur = raison

    @staticmethod
    def _borner(valeur):
        """Borne un entier dans la plage d'un int16 signé."""
        v = int(valeur)
        if v > _LIMITE_INT16:
            return _LIMITE_INT16
        if v < -_LIMITE_INT16 - 1:
            return -_LIMITE_INT16 - 1
        return v

    def ajouter(self, t_us, ax, ay, az, gx, gy, gz):
        """
        Mémorise un échantillon (µs, m/s², °/s) dans le tampon circulaire,
        et vide la fenêtre si un événement déclenché est arrivé à son terme.
        """
        if not self.actif:
            return
        struct.pack_into(
            _FMT_ENREG, self._anneau, self._position * TAILLE_ENREG,
            t_us & 0x7FFFFFFF,
            self._borner(ax * _LSB_PAR_MS2), self._borner(ay * _LSB_PAR_MS2),
            self._borner(az * _LSB_PAR_MS2), self._borner(gx * _LSB_PAR_DPS),
            self._borner(gy * _LSB_PAR_DPS), self._borner(gz * _LSB_PAR_DPS))
        self._position = (self._position + 1) % self.taille_fenetre
        if self._remplissage < self.taille_fenetre:
            self._remplissage += 1
        if self._restant is not None:
            self._restant -= 1
            if self._restant <= 0:
                self._restant = None
                self._vider()

    def declencher(self):
        """
        Signale un événement : la fenêtre sera écrite une fois les
        `nb_apres` échantillons suivants mémorisés. Un déclenchement pendant
        l'attente est ignoré (la fenêtre en cours le couvre déjà).
        """
        if not self.actif or self._restant is not None:
            return
        if self.nb_evenements >= self.nb_evenements_max:
            return
        self._restant = self.nb_apres

    def _vider(self):
        """Écrit la fenêtre courante, du plus ancien au plus récent."""
        libre = espace_libre_octets()
        if libre is not None and libre < self.octets_min_libres:
            self._desactiver("espace insuffisant ({} octets libres)".format(libre))
            return
        debut = (self._position - self._remplissage) % self.taille_fenetre
        try:
            with open(self.nom_fichier, "ab") as f:
                for i in range(self._remplissage):
                    j = (debut + i) % self.taille_fenetre
                    f.write(self._anneau[j * TAILLE_ENREG:(j + 1) * TAILLE_ENREG])
        except OSError as e:
            self._desactiver("ecriture impossible : {}".format(e))
            return
        self.nb_evenements += 1

    def fermer(self):
        """Écrit la fenêtre en cours si un événement attendait sa fin."""
        if self.actif and self._restant is not None:
            self._restant = None
            self._vider()

    def resume(self):
        """État du journal, à joindre au résumé de séance."""
        return {"fichier": self.nom_fichier, "evenements": self.nb_evenements,
                "echantillons_par_evenement": self.taille_fenetre,
                "actif": self.actif, "erreur": self.erreur}


# =============================================================================
# Relecture
# =============================================================================

def convertir_csv(nom_binaire, nom_csv, taille_fenetre=None):
    """
    Convertit un fichier d'événements en CSV lisible dans un tableur.
    Fonctionne sur le Pico comme sur un PC (Python 3).

    Colonnes : evenement, t_us, ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, gz_dps
    `taille_fenetre` (échantillons par événement) numérote les événements ;
    la valeur figure dans le résumé de séance. Sans elle, tout porte le
    numéro 0.

    Retour : nombre de lignes écrites.
    """
    n = 0
    with open(nom_binaire, "rb") as src, open(nom_csv, "w") as dst:
        dst.write("evenement,t_us,ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps\n")
        while True:
            bloc = src.read(TAILLE_ENREG)
            if len(bloc) < TAILLE_ENREG:
                break
            t, ax, ay, az, gx, gy, gz = struct.unpack(_FMT_ENREG, bloc)
            evenement = n // taille_fenetre if taille_fenetre else 0
            dst.write("{},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
                evenement, t,
                ax / _LSB_PAR_MS2, ay / _LSB_PAR_MS2, az / _LSB_PAR_MS2,
                gx / _LSB_PAR_DPS, gy / _LSB_PAR_DPS, gz / _LSB_PAR_DPS))
            n += 1
    return n

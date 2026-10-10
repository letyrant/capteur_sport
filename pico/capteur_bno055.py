"""
capteur_bno055.py
=================

Accès à la centrale inertielle BNO055 (breakout Adafruit) sur Raspberry Pi
Pico 2 W, en MicroPython. Ce fichier ne contient que ce qui est propre au
capteur ; les traitements sont dans mode_foot.py et mode_rugby.py.

Fichiers du projet (à copier sur le Pico, racine ou /lib)
---------------------------------------------------------
    bno055_base.py      pilote de base de Peter Hinch (dépôt micropython-bno055)
    bno055.py           pilote de Peter Hinch (celui fourni)
    capteur_bno055.py   ce fichier : accès au BNO055
    capteur_gps.py      accès au GPS (optionnel)
    deplacement.py      tronc commun : pas/foulées, vitesse, indicateurs, GPS
    mode_foot.py        capteur sur le mollet : frappes + course
    mode_rugby.py       capteur dans le dos (col du maillot) : course

Câblage (breakout BNO055 -> Pico 2 W), bus I2C0
-----------------------------------------------
    VIN -> 3V3(OUT)  broche physique 36
    GND -> GND       broche physique 38
    SDA -> GP4       broche physique 6
    SCL -> GP5       broche physique 7
    ADR non connecté -> adresse 0x28 (ADR relié à 3,3 V -> 0x29)

Réglages du capteur
-------------------
    passer_en_mode_foot()   ACCGYRO, ±16 g, ±2000 °/s, bandes passantes
                            62 / 47 Hz pour un échantillonnage à 125 Hz
    passer_en_mode_rugby()  ACCGYRO, ±16 g, ±2000 °/s, bandes passantes
                            31 / 32 Hz pour un échantillonnage à 100 Hz
    passer_en_mode_fusion() NDOF ou IMUPLUS : le BNO055 calcule lui-même
                            l'orientation, à 100 Hz, mais limite
                            l'accéléromètre à ±4 g (inutilisable pour une
                            frappe ou des contacts)

Vitesse de lecture du BNO055 (mesurée sur le montage)
-----------------------------------------------------
Une lecture I2C dure environ 0,8 ms + 0,22 ms par octet, à 100 kHz comme
à 400 kHz : le BNO055 ralentit lui-même chaque octet. Une lecture
complète (accéléromètre + gyroscope, 18 octets) prend ≈ 4,7 ms, soit
≈ 210 lectures/s au maximum : d'où 125 Hz en mode foot et 100 Hz en
mode rugby.

Unités (réglage par défaut du BNO055 après reset)
-------------------------------------------------
    accélération          m/s²   (1 m/s² = 100 LSB)
    vitesse angulaire     °/s    (1 °/s  = 16 LSB)
    champ magnétique      µT     (1 µT   = 16 LSB)
    angles d'Euler        °      (cap, roulis, tangage)
    quaternion            sans unité (w, x, y, z)
    température           °C

Remarque sur bno055.py
----------------------
Ce module n'utilise pas BNO055.iget() : sa conversion _bytes_toint() est
fausse pour certaines valeurs négatives (priorité de "+" sur "|" quand
l'octet de poids faible vaut 0, ex. 0xFE00 donne -256 au lieu de -512).
Les conversions sont faites ici avec struct.unpack, qui est exact.
"""

import gc
import math
import struct
import time

from machine import I2C, Pin
from micropython import const

from bno055 import (
    BNO055,
    ACC,
    GYRO,
    CONFIG_MODE,
    ACCGYRO_MODE,
    IMUPLUS_MODE,
    NDOF_MODE,
)


# =============================================================================
# Constantes
# =============================================================================

G = 9.80665  # accélération de la pesanteur standard (m/s²)

ADRESSE_BNO055 = const(0x28)  # adresse I2C par défaut (ADR non connecté)

# Registres du BNO055 (page 0)
_REG_ACC_DATA = const(0x08)   # début du bloc ACC(6) + MAG(6) + GYR(6) octets
_REG_OFFSETS = const(0x55)    # début des 22 octets de calibration
_NB_OCTETS_OFFSETS = const(22)

# Facteurs d'échelle (LSB par unité)
_LSB_PAR_MS2 = 100.0
_LSB_PAR_DPS = 16.0

# Plages utilisées par les modes foot et rugby (maximums du BNO055)
ACC_PLAGE_G = 16          # accéléromètre ±16 g
GYRO_PLAGE_DPS = 2000     # gyroscope ±2000 °/s

# Réglages du mode foot (échantillonnage 125 Hz -> bande passante < 62,5 Hz)
ACC_BP_FOOT_HZ = 62
GYRO_BP_FOOT_HZ = 47

# Réglages du mode rugby (échantillonnage 100 Hz -> bande passante <= 50 Hz)
ACC_BP_RUGBY_HZ = 31
GYRO_BP_RUGBY_HZ = 32

# Seuil (fraction de la pleine échelle) au-delà duquel on considère
# qu'un axe est saturé
_FRACTION_SATURATION = 0.98

# Formats struct
_FMT_BLOC = "<hhhhhhhhh"   # 9 entiers 16 bits signés little-endian
_FMT_ENREG = "<ihhhhhh"    # t_us (int32) + ax ay az gx gy gz bruts (int16)
_TAILLE_ENREG = const(16)  # 4 + 6 × 2 octets

_MODES_FUSION = (NDOF_MODE, IMUPLUS_MODE)

NOMS_MODES = {
    CONFIG_MODE: "CONFIG",
    ACCGYRO_MODE: "ACCGYRO (foot / rugby)",
    IMUPLUS_MODE: "IMUPLUS (fusion 6 axes)",
    NDOF_MODE: "NDOF (fusion 9 axes)",
}


# =============================================================================
# Fonctions utilitaires
# =============================================================================

def norme3(x, y, z):
    """Norme euclidienne d'un vecteur 3D (même unité que les composantes)."""
    return math.sqrt(x * x + y * y + z * z)


def ms2_vers_g(a_ms2):
    """Convertit une accélération de m/s² en g."""
    return a_ms2 / G


def dps_vers_rads(w_dps):
    """Convertit une vitesse angulaire de °/s en rad/s."""
    return w_dps * math.pi / 180.0


def ms_vers_kmh(v_ms):
    """Convertit une vitesse de m/s en km/h."""
    return v_ms * 3.6


def creer_bus_i2c(id_bus=0, broche_sda=4, broche_scl=5, frequence_hz=400000):
    """
    Crée le bus I2C matériel du Pico.

    Paramètres
        id_bus        0 ou 1 (I2C0 : GP4/GP5 par défaut)
        broche_sda    numéro GPIO de SDA
        broche_scl    numéro GPIO de SCL
        frequence_hz  400 kHz (maximum du BNO055). Si des OSError
                      apparaissent (câbles longs), descendre à 100000.

    Retour
        objet machine.I2C
    """
    return I2C(id_bus, sda=Pin(broche_sda), scl=Pin(broche_scl), freq=frequence_hz)


def sauvegarder_csv(donnees, nb, nom_fichier, biais_gyro=(0.0, 0.0, 0.0)):
    """
    Écrit un enregistrement (voir CapteurBNO055.enregistrer) en CSV.

    Paramètres
        donnees      bytearray renvoyé par enregistrer()
        nb           nombre d'échantillons valides
        nom_fichier  ex. "essai_01.csv"
        biais_gyro   biais à retrancher au gyroscope (°/s), en général
                     capteur.biais_gyro

    Colonnes : t_us, ax, ay, az (m/s²), gx, gy, gz (°/s)
    Compter environ 50 octets par ligne sur la flash du Pico.
    """
    bx, by, bz = biais_gyro
    with open(nom_fichier, "w") as f:
        f.write("t_us,ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps\n")
        for i in range(nb):
            t, ax, ay, az, gx, gy, gz = struct.unpack_from(
                _FMT_ENREG, donnees, i * _TAILLE_ENREG)
            f.write("{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
                t,
                ax / _LSB_PAR_MS2, ay / _LSB_PAR_MS2, az / _LSB_PAR_MS2,
                gx / _LSB_PAR_DPS - bx, gy / _LSB_PAR_DPS - by,
                gz / _LSB_PAR_DPS - bz))


# =============================================================================
# Classe principale : accès au capteur
# =============================================================================

class CapteurBNO055:
    """
    Interface haut niveau du BNO055.

    Exemple
        i2c = creer_bus_i2c()
        capteur = CapteurBNO055(i2c)
        capteur.passer_en_mode_foot()
        capteur.estimer_biais_gyro()
        for t_us, ax, ay, az, gx, gy, gz in capteur.echantillons(125, 10):
            ...

    Attributs publics
        imu              objet BNO055 du pilote (accès bas niveau)
        mode_courant     mode de fonctionnement actuel
        biais_gyro       biais du gyroscope (°/s) retranché en modes foot et rugby
        nb_echantillons  échantillons produits par le dernier echantillons()
        nb_manques       échantillons manqués (retards) pendant echantillons()
        nb_erreurs       échantillons perdus sur erreur de lecture I2C
    """

    def __init__(self, i2c, adresse=ADRESSE_BNO055, quartz_externe=True,
                 transposition=(0, 1, 2), signe=(0, 0, 0),
                 delai_demarrage_ms=1000):
        """
        Initialise le capteur.

        Paramètres
            i2c                 bus machine.I2C (voir creer_bus_i2c)
            adresse             0x28 ou 0x29
            quartz_externe      True pour le breakout Adafruit (quartz 32 kHz)
            transposition       remappage des axes (voir bno055.py),
                                (0, 1, 2) = pas de changement
            signe               inversion des axes, (0, 0, 0) = aucune
            delai_demarrage_ms  temps maximal d'attente de la réponse du
                                capteur après mise sous tension

        Le constructeur du pilote vérifie l'identifiant de la puce, effectue
        un reset logiciel (environ 1,2 s) et laisse le capteur en mode NDOF.

        Lève RuntimeError si le capteur ne répond pas.
        """
        self.i2c = i2c
        self.adresse = adresse
        self._attendre_presence(delai_demarrage_ms)
        self.imu = BNO055(i2c, address=adresse, crystal=quartz_externe,
                          transpose=transposition, sign=signe)
        self.mode_courant = NDOF_MODE
        self.biais_gyro = (0.0, 0.0, 0.0)
        self.nb_echantillons = 0
        self.nb_manques = 0
        self.nb_erreurs = 0
        self._buf = bytearray(18)

    # ------------------------------------------------------------------
    # Outils internes
    # ------------------------------------------------------------------

    def _attendre_presence(self, delai_ms):
        """Attend que le BNO055 réponde sur le bus (démarrage ~650 ms)."""
        t0 = time.ticks_ms()
        trouves = []
        while True:
            try:
                trouves = self.i2c.scan()
            except OSError:
                trouves = []
            if self.adresse in trouves:
                return
            if time.ticks_diff(time.ticks_ms(), t0) >= delai_ms:
                raise RuntimeError(
                    "BNO055 introuvable a l'adresse 0x{:02X}. Adresses vues : {}. "
                    "Verifier cablage SDA/SCL, alimentation et adresse.".format(
                        self.adresse, [hex(a) for a in trouves]))
            time.sleep_ms(50)

    def _verifier_mode(self, modes_autorises, nom_fonction):
        """Lève RuntimeError si le mode courant n'est pas autorisé."""
        if self.mode_courant not in modes_autorises:
            raise RuntimeError("{}() impossible en mode {} ; modes valides : {}".format(
                nom_fonction,
                NOMS_MODES.get(self.mode_courant, self.mode_courant),
                [NOMS_MODES.get(m, m) for m in modes_autorises]))

    def _lire_brut(self):
        """
        Lit en une seule transaction I2C les 18 octets 0x08 à 0x19
        (accéléromètre, magnétomètre, gyroscope) et renvoie les valeurs
        brutes (ax, ay, az, gx, gy, gz) en LSB.

        Réessaie 2 fois en cas d'erreur I2C passagère.
        """
        for essai in range(3):
            try:
                self.i2c.readfrom_mem_into(self.adresse, _REG_ACC_DATA, self._buf)
                break
            except OSError:
                if essai == 2:
                    raise
        v = struct.unpack(_FMT_BLOC, self._buf)
        # v[3:6] = magnétomètre, ignoré ici
        return (v[0], v[1], v[2], v[6], v[7], v[8])

    def _convertir(self, brut):
        """Convertit (ax, ay, az, gx, gy, gz) bruts en m/s² et °/s."""
        if self.mode_courant == ACCGYRO_MODE:
            bx, by, bz = self.biais_gyro
        else:
            bx = by = bz = 0.0
        return (brut[0] / _LSB_PAR_MS2,
                brut[1] / _LSB_PAR_MS2,
                brut[2] / _LSB_PAR_MS2,
                brut[3] / _LSB_PAR_DPS - bx,
                brut[4] / _LSB_PAR_DPS - by,
                brut[5] / _LSB_PAR_DPS - bz)

    # ------------------------------------------------------------------
    # Modes de fonctionnement
    # ------------------------------------------------------------------

    def passer_en_mode_foot(self, bp_acc_hz=ACC_BP_FOOT_HZ,
                            bp_gyro_hz=GYRO_BP_FOOT_HZ):
        """
        Configure le capteur pour le mode foot (capteur sur le mollet,
        frappes et course, échantillonnage 125 Hz), mode ACCGYRO.

        Accéléromètre ±16 g, gyroscope ±2000 °/s, magnétomètre éteint,
        pas de fusion.

        Paramètres
            bp_acc_hz   bande passante accéléromètre :
                        8, 16, 31, 62, 125, 250, 500 ou 1000 Hz
            bp_gyro_hz  bande passante gyroscope :
                        12, 23, 32, 47, 64, 116, 230 ou 523 Hz
            Règle pratique : bande passante <= moitié de la fréquence
            d'échantillonnage (62 Hz pour 125 Hz).

        Lève ValueError (paramètre invalide) ou RuntimeError (réglage
        non pris en compte par le capteur).
        """
        self._configurer_acc_gyro(ACC_PLAGE_G, bp_acc_hz,
                                  GYRO_PLAGE_DPS, bp_gyro_hz)

    def passer_en_mode_rugby(self, bp_acc_hz=ACC_BP_RUGBY_HZ,
                             bp_gyro_hz=GYRO_BP_RUGBY_HZ):
        """
        Configure le capteur pour le mode rugby (capteur dans le dos,
        au col du maillot), utilisé par mode_rugby.py (avec ou sans GPS).

        Mode ACCGYRO : accéléromètre ±16 g (les foulées et les contacts
        dépassent les ±4 g du mode fusion), gyroscope ±2000 °/s, bandes
        passantes basses adaptées à un échantillonnage à 100 Hz.

        Paramètres : voir passer_en_mode_foot().
        """
        self._configurer_acc_gyro(ACC_PLAGE_G, bp_acc_hz,
                                  GYRO_PLAGE_DPS, bp_gyro_hz)

    def _configurer_acc_gyro(self, plage_acc_g, bp_acc_hz, plage_gyro_dps, bp_gyro_hz):
        """
        Passe en mode ACCGYRO, règle accéléromètre et gyroscope, puis relit
        les registres pour vérifier que le capteur a accepté les réglages.
        """
        if plage_acc_g not in BNO055.acc_range:
            raise ValueError("plage_acc_g doit valoir l'un de {}".format(BNO055.acc_range))
        if bp_acc_hz not in BNO055.acc_bw:
            raise ValueError("bp_acc_hz doit valoir l'un de {}".format(BNO055.acc_bw))
        if plage_gyro_dps not in BNO055.gyro_range:
            raise ValueError("plage_gyro_dps doit valoir l'un de {}".format(BNO055.gyro_range))
        if bp_gyro_hz not in BNO055.gyro_bw:
            raise ValueError("bp_gyro_hz doit valoir l'un de {}".format(BNO055.gyro_bw))

        self.imu.mode(ACCGYRO_MODE)
        self.mode_courant = ACCGYRO_MODE
        self.imu.config(ACC, (plage_acc_g, bp_acc_hz))
        self.imu.config(GYRO, (plage_gyro_dps, bp_gyro_hz))

        # Relecture pour vérifier que le capteur a bien accepté les réglages
        acc_lu = self.imu.config(ACC)
        gyro_lu = self.imu.config(GYRO)
        if acc_lu != (plage_acc_g, bp_acc_hz) or \
                gyro_lu != (plage_gyro_dps, bp_gyro_hz):
            raise RuntimeError("Configuration refusee : acc={} gyro={}".format(
                acc_lu, gyro_lu))
        self._attendre_donnees(500)

    def _attendre_donnees(self, delai_ms):
        """
        Après un changement de mode, les registres de mesure valent 0 tant que
        l'accéléromètre n'a pas produit sa première mesure (constaté sur le
        matériel : 1re lecture nulle, puis ≈ 9,8 m/s²). Attend une mesure non
        nulle ; lève RuntimeError si rien n'arrive dans delai_ms.
        """
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < delai_ms:
            b = self._lire_brut()
            if b[0] or b[1] or b[2]:
                return
            time.sleep_ms(5)
        raise RuntimeError("Aucune mesure de l'accelerometre {} ms apres le "
                           "changement de mode".format(delai_ms))

    def passer_en_mode_fusion(self, avec_magnetometre=True):
        """
        Active la fusion de données interne du BNO055 (sorties à 100 Hz).

        Paramètres
            avec_magnetometre  True  -> NDOF : orientation absolue (cap
                                        référencé au nord magnétique)
                               False -> IMUPLUS : orientation relative,
                                        insensible aux perturbations
                                        magnétiques (poteaux, tribunes)

        En fusion, l'accéléromètre est limité à ±4 g par le BNO055.
        """
        mode = NDOF_MODE if avec_magnetometre else IMUPLUS_MODE
        self.imu.mode(mode)
        self.mode_courant = mode
        self._attendre_donnees(1500)   # la fusion démarre plus lentement

    def mode_lu(self):
        """Mode réellement actif, relu dans le registre du capteur."""
        return self.imu.mode() & 0x0F

    # ------------------------------------------------------------------
    # Lectures simples (tous modes sauf CONFIG)
    # ------------------------------------------------------------------

    def lire_acceleration(self):
        """Accélération totale (gravité incluse) (ax, ay, az) en m/s²."""
        self._verifier_mode((ACCGYRO_MODE,) + _MODES_FUSION, "lire_acceleration")
        return self.imu.accel()

    def lire_gyroscope(self):
        """
        Vitesse angulaire (gx, gy, gz) en °/s.
        En modes foot et rugby, le biais estimé par estimer_biais_gyro() est retranché.
        """
        self._verifier_mode((ACCGYRO_MODE,) + _MODES_FUSION, "lire_gyroscope")
        gx, gy, gz = self.imu.gyro()
        if self.mode_courant == ACCGYRO_MODE:
            bx, by, bz = self.biais_gyro
            return (gx - bx, gy - by, gz - bz)
        return (gx, gy, gz)

    def lire_acc_gyro_rapide(self):
        """
        Lecture la plus rapide : (ax, ay, az, gx, gy, gz) en m/s² et °/s,
        en une seule transaction I2C (accélération et rotation du même
        instant).
        """
        self._verifier_mode((ACCGYRO_MODE,) + _MODES_FUSION, "lire_acc_gyro_rapide")
        return self._convertir(self._lire_brut())

    def lire_temperature(self):
        """Température de la puce en °C (entier)."""
        return self.imu.temperature()

    # ------------------------------------------------------------------
    # Lectures disponibles uniquement en fusion
    # ------------------------------------------------------------------

    def lire_euler(self):
        """
        Angles d'Euler (cap, roulis, tangage) en degrés.
        En IMUPLUS, le cap est relatif à la position de départ.
        """
        self._verifier_mode(_MODES_FUSION, "lire_euler")
        return self.imu.euler()

    def lire_quaternion(self):
        """Quaternion unitaire (w, x, y, z) ; sans singularité, à préférer aux angles d'Euler pour les calculs."""
        self._verifier_mode(_MODES_FUSION, "lire_quaternion")
        return self.imu.quaternion()

    def lire_acceleration_lineaire(self):
        """Accélération sans la gravité (ax, ay, az) en m/s², repère capteur."""
        self._verifier_mode(_MODES_FUSION, "lire_acceleration_lineaire")
        return self.imu.lin_acc()

    def lire_gravite(self):
        """Vecteur gravité (gx, gy, gz) en m/s², repère capteur."""
        self._verifier_mode(_MODES_FUSION, "lire_gravite")
        return self.imu.gravity()

    def lire_magnetometre(self):
        """Champ magnétique (mx, my, mz) en µT (mode NDOF uniquement)."""
        self._verifier_mode((NDOF_MODE,), "lire_magnetometre")
        return self.imu.mag()

    # ------------------------------------------------------------------
    # Biais du gyroscope (modes foot et rugby)
    # ------------------------------------------------------------------

    def estimer_biais_gyro(self, nb_mesures=200, periode_ms=5,
                           seuil_mouvement_dps=10.0):
        """
        Mesure le biais du gyroscope, capteur IMMOBILE, et le mémorise dans
        self.biais_gyro (retranché ensuite à toutes les lectures en modes
        foot et rugby). Durée : nb_mesures × periode_ms (1 s par défaut).

        Lève RuntimeError si une vitesse angulaire supérieure à
        seuil_mouvement_dps est vue (capteur pas immobile).

        Retour : (bx, by, bz) en °/s
        """
        self._verifier_mode((ACCGYRO_MODE,), "estimer_biais_gyro")
        if nb_mesures < 1:
            raise ValueError("nb_mesures doit etre >= 1")
        sx = sy = sz = 0.0
        for _ in range(nb_mesures):
            brut = self._lire_brut()
            gx = brut[3] / _LSB_PAR_DPS
            gy = brut[4] / _LSB_PAR_DPS
            gz = brut[5] / _LSB_PAR_DPS
            if max(abs(gx), abs(gy), abs(gz)) > seuil_mouvement_dps:
                raise RuntimeError("Mouvement detecte pendant l'estimation du biais : "
                                   "laisser le capteur immobile et recommencer.")
            sx += gx
            sy += gy
            sz += gz
            time.sleep_ms(periode_ms)
        self.biais_gyro = (sx / nb_mesures, sy / nb_mesures, sz / nb_mesures)
        return self.biais_gyro

    # ------------------------------------------------------------------
    # Acquisition cadencée (modes foot et rugby)
    # ------------------------------------------------------------------

    def echantillons(self, frequence_hz=125, duree_s=None, brut=False,
                     erreurs_max_consecutives=50):
        """
        Générateur d'échantillons à fréquence fixe.

        Paramètres
            frequence_hz  fréquence visée (vérifier qu'elle est atteignable
                          avec mesurer_frequence_max())
            duree_s       durée d'acquisition en s ; None = sans fin
                          (sortir de la boucle avec break)
            brut          False -> m/s² et °/s, biais gyro retranché
                          True  -> entiers bruts (LSB), sans correction
            erreurs_max_consecutives
                          nombre d'erreurs I2C d'affilée tolérées avant
                          d'abandonner (0 = ne jamais abandonner)

        Produit des tuples (t_us, ax, ay, az, gx, gy, gz) ; t_us est le temps
        écoulé depuis le début en microsecondes (entier croissant, sans
        débordement).

        Tolérance aux erreurs : _lire_brut() réessaie déjà 3 fois. Si la
        lecture échoue quand même, l'échantillon est abandonné, compté dans
        nb_erreurs, et l'acquisition CONTINUE : un faux contact passager ne
        doit pas faire perdre une séance entière. L'exception n'est relancée
        qu'après erreurs_max_consecutives échecs d'affilée (capteur
        débranché), car il n'y a alors plus rien à mesurer.

        Après (ou pendant) la boucle :
            self.nb_echantillons  échantillons produits
            self.nb_manques       échantillons sautés parce que le traitement
                                  entre deux échantillons était trop long
            self.nb_erreurs       échantillons perdus sur erreur de lecture

        Utilisation
            for t_us, ax, ay, az, gx, gy, gz in capteur.echantillons(125, 10):
                ...
        """
        self._verifier_mode((ACCGYRO_MODE,), "echantillons")
        if frequence_hz <= 0:
            raise ValueError("frequence_hz doit etre > 0")
        periode_us = int(1000000 / frequence_hz)
        if periode_us < 1:
            raise ValueError("frequence_hz trop elevee")
        if erreurs_max_consecutives < 0:
            raise ValueError("erreurs_max_consecutives doit etre >= 0")
        duree_ms = None if duree_s is None else int(duree_s * 1000)

        self.nb_echantillons = 0
        self.nb_manques = 0
        self.nb_erreurs = 0
        erreurs_suite = 0
        t_debut_ms = time.ticks_ms()
        t_prec = time.ticks_us()
        prochaine = t_prec
        t_us = 0

        while duree_ms is None or time.ticks_diff(time.ticks_ms(), t_debut_ms) < duree_ms:
            attente = time.ticks_diff(prochaine, time.ticks_us())
            if attente > 0:
                time.sleep_us(attente)
            elif attente <= -periode_us:
                # Retard d'au moins une période : on compte les échantillons
                # perdus et on se recale sur l'instant présent.
                self.nb_manques += (-attente) // periode_us
                prochaine = time.ticks_us()

            maintenant = time.ticks_us()
            t_us += time.ticks_diff(maintenant, t_prec)
            t_prec = maintenant
            try:
                valeurs = self._lire_brut()
            except OSError:
                # Lecture perdue : on saute l'échantillon et on poursuit.
                self.nb_erreurs += 1
                erreurs_suite += 1
                if erreurs_max_consecutives and erreurs_suite >= erreurs_max_consecutives:
                    raise
                prochaine = time.ticks_add(prochaine, periode_us)
                continue
            erreurs_suite = 0
            prochaine = time.ticks_add(prochaine, periode_us)
            self.nb_echantillons += 1

            if brut:
                yield (t_us,) + valeurs
            else:
                yield (t_us,) + self._convertir(valeurs)

    def mesurer_frequence_max(self, nb_lectures=1000):
        """
        Mesure la fréquence maximale de lecture (Hz) atteignable sur ce
        montage, sans traitement. Viser au plus ~70 % de cette valeur pour
        laisser du temps au calcul.
        """
        self._verifier_mode((ACCGYRO_MODE,) + _MODES_FUSION, "mesurer_frequence_max")
        t0 = time.ticks_us()
        for _ in range(nb_lectures):
            self._lire_brut()
        dt_us = time.ticks_diff(time.ticks_us(), t0)
        return nb_lectures * 1000000 / dt_us

    def enregistrer(self, duree_s, frequence_hz=125):
        """
        Enregistre les données brutes en RAM (pour régler les seuils des
        détecteurs à partir de vrais gestes : frappes, foulées, pas).

        Paramètres
            duree_s       durée en s (2000 s maximum)
            frequence_hz  fréquence d'échantillonnage

        Retour
            (donnees, nb) : bytearray de 16 octets par échantillon et nombre
            d'échantillons. Sauvegarde : sauvegarder_csv(donnees, nb, "f.csv",
            capteur.biais_gyro).

        Lève MemoryError si la RAM libre est insuffisante
        (16 octets × duree_s × frequence_hz ; 60 s à 125 Hz = 120 ko).
        """
        if duree_s <= 0 or duree_s > 2000:
            raise ValueError("duree_s doit etre dans ]0, 2000] (temps stocke sur 32 bits)")
        nb_max = int(duree_s * frequence_hz)
        taille = nb_max * _TAILLE_ENREG
        gc.collect()
        libre = gc.mem_free()
        if taille > 0.8 * libre:
            raise MemoryError("Enregistrement de {} octets impossible ({} octets libres) : "
                              "reduire la duree ou la frequence.".format(taille, libre))
        donnees = bytearray(taille)
        nb = 0
        for ech in self.echantillons(frequence_hz, None, brut=True):
            struct.pack_into(_FMT_ENREG, donnees, nb * _TAILLE_ENREG, *ech)
            nb += 1
            if nb >= nb_max:
                break
        return donnees, nb

    # ------------------------------------------------------------------
    # Calibration (utile en mode fusion)
    # ------------------------------------------------------------------

    def etat_calibration(self):
        """
        Niveaux de calibration, de 0 (non calibré) à 3 (calibré) :
        {"systeme": s, "gyro": g, "acc": a, "mag": m}

        Pour progresser :
            gyro : laisser le capteur immobile quelques secondes
            acc  : le poser dans ~6 orientations différentes, immobile
                   quelques secondes dans chacune
            mag  : décrire des "8" dans l'air (NDOF uniquement)
        """
        s = self.imu.cal_status()
        return {"systeme": s[0], "gyro": s[1], "acc": s[2], "mag": s[3]}

    def est_calibre(self):
        """
        True si la calibration est suffisante pour le mode courant :
            NDOF    : gyro, acc et mag à 3 et système > 0
            IMUPLUS : gyro et acc à 3
            ACCGYRO : toujours False (pas de fusion)
        """
        e = self.etat_calibration()
        if self.mode_courant == NDOF_MODE:
            return e["gyro"] == 3 and e["acc"] == 3 and e["mag"] == 3 and e["systeme"] > 0
        if self.mode_courant == IMUPLUS_MODE:
            return e["gyro"] == 3 and e["acc"] == 3
        return False

    def attendre_calibration(self, delai_max_s=120, afficher=True):
        """
        Attend que est_calibre() devienne vrai (mode fusion uniquement).

        Retour : True si calibré, False si le délai est écoulé.
        """
        self._verifier_mode(_MODES_FUSION, "attendre_calibration")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < delai_max_s * 1000:
            if self.est_calibre():
                if afficher:
                    print("Calibration OK :", self.etat_calibration())
                return True
            if afficher:
                print("Calibration :", self.etat_calibration())
            time.sleep_ms(500)
        return False

    def lire_offsets(self):
        """
        Lit les 22 octets de calibration (offsets acc, mag, gyro et rayons).
        Le capteur passe brièvement en mode CONFIG puis revient au mode courant.
        """
        self.imu.mode(CONFIG_MODE)
        donnees = bytearray(_NB_OCTETS_OFFSETS)
        try:
            self.i2c.readfrom_mem_into(self.adresse, _REG_OFFSETS, donnees)
        finally:
            self.imu.mode(self.mode_courant)
        return bytes(donnees)

    def ecrire_offsets(self, donnees):
        """
        Recharge 22 octets de calibration obtenus avec lire_offsets().
        Le capteur passe brièvement en mode CONFIG puis revient au mode courant.
        """
        if len(donnees) != _NB_OCTETS_OFFSETS:
            raise ValueError("Il faut exactement {} octets".format(_NB_OCTETS_OFFSETS))
        self.imu.mode(CONFIG_MODE)
        try:
            time.sleep_ms(25)
            self.i2c.writeto_mem(self.adresse, _REG_OFFSETS, bytes(donnees))
        finally:
            self.imu.mode(self.mode_courant)

    def sauvegarder_calibration(self, nom_fichier="calib_bno055.bin"):
        """
        Enregistre la calibration dans la flash du Pico, pour éviter de
        recalibrer à chaque démarrage. À faire une fois est_calibre() vrai,
        en mode fusion.
        """
        self._verifier_mode(_MODES_FUSION, "sauvegarder_calibration")
        donnees = self.lire_offsets()
        with open(nom_fichier, "wb") as f:
            f.write(donnees)
        return donnees

    def charger_calibration(self, nom_fichier="calib_bno055.bin"):
        """
        Recharge une calibration sauvegardée.

        Retour : True si chargée, False si le fichier n'existe pas.
        Le magnétomètre peut demander quelques mouvements pour revenir à 3.
        """
        try:
            with open(nom_fichier, "rb") as f:
                donnees = f.read()
        except OSError:
            return False
        self.ecrire_offsets(donnees)
        return True


# =============================================================================
# Programmes de démonstration
# =============================================================================

def demo_orientation(duree_s=30, avec_magnetometre=True):
    """Calibre (ou recharge la calibration) puis affiche l'orientation à 10 Hz."""
    capteur = CapteurBNO055(creer_bus_i2c())
    capteur.passer_en_mode_fusion(avec_magnetometre)
    if capteur.charger_calibration():
        print("Calibration rechargee depuis la flash")
    if capteur.attendre_calibration(delai_max_s=120):
        capteur.sauvegarder_calibration()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < duree_s * 1000:
        cap, roulis, tangage = capteur.lire_euler()
        lx, ly, lz = capteur.lire_acceleration_lineaire()
        print("cap {:6.1f} | roulis {:6.1f} | tangage {:6.1f} | a_lin {:5.2f} m/s2".format(
            cap, roulis, tangage, norme3(lx, ly, lz)))
        time.sleep_ms(100)


def demo_enregistrement(duree_s=10, frequence_hz=125, nom_fichier="essai.csv"):
    """Enregistre des données brutes puis les écrit en CSV (réglage des seuils)."""
    capteur = CapteurBNO055(creer_bus_i2c())
    capteur.passer_en_mode_foot()
    capteur.estimer_biais_gyro()
    print("Enregistrement {} s...".format(duree_s))
    donnees, nb = capteur.enregistrer(duree_s, frequence_hz)
    print("{} echantillons ({} manques), ecriture de {}...".format(
        nb, capteur.nb_manques, nom_fichier))
    sauvegarder_csv(donnees, nb, nom_fichier, capteur.biais_gyro)
    print("Termine")


if __name__ == "__main__":
    demo_orientation()

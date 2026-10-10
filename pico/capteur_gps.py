"""
capteur_gps.py
==============

Récepteur GPS (GNSS) à liaison série NMEA pour le capteur C1, en MicroPython
sur Raspberry Pi Pico 2 W. Utilisé en option par mode_rugby.py.

Modèle actuel : GY-NEO6MV2 (puce u-blox NEO-6M)
    - liaison série 9600 bauds, 1 position par seconde par défaut
    - jusqu'à 5 positions par seconde après configuration (configurer_ublox)
    - fiche technique NEO-6 : position ≈ 2,5 m (CEP), vitesse ≈ 0,1 m/s

Prévu pour un changement de modèle
----------------------------------
    - La lecture n'utilise QUE les trames NMEA 0183 standard RMC et GGA,
      émises par presque tous les récepteurs (u-blox NEO-6M/7M/M8N/M9N,
      MediaTek, Quectel...). Tous les préfixes sont acceptés
      (GP = GPS, GN = multi-constellation, GL, GA, GB, BD...).
    - Seule configurer_ublox() est propre aux puces u-blox (protocole UBX).
      Pour un autre fabricant, écrire l'équivalent (ex. commandes PMTK pour
      MediaTek) : le reste du code ne change pas.
    - Pour un récepteur très différent, il suffit d'écrire une classe qui
      respecte la même interface (ci-dessous) : mode_rugby.py n'utilise
      rien d'autre.

Interface utilisée par mode_rugby.SeanceRugbyGPS
------------------------------------------------
    mettre_a_jour() -> bool       lit la liaison série ; True si une nouvelle
                                  trame de position (RMC) vient d'être lue
    est_exploitable(hdop_max, nb_sat_min, age_max_ms) -> bool
                                  position et vitesse fiables et récentes
    origine_definie() -> bool
    definir_origine()             fixe l'origine du repère local
    position_locale() -> (x, y)   position en m (x vers l'est, y vers le
                                  nord) par rapport à l'origine, ou None
    vitesse_ms                    vitesse sol (m/s), mesurée par effet Doppler
    cap_deg                       cap sol (° depuis le nord, sens horaire)
                                  ou None si inconnu

Câblage (GY-NEO6MV2 -> Pico 2 W), UART0
---------------------------------------
    VCC -> 3V3(OUT)  broche physique 36 (vérifier sur la fiche de ton
                     module la plage d'alimentation admise)
    GND -> GND       broche physique 38
    TX  -> GP1       broche physique 2  (RX de l'UART0 du Pico)
    RX  -> GP0       broche physique 1  (TX de l'UART0 du Pico)
    Ces broches sont distinctes de celles de l'I2C du BNO055 (GP4/GP5).

Précision des calculs
---------------------
Les nombres à virgule de MicroPython sur le Pico sont en simple précision
(~7 chiffres significatifs) : une latitude stockée en flottant ne serait
précise qu'à ~0,4 m près. Les coordonnées sont donc stockées en ENTIERS,
en 1e-5 minute d'arc (≈ 1,85 cm), et seules les différences par rapport à
l'origine sont converties en flottants.
"""

import math
import struct
import time

from machine import UART, Pin


# =============================================================================
# Constantes
# =============================================================================

_RAYON_TERRE_M = 6371000.0

# Mètres par unité de coordonnée (1e-5 minute d'arc) le long d'un méridien
_M_PAR_UNITE = _RAYON_TERRE_M * math.pi / (180.0 * 60.0 * 100000.0)

# Unités de coordonnée par degré : 60 minutes × 100000
_UNITES_PAR_DEGRE = 6000000

_NOEUD_VERS_MS = 1852.0 / 3600.0  # 1 nœud = 1852 m/h

_TAILLE_MAX_TAMPON = 512  # octets sans fin de ligne avant purge

# Identifiants des trames NMEA standard dans le protocole UBX (classe 0xF0)
_ID_NMEA_UBX = {
    "GGA": 0x00,
    "GLL": 0x01,
    "GSA": 0x02,
    "GSV": 0x03,
    "RMC": 0x04,
    "VTG": 0x05,
}


# =============================================================================
# Fonctions utilitaires
# =============================================================================

def creer_uart_gps(id_uart=0, broche_tx=0, broche_rx=1, debit=9600):
    """
    Crée la liaison série vers le GPS.

    Paramètres
        id_uart    0 ou 1
        broche_tx  GPIO TX du Pico (reliée au RX du GPS)
        broche_rx  GPIO RX du Pico (reliée au TX du GPS)
        debit      9600 bauds par défaut pour le NEO-6M

    Le tampon de réception (1024 octets) absorbe environ 1 s de trames.
    """
    return UART(id_uart, baudrate=debit, tx=Pin(broche_tx), rx=Pin(broche_rx),
                rxbuf=1024)


def _texte_ascii(ligne):
    """Convertit des octets en str si tous sont des caractères ASCII imprimables, sinon None."""
    for b in ligne:
        if b < 32 or b > 126:
            return None
    return ligne.decode()


def _checksum_valide(texte):
    """
    Vérifie la somme de contrôle d'une trame NMEA "$....*hh" :
    XOR de tous les caractères entre '$' et '*'.
    """
    etoile = texte.find("*")
    if not texte.startswith("$") or etoile < 0 or etoile + 3 > len(texte):
        return False
    somme = 0
    for c in texte[1:etoile]:
        somme ^= ord(c)
    try:
        attendu = int(texte[etoile + 1:etoile + 3], 16)
    except ValueError:
        return False
    return somme == attendu


def _coord_vers_unites(valeur, hemisphere):
    """
    Convertit une coordonnée NMEA en entier signé (unité : 1e-5 minute d'arc).

    "4807.038", "N"   -> 48° 07,038'  -> (48×60 + 7)×100000 + 3800
    "01131.000", "E"  -> 11° 31,000'
    Sud et Ouest donnent des valeurs négatives. Renvoie None si invalide.
    """
    if not valeur or hemisphere not in ("N", "S", "E", "W"):
        return None
    if "." in valeur:
        entier, decimales = valeur.split(".", 1)
    else:
        entier, decimales = valeur, ""
    if not entier.isdigit() or (decimales and not decimales.isdigit()):
        return None
    e = int(entier)
    degres = e // 100
    minutes = e % 100
    if minutes >= 60:
        return None
    fraction = int((decimales + "00000")[:5])
    unites = (degres * 60 + minutes) * 100000 + fraction
    if hemisphere in ("S", "W"):
        unites = -unites
    return unites


def _flottant(texte, defaut=None):
    """float(texte), ou defaut si le champ est vide ou invalide."""
    if not texte:
        return defaut
    try:
        return float(texte)
    except ValueError:
        return defaut


def _entier(texte, defaut=0):
    """int(texte), ou defaut si le champ est vide ou invalide."""
    if not texte:
        return defaut
    try:
        return int(texte)
    except ValueError:
        return defaut


# =============================================================================
# Récepteur NMEA générique
# =============================================================================

class RecepteurGPS:
    """
    Lecture non bloquante d'un récepteur GPS NMEA.

    Exemple
        gps = RecepteurGPS(creer_uart_gps())
        while True:
            if gps.mettre_a_jour() and gps.est_exploitable():
                print(gps.latitude_deg(), gps.longitude_deg(), gps.vitesse_ms)

    Attributs publics (mis à jour par mettre_a_jour)
        fix_valide          True si la dernière trame RMC est valide ("A")
        qualite_fix         0 = pas de position, 1 = GPS, 2 = DGPS... (GGA)
        nb_satellites       satellites utilisés (GGA)
        hdop                dilution horizontale de précision (GGA),
                            ~1 = très bon, > 3 = médiocre
        altitude_m          altitude (GGA) ou None
        lat_unites          latitude, entier en 1e-5 minute d'arc, ou None
        lon_unites          longitude, entier en 1e-5 minute d'arc, ou None
        vitesse_ms          vitesse sol (m/s)
        cap_deg             cap sol (°, nord = 0, sens horaire) ou None
        heure_utc           "hhmmss.ss" (texte brut de la trame)
        date                "jjmmaa" (texte brut de la trame)
        nb_trames_ok        trames valides reçues
        nb_erreurs          trames rejetées (somme de contrôle, format)
    """

    def __init__(self, uart):
        """uart : objet machine.UART (voir creer_uart_gps)."""
        self.uart = uart
        self._tampon = b""
        self.fix_valide = False
        self.qualite_fix = 0
        self.nb_satellites = 0
        self.hdop = 99.9
        self.altitude_m = None
        self.lat_unites = None
        self.lon_unites = None
        self.vitesse_ms = 0.0
        self.cap_deg = None
        self.heure_utc = ""
        self.date = ""
        self.nb_trames_ok = 0
        self.nb_erreurs = 0
        self._t_rmc_ms = None
        self._origine = None
        self._cos_lat0 = 1.0

    # ------------------------------------------------------------------
    # Lecture
    # ------------------------------------------------------------------

    def mettre_a_jour(self):
        """
        Lit les octets disponibles sur la liaison série (sans attendre) et
        décode les lignes complètes.

        Retour : True si au moins une trame RMC (position + vitesse) a été
        décodée pendant cet appel, valide ou non.
        """
        nouveau = False
        n = self.uart.any()
        if n:
            donnees = self.uart.read(n)
            if donnees:
                self._tampon += donnees
        while True:
            i = self._tampon.find(b"\n")
            if i < 0:
                break
            ligne = self._tampon[:i]
            self._tampon = self._tampon[i + 1:]
            if self._traiter_ligne(ligne):
                nouveau = True
        if len(self._tampon) > _TAILLE_MAX_TAMPON:
            self._tampon = b""
        return nouveau

    def _traiter_ligne(self, ligne):
        """Décode une ligne ; renvoie True si c'était une trame RMC."""
        if ligne.endswith(b"\r"):
            ligne = ligne[:-1]
        debut = ligne.find(b"$")
        if debut < 0:
            return False
        texte = _texte_ascii(ligne[debut:])
        if texte is None or not _checksum_valide(texte):
            self.nb_erreurs += 1
            return False
        champs = texte[1:texte.find("*")].split(",")
        type_trame = champs[0][-3:]
        if type_trame == "RMC":
            ok = self._decoder_rmc(champs)
        elif type_trame == "GGA":
            ok = self._decoder_gga(champs)
        else:
            return False  # trame valide mais non utilisée
        if ok:
            self.nb_trames_ok += 1
        else:
            self.nb_erreurs += 1
        return ok and type_trame == "RMC"

    def _decoder_rmc(self, f):
        """
        $xxRMC,heure,statut,lat,N/S,lon,E/W,vitesse_noeuds,cap,date,...
        """
        if len(f) < 10:
            return False
        self._t_rmc_ms = time.ticks_ms()
        self.heure_utc = f[1]
        self.date = f[9]
        lat = _coord_vers_unites(f[3], f[4])
        lon = _coord_vers_unites(f[5], f[6])
        if f[2] != "A" or lat is None or lon is None:
            self.fix_valide = False
            return True
        self.fix_valide = True
        self.lat_unites = lat
        self.lon_unites = lon
        self.vitesse_ms = _flottant(f[7], 0.0) * _NOEUD_VERS_MS
        self.cap_deg = _flottant(f[8], None)
        return True

    def _decoder_gga(self, f):
        """
        $xxGGA,heure,lat,N/S,lon,E/W,qualite,nb_sat,hdop,altitude,M,...
        """
        if len(f) < 10:
            return False
        self.qualite_fix = _entier(f[6], 0)
        self.nb_satellites = _entier(f[7], 0)
        self.hdop = _flottant(f[8], 99.9)
        self.altitude_m = _flottant(f[9], None)
        return True

    # ------------------------------------------------------------------
    # Qualité
    # ------------------------------------------------------------------

    def age_ms(self):
        """Temps écoulé depuis la dernière trame RMC (ms), None si aucune."""
        if self._t_rmc_ms is None:
            return None
        return time.ticks_diff(time.ticks_ms(), self._t_rmc_ms)

    def est_exploitable(self, hdop_max=2.5, nb_sat_min=5, age_max_ms=1500):
        """
        True si la position et la vitesse sont fiables et récentes :
        trame RMC valide de moins de age_max_ms, qualité GGA >= 1,
        au moins nb_sat_min satellites, HDOP <= hdop_max.
        """
        age = self.age_ms()
        return (self.fix_valide
                and age is not None and age <= age_max_ms
                and self.qualite_fix >= 1
                and self.nb_satellites >= nb_sat_min
                and self.hdop <= hdop_max)

    # ------------------------------------------------------------------
    # Position
    # ------------------------------------------------------------------

    def latitude_deg(self):
        """Latitude en degrés décimaux (affichage uniquement), ou None."""
        if self.lat_unites is None:
            return None
        return self.lat_unites / _UNITES_PAR_DEGRE

    def longitude_deg(self):
        """Longitude en degrés décimaux (affichage uniquement), ou None."""
        if self.lon_unites is None:
            return None
        return self.lon_unites / _UNITES_PAR_DEGRE

    def origine_definie(self):
        """True si l'origine du repère local est fixée."""
        return self._origine is not None

    def definir_origine(self):
        """
        Prend la position actuelle comme origine du repère local
        (x vers l'est, y vers le nord). Lève RuntimeError sans position.
        """
        if self.lat_unites is None or self.lon_unites is None:
            raise RuntimeError("Pas encore de position GPS valide")
        self._origine = (self.lat_unites, self.lon_unites)
        lat0_rad = (self.lat_unites / _UNITES_PAR_DEGRE) * math.pi / 180.0
        self._cos_lat0 = math.cos(lat0_rad)

    def position_locale(self):
        """
        Position (x_est, y_nord) en mètres par rapport à l'origine
        (projection plane, valable à l'échelle d'un terrain), ou None.
        """
        if self._origine is None or self.lat_unites is None:
            return None
        d_lat = self.lat_unites - self._origine[0]   # entiers : calcul exact
        d_lon = self.lon_unites - self._origine[1]
        y = d_lat * _M_PAR_UNITE
        x = d_lon * _M_PAR_UNITE * self._cos_lat0
        return (x, y)


# =============================================================================
# Configuration propre aux récepteurs u-blox (protocole UBX)
# =============================================================================

def _trame_ubx(classe, ident, charge):
    """
    Construit une trame UBX : 0xB5 0x62, classe, id, longueur (2 octets,
    little-endian), charge utile, somme de contrôle Fletcher 8 bits
    calculée sur classe + id + longueur + charge.
    """
    longueur = len(charge)
    corps = bytes((classe, ident, longueur & 0xFF, (longueur >> 8) & 0xFF)) + bytes(charge)
    ck_a = 0
    ck_b = 0
    for b in corps:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xb5\x62" + corps + bytes((ck_a, ck_b))


def _attendre_ack(uart, classe, ident, delai_ms=500):
    """
    Attend la réponse du récepteur à une commande UBX.

    Retour : True (ACK-ACK), False (ACK-NAK, commande refusée) ou
    None (pas de réponse dans le délai).
    """
    ack = b"\xb5\x62\x05\x01\x02\x00" + bytes((classe, ident))
    nak = b"\xb5\x62\x05\x00\x02\x00" + bytes((classe, ident))
    tampon = b""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < delai_ms:
        n = uart.any()
        if n:
            donnees = uart.read(n)
            if donnees:
                tampon += donnees
                if tampon.find(ack) >= 0:
                    return True
                if tampon.find(nak) >= 0:
                    return False
                tampon = tampon[-64:]
        else:
            time.sleep_ms(5)
    return None


def configurer_ublox(uart, frequence_hz=5, trames_gardees=("GGA", "RMC"),
                     frequence_max_hz=5):
    """
    Configure un récepteur u-blox (NEO-6M, et aussi NEO-7M, M8N...).
    Les réglages sont perdus à la coupure d'alimentation : appeler cette
    fonction à chaque démarrage.

    1. Désactive les trames NMEA inutiles (GLL, GSA, GSV, VTG) : à 9600
       bauds la liaison transmet ~960 caractères/s ; RMC + GGA ≈ 150
       caractères par position, soit ~750 caractères/s à 5 Hz.
    2. Règle la fréquence des positions (commande CFG-RATE).

    Paramètres
        frequence_hz      positions par seconde (5 au maximum pour le NEO-6M)
        trames_gardees    trames NMEA laissées actives (RMC et GGA nécessaires)
        frequence_max_hz  limite du modèle (5 pour NEO-6M, 10 pour NEO-M8N)

    Retour : dict {commande: True/False/None} (voir _attendre_ack).
    """
    if frequence_hz <= 0 or frequence_hz > frequence_max_hz:
        raise ValueError("frequence_hz doit etre dans ]0, {}]".format(frequence_max_hz))
    for nom in ("GGA", "RMC"):
        if nom not in trames_gardees:
            raise ValueError("RMC et GGA doivent rester actives")

    resultats = {}
    for nom, ident in _ID_NMEA_UBX.items():
        taux = 1 if nom in trames_gardees else 0
        # CFG-MSG (0x06 0x01), forme courte : classe, id, taux sur le port courant
        uart.write(_trame_ubx(0x06, 0x01, bytes((0xF0, ident, taux))))
        resultats["MSG_" + nom] = _attendre_ack(uart, 0x06, 0x01)

    # CFG-RATE (0x06 0x08) : période de mesure (ms), 1 navigation par mesure,
    # référence de temps GPS (1)
    periode_ms = int(1000 / frequence_hz)
    uart.write(_trame_ubx(0x06, 0x08, struct.pack("<HHH", periode_ms, 1, 1)))
    resultats["RATE"] = _attendre_ack(uart, 0x06, 0x08)
    return resultats


# =============================================================================
# Démonstration
# =============================================================================

def demo_gps(duree_s=60, frequence_hz=5):
    """Configure le GPS puis affiche son état une fois par seconde."""
    uart = creer_uart_gps()
    print("Configuration u-blox :", configurer_ublox(uart, frequence_hz))
    gps = RecepteurGPS(uart)
    t0 = time.ticks_ms()
    t_affichage = t0
    while time.ticks_diff(time.ticks_ms(), t0) < duree_s * 1000:
        gps.mettre_a_jour()
        if gps.est_exploitable() and not gps.origine_definie():
            gps.definir_origine()
        if time.ticks_diff(time.ticks_ms(), t_affichage) >= 1000:
            t_affichage = time.ticks_ms()
            pos = gps.position_locale()
            print("fix {} | sat {} | hdop {:.1f} | v {:.1f} km/h | pos {}".format(
                "oui" if gps.est_exploitable() else "non",
                gps.nb_satellites, gps.hdop, gps.vitesse_ms * 3.6,
                "-" if pos is None else "({:.1f}, {:.1f}) m".format(pos[0], pos[1])))
        time.sleep_ms(20)


if __name__ == "__main__":
    demo_gps()

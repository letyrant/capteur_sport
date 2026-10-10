"""
gps_relatif.py — base C0
========================

GPS de la base fixe et position du joueur RELATIVE à la base (FP3).

Principe
--------
La base et le joueur voient les mêmes satellites à travers la même
atmosphère : une grande partie de l'erreur GPS (ionosphère, troposphère,
horloges et orbites des satellites) est commune aux deux récepteurs et
varie lentement (échelle de la minute). La différence
    r(t) = position_C1(t) - position_C0(t)
en élimine une partie : r est la position du joueur PAR RAPPORT À LA BASE,
sans avoir besoin de connaître la position exacte de la base.

Ne s'éliminent pas : le bruit propre à chaque récepteur, les trajets
multiples (réflexions), et les écarts quand les deux récepteurs n'utilisent
pas les mêmes satellites (le corps du joueur en masque une partie). Le gain
réel est donc à MESURER (essai statique : deux GPS côte à côte).

Les positions sont appariées sur l'heure UTC de la trame (même époque) ;
à défaut, la dernière position de la base est utilisée (base immobile).

Repère du terrain
-----------------
Sans calibrage : x vers l'est, y vers le nord, base placée en
(base_x_m, base_y_m) : la carte est juste à une rotation près.
Calibrage : le joueur se tient immobile sur la ligne de touche de la base,
à DROITE de la base quand on regarde le terrain depuis la base, à au moins
distance_min_m (idéalement au drapeau de coin). La direction base -> joueur
devient l'axe X (le long de la touche) ; Y pointe vers l'intérieur du
terrain. Origine : coin gauche de la touche de la base.

Coordonnées : entiers en 1e-5 minute d'arc (voir commun/capteur_gps.py) ;
seules les DIFFÉRENCES sont converties en flottants (simple précision).
"""

import math

from machine import RTC

_RAYON_TERRE_M = 6371000.0
_M_PAR_UNITE = _RAYON_TERRE_M * math.pi / (180.0 * 60.0 * 100000.0)  # 1e-5 min d'arc
_UNITES_PAR_DEGRE = 6000000
_DISTANCE_MAX_PLAUSIBLE_M = 2000.0


def regler_horloge(date, heure):
    """
    Règle l'horloge du Pico sur l'heure UTC du GPS ("jjmmaa", "hhmmss.ss").
    Le Pico n'a pas de pile d'horloge : sans cela, la date est fausse dès
    qu'il démarre sans PC. Retour : True si l'horloge a été réglée.
    """
    try:
        j, m, a = int(date[0:2]), int(date[2:4]), 2000 + int(date[4:6])
        h, mi, s = int(heure[0:2]), int(heure[2:4]), int(heure[4:6])
        RTC().datetime((a, m, j, 0, h, mi, s, 0))
        return True
    except (ValueError, IndexError, OSError):
        return False


class GPSRelatif:
    """
    GPS de la base et calcul de la position du joueur par rapport à elle.
    Garde les dernières positions de la base, indexées par heure UTC, pour
    les apparier avec celles de C1 prises au même instant.

    Attributs publics
        gps            RecepteurGPS de la base (commun/capteur_gps.py)
        nb_apparies    positions calculées avec la base à la même heure UTC
        nb_approches   positions calculées avec la dernière position de la base
        horloge_reglee True une fois l'horloge du Pico réglée par le GPS
    """

    def __init__(self, gps, taille_historique=30):
        self.gps = gps
        self.nb_apparies = 0
        self.nb_approches = 0
        self.horloge_reglee = False
        self._historique = {}     # heure_utc -> (lat, lon)
        self._ordre = []
        self._taille = taille_historique

    def mettre_a_jour(self):
        """Lit le GPS de la base (non bloquant) et mémorise les bonnes positions."""
        gps = self.gps
        if not (gps.mettre_a_jour() and gps.est_exploitable()):
            return
        h = gps.heure_utc
        if h not in self._historique:
            self._ordre.append(h)
            if len(self._ordre) > self._taille:
                del self._historique[self._ordre.pop(0)]
        self._historique[h] = (gps.lat_unites, gps.lon_unites)
        if not self.horloge_reglee:
            self.horloge_reglee = regler_horloge(gps.date, h)

    def base_ok(self):
        """True si la position de la base est fiable et récente."""
        return self.gps.est_exploitable()

    def position_relative(self, lat, lon, heure_utc):
        """
        Position du joueur par rapport à la base.
        lat, lon : coordonnées de C1 (entiers, 1e-5 minute d'arc).
        heure_utc : heure de la trame de C1 ("hhmmss.ss").
        Retour : (est_m, nord_m, apparie) ou None ; apparie = True si la
        position de la base a exactement la même heure UTC.
        """
        ref = self._historique.get(heure_utc)
        apparie = ref is not None
        if ref is None:
            if not self.base_ok():
                return None
            ref = (self.gps.lat_unites, self.gps.lon_unites)
        lat0, lon0 = ref
        cos_lat = math.cos(lat0 / _UNITES_PAR_DEGRE * math.pi / 180.0)
        est = (lon - lon0) * _M_PAR_UNITE * cos_lat     # différences d'entiers : exactes
        nord = (lat - lat0) * _M_PAR_UNITE
        if abs(est) > _DISTANCE_MAX_PLAUSIBLE_M or abs(nord) > _DISTANCE_MAX_PLAUSIBLE_M:
            return None
        if apparie:
            self.nb_apparies += 1
        else:
            self.nb_approches += 1
        return est, nord, apparie


class RepereTerrain:
    """Passage (est, nord) relatif à la base -> (X, Y) dans le repère du terrain."""

    def __init__(self, base_x_m, base_y_m, distance_min_m=20.0):
        self.base_x = base_x_m
        self.base_y = base_y_m
        self.distance_min_m = distance_min_m
        self.calibre = False
        self._cos = 1.0
        self._sin = 0.0

    def calibrer(self, est, nord):
        """
        Le joueur est sur la touche, à droite de la base : la direction
        base -> joueur devient l'axe X. Retour : (réussi, distance en m).
        """
        d = math.sqrt(est * est + nord * nord)
        if d < self.distance_min_m:
            return False, d
        theta = math.atan2(nord, est)
        self._cos = math.cos(theta)
        self._sin = math.sin(theta)
        self.calibre = True
        return True, d

    def vers_terrain(self, est, nord):
        """Rotation de -theta, puis translation de la position de la base."""
        x = est * self._cos + nord * self._sin
        y = -est * self._sin + nord * self._cos
        return self.base_x + x, self.base_y + y

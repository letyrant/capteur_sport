"""
journal_base.py — base C0
=========================

Enregistrement sur la base des données reçues de C1 (FP4) : une ligne CSV
par état reçu (reception_001.csv, reception_002.csv...).

    - Le fichier n'est créé qu'au premier état reçu : un démarrage de la
      base sans capteur ne laisse pas de fichier vide.
    - Écriture par blocs (moins d'accès à la mémoire flash, qui bloquent
      le processeur quelques dizaines de millisecondes).
    - Aucune écriture ne doit arrêter la base : en cas d'erreur ou de
      mémoire pleine, le journal se désactive et note la raison (erreur).
"""

import os
import time

ENTETE = ("t_base_s,id,t_seance_s,vitesse_kmh,distance_km,"
          "est_m,nord_m,x_terrain_m,y_terrain_m,apparie,rssi\n")


def espace_libre_octets():
    """Octets libres sur la mémoire du Pico, ou None si l'info est indisponible."""
    try:
        s = os.statvfs("/")
        return s[0] * s[3]
    except (OSError, AttributeError):
        return None


def nom_fichier_libre(prefixe, extension):
    """prefixe_001.extension, prefixe_002... (plus grand numéro présent + 1)."""
    debut = prefixe + "_"
    fin = "." + extension
    n_max = 0
    for nom in os.listdir():
        if nom.startswith(debut) and nom.endswith(fin):
            milieu = nom[len(debut):len(nom) - len(fin)]
            if milieu.isdigit():
                n_max = max(n_max, int(milieu))
    return "{}{:03d}{}".format(debut, n_max + 1, fin)


def _champ(v):
    return "" if v is None else v


class JournalCSV:
    """
    Attributs publics
        actif       False si désactivé (réglage, erreur ou mémoire pleine)
        nom         nom du fichier, None tant qu'aucun état n'a été reçu
        nb_lignes   lignes enregistrées
        erreur      raison de la désactivation, ou None
    """

    def __init__(self, actif=True, lignes_par_bloc=20, octets_min_libres=100000):
        self.actif = actif
        self.nom = None
        self.nb_lignes = 0
        self.erreur = None
        self._tampon = []
        self._par_bloc = lignes_par_bloc
        self._min_libre = octets_min_libres
        self._t0 = time.ticks_ms()

    def _desactiver(self, raison):
        self.actif = False
        self.erreur = raison
        self._tampon = []

    def _creer(self):
        try:
            self.nom = nom_fichier_libre("reception", "csv")
            with open(self.nom, "w") as f:
                f.write(ENTETE)
            return True
        except OSError as e:
            self._desactiver("creation impossible : {}".format(e))
            return False

    def ajouter(self, id_capteur, t_seance_s, etat, rel, terrain, rssi=None):
        """
        Une ligne par état reçu.
        rel : (est_m, nord_m, apparie) ou None ; terrain : (X, Y) ou None.
        """
        if not self.actif:
            return
        if self.nom is None and not self._creer():
            return
        if rel is None or terrain is None:
            pos = ",,,,"
        else:
            pos = "{:.2f},{:.2f},{:.2f},{:.2f},{}".format(
                rel[0], rel[1], terrain[0], terrain[1], int(rel[2]))
        e = etat or {}
        self._tampon.append("{:.1f},{},{},{},{},{},{}\n".format(
            time.ticks_diff(time.ticks_ms(), self._t0) / 1000.0,
            _champ(id_capteur), _champ(t_seance_s),
            _champ(e.get("vitesse_kmh")), _champ(e.get("distance_km")),
            pos, _champ(rssi)))
        self.nb_lignes += 1
        if len(self._tampon) >= self._par_bloc:
            self.vider()

    def vider(self):
        """Écrit les lignes en attente."""
        if not self.actif or not self._tampon or self.nom is None:
            return
        libre = espace_libre_octets()
        if libre is not None and libre < self._min_libre:
            self._desactiver("memoire pleine")
            return
        try:
            with open(self.nom, "a") as f:
                f.write("".join(self._tampon))
        except OSError as e:
            self._desactiver("ecriture impossible : {}".format(e))
        self._tampon = []

"""
reception.py — base C0
======================

Données reçues du capteur C1.

    ##################################################################
    #  EMPLACEMENT RÉSERVÉ À LA LIAISON C1 -> C0 (PAS ENCORE RÉALISÉE) #
    #  Les zones à compléter sont marquées « A FAIRE (liaison) ».     #
    ##################################################################

Ce qui reste à écrire (le transport)
------------------------------------
    1. Choisir le lien : Wi-Fi (la base en point d'accès, envoi UDP), BLE,
       LoRa... Le corps du joueur absorbe fortement le 2,4 GHz : prévoir un
       test de portée joueur de face / de dos.
    2. demarrer() : ouvrir le lien.
    3. lire()     : lire les messages arrivés depuis C1 SANS ATTENDRE, les
                    décoder, puis pour chacun appeler
                        self.recevoir_etat(...)    pendant la séance
                        self.recevoir_resume(...)  en fin de séance
    4. Si les messages sont numérotés : compter les pertes dans nb_perdus.
    5. Côté C1 : envoyer seance.etat() (et la position GPS brute, voir
       recevoir_etat) quelques fois par seconde, sans bloquer la mesure.

Ce qui est déjà prêt (indépendant du transport)
-----------------------------------------------
    recevoir_etat     mémorise l'état, calcule la position relative (GPS
                      relatif), la trace sur la carte, enregistre en CSV
    recevoir_resume   résumé de fin de séance, affiché sur l'écran
    calibrer          calibrage du repère terrain (appui long, page CARTE)
    age_s, liaison_ok, pertes_pct, texte_liaison
    SimulateurC1      faux états pour tester l'écran AVANT la liaison
"""

import math
import time

# A FAIRE (liaison) : passer à True une fois demarrer() et lire() écrits.
LIAISON_REALISEE = False


class Reception:
    """
    Dernières données reçues de C1, lues par l'affichage (ecran.py).

    Attributs publics
        etat          dict seance.etat() de C1 (voir c1_capteur/deplacement.py),
                      ou None tant que rien n'est reçu
        mode          "foot" ou "rugby", ou None
        t_seance_s    temps depuis le début de la séance de C1 (s), ou None
        rssi          puissance du signal reçu (dBm), ou None
        resume        résumé de fin de séance (dict), ou None
        rel           (est_m, nord_m, apparie) du dernier état, ou None
        terrain       dernière position (X, Y) connue, repère terrain, ou None
        trace         dernières positions (X, Y)
        nb_recus      états reçus
        nb_perdus     états perdus (si le transport sait les compter)
    """

    def __init__(self, gps_rel, repere, journal, age_max_s=3.0, longueur_trace=40):
        self.gps_rel = gps_rel
        self.repere = repere
        self.journal = journal
        self.age_max_s = age_max_s
        self.longueur_trace = longueur_trace
        self.nb_recus = 0
        self.nb_perdus = 0
        self._t_dernier_ms = None
        self.nouvelle_seance()

    # ------------------------------------------------------------------
    # Transport : A FAIRE (liaison)
    # ------------------------------------------------------------------

    def demarrer(self):
        """
        Ouvre le lien avec C1.
        Retour : texte court (16 caractères) affiché au démarrage de la base.
        """
        # A FAIRE (liaison) : créer le lien (point d'accès Wi-Fi + socket
        # UDP non bloquante, BLE, LoRa...).
        return "Liaison: a faire"

    def lire(self):
        """
        Lit les messages de C1 arrivés depuis l'appel précédent, sans attendre.
        Appelé en boucle par main.py (toutes les ~5 ms).
        Retour : "resume" si un résumé de fin de séance vient d'arriver
        (l'écran l'affiche alors), sinon None.
        """
        # A FAIRE (liaison) : pour chaque message reçu et décodé :
        #     self.recevoir_etat(etat, mode, t_seance_s, gps_c1, rssi, id_capteur)
        # ou, pour un résumé de fin de séance :
        #     if self.recevoir_resume(resume): evenement = "resume"
        return None

    # ------------------------------------------------------------------
    # Traitement des données reçues : prêt
    # ------------------------------------------------------------------

    def nouvelle_seance(self):
        """Efface les données de la séance précédente (C1 a redémarré)."""
        self.etat = None
        self.mode = None
        self.t_seance_s = None
        self.rssi = None
        self.resume = None
        self.rel = None
        self.terrain = None
        self.trace = []
        self._rels_recents = []   # dernières (est, nord), pour le calibrage

    def recevoir_etat(self, etat, mode=None, t_seance_s=None, gps_c1=None,
                      rssi=None, id_capteur=1):
        """
        À appeler pour chaque état reçu de C1.

        Paramètres
            etat        dict seance.etat() de C1
            mode        "foot" ou "rugby"
            t_seance_s  temps depuis le début de la séance (s)
            gps_c1      position BRUTE du GPS de C1 : (lat_unites, lon_unites,
                        heure_utc), entiers en 1e-5 minute d'arc et
                        "hhmmss.ss" (attributs lat_unites, lon_unites,
                        heure_utc de RecepteurGPS), ou None sans GPS valide
            rssi        puissance du signal (dBm), ou None
            id_capteur  numéro du capteur (plusieurs joueurs plus tard)
        """
        self.nb_recus += 1
        self._t_dernier_ms = time.ticks_ms()
        self.etat = etat or {}
        self.mode = mode
        self.t_seance_s = t_seance_s
        self.rssi = rssi
        self._calculer_position(gps_c1)
        self.journal.ajouter(id_capteur, t_seance_s, self.etat, self.rel,
                             self.terrain, rssi)

    def recevoir_resume(self, resume):
        """
        À appeler à la réception du résumé de fin de séance (seance.resume()).
        Retour : True (l'écran affiche alors la page RESUME).
        """
        self.resume = resume or {}
        self.journal.vider()
        return True

    def _calculer_position(self, gps_c1):
        self.rel = None
        if gps_c1:
            try:
                lat, lon, heure = int(gps_c1[0]), int(gps_c1[1]), str(gps_c1[2])
                self.rel = self.gps_rel.position_relative(lat, lon, heure)
            except (TypeError, ValueError, IndexError):
                self.rel = None
        if self.rel is None:
            return
        self._rels_recents.append((self.rel[0], self.rel[1]))
        if len(self._rels_recents) > 6:
            self._rels_recents.pop(0)
        self.terrain = self.repere.vers_terrain(self.rel[0], self.rel[1])
        self.trace.append(self.terrain)
        if len(self.trace) > self.longueur_trace:
            self.trace.pop(0)

    # ------------------------------------------------------------------
    # État de la liaison
    # ------------------------------------------------------------------

    def age_s(self):
        """Secondes depuis le dernier état reçu, ou None."""
        if self._t_dernier_ms is None:
            return None
        return time.ticks_diff(time.ticks_ms(), self._t_dernier_ms) / 1000.0

    def liaison_ok(self):
        age = self.age_s()
        return age is not None and age <= self.age_max_s

    def pertes_pct(self):
        total = self.nb_recus + self.nb_perdus
        return 100.0 * self.nb_perdus / total if total else 0.0

    def texte_liaison(self):
        """Texte court pour le bas de l'écran."""
        if self.age_s() is None:
            return "C1 --" if LIAISON_REALISEE else "C1 a faire"
        if not self.liaison_ok():
            return "C1 PERDU"
        if self.rssi is not None:
            return "C1 {}dB".format(self.rssi)
        return "C1 ok"

    # ------------------------------------------------------------------
    # Calibrage du repère terrain
    # ------------------------------------------------------------------

    def calibrer(self):
        """
        Calibre le repère sur la moyenne des derniers états (joueur immobile
        au drapeau de coin). Retour : texte à afficher (15 caractères).
        """
        if not self.liaison_ok() or self.rel is None or len(self._rels_recents) < 4:
            return "Echec: pas GPS"
        n = len(self._rels_recents)
        est = sum(r[0] for r in self._rels_recents) / n
        nord = sum(r[1] for r in self._rels_recents) / n
        ok, d = self.repere.calibrer(est, nord)
        if not ok:
            return "Trop pres {:.0f}m".format(d)
        self.trace = []                    # trace dans l'ancien repère : effacée
        self.terrain = self.repere.vers_terrain(self.rel[0], self.rel[1])
        return "Calibre d={:.0f}m".format(d)


# =============================================================================
# Simulation de C1 (tests de l'écran avant la liaison)
# =============================================================================

class SimulateurC1:
    """
    Fabrique de faux états de C1, 2 fois par seconde, pour tester l'écran et
    le journal sans capteur. Si le GPS de la base a une position, le faux
    joueur parcourt un cercle de 25 m de rayon centré à 25 m au nord de la
    base : la page CARTE et le calibrage sont alors testables (positions
    appariées sur l'heure de la base).
    Activer avec SIMULATION_C1 = True dans main.py. À retirer ensuite.
    """

    def __init__(self, rec, mode="foot", periode_s=0.5):
        self.rec = rec
        self.mode = mode
        self.periode_ms = int(periode_s * 1000)
        self._t0 = time.ticks_ms()
        self._t_prochain = self._t0
        self._distance_km = 0.0
        self._cycles = 0

    def mettre_a_jour(self):
        maintenant = time.ticks_ms()
        if time.ticks_diff(maintenant, self._t_prochain) < 0:
            return
        self._t_prochain = time.ticks_add(maintenant, self.periode_ms)
        t = time.ticks_diff(maintenant, self._t0) / 1000.0
        v = 12.0 + 8.0 * math.sin(t / 5.0)                   # km/h
        self._distance_km += v / 3600.0 * self.periode_ms / 1000.0
        self._cycles += 1
        etat = {"vitesse_kmh": round(v, 1),
                "acceleration_ms2": round(8.0 / 5.0 / 3.6 * math.cos(t / 5.0), 2),
                "distance_km": round(self._distance_km, 3),
                "x_m": 0.0, "y_m": 0.0, "cap_deg": int(t * 10) % 360,
                "cadence_hz": 1.3, "source": "simulation"}
        if self.mode == "foot":
            etat["nb_foulees"] = self._cycles
            etat["tirs"] = int(t // 20)
        else:
            etat["nb_pas"] = 2 * self._cycles
        gps = self.rec.gps_rel.gps
        gps_c1 = None
        if gps.lat_unites is not None and gps.heure_utc:
            # cercle de 25 m de rayon centré à 25 m au nord de la base :
            # 1 unité = 1,85 cm en latitude, 1,22 cm en longitude à 49° N
            gps_c1 = (gps.lat_unites + int(1350 * (1 + math.sin(t / 10.0))),
                      gps.lon_unites + int(2000 * math.cos(t / 10.0)),
                      gps.heure_utc)
        self.rec.recevoir_etat(etat, self.mode, round(t, 1), gps_c1, rssi=-60)

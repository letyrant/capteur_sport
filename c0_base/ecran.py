"""
ecran.py — base C0
==================

Affichage temps réel sur l'écran OLED (FP5) et bouton de navigation.

Écran : Keyestudio KS0271, puce SSD1306, 128x64 px, I2C, adresse 0x3C
(0x3D sur certains modules), alimentation 3,3 à 5 V.
Pilote : ssd1306.py (micropython-lib), à copier sur le Pico (voir README).

Pages
-----
    LIVE      vitesse en grand, distance, tirs (foot) ou pas (rugby)
    CARTE     terrain vu de la base, position et trace du joueur
    STATS     état complet reçu de C1
    SYSTEME   GPS de la base, liaison, calibrage, enregistrement
    RESUME    résumé de fin de séance (affiché dès sa réception)

Texte normal : 8x8 px, soit 16 caractères par ligne et 8 lignes.
"""

import time

import framebuf
from machine import Pin

TERRAINS = {"foot": (105.0, 68.0), "rugby": (100.0, 70.0)}   # longueur, largeur (m)
PAGES = ("LIVE", "CARTE", "STATS", "SYSTEME", "RESUME")


# =============================================================================
# Utilitaires
# =============================================================================

def format_min_s(secondes):
    """Durée en "min:ss" (ex. 754 -> "12:34")."""
    s = int(secondes + 0.5)
    return "{}:{:02d}".format(s // 60, s % 60)


def valeur(d, cle, defaut=0):
    """d[cle], ou defaut si absent ou None."""
    v = d.get(cle) if d else None
    return defaut if v is None else v


def nombre_cycles(e):
    """Pas (rugby) ou foulées (foot), selon la clé présente dans l'état."""
    for cle in ("nb_pas", "nb_foulees", "nb_cycles"):
        if cle in e:
            return e[cle]
    return 0


# =============================================================================
# Écran et bouton
# =============================================================================

def trouver_adresse_oled(i2c):
    """Adresse I2C de l'écran (0x3C ou 0x3D) ; RuntimeError s'il est absent."""
    adresses = i2c.scan()
    for a in (0x3C, 0x3D):
        if a in adresses:
            return a
    raise RuntimeError("Ecran OLED introuvable. Adresses vues : {}".format(
        [hex(a) for a in adresses]))


class Ecran:
    """Écran SSD1306 128x64."""

    def __init__(self, i2c, adresse):
        from ssd1306 import SSD1306_I2C
        self.o = SSD1306_I2C(128, 64, i2c, addr=adresse)
        self._car = framebuf.FrameBuffer(bytearray(8), 8, 8, framebuf.MONO_HLSB)

    def texte(self, s, x, y):
        self.o.text(s, x, y, 1)

    def texte_droite(self, s, y):
        self.o.text(s, 128 - 8 * len(s), y, 1)

    def texte_grand(self, s, x, y, k=3):
        """Texte agrandi k fois (k=3 : caractères de 24x24 px)."""
        car = self._car
        for c in s:
            car.fill(0)
            car.text(c, 0, 0, 1)
            for j in range(8):
                for i in range(8):
                    if car.pixel(i, j):
                        self.o.fill_rect(x + i * k, y + j * k, k, k, 1)
            x += 8 * k

    def message(self, lignes):
        """Écran de texte simple (6 lignes au plus)."""
        self.o.fill(0)
        for i, ligne in enumerate(lignes[:6]):
            self.texte(ligne[:16], 0, i * 10)
        self.o.show()


class Bouton:
    """Bouton entre la broche et GND (pull-up interne), anti-rebond par durée."""

    def __init__(self, gpio, duree_longue_ms=2000, duree_min_ms=30):
        self.pin = Pin(gpio, Pin.IN, Pin.PULL_UP)
        self._longue = duree_longue_ms
        self._min = duree_min_ms
        self._appuye = False
        self._long_signale = False
        self._t0 = 0

    def lire(self):
        """Retour : None, "court" (au relâchement) ou "long" (dès 2 s d'appui)."""
        bas = self.pin.value() == 0
        t = time.ticks_ms()
        if bas:
            if not self._appuye:
                self._appuye = True
                self._long_signale = False
                self._t0 = t
            elif not self._long_signale and time.ticks_diff(t, self._t0) >= self._longue:
                self._long_signale = True
                return "long"
            return None
        if self._appuye:
            self._appuye = False
            if not self._long_signale and time.ticks_diff(t, self._t0) >= self._min:
                return "court"
        return None


# =============================================================================
# Pages
# =============================================================================

class Affichage:
    """Dessine les pages et gère la navigation."""

    def __init__(self, ecran, rec, gps_rel, repere, journal, defilement_auto_s=6):
        self.e = ecran
        self.rec = rec
        self.gps_rel = gps_rel
        self.repere = repere
        self.journal = journal
        self.defilement_auto_ms = int(defilement_auto_s * 1000)
        self.page = "LIVE"
        self._t_page_ms = time.ticks_ms()
        self._message = None
        self._t_fin_message_ms = 0

    # ---------------- navigation ----------------

    def pages_disponibles(self):
        return PAGES if self.rec.resume is not None else PAGES[:-1]

    def page_suivante(self):
        pages = self.pages_disponibles()
        i = pages.index(self.page) if self.page in pages else -1
        self.montrer(pages[(i + 1) % len(pages)])

    def montrer(self, page):
        self.page = page
        self._t_page_ms = time.ticks_ms()

    def afficher_message(self, texte, duree_ms=2500):
        """Bandeau temporaire au milieu de l'écran (15 caractères)."""
        self._message = texte
        self._t_fin_message_ms = time.ticks_add(time.ticks_ms(), duree_ms)

    def action(self, appui):
        """appui : None, "court" (page suivante) ou "long" (calibrage sur CARTE)."""
        if appui == "court":
            self.page_suivante()
        elif appui == "long" and self.page == "CARTE":
            self.afficher_message(self.rec.calibrer())

    def defilement_auto(self):
        """Sans bouton : page suivante toutes les defilement_auto_s."""
        if time.ticks_diff(time.ticks_ms(), self._t_page_ms) >= self.defilement_auto_ms:
            self.page_suivante()

    # ---------------- dessin ----------------

    def dessiner(self):
        o = self.e.o
        o.fill(0)
        if self.page == "RESUME" and self.rec.resume is None:
            self.page = "LIVE"             # nouvelle séance : résumé effacé
        if self.page == "LIVE":
            self._live()
        elif self.page == "CARTE":
            self._carte()
        elif self.page == "STATS":
            self._stats()
        elif self.page == "SYSTEME":
            self._systeme()
        else:
            self._resume()
        if self._message is not None:
            if time.ticks_diff(self._t_fin_message_ms, time.ticks_ms()) > 0:
                o.fill_rect(0, 22, 128, 20, 0)
                o.rect(0, 22, 128, 20, 1)
                self.e.texte(self._message[:15], 4, 28)
            else:
                self._message = None
        o.show()

    def _ligne_etat(self, y=56):
        """Bas d'écran : liaison avec C1 à gauche, GPS de la base à droite."""
        gps = self.gps_rel.gps
        sat = "B{}{}".format(gps.nb_satellites, "*" if self.gps_rel.base_ok() else "")
        self.e.texte(self.rec.texte_liaison(), 0, y)
        self.e.texte_droite(sat, y)

    def _live(self):
        rec = self.rec
        e = rec.etat
        self.e.texte((rec.mode or "BASE C0").upper(), 0, 0)
        if rec.t_seance_s is not None:
            self.e.texte_droite(format_min_s(rec.t_seance_s), 0)
        self.e.o.hline(0, 10, 128, 1)
        if e is None:
            self.e.texte("En attente du", 0, 22)
            self.e.texte("capteur C1...", 0, 34)
        else:
            s = "{:.1f}".format(valeur(e, "vitesse_kmh"))[:4]
            self.e.texte_grand(s, 96 - 24 * len(s), 14, 3)
            self.e.texte("km/h", 96, 30)
            ligne = "D{:5.2f}km".format(valeur(e, "distance_km"))
            if "tirs" in e:
                ligne += " Tirs{:3d}".format(e["tirs"])
            else:
                ligne += " Pas{:4d}".format(nombre_cycles(e))
            self.e.texte(ligne, 0, 44)
        self._ligne_etat()

    def _carte(self):
        rec = self.rec
        o = self.e.o
        longueur, largeur = TERRAINS.get(rec.mode, TERRAINS["foot"])
        k = min(100.0 / longueur, 63.0 / largeur)       # pixels par mètre
        w = int(longueur * k)
        h = int(largeur * k)
        y0 = 63 - h

        def vers_ecran(x, y):
            px = int(x * k)
            py = y0 + h - int(y * k)
            return min(max(px, 0), w), min(max(py, 0), 63)

        o.rect(0, y0, w + 1, h + 1, 1)
        cx = int(longueur / 2 * k)
        cy = y0 + h // 2
        o.vline(cx, y0, h, 1)
        if rec.mode == "rugby":
            for x in (22.0, longueur - 22.0):
                o.vline(int(x * k), y0, h, 1)
        else:
            r = int(9.15 * k)
            try:
                o.ellipse(cx, cy, r, r, 1)
            except AttributeError:
                pass                       # MicroPython trop ancien : pas de rond central
            sw, sh = int(16.5 * k), int(40.3 * k)
            o.rect(0, cy - sh // 2, sw, sh, 1)
            o.rect(w - sw, cy - sh // 2, sw + 1, sh, 1)

        bx, by = vers_ecran(self.repere.base_x, max(self.repere.base_y, 0.0))
        o.fill_rect(bx - 3, by - 3, 7, 4, 1)          # base : pavé plein sur sa touche

        for x, y in rec.trace:
            px, py = vers_ecran(x, y)
            o.pixel(px, py, 1)
        if rec.terrain is not None:
            px, py = vers_ecran(rec.terrain[0], rec.terrain[1])
            if rec.rel is not None and rec.liaison_ok():
                o.fill_rect(px - 1, py - 1, 3, 3, 1)    # position à jour
            else:
                o.rect(px - 2, py - 2, 5, 5, 1)         # dernière position connue

        xt = 104
        v = valeur(rec.etat, "vitesse_kmh") if rec.etat else 0
        self.e.texte("{:3.0f}".format(v)[:3], xt, 0)
        self.e.texte("kmh", xt, 10)
        self.e.texte("CAL" if self.repere.calibre else "N-C", xt, 28)
        self.e.texte("REL" if rec.rel is not None else "---", xt, 40)
        if not rec.liaison_ok():
            self.e.texte(" !", xt, 54)

    def _stats(self):
        e = self.rec.etat
        if e is None:
            self.e.texte("STATS", 0, 0)
            self.e.texte("Aucune donnee", 0, 24)
            self._ligne_etat()
            return
        lignes = [
            "Vit {:6.1f} km/h".format(valeur(e, "vitesse_kmh")),
            "Acc {:+6.2f} m/s2".format(valeur(e, "acceleration_ms2")),
            "Dist {:6.3f} km".format(valeur(e, "distance_km")),
            "Cad {:6.2f} Hz".format(valeur(e, "cadence_hz")),
            "Cycles {:6d}".format(nombre_cycles(e)),
            "Cap {:5d} deg".format(int(valeur(e, "cap_deg"))),
            "Src {}".format(str(valeur(e, "source", "-"))[:12]),
        ]
        if "tirs" in e:
            lignes.append("Tirs {:5d}".format(e["tirs"]))
        elif "nb_satellites" in e:
            lignes.append("GPS C1 sat {}".format(valeur(e, "nb_satellites", "-")))
        for i, ligne in enumerate(lignes):
            self.e.texte(ligne[:16], 0, i * 8)

    def _systeme(self):
        rec = self.rec
        gps = self.gps_rel.gps
        lignes = [
            "GPS base {}".format("OK" if self.gps_rel.base_ok() else "--"),
            "Sat{:3d} HDOP{:4.1f}".format(gps.nb_satellites, gps.hdop),
            rec.texte_liaison(),
            "Recus{:5d} P{:3.0f}%".format(rec.nb_recus, rec.pertes_pct()),
            "Appar {}/{}".format(self.gps_rel.nb_apparies,
                                 self.gps_rel.nb_apparies + self.gps_rel.nb_approches),
            "Repere {}".format("calibre" if self.repere.calibre else "non cal."),
            "CSV {}".format(self.journal.nb_lignes if self.journal.actif else "off"),
            "Heure {}".format("GPS" if self.gps_rel.horloge_reglee else "--"),
        ]
        for i, ligne in enumerate(lignes):
            self.e.texte(ligne[:16], 0, i * 8)

    def _resume(self):
        r = self.rec.resume or {}
        lignes = [
            "FIN DE SEANCE",
            "Dist {:6.2f} km".format(valeur(r, "distance_km")),
            "Activite {:4.0f} %".format(valeur(r, "activite_pct")),
            "Vmax {:5.1f} km/h".format(valeur(r, "sprint_max_kmh")),
            "Sprints {}".format(valeur(r, "sprints")),
            "Acc {} Dec {}".format(valeur(r, "accelerations"), valeur(r, "decelerations")),
            "Course {}".format(valeur(r, "course", "-")),
        ]
        if "tirs" in r:
            lignes.append("Tirs {} max{}{:.0f}".format(
                r["tirs"], ">" if r.get("tir_max_minore") else " ",
                valeur(r, "tir_max_pied_kmh")))
        for i, ligne in enumerate(lignes):
            self.e.texte(ligne[:16], 0, i * 8)

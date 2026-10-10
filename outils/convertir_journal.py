#!/usr/bin/env python3
"""
convertir_journal.py — s'exécute sur le PC, pas sur le Pico.

Convertit un journal d'événements (frappes_NNNN.bin) en CSV lisible dans un
tableur, et affiche un résumé par événement : pic de vitesse angulaire,
accélération maximale, durée, saturations.

Ce résumé sert à régler les seuils de DetecteurFrappe sans retourner sur le
terrain : comparer les pics d'une séance de frappes à ceux d'une séance de
sprints, puis choisir omega_min_dps entre les deux populations.

Usage
    python outils/convertir_journal.py frappes_0007.bin
    python outils/convertir_journal.py frappes_0007.bin --fenetre 375
    python outils/convertir_journal.py frappes_0007.bin --sortie ailleurs.csv
    python outils/convertir_journal.py frappes_0007.bin --tracer   (matplotlib)

La taille de fenêtre (échantillons par événement) figure dans le résumé de
séance, champ journal_evenements.echantillons_par_evenement. Par défaut 375
(2 s avant + 1 s après, à 125 Hz).
"""

import argparse
import math
import struct
import sys

FMT = "<ihhhhhh"
TAILLE = 16
LSB_PAR_MS2 = 100.0
LSB_PAR_DPS = 16.0
G = 9.80665

# Pleines échelles du BNO055 en modes foot et rugby
PLAGE_ACC_G = 16
PLAGE_GYRO_DPS = 2000
FRACTION_SATURATION = 0.98


def lire(nom_binaire):
    """Renvoie la liste des échantillons (t_us, ax, ay, az, gx, gy, gz) en unités SI."""
    echantillons = []
    with open(nom_binaire, "rb") as f:
        while True:
            bloc = f.read(TAILLE)
            if len(bloc) < TAILLE:
                break
            t, ax, ay, az, gx, gy, gz = struct.unpack(FMT, bloc)
            echantillons.append((
                t,
                ax / LSB_PAR_MS2, ay / LSB_PAR_MS2, az / LSB_PAR_MS2,
                gx / LSB_PAR_DPS, gy / LSB_PAR_DPS, gz / LSB_PAR_DPS))
    return echantillons


def ecrire_csv(echantillons, nom_csv, fenetre):
    """Écrit le CSV ; renvoie le nombre de lignes."""
    with open(nom_csv, "w") as f:
        f.write("evenement,t_us,ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps\n")
        for i, e in enumerate(echantillons):
            f.write("{},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
                i // fenetre if fenetre else 0, *e))
    return len(echantillons)


def resumer(echantillons, fenetre, longueur_levier_m):
    """Un dict de statistiques par événement."""
    seuil_gyro = FRACTION_SATURATION * PLAGE_GYRO_DPS
    seuil_acc = FRACTION_SATURATION * PLAGE_ACC_G * G
    resultats = []
    for debut in range(0, len(echantillons), fenetre):
        bloc = echantillons[debut:debut + fenetre]
        if not bloc:
            continue
        omega_max = acc_max = 0.0
        gyro_sature = acc_sature = False
        for _, ax, ay, az, gx, gy, gz in bloc:
            omega = math.sqrt(gx * gx + gy * gy + gz * gz)
            acc = math.sqrt(ax * ax + ay * ay + az * az)
            omega_max = max(omega_max, omega)
            acc_max = max(acc_max, acc)
            gyro_sature |= max(abs(gx), abs(gy), abs(gz)) >= seuil_gyro
            acc_sature |= max(abs(ax), abs(ay), abs(az)) >= seuil_acc
        resultats.append({
            "numero": len(resultats) + 1,
            "duree_s": (bloc[-1][0] - bloc[0][0]) / 1e6,
            "omega_max_dps": omega_max,
            "acc_max_g": acc_max / G,
            "vitesse_pied_kmh": math.radians(omega_max) * longueur_levier_m * 3.6,
            "gyro_sature": gyro_sature,
            "acc_sature": acc_sature,
        })
    return resultats


def tracer(echantillons, fenetre):
    """Trace |omega| et |a| pour chaque événement (matplotlib requis)."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib n'est pas installe : pip install matplotlib", file=sys.stderr)
        return
    nb = max(1, (len(echantillons) + fenetre - 1) // fenetre)
    fig, axes = plt.subplots(nb, 1, figsize=(10, 2.6 * nb), squeeze=False, sharex=False)
    for k in range(nb):
        bloc = echantillons[k * fenetre:(k + 1) * fenetre]
        if not bloc:
            continue
        t0 = bloc[0][0]
        t = [(e[0] - t0) / 1e6 for e in bloc]
        omega = [math.sqrt(e[4] ** 2 + e[5] ** 2 + e[6] ** 2) for e in bloc]
        acc = [math.sqrt(e[1] ** 2 + e[2] ** 2 + e[3] ** 2) / G for e in bloc]
        a = axes[k][0]
        a.plot(t, omega, label="|ω| (°/s)")
        a.axhline(PLAGE_GYRO_DPS, linestyle="--", linewidth=0.8, label="saturation gyro")
        a.set_ylabel("|ω| (°/s)")
        a.set_title("Événement {}".format(k + 1))
        jumelle = a.twinx()
        jumelle.plot(t, acc, linestyle=":", label="|a| (g)")
        jumelle.set_ylabel("|a| (g)")
        a.set_xlabel("temps (s)")
        a.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    plt.show()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("binaire", help="fichier frappes_NNNN.bin recupere du Pico")
    p.add_argument("--sortie", help="fichier CSV produit (defaut : meme nom en .csv)")
    p.add_argument("--fenetre", type=int, default=375,
                   help="echantillons par evenement (defaut 375 : 2 s + 1 s a 125 Hz)")
    p.add_argument("--levier", type=float, default=0.50,
                   help="distance genou -> point de contact du pied, en m (defaut 0,50)")
    p.add_argument("--tracer", action="store_true", help="tracer les courbes (matplotlib)")
    args = p.parse_args()

    if args.fenetre < 1:
        p.error("--fenetre doit etre >= 1")
    echantillons = lire(args.binaire)
    if not echantillons:
        print("Fichier vide : aucune frappe enregistree.", file=sys.stderr)
        return 1

    sortie = args.sortie or (args.binaire.rsplit(".", 1)[0] + ".csv")
    n = ecrire_csv(echantillons, sortie, args.fenetre)
    print("{} echantillons -> {}".format(n, sortie))
    if len(echantillons) % args.fenetre:
        print("ATTENTION : le fichier ne contient pas un nombre entier de fenetres de {} "
              "echantillons. Verifier --fenetre (champ "
              "journal_evenements.echantillons_par_evenement du resume).".format(args.fenetre),
              file=sys.stderr)

    print("\nEvenement | duree  | omega max   | acc max | vitesse pied  | saturation")
    print("----------+--------+-------------+---------+---------------+-----------")
    for r in resumer(echantillons, args.fenetre, args.levier):
        saturations = []
        if r["gyro_sature"]:
            saturations.append("gyro")
        if r["acc_sature"]:
            saturations.append("acc")
        vitesse = "{}{:5.1f} km/h".format(
            ">=" if r["gyro_sature"] else "  ", r["vitesse_pied_kmh"])
        print("{:>9} | {:4.2f} s | {:7.0f} °/s | {:5.1f} g | {:>13} | {}".format(
            r["numero"], r["duree_s"], r["omega_max_dps"], r["acc_max_g"], vitesse,
            ", ".join(saturations) if saturations else "-"))

    if args.tracer:
        tracer(echantillons, args.fenetre)
    return 0


if __name__ == "__main__":
    sys.exit(main())

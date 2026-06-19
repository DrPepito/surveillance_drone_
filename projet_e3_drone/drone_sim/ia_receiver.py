#!/usr/bin/env python3
# =============================================================================
# ia_receiver.py
# Récepteur UDP de démonstration — côté IA observatrice
#
# Lance ce script dans un terminal pendant que le simulateur tourne.
# Il affiche la télémétrie reçue à 50 Hz et calcule des métriques simples.
#
# Usage :
#   python ia_receiver.py                  # port 5005 par défaut
#   python ia_receiver.py --port 5005
#   python ia_receiver.py --verbose        # affiche chaque paquet
#
# Structure de chaque paquet JSON reçu :
#   { "t", "pos"[3], "vel"[3], "att_rad"[3], "att_deg"[3],
#     "att_rate"[3], "acc"[3], "moteurs"[4], "cmd"{...},
#     "cible_alt", "bat_pct", "bat_v", "bat_mah",
#     "mode", "arme", "_cnt" }
# =============================================================================

import argparse
import json
import math
import socket
import time
from collections import deque


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

DEFAULT_PORT    = 5005
BUFFER_SIZE     = 4096
STATS_INTERVAL  = 2.0   # secondes entre les affichages de stats
HISTORY_LEN     = 100   # taille fenêtre glissante pour les métriques


# ---------------------------------------------------------------------------
# Métriques temps réel (exemple de ce que peut faire l'IA)
# ---------------------------------------------------------------------------

class MetriquesVol:
    """Calcule quelques indicateurs simples sur la fenêtre glissante."""

    def __init__(self, n=HISTORY_LEN):
        self.alts     = deque(maxlen=n)
        self.vzs      = deque(maxlen=n)
        self.rolls    = deque(maxlen=n)
        self.pitchs   = deque(maxlen=n)
        self.thrs     = deque(maxlen=n)
        self.pkts_rx  = 0
        self.pkts_lost= 0
        self._last_cnt= -1
        self._t_debut = None

    def push(self, tel: dict):
        if self._t_debut is None:
            self._t_debut = time.monotonic()

        # Détection pertes de paquets
        cnt = tel.get("_cnt", 0)
        if self._last_cnt >= 0:
            delta = (cnt - self._last_cnt) & 0xFFFF
            if delta > 1:
                self.pkts_lost += delta - 1
        self._last_cnt = cnt
        self.pkts_rx  += 1

        self.alts.append(tel["pos"][2])
        self.vzs.append(tel["vel"][2])
        self.rolls.append(tel["att_deg"][0])
        self.pitchs.append(tel["att_deg"][1])
        self.thrs.append(tel["cmd"]["throttle"])

    def resume(self) -> str:
        if not self.alts:
            return "(pas encore de données)"

        perte_pct = 100.0 * self.pkts_lost / max(self.pkts_rx + self.pkts_lost, 1)
        alt_moy   = sum(self.alts)  / len(self.alts)
        vz_rms    = math.sqrt(sum(v**2 for v in self.vzs) / len(self.vzs))
        roll_max  = max(abs(r) for r in self.rolls)
        pitch_max = max(abs(p) for p in self.pitchs)
        thr_moy   = sum(self.thrs) / len(self.thrs)

        dt = time.monotonic() - self._t_debut if self._t_debut else 0
        freq = self.pkts_rx / dt if dt > 0 else 0

        return (
            f"  Fréquence     : {freq:5.1f} Hz   "
            f"(pertes {self.pkts_lost} paquets, {perte_pct:.1f}%)\n"
            f"  Alt moyenne   : {alt_moy:+6.2f} m\n"
            f"  Vz RMS        : {vz_rms:6.3f} m/s\n"
            f"  Roll max      : {roll_max:5.1f}°    "
            f"Pitch max : {pitch_max:5.1f}°\n"
            f"  Throttle moy  : {thr_moy*100:5.1f} %\n"
            f"  Paquets reçus : {self.pkts_rx}"
        )


# ---------------------------------------------------------------------------
# Boucle principale
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Récepteur HIL — IA observatrice")
    parser.add_argument("--port",    type=int,  default=DEFAULT_PORT)
    parser.add_argument("--host",    type=str,  default="0.0.0.0")
    parser.add_argument("--verbose", action="store_true",
                        help="Affiche chaque paquet reçu")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(1.0)   # timeout pour permettre Ctrl+C propre

    metriques = MetriquesVol()
    t_last_stats = time.monotonic()

    print(f"╔══════════════════════════════════════════════════╗")
    print(f"║  IA Receiver — écoute UDP {args.host}:{args.port:<5}      ║")
    print(f"║  Ctrl+C pour quitter                             ║")
    print(f"╚══════════════════════════════════════════════════╝\n")

    try:
        while True:
            # ── Réception ───────────────────────────────────────────
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                continue

            try:
                tel = json.loads(data.decode())
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"[WARN] paquet malformé : {e}")
                continue

            metriques.push(tel)

            # ── Affichage verbose ────────────────────────────────────
            if args.verbose:
                print(
                    f"[#{tel.get('_cnt',0):05d}] "
                    f"t={tel['t']:.2f}s  "
                    f"mode={tel['mode']:8s}  "
                    f"alt={tel['pos'][2]:5.2f}m  "
                    f"roll={tel['att_deg'][0]:+5.1f}°  "
                    f"pitch={tel['att_deg'][1]:+5.1f}°  "
                    f"yaw={tel['att_deg'][2]:+6.1f}°  "
                    f"thr={tel['cmd']['throttle']*100:.1f}%"
                )

            # ── Affichage stats périodique ───────────────────────────
            now = time.monotonic()
            if now - t_last_stats >= STATS_INTERVAL:
                t_last_stats = now
                print(f"\n── Stats (fenêtre {HISTORY_LEN} trames) ──────────────")
                print(metriques.resume())
                print(f"──────────────────────────────────────────────────")

                # Exemple : détection d'anomalie simple
                if metriques.rolls and max(abs(r) for r in metriques.rolls) > 28:
                    print("  ⚠  ALERTE : roll > 28° détecté !")
                if metriques.vzs and any(abs(v) > 3.0 for v in metriques.vzs):
                    print("  ⚠  ALERTE : vitesse verticale > 3 m/s !")

    except KeyboardInterrupt:
        print("\n\nRécepteur arrêté.")
    finally:
        sock.close()
        print("Stats finales :")
        print(metriques.resume())


if __name__ == "__main__":
    main()

# =============================================================================
# drone_state.py  (version HIL)
# État complet du drone à chaque instant T
# Ajout par rapport à la version originale :
#   - exporter_telemetrie() → dict JSON-sérialisable pour HilBridge
#   - _frame_cnt            → compteur de trame (uint16, wrapping)
# =============================================================================

import math


class Vec3:
    """Vecteur 3D — opérations de base dont on a besoin, c'est tout."""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def norme(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def reset(self):
        self.x = self.y = self.z = 0.0

    def __repr__(self):
        return f"Vec3({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"


class DroneState:
    """
    Toute la donnée du drone à un instant donné.
    Séparé en blocs clairs : cinématique / attitude / commandes / systèmes.
    Unités SI : mètres, m/s, radians.
    """

    # Modes de vol
    MODE_SOL       = "SOL"
    MODE_DECOLLAGE = "DECOLLAGE"
    MODE_VOL       = "VOL"
    MODE_ATTERRO   = "ATTERRO"
    MODE_URGENCE   = "URGENCE"

    def __init__(self):

        # --- Cinématique ---
        self.position     = Vec3()   # x=droite y=avant z=altitude (m)
        self.vitesse      = Vec3()   # m/s
        self.acceleration = Vec3()   # m/s²

        # --- Attitude (radians) ---
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        # Vitesses angulaires (rad/s)
        self.roll_rate  = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate   = 0.0

        # --- Commandes pilote (normalisées -1.0 à +1.0) ---
        self.cmd_roll     = 0.0
        self.cmd_pitch    = 0.0
        self.cmd_yaw      = 0.0
        self.cmd_throttle = 0.0

        # --- Moteurs quadcopter X-frame (0.0 → 1.0) ---
        self.moteurs = [0.0, 0.0, 0.0, 0.0]
        self.moteurs_armes = False

        # --- Batterie ---
        self.batterie_pct     = 100.0
        self.batterie_tension = 12.6
        self.batterie_mah     = 0.0
        self.batterie_courant = 0.0

        # --- Systèmes ---
        self.mode_vol         = DroneState.MODE_SOL
        self.temps_vol        = 0.0
        self.distance_origine = 0.0

        # --- Consignes internes PID ---
        self.cible_altitude = 0.0
        self.cible_yaw      = 0.0

        # --- Historique graphiques ---
        N = 200
        self.hist_altitude  = [0.0] * N
        self.hist_vitesse_z = [0.0] * N
        self.hist_roll      = [0.0] * N
        self.hist_pitch     = [0.0] * N

        # --- Compteur de trame HIL (uint16 wrapping) ---
        self._frame_cnt = 0

    # ------------------------------------------------------------------
    # Historique / distance
    # ------------------------------------------------------------------

    def push_historique(self):
        def push(lst, val):
            lst.pop(0)
            lst.append(val)
        push(self.hist_altitude,  self.position.z)
        push(self.hist_vitesse_z, self.vitesse.z)
        push(self.hist_roll,      math.degrees(self.roll))
        push(self.hist_pitch,     math.degrees(self.pitch))

    def update_distance(self):
        self.distance_origine = math.sqrt(
            self.position.x**2 + self.position.y**2)

    # ------------------------------------------------------------------
    # Export HIL — snapshot JSON-sérialisable
    # ------------------------------------------------------------------

    def exporter_telemetrie(self) -> dict:
        """
        Retourne un dict entièrement JSON-sérialisable (float, list, str, int).
        Appelé par HilBridge.envoyer() à chaque tick 50 Hz.

        Champs :
          t          — temps de vol (s)
          pos        — [x, y, z] position monde (m)
          vel        — [vx, vy, vz] vitesse monde (m/s)
          att_rad    — [roll, pitch, yaw] en radians
          att_deg    — [roll, pitch, yaw] en degrés  (commodité pour l'IA)
          att_rate   — [roll_rate, pitch_rate, yaw_rate] (rad/s)
          acc        — [ax, ay, az] accélération (m/s²)
          moteurs    — [M1, M2, M3, M4] puissance normalisée [0-1]
          cmd        — {throttle, roll, pitch, yaw} commandes normalisées
          cible_alt  — consigne altitude PID (m)
          bat_pct    — batterie (%)
          bat_v      — tension batterie (V)
          bat_mah    — capacité consommée (mAh)
          mode       — "SOL"|"DECOLLAGE"|"VOL"|"ATTERRO"|"URGENCE"
          arme       — bool moteurs armés
          _cnt       — compteur de trame uint16
        """
        self._frame_cnt = (self._frame_cnt + 1) & 0xFFFF

        return {
            "t"         : round(self.temps_vol, 4),
            "pos"       : [round(self.position.x, 4),
                           round(self.position.y, 4),
                           round(self.position.z, 4)],
            "vel"       : [round(self.vitesse.x, 4),
                           round(self.vitesse.y, 4),
                           round(self.vitesse.z, 4)],
            "att_rad"   : [round(self.roll,  6),
                           round(self.pitch, 6),
                           round(self.yaw,   6)],
            "att_deg"   : [round(math.degrees(self.roll),  3),
                           round(math.degrees(self.pitch), 3),
                           round(math.degrees(self.yaw),   3)],
            "att_rate"  : [round(self.roll_rate,  5),
                           round(self.pitch_rate, 5),
                           round(self.yaw_rate,   5)],
            "acc"       : [round(self.acceleration.x, 4),
                           round(self.acceleration.y, 4),
                           round(self.acceleration.z, 4)],
            "moteurs"   : [round(m, 4) for m in self.moteurs],
            "cmd"       : {
                "throttle": round(self.cmd_throttle, 4),
                "roll"    : round(self.cmd_roll,     4),
                "pitch"   : round(self.cmd_pitch,    4),
                "yaw"     : round(self.cmd_yaw,      4),
            },
            "cible_alt" : round(self.cible_altitude, 4),
            "bat_pct"   : round(self.batterie_pct, 2),
            "bat_v"     : round(self.batterie_tension, 3),
            "bat_mah"   : round(self.batterie_mah, 2),
            "mode"      : self.mode_vol,
            "arme"      : self.moteurs_armes,
            "_cnt"      : self._frame_cnt,
        }

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(self):
        """Touche R — remet à l'origine sans perdre le cap."""
        yaw_sauvegarde = self.yaw
        self.__init__()
        self.yaw = yaw_sauvegarde

    def __repr__(self):
        return (f"[{self.mode_vol}] "
                f"pos={self.position} "
                f"roll={math.degrees(self.roll):.1f}° "
                f"pitch={math.degrees(self.pitch):.1f}° "
                f"bat={self.batterie_pct:.0f}%")

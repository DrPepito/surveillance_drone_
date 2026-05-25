# =============================================================================
# keyboard_controller.py
# Entrées clavier AZERTY → consignes de VITESSE (mode DJI Position Hold)
#
# COMPORTEMENT DJI-STYLE :
#   Flèche ↑      → consigne vitesse +Y (avancer)
#   Flèche ↓      → consigne vitesse -Y (reculer)
#   Flèche ←/→    → consigne vitesse ±X (gauche/droite)
#   Relâche stick → consigne vitesse = 0 → PID freine → drone se remet droit
#
#   Z             → monter    (augmente consigne altitude)
#   S             → descendre (diminue consigne altitude)
#   Q / D         → yaw gauche / droite (vitesse angulaire)
#
#   ESPACE        → arrêt urgence
#   R             → reset position
#   T             → décollage automatique
#   L             → atterrissage automatique
#   H             → retour home
# =============================================================================

from PyQt6.QtCore import Qt
from physics_engine import MASSE, THRUST_MAX, G, VITESSE_MAX_XY, VITESSE_MAX_Z

THROTTLE_HOVER = (MASSE * G) / THRUST_MAX   # 0.3924

# ── Vitesses max commandables depuis le clavier ──────────────────────────────
VIT_MAX_CMD_XY   = 4.0    # m/s en horizontal (stick à fond)
VIT_MAX_CMD_Z    = 2.0    # m/s en vertical
YAW_RATE_MAX     = 1.2    # rad/s vitesse rotation yaw max

# ── Pas d'évolution par tick (50 Hz) ─────────────────────────────────────────
PAS_VIT_XY       = 0.12   # m/s par tick → temps montée ≈ 0.7s
PAS_VIT_Z        = 0.05   # m/s par tick pour altitude
PAS_YAW          = 0.06   # rad/s par tick

# ── Rappel vers zéro quand stick relâché ─────────────────────────────────────
PAS_RETOUR_XY    = 0.25   # m/s par tick — retour rapide (DJI brake)
PAS_RETOUR_YAW   = 0.10   # rad/s par tick


class KeyboardController:
    """
    Traduit les touches AZERTY en CONSIGNES DE VITESSE.
    Le PID vel_x/vel_y (dans main) convertit ces consignes en angles.
    Résultat : relâche la touche → vitesse cible = 0 → drone freine et se stabilise.
    """

    def __init__(self):
        self._touches = set()

        # Consignes vitesse horizontale (m/s, repère monde)
        self.consigne_vx = 0.0   # +X = droite
        self.consigne_vy = 0.0   # +Y = avant

        # Consigne vitesse yaw (rad/s)
        self.consigne_yaw_rate = 0.0

        # Throttle géré en altitude-hold : Z/S modifient la consigne altitude
        # On garde un throttle interne uniquement pour décollage/urgence
        self._throttle_cmd = THROTTLE_HOVER

        # Événements one-shot
        self.evt_urgence      = False
        self.evt_reset        = False
        self.evt_decollage    = False
        self.evt_atterrissage = False
        self.evt_home         = False

    # ------------------------------------------------------------------
    # Événements Qt
    # ------------------------------------------------------------------

    def key_press(self, key):
        self._touches.add(key)
        if   key == Qt.Key.Key_Space : self.evt_urgence      = True
        elif key == Qt.Key.Key_R     : self.evt_reset        = True
        elif key == Qt.Key.Key_T     : self.evt_decollage    = True
        elif key == Qt.Key.Key_L     : self.evt_atterrissage = True
        elif key == Qt.Key.Key_H     : self.evt_home         = True

    def key_release(self, key):
        self._touches.discard(key)

    # ------------------------------------------------------------------
    # Update 50 Hz — génère les consignes vitesse
    # ------------------------------------------------------------------

    def update(self, state):
        a = self._touches

        # ── Vitesse Y (avant/arrière) — flèches haut/bas ─────────────
        if Qt.Key.Key_Up in a:
            self.consigne_vy = _rampe(self.consigne_vy,  VIT_MAX_CMD_XY, PAS_VIT_XY)
        elif Qt.Key.Key_Down in a:
            self.consigne_vy = _rampe(self.consigne_vy, -VIT_MAX_CMD_XY, PAS_VIT_XY)
        else:
            # Relâché → retour progressif vers 0 (freinant)
            self.consigne_vy = _retour_neutre(self.consigne_vy, PAS_RETOUR_XY)

        # ── Vitesse X (gauche/droite) — flèches gauche/droite ────────
        if Qt.Key.Key_Right in a:
            self.consigne_vx = _rampe(self.consigne_vx,  VIT_MAX_CMD_XY, PAS_VIT_XY)
        elif Qt.Key.Key_Left in a:
            self.consigne_vx = _rampe(self.consigne_vx, -VIT_MAX_CMD_XY, PAS_VIT_XY)
        else:
            self.consigne_vx = _retour_neutre(self.consigne_vx, PAS_RETOUR_XY)

        # ── Yaw — Q gauche / D droite ────────────────────────────────
        if Qt.Key.Key_Q in a:
            self.consigne_yaw_rate = _rampe(self.consigne_yaw_rate, -YAW_RATE_MAX, PAS_YAW)
        elif Qt.Key.Key_D in a:
            self.consigne_yaw_rate = _rampe(self.consigne_yaw_rate,  YAW_RATE_MAX, PAS_YAW)
        else:
            self.consigne_yaw_rate = _retour_neutre(self.consigne_yaw_rate, PAS_RETOUR_YAW)

        # ── Altitude — Z monter / S descendre ────────────────────────
        # On incrémente directement la cible altitude dans le state
        # (gérée par main.py via PID altitude)
        if Qt.Key.Key_Z in a:
            state.cible_altitude = _clamp(
                state.cible_altitude + VIT_MAX_CMD_Z * (1.0/50.0),
                0.0, 30.0)
        elif Qt.Key.Key_S in a:
            state.cible_altitude = _clamp(
                state.cible_altitude - VIT_MAX_CMD_Z * (1.0/50.0),
                0.0, 30.0)

        # Écriture throttle brut (seulement pour urgence/sol)
        state.cmd_throttle = self._throttle_cmd

    # ------------------------------------------------------------------
    # Consommation événements one-shot
    # ------------------------------------------------------------------

    def consommer_urgence(self):
        v = self.evt_urgence      ; self.evt_urgence      = False ; return v

    def consommer_reset(self):
        v = self.evt_reset        ; self.evt_reset        = False ; return v

    def consommer_decollage(self):
        v = self.evt_decollage    ; self.evt_decollage    = False ; return v

    def consommer_atterrissage(self):
        v = self.evt_atterrissage ; self.evt_atterrissage = False ; return v

    def consommer_home(self):
        v = self.evt_home         ; self.evt_home         = False ; return v

    # ------------------------------------------------------------------
    # Utilitaires
    # ------------------------------------------------------------------

    def reset_commandes(self):
        self.consigne_vx = 0.0
        self.consigne_vy = 0.0
        self.consigne_yaw_rate = 0.0
        self._throttle_cmd = 0.0
        self._touches.clear()

    def set_throttle_hover(self):
        self._throttle_cmd = THROTTLE_HOVER


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rampe(val, cible, pas):
    if val < cible:
        return min(val + pas, cible)
    return max(val - pas, cible)


def _retour_neutre(val, pas):
    if abs(val) < pas:
        return 0.0
    return val - pas if val > 0 else val + pas


def _clamp(val, mini, maxi):
    return max(mini, min(maxi, val))
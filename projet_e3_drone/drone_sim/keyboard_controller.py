# =============================================================================
# keyboard_controller.py
# Entrées clavier AZERTY → consignes de VITESSE (mode DJI Position Hold)
#
# CORRECTIONS v2 :
#   ✓ DT dynamique passé en paramètre à update()
#   ✓ Gel des consignes en MODE_SOL / MODE_URGENCE
#   ✓ cmd_throttle n'est plus écrasé inutilement en vol
#   ✓ Commentaire throttle clarifié
#   ✓ HOME → navigation réelle en 3 étapes (voir main_combined.py)
# =============================================================================

from PyQt6.QtCore import Qt
from physics_engine import MASSE, THRUST_MAX, G, VITESSE_MAX_XY, VITESSE_MAX_Z

THROTTLE_HOVER = (MASSE * G) / THRUST_MAX   # 0.3924

# ── Vitesses max commandables depuis le clavier ──────────────────────────────
VIT_MAX_CMD_XY   = 4.0    # m/s en horizontal (stick à fond)
VIT_MAX_CMD_Z    = 2.0    # m/s en vertical
YAW_RATE_MAX     = 1.2    # rad/s vitesse rotation yaw max

# ── Pas d'évolution par seconde (sera multiplié par dt réel) ─────────────────
PAS_VIT_XY_S     = 6.0    # m/s²  → temps montée ≈ 0.67s
PAS_VIT_Z_S      = 2.5    # m/s²
PAS_YAW_S        = 3.0    # rad/s²

# ── Rappel vers zéro quand stick relâché (par seconde) ───────────────────────
PAS_RETOUR_XY_S  = 12.0   # m/s² — frein rapide DJI style
PAS_RETOUR_YAW_S = 5.0    # rad/s²

# ── Altitude de sécurité retour HOME ─────────────────────────────────────────
Z_HOME_SAFE      = 4.0    # m — altitude minimale avant navigation horizontale
HOME_DIST_TOL    = 0.3    # m — tolérance arrivée horizontale
HOME_VIT_MAX     = 2.0    # m/s — vitesse max en navigation HOME


class KeyboardController:
    """
    Traduit les touches AZERTY en consignes de vitesse.
    Le PID vel_x/vel_y (dans main) convertit ces consignes en angles.
    Résultat : relâche la touche → vitesse cible = 0 → drone freine.
    """

    def __init__(self):
        self._touches = set()

        # Consignes vitesse horizontale (m/s, repère drone)
        self.consigne_vx = 0.0   # +X = droite
        self.consigne_vy = 0.0   # +Y = avant

        # Consigne vitesse yaw (rad/s)
        self.consigne_yaw_rate = 0.0

        # Throttle interne — utilisé uniquement en SOL/URGENCE
        # En vol, c'est le PID altitude qui écrit cmd_throttle directement
        self._throttle_sol = THROTTLE_HOVER

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
    # Update — dt réel en paramètre (plus de hardcode 1/50)
    # ------------------------------------------------------------------

    def update(self, state, dt: float):
        from drone_state import DroneState

        # ── Gel complet des consignes si drone non en vol ─────────────
        if state.mode_vol in (DroneState.MODE_SOL, DroneState.MODE_URGENCE):
            self.consigne_vx       = 0.0
            self.consigne_vy       = 0.0
            self.consigne_yaw_rate = 0.0
            # En SOL/URGENCE le PID ne tourne pas,
            # on écrit directement 0 pour être explicite
            state.cmd_throttle = 0.0
            return

        # ── En mode HOME : consignes pilotées automatiquement ─────────
        if state.mode_vol == DroneState.MODE_HOME:
            self._update_home(state, dt)
            return

        a = self._touches

        # ── Vitesse Y (avant/arrière) ─────────────────────────────────
        if Qt.Key.Key_Up in a:
            self.consigne_vy = _rampe(
                self.consigne_vy,  VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        elif Qt.Key.Key_Down in a:
            self.consigne_vy = _rampe(
                self.consigne_vy, -VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        else:
            self.consigne_vy = _retour_neutre(
                self.consigne_vy, PAS_RETOUR_XY_S * dt)

        # ── Vitesse X (gauche/droite) ─────────────────────────────────
        if Qt.Key.Key_Right in a:
            self.consigne_vx = _rampe(
                self.consigne_vx,  VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        elif Qt.Key.Key_Left in a:
            self.consigne_vx = _rampe(
                self.consigne_vx, -VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        else:
            self.consigne_vx = _retour_neutre(
                self.consigne_vx, PAS_RETOUR_XY_S * dt)

        # ── Yaw ──────────────────────────────────────────────────────
        if Qt.Key.Key_Q in a:
            self.consigne_yaw_rate = _rampe(
                self.consigne_yaw_rate, -YAW_RATE_MAX, PAS_YAW_S * dt)
        elif Qt.Key.Key_D in a:
            self.consigne_yaw_rate = _rampe(
                self.consigne_yaw_rate,  YAW_RATE_MAX, PAS_YAW_S * dt)
        else:
            self.consigne_yaw_rate = _retour_neutre(
                self.consigne_yaw_rate, PAS_RETOUR_YAW_S * dt)

        # ── Altitude — Z monter / S descendre (dt réel) ───────────────
        if Qt.Key.Key_Z in a:
            state.cible_altitude = _clamp(
                state.cible_altitude + VIT_MAX_CMD_Z * dt, 0.0, 30.0)
        elif Qt.Key.Key_S in a:
            state.cible_altitude = _clamp(
                state.cible_altitude - VIT_MAX_CMD_Z * dt, 0.0, 30.0)

        # ── cmd_throttle NON écrit ici en vol ─────────────────────────
        # C'est _appliquer_pids() dans main_combined.py qui s'en charge.
        # _throttle_sol est réservé aux transitions SOL/URGENCE.

    # ------------------------------------------------------------------
    # Navigation HOME automatique (3 étapes)
    # ------------------------------------------------------------------

    def _update_home(self, state, dt: float):
        import math

        dx_monde = 0.0 - state.position.x
        dy_monde = 0.0 - state.position.y
        dist_horiz = math.sqrt(dx_monde**2 + dy_monde**2)

        # Étape 1 — Sécurisation altitude
        if state.position.z < Z_HOME_SAFE - 0.3:
            state.cible_altitude   = Z_HOME_SAFE
            self.consigne_vx       = 0.0
            self.consigne_vy       = 0.0
            self.consigne_yaw_rate = 0.0
            return

        # Étape 2 — Navigation horizontale
        if dist_horiz > HOME_DIST_TOL:
            state.cible_altitude = Z_HOME_SAFE

            norm     = max(dist_horiz, 0.01)
            vitesse  = min(dist_horiz, HOME_VIT_MAX)

            # ── Repère MONDE directement ──────────────────────────────
            # _appliquer_pids() fait déjà la rotation monde→drone
            # donc on écrit ici comme si c'était un stick monde
            cy = math.cos(state.yaw)
            sy = math.sin(state.yaw)

            # dx_monde/dy_monde → repère drone (ce que consigne_vx/vy représente)
            vx_drone = ( dx_monde * cy + dy_monde * sy) / norm * vitesse
            vy_drone = (-dx_monde * sy + dy_monde * cy) / norm * vitesse

            self.consigne_vx = _rampe(self.consigne_vx, vx_drone, PAS_VIT_XY_S * dt)
            self.consigne_vy = _rampe(self.consigne_vy, vy_drone, PAS_VIT_XY_S * dt)
            self.consigne_yaw_rate = 0.0
            return

        # Étape 3 — Arrivée hover (0, 0, Z_HOME_SAFE)
        self.consigne_vx       = 0.0
        self.consigne_vy       = 0.0
        self.consigne_yaw_rate = 0.0
        state.cible_altitude   = Z_HOME_SAFE

        from drone_state import DroneState
        state.mode_vol = DroneState.MODE_VOL
    # -----------------------------------------------   -------------------
    # Consommation événements one-shot
    # ------------------------------------------------------------------

    def consommer_urgence(self):
            v = self.evt_urgence
            self.evt_urgence = False
            return v

    def consommer_reset(self):
        v = self.evt_reset
        self.evt_reset = False
        return v

    def consommer_decollage(self):
        v = self.evt_decollage
        self.evt_decollage = False
        return v

    def consommer_atterrissage(self):
        v = self.evt_atterrissage
        self.evt_atterrissage = False
        return v

    def consommer_home(self):
        v = self.evt_home
        self.evt_home = False
        return v
    # ------------------------------------------------------------------
    # Utilitaires
    # ------------------------------------------------------------------

    def reset_commandes(self):
        self.consigne_vx       = 0.0
        self.consigne_vy       = 0.0
        self.consigne_yaw_rate = 0.0
        self._throttle_sol     = 0.0
        self._touches.clear()

    def set_throttle_hover(self):
        self._throttle_sol = THROTTLE_HOVER


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rampe(val, cible, pas):
    """Approche val vers cible par pas maximum."""
    if val < cible:
        return min(val + pas, cible)
    return max(val - pas, cible)


def _retour_neutre(val, pas):
    """Retour progressif vers 0."""
    if abs(val) < pas:
        return 0.0
    return val - pas if val > 0 else val + pas


def _clamp(val, mini, maxi):
    return max(mini, min(maxi, val))
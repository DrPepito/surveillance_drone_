# keyboard_controller.py
# Entrées clavier AZERTY -->consignes de VITESSE (mode DJI Position Hold)
#

#
# Règle : la rampe clavier doit être plus lente que le filtre PID (omega_n=1.0 ≈ 1s)
# sinon on dépasse la trajectoire de référence et ça oscille

from PyQt6.QtCore import Qt
from physics_engine import MASSE, THRUST_MAX, G, VITESSE_MAX_XY, VITESSE_MAX_Z

THROTTLE_HOVER = (MASSE * G) / THRUST_MAX   # 0.3924

VIT_MAX_CMD_XY   = 4.0    # m/s horizontal max
VIT_MAX_CMD_Z    = 2.0    # m/s vertical max
YAW_RATE_MAX     = 1.2    # rad/s yaw max

# rampes d'accélération — volontairement douces pour ne pas dépasser le filtre PID
# si c'est trop lent à ton goût, remonte PAS_VIT_XY_S mais pas au-dessus de 4.0
PAS_VIT_XY_S     = 0.8    # m/s²  montée en ~1.6s (était 6.0 -->trop rapide)
PAS_VIT_Z_S      = 0.8    # m/s²
PAS_YAW_S        = 1    # rad/s² (était 3.0)

# freinage quand on lâche la touche — doit rester proche de PAS_VIT_XY_S
# un freinage trop brutal par rapport à la montée crée des oscillations au relâché
PAS_RETOUR_XY_S  = 4.0    # m/s²  (était 12.0 -->beaucoup trop brutal)
PAS_RETOUR_YAW_S = 3.0    # rad/s² (était 5.0)

Z_HOME_SAFE      = 4.0    # m  altitude sécurité HOME
HOME_DIST_TOL    = 0.3    # m  tolérance arrivée
HOME_VIT_MAX     = 2.0    # m/s vitesse navigation HOME


class KeyboardController:
    """
    Traduit les touches AZERTY en consignes de vitesse.
    Le PID vel_x/vel_y convertit ensuite ces consignes en angles.
    Relâche la touche -->consigne retombe à 0 -->drone freine progressivement.
    """

    def __init__(self):
        self._touches = set()

        self.consigne_vx       = 0.0
        self.consigne_vy       = 0.0
        self.consigne_yaw_rate = 0.0

        # utilisé uniquement en SOL/URGENCE, pas en vol
        self._throttle_sol = THROTTLE_HOVER

        self.evt_urgence      = False
        self.evt_reset        = False
        self.evt_decollage    = False
        self.evt_atterrissage = False
        self.evt_home         = False

    def key_press(self, key):
        self._touches.add(key)
        if   key == Qt.Key.Key_Space : self.evt_urgence      = True
        elif key == Qt.Key.Key_R     : self.evt_reset        = True
        elif key == Qt.Key.Key_T     : self.evt_decollage    = True
        elif key == Qt.Key.Key_L     : self.evt_atterrissage = True
        elif key == Qt.Key.Key_H     : self.evt_home         = True

    def key_release(self, key):
        self._touches.discard(key)

    def update(self, state, dt: float):
        from drone_state import DroneState

        # gel complet si pas en vol
        if state.mode_vol in (DroneState.MODE_SOL, DroneState.MODE_URGENCE):
            self.consigne_vx       = 0.0
            self.consigne_vy       = 0.0
            self.consigne_yaw_rate = 0.0
            state.cmd_throttle     = 0.0
            return

        if state.mode_vol == DroneState.MODE_HOME:
            self._update_home(state, dt)
            return

        a = self._touches

        # avant / arrière
        if Qt.Key.Key_Up in a:
            self.consigne_vy = _rampe(self.consigne_vy,  VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        elif Qt.Key.Key_Down in a:
            self.consigne_vy = _rampe(self.consigne_vy, -VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        else:
            # freinage progressif quand on lâche — pas trop vite sinon ça rebondit
            self.consigne_vy = _retour_neutre(self.consigne_vy, PAS_RETOUR_XY_S * dt)

        # gauche / droite
        if Qt.Key.Key_Right in a:
            self.consigne_vx = _rampe(self.consigne_vx,  VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        elif Qt.Key.Key_Left in a:
            self.consigne_vx = _rampe(self.consigne_vx, -VIT_MAX_CMD_XY, PAS_VIT_XY_S * dt)
        else:
            self.consigne_vx = _retour_neutre(self.consigne_vx, PAS_RETOUR_XY_S * dt)

        # yaw
        if Qt.Key.Key_Q in a:
            self.consigne_yaw_rate = _rampe(self.consigne_yaw_rate, -YAW_RATE_MAX, PAS_YAW_S * dt)
        elif Qt.Key.Key_D in a:
            self.consigne_yaw_rate = _rampe(self.consigne_yaw_rate,  YAW_RATE_MAX, PAS_YAW_S * dt)
        else:
            self.consigne_yaw_rate = _retour_neutre(self.consigne_yaw_rate, PAS_RETOUR_YAW_S * dt)

        # altitude : Z monte, S descend
        if Qt.Key.Key_Z in a:
            state.cible_altitude = _clamp(state.cible_altitude + VIT_MAX_CMD_Z * dt, 0.0, 30.0)
        elif Qt.Key.Key_S in a:
            state.cible_altitude = _clamp(state.cible_altitude - VIT_MAX_CMD_Z * dt, 0.0, 30.0)

        # cmd_throttle écrit par le PID altitude dans main, pas ici

    def _update_home(self, state, dt: float):
        import math

        dx_monde   = 0.0 - state.position.x
        dy_monde   = 0.0 - state.position.y
        dist_horiz = math.sqrt(dx_monde**2 + dy_monde**2)

        # étape 1 : monter à l'altitude de sécurité avant de naviguer
        if state.position.z < Z_HOME_SAFE - 0.3:
            state.cible_altitude   = Z_HOME_SAFE
            self.consigne_vx       = 0.0
            self.consigne_vy       = 0.0
            self.consigne_yaw_rate = 0.0
            return

        # étape 2 : navigation horizontale vers (0, 0)
        if dist_horiz > HOME_DIST_TOL:
            state.cible_altitude = Z_HOME_SAFE
            norm    = max(dist_horiz, 0.01)
            vitesse = min(dist_horiz, HOME_VIT_MAX)

            cy = math.cos(state.yaw)
            sy = math.sin(state.yaw)
            vx_drone = ( dx_monde * cy + dy_monde * sy) / norm * vitesse
            vy_drone = (-dx_monde * sy + dy_monde * cy) / norm * vitesse

            self.consigne_vx       = _rampe(self.consigne_vx, vx_drone, PAS_VIT_XY_S * dt)
            self.consigne_vy       = _rampe(self.consigne_vy, vy_drone, PAS_VIT_XY_S * dt)
            self.consigne_yaw_rate = 0.0
            return

        # étape 3 : arrivée, on repasse en vol normal
        self.consigne_vx       = 0.0
        self.consigne_vy       = 0.0
        self.consigne_yaw_rate = 0.0
        state.cible_altitude   = Z_HOME_SAFE

        from drone_state import DroneState
        state.mode_vol = DroneState.MODE_VOL

    def consommer_urgence(self):
        v = self.evt_urgence; self.evt_urgence = False; return v

    def consommer_reset(self):
        v = self.evt_reset; self.evt_reset = False; return v

    def consommer_decollage(self):
        v = self.evt_decollage; self.evt_decollage = False; return v

    def consommer_atterrissage(self):
        v = self.evt_atterrissage; self.evt_atterrissage = False; return v

    def consommer_home(self):
        v = self.evt_home; self.evt_home = False; return v

    def reset_commandes(self):
        self.consigne_vx       = 0.0
        self.consigne_vy       = 0.0
        self.consigne_yaw_rate = 0.0
        self._throttle_sol     = 0.0
        self._touches.clear()

    def set_throttle_hover(self):
        self._throttle_sol = THROTTLE_HOVER


def _rampe(val, cible, pas):
    """Approche val vers cible par pas maximum."""
    if val < cible:
        return min(val + pas, cible)
    return max(val - pas, cible)


def _retour_neutre(val, pas):
    """Retour progressif vers zéro quand la touche est relâchée."""
    if abs(val) < pas:
        return 0.0
    return val - pas if val > 0 else val + pas


def _clamp(val, mini, maxi):
    return max(mini, min(maxi, val))
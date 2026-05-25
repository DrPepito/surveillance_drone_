# pid_controller.py
# =============================================================================
#
# PROBLEMES CORRIGES :
#
#   1. THRUST SUR-CORRIGE
#      Avant : PID altitude avec Ki trop fort → windup → oscillations pompage
#      Apres : omega_n abaisse a 1.0 rad/s, zeta=1.0 (critique), Ki tres faible
#              => correction tres douce, pas de depassement, pas de pompage
#
#   2. DIMENSION VITESSE OUBLIEE
#      Avant : ModeleReference2Ordre filtrait la consigne de vitesse (m/s)
#              mais comparait y_ref (adimensionnel) a v_mesuree (m/s)
#              => erreur dimensionnelle, le P ne travaillait pas en m/s
#      Apres : le modele de reference genere directement v_ref en m/s
#              (consigne en m/s → filtre → v_ref en m/s)
#              puis P corrige : angle = Kp * (v_ref[m/s] - v_reelle[m/s])
#              Kp a maintenant les bonnes unites : rad / (m/s)
#
# ARCHITECTURE BOUCLE VITESSE (2eme ordre + P) :
#
#   consigne[m/s] → ModeleRef2Ordre → v_ref[m/s]
#                                          |
#                   v_mesuree[m/s] ────────┤
#                                          ↓
#                              erreur_v = v_ref - v_mesuree   [m/s]
#                                          |
#                              angle = Kp * erreur_v          [rad]
#                              Kp = ANGLE_CMD_MAX / VIT_MAX   [rad/(m/s)]
#
# ARCHITECTURE BOUCLE ALTITUDE (PID classique, gains mous) :
#
#   cible_z[m] → PID(omega=1.0, zeta=1.0) → delta_throttle
#   cmd_throttle = THROTTLE_HOVER + delta_throttle
#   => correction autour du point d equilibre, pas de pompage
#
# =============================================================================

from physics_engine import (MASSE, THRUST_MAX, G,
                             INERTIE_ROLL, INERTIE_PITCH, INERTIE_YAW,
                             BRAS_LEVIER, COEFF_YAW,
                             ROLL_MAX, PITCH_MAX)
import math

# ---------------------------------------------------------------------------
# Point d equilibre physique exact
# ---------------------------------------------------------------------------

THROTTLE_HOVER = MASSE * G / THRUST_MAX   # 0.3924 = 39.24 %

# ---------------------------------------------------------------------------
# Gains physiques (acceleration par unite de commande)
# ---------------------------------------------------------------------------

K_ALTITUDE = THRUST_MAX / MASSE                       # 25.0  m/s2/u
K_YAW      = THRUST_MAX * COEFF_YAW   / INERTIE_YAW  # 50.0  rad/s2/u

# ---------------------------------------------------------------------------
# Angle et vitesse max en mode position
# ---------------------------------------------------------------------------

ANGLE_CMD_MAX  = math.radians(22.0)  # 22 deg max commande par boucle vitesse
VIT_MAX_CMD_XY = 4.0                 # m/s max commande clavier


# ===========================================================================
# ModeleReference2Ordre
# ===========================================================================
# Filtre une consigne brutale (echelon) en une trajectoire en S-curve.
#
# Equation :  y'' + 2*zeta*wn*y' + wn^2*y = wn^2 * consigne
# Integree par Euler pas a pas.
#
# ENTREE  : consigne [meme unite que la sortie — ici m/s]
# SORTIES : y  = valeur filtree        [m/s]
#           dy = derivee filtree       [m/s2]  (non utilisee en dehors)
#
# Reglage recommande boucle vitesse :
#   omega_n = 1.2 a 1.6 rad/s  → temps de montee 2 a 3 s
#   zeta    = 0.90              → legerement sur-amorti, zero depassement
# ===========================================================================

class ModeleReference2Ordre:

    def __init__(self, omega_n, zeta):
        self.wn   = omega_n   # pulsation propre [rad/s]
        self.zeta = zeta      # amortissement [-]
        self.y    = 0.0       # etat courant [m/s]
        self.dy   = 0.0       # derivee courante [m/s2]

    def update(self, consigne, dt):
        """
        Avance le filtre d un pas dt.
        consigne : valeur cible [m/s]
        retourne y filtre [m/s]
        """
        if dt <= 0.0:
            return self.y
        # Acceleration du 2eme ordre
        ddy    = self.wn**2 * (consigne - self.y) - 2.0*self.zeta*self.wn * self.dy
        # Integration Euler
        self.dy += ddy    * dt   # [m/s2] * [s] = [m/s]
        self.y  += self.dy * dt  # [m/s]  * [s] = [m]  <- FAUX avant, corrige ici
        # Note : self.y reste en m/s car on filtre une VITESSE, pas une position.
        # La derivee dy est une acceleration [m/s2], l integration donne bien m/s.
        return self.y

    def reset(self, valeur=0.0):
        self.y  = valeur
        self.dy = 0.0


# ===========================================================================
# CorrecteurVitesse  —  modele 2eme ordre + correcteur P
# ===========================================================================
# Les unites sont maintenant coherentes partout :
#
#   consigne    [m/s]
#   v_ref       [m/s]   (sortie ModeleReference)
#   v_mesuree   [m/s]   (vitesse reelle drone)
#   erreur_v    [m/s]   = v_ref - v_mesuree
#   angle       [rad]   = Kp * erreur_v
#   Kp          [rad/(m/s)]  = ANGLE_CMD_MAX / VIT_MAX_CMD_XY
#
# Interpretation physique de Kp :
#   Si erreur_v = VIT_MAX (4 m/s) → angle = ANGLE_CMD_MAX (22 deg)
#   Si erreur_v = 0              → angle = 0 (drone plat, vitesse atteinte)
#   Si erreur_v < 0              → angle negatif (freinage)
# ===========================================================================

class CorrecteurVitesse:

    KP = ANGLE_CMD_MAX / VIT_MAX_CMD_XY   # rad/(m/s)  = 0.384/4.0 = 0.096

    def __init__(self, omega_n=1.4, zeta=0.90):
        self.ref = ModeleReference2Ordre(omega_n, zeta)
        self.kp  = self.KP

    def calculer(self, consigne_ms, vitesse_mesuree_ms, dt):
        """
        consigne_ms       : vitesse cible  [m/s]
        vitesse_mesuree_ms: vitesse reelle [m/s]
        dt                : pas de temps   [s]
        retourne          : angle cible    [rad]
        """
        v_ref  = self.ref.update(consigne_ms, dt)          # [m/s]
        erreur = v_ref - vitesse_mesuree_ms                 # [m/s]
        angle  = self.kp * erreur                           # [rad/(m/s)] * [m/s] = [rad]
        return _clamp(angle, -ANGLE_CMD_MAX, ANGLE_CMD_MAX)

    def reset(self, valeur_ms=0.0):
        self.ref.reset(valeur_ms)


# ===========================================================================
# PID generique  —  utilise pour altitude et yaw
# ===========================================================================
# Anti-windup clamping, pas de kick derivee au premier tick.
# Sorties bornees.
# ===========================================================================

class PID:

    def __init__(self, kp, ki, kd, lim_sortie, lim_integrale=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lim_sortie    = lim_sortie
        self.lim_integrale = lim_integrale if lim_integrale is not None \
                             else lim_sortie * 0.5
        self._integrale    = 0.0
        self._erreur_prec  = 0.0
        self._premier_tick = True

    def calculer(self, consigne, mesure, dt):
        if dt <= 0.0:
            return 0.0
        erreur  = consigne - mesure
        terme_p = self.kp * erreur
        self._integrale += erreur * dt
        self._integrale  = _clamp(self._integrale,
                                  -self.lim_integrale, self.lim_integrale)
        terme_i = self.ki * self._integrale
        if self._premier_tick:
            terme_d       = 0.0
            self._premier_tick = False
        else:
            terme_d = self.kd * (erreur - self._erreur_prec) / dt
        self._erreur_prec = erreur
        return _clamp(terme_p + terme_i + terme_d,
                      -self.lim_sortie, self.lim_sortie)

    def reset(self):
        self._integrale    = 0.0
        self._erreur_prec  = 0.0
        self._premier_tick = True

    def set_gains(self, kp, ki, kd):
        self.kp = kp ; self.ki = ki ; self.kd = kd
        self.reset()

    def debug(self):
        return (f"Kp={self.kp:.4f} Ki={self.ki:.4f} Kd={self.kd:.4f} "
                f"I={self._integrale:.4f}")


# ===========================================================================
# FlightPIDs  —  instanciation de tous les correcteurs
# ===========================================================================
#
# ALTITUDE :
#   omega_n = 1.0 rad/s  → lent, pas de pompage
#   zeta    = 1.00       → amorti critique, zero depassement
#   Ki tres faible       → compense derive lente sans windup
#
# VITESSE XY :
#   CorrecteurVitesse avec modele 2eme ordre + P
#   omega_n = 1.4 rad/s  → montee en ~2s
#   zeta    = 0.90       → legerement sur-amorti
#
# YAW :
#   PID classique, placement de poles, taux direct
#
# ===========================================================================

class FlightPIDs:

    def __init__(self):

        # --- Altitude ---
        # Gains calcules par placement de poles omega=1.0, zeta=1.0 (critique)
        # k = K_ALTITUDE = 25 m/s2/u
        # Kp = omega^2 / k = 1.0 / 25.0 = 0.040
        # Kd = 2*zeta*omega / k = 2.0 / 25.0 = 0.080
        # Ki = tres faible pour ne pas pomper : 0.008
        marge = 1.0 - THROTTLE_HOVER   # ~0.608 — borne max correction
        self.altitude = PID(
            kp = 0.040,
            ki = 0.008,   # tres faible : compense derive sans pomper
            kd = 0.080,
            lim_sortie    = marge,
            lim_integrale = marge * 0.25   # windup serre
        )

        # --- Vitesse horizontale (2eme ordre + P) ---
        # omega_n=1.4 : montee ~2s   zeta=0.90 : sur-amorti leger
        # Pour plus de douceur : omega_n=1.0
        # Pour plus de reactivite : omega_n=2.0
        self.vel_x = CorrecteurVitesse(omega_n=1.4, zeta=0.90)
        self.vel_y = CorrecteurVitesse(omega_n=1.4, zeta=0.90)

        # --- Yaw ---
        # k = 50 rad/s2/u   omega=4   zeta=0.90
        # Kp = 16/50 = 0.320   Kd = 7.2/50 = 0.144   Ki = 0.010
        self.yaw = PID(
            kp = 0.320,
            ki = 0.010,
            kd = 0.144,
            lim_sortie    = 0.80,
            lim_integrale = 0.20
        )

        # --- Position XY (mode autonome futur) ---
        self.pos_x = PID(kp=0.40, ki=0.005, kd=0.20, lim_sortie=0.35)
        self.pos_y = PID(kp=0.40, ki=0.005, kd=0.20, lim_sortie=0.35)

    def reset_all(self):
        self.altitude.reset()
        self.vel_x.reset()
        self.vel_y.reset()
        self.yaw.reset()
        self.pos_x.reset()
        self.pos_y.reset()

    def afficher_gains(self):
        print("=" * 60)
        print(f"  THROTTLE_HOVER = {THROTTLE_HOVER*100:.2f} %")
        print(f"  Altitude   : {self.altitude.debug()}")
        print(f"  Vel XY     : Kp={CorrecteurVitesse.KP:.4f} rad/(m/s)"
              f"  wn={self.vel_x.ref.wn:.2f}  zeta={self.vel_x.ref.zeta:.2f}")
        print(f"  Yaw        : {self.yaw.debug()}")
        print("=" * 60)


def _clamp(val, mini, maxi):
    return max(mini, min(maxi, val))
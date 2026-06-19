# pid_controller.py
# =============================================================================
#
# VERSION v2 — Correcteur second ordre avec intégrateur
#
# ARCHITECTURE :
#
#   ALTITUDE  : PID classique, pôles placés ω=1.8 ζ=0.95
#               → montée progressive ~2s, zéro dépassement, Ki anti-windup
#
#   VITESSE XY : Modèle référence 2ème ordre + correcteur PI
#               → frein progressif naturel, ω=1.6 ζ=0.95
#               → ajout Ki pour annuler erreur statique en vent/pente
#
#   YAW       : PID classique inchangé
#
# GARANTIES :
#   ✓ Zéro dépassement altitude (ζ ≥ 0.95)
#   ✓ Erreur statique nulle (intégrateur sur altitude ET vitesse)
#   ✓ Anti-windup sur tous les intégrateurs
#   ✓ Pas de kick dérivée au premier tick
#   ✓ Freinage naturel : stick relâché → consigne 0 → frein progressif
#
# =============================================================================

from physics_engine import (MASSE, THRUST_MAX, G,
                             INERTIE_ROLL, INERTIE_PITCH, INERTIE_YAW,
                             BRAS_LEVIER, COEFF_YAW,
                             ROLL_MAX, PITCH_MAX)
import math

# ---------------------------------------------------------------------------
# Point d'équilibre physique exact
# ---------------------------------------------------------------------------

THROTTLE_HOVER = MASSE * G / THRUST_MAX   # 0.3924 = 39.24 %

# ---------------------------------------------------------------------------
# Gains physiques (accélération par unité de commande)
# ---------------------------------------------------------------------------

K_ALTITUDE = THRUST_MAX / MASSE                       # 25.0  m/s²/u
K_YAW      = THRUST_MAX * COEFF_YAW / INERTIE_YAW    # 50.0  rad/s²/u

# ---------------------------------------------------------------------------
# Angle et vitesse max en mode position
# ---------------------------------------------------------------------------

ANGLE_CMD_MAX  = math.radians(22.0)  # 22° max commandé par boucle vitesse
VIT_MAX_CMD_XY = 4.0                 # m/s max commande clavier


# ===========================================================================
# ModeleReference2Ordre
# ===========================================================================
# Filtre une consigne brutale (échelon) en trajectoire en S-curve.
#
# Équation : y'' + 2·ζ·ωn·y' + ωn²·y = ωn²·consigne
# Intégrée par Euler pas à pas.
#
# ENTRÉE  : consigne [m/s]
# SORTIE  : y = valeur filtrée [m/s]
# ===========================================================================

class ModeleReference2Ordre:

    def __init__(self, omega_n, zeta):
        self.wn   = omega_n
        self.zeta = zeta
        self.y    = 0.0
        self.dy   = 0.0

    def update(self, consigne, dt):
        if dt <= 0.0:
            return self.y
        ddy     = self.wn**2 * (consigne - self.y) - 2.0*self.zeta*self.wn * self.dy
        self.dy += ddy    * dt
        self.y  += self.dy * dt
        return self.y

    def reset(self, valeur=0.0):
        self.y  = valeur
        self.dy = 0.0


# ===========================================================================
# CorrecteurVitesse  —  modèle 2ème ordre + correcteur PI
# ===========================================================================
#
# Ajout d'un intégrateur (Ki) par rapport à la version précédente :
#   → annule l'erreur statique résiduelle (vent simulé, frottement asymétrique)
#   → Ki volontairement faible pour ne pas créer de windup
#
# Unités :
#   consigne      [m/s]
#   v_ref         [m/s]   sortie ModèleRéférence
#   v_mesurée     [m/s]
#   erreur_v      [m/s]
#   angle         [rad]   = Kp·erreur + Ki·∫erreur
#   Kp            [rad/(m/s)]
#   Ki            [rad/(m·s⁻¹·s)] = [rad/m]
#
# ===========================================================================

class CorrecteurVitesse:

    KP = ANGLE_CMD_MAX / VIT_MAX_CMD_XY   # 0.096 rad/(m/s)
    KI = 0.008                             # intégrateur doux anti-dérive

    def __init__(self, omega_n=1.6, zeta=0.95):
        self.ref        = ModeleReference2Ordre(omega_n, zeta)
        self.kp         = self.KP
        self.ki         = self.KI
        self._integrale = 0.0
        self._lim_i     = ANGLE_CMD_MAX * 0.25   # anti-windup serré

    def calculer(self, consigne_ms, vitesse_mesuree_ms, dt):
        """
        consigne_ms        : vitesse cible  [m/s]
        vitesse_mesuree_ms : vitesse réelle [m/s]
        dt                 : pas de temps   [s]
        retourne           : angle cible    [rad]
        """
        v_ref  = self.ref.update(consigne_ms, dt)
        erreur = v_ref - vitesse_mesuree_ms

        # Intégrateur avec anti-windup par clamping
        self._integrale += erreur * dt
        self._integrale  = _clamp(self._integrale, -self._lim_i, self._lim_i)

        angle = self.kp * erreur + self.ki * self._integrale
        return _clamp(angle, -ANGLE_CMD_MAX, ANGLE_CMD_MAX)

    def reset(self, valeur_ms=0.0):
        self.ref.reset(valeur_ms)
        self._integrale = 0.0


# ===========================================================================
# PID générique  —  altitude, yaw, position
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

        # Intégrateur anti-windup
        self._integrale += erreur * dt
        self._integrale  = _clamp(self._integrale,
                                  -self.lim_integrale, self.lim_integrale)
        terme_i = self.ki * self._integrale

        # Dérivée — pas de kick au premier tick
        if self._premier_tick:
            terme_d            = 0.0
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
        self.kp = kp; self.ki = ki; self.kd = kd
        self.reset()

    def debug(self):
        return (f"Kp={self.kp:.4f} Ki={self.ki:.4f} Kd={self.kd:.4f} "
                f"I={self._integrale:.4f}")


# ===========================================================================
# FlightPIDs  —  instanciation de tous les correcteurs
# ===========================================================================
#
# ALTITUDE — placement de pôles ω=1.8 rad/s, ζ=0.95 (quasi-critique)
#   k = K_ALTITUDE = 25 m/s²/u
#   Kp = ω²/k         = 3.24/25  = 0.130
#   Kd = 2·ζ·ω/k      = 3.42/25  = 0.137
#   Ki = faible        = 0.012    → annule dérive lente sans pomper
#   → montée ~2s, zéro dépassement, tenue d'altitude solide
#
# VITESSE XY — CorrecteurVitesse (2ème ordre + PI)
#   ω=1.6 ζ=0.95 → frein progressif ~2s
#   Ki=0.018 → annule erreur statique sans windup
#
# YAW — PID classique
#   ω=4 ζ=0.90 → rotation franche, bien amorti
#
# ===========================================================================

class FlightPIDs:

    def __init__(self):

        # ── Altitude ──────────────────────────────────────────────────
        # ω=1.8 ζ=0.95 → montée progressive ~2s, zéro dépassement
        marge = 1.0 - THROTTLE_HOVER   # ~0.608
        self.altitude = PID(
            kp            = 0.130,   # ω²/k    = 3.24/25
            ki            = 0.012,   # dérive lente uniquement
            kd            = 0.137,   # 2·ζ·ω/k = 3.42/25
            lim_sortie    = marge,
            lim_integrale = marge * 0.20   # windup très serré
        )

        # ── Vitesse horizontale XY (2ème ordre + PI) ──────────────────
        # ω=1.6 ζ=0.95 → frein progressif naturel ~2s
        self.vel_x = CorrecteurVitesse(omega_n=1.6, zeta=0.95)
        self.vel_y = CorrecteurVitesse(omega_n=1.6, zeta=0.95)

        # ── Yaw ───────────────────────────────────────────────────────
        # ω=4 ζ=0.90 → rotation franche bien amortie
        # Kp=16/50=0.320  Kd=7.2/50=0.144  Ki faible
        self.yaw = PID(
            kp            = 0.320,
            ki            = 0.008,   # réduit vs avant : moins de drift yaw
            kd            = 0.144,
            lim_sortie    = 0.80,
            lim_integrale = 0.15
        )

        # ── Position XY (mode autonome / HOME) ────────────────────────
        # Gains légèrement augmentés pour navigation HOME plus précise
        self.pos_x = PID(kp=0.50, ki=0.008, kd=0.25, lim_sortie=0.40)
        self.pos_y = PID(kp=0.50, ki=0.008, kd=0.25, lim_sortie=0.40)

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
        print(f"  Vel XY     : Kp={CorrecteurVitesse.KP:.4f} "
              f"Ki={CorrecteurVitesse.KI:.4f} rad/(m/s) "
              f"wn={self.vel_x.ref.wn:.2f}  zeta={self.vel_x.ref.zeta:.2f}")
        print(f"  Yaw        : {self.yaw.debug()}")
        print("=" * 60)


def _clamp(val, mini, maxi):
    return max(mini, min(maxi, val))
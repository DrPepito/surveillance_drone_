# =============================================================================
# physics_engine.py  —  v3
#
# CORRECTIFS :
#   • Mixer réécrit : pitch inversé (AV/AR) + ratio asymétrique 60/40
#     → pitch+ (avancer) booste AR à 60%, réduit AV à 40%
#     → couple_pitch négatif → angle pitch négatif
#     → force_y = -thrust*sin(pitch) > 0  ✓ drone avance
#   • Même logique 60/40 pour roll et yaw
#   • force_y conserve le signe négatif du correctif v2
# =============================================================================

import math

G             = 9.81
MASSE         = 0.8
THRUST_MAX    = 20.0
DRAG_LIN      = 0.35
DRAG_ANG      = 0.60
INERTIE_ROLL  = 0.01
INERTIE_PITCH = 0.01
INERTIE_YAW   = 0.02
BRAS_LEVIER   = 0.15
COEFF_YAW     = 0.05

ROLL_MAX       = math.radians(35)
PITCH_MAX      = math.radians(35)
VITESSE_MAX_XY = 8.0
VITESSE_MAX_Z  = 4.0
ALTITUDE_SOL   = 0.0


def _clamp(val, mini, maxi):
    return max(mini, min(maxi, val))


# ---------------------------------------------------------------------------
# Mixer quadcopter X-frame  —  v3 avec ratio 60/40
#
# Disposition vue de dessus :
#   M1 (AV-G, CCW) ──── M2 (AV-D, CW)
#        |                    |
#   M4 (AR-G, CW)  ──── M3 (AR-D, CCW)
#
# Règle 60/40 :
#   Le groupe "actif" (celui qui génère le couple voulu) reçoit +60% de la cmd.
#   Le groupe "passif" (côté opposé)                    reçoit -40% de la cmd.
#   → somme nette = +20% * cmd → thrust légèrement augmenté côté actif
#   → asymétrie réaliste, évite la saturation unilatérale
#
# Convention commandes :
#   cmd_pitch > 0  →  avancer   (AR boosté → couple pitch− → nez vers avant)
#   cmd_pitch < 0  →  reculer   (AV boosté → couple pitch+ → nez vers arrière)
#   cmd_roll  > 0  →  droite    (G  boosté → couple roll−  → inclinaison droite)
#   cmd_roll  < 0  →  gauche
#   cmd_yaw   > 0  →  rotation horaire vue dessus
# ---------------------------------------------------------------------------

def mixer_moteurs(throttle, cmd_roll, cmd_pitch, cmd_yaw):

    RATIO_HAUT = 0.60   # groupe actif  : reçoit 60% de l'amplitude
    RATIO_BAS  = 0.40   # groupe passif : réduit de 40%

    p = cmd_pitch
    r = cmd_roll

    # ── Pitch : cmd_pitch > 0 = avancer ──────────────────────────────
    # Pour avancer, les moteurs AR (M3, M4) sont boostés → nez s'incline AV
    # couple_pitch = (AV − AR) * ... → si AR > AV → couple_pitch < 0 → pitch < 0
    # force_y = −thrust * sin(pitch) → si pitch < 0 → force_y > 0 → avance ✓
    pitch_av = -RATIO_BAS  * p    # AV (M1, M2) : réduits quand p > 0
    pitch_ar =  RATIO_HAUT * p    # AR (M3, M4) : boostés  quand p > 0

    # ── Roll : cmd_roll > 0 = droite ─────────────────────────────────
    # Moteurs gauche (M1, M4) boostés → couple roll− → inclinaison droite
    # force_x = thrust * sin(roll) → si roll > 0 → force_x > 0 → drift droite ✓
    roll_g =  RATIO_HAUT * r    # G (M1, M4) : boostés  quand r > 0
    roll_d = -RATIO_BAS  * r    # D (M2, M3) : réduits  quand r > 0

    # ── Yaw : limité pour garder tous les moteurs positifs ────────────
    RATIO_YAW_MAX = 0.50
    marge       = throttle * RATIO_YAW_MAX - abs(p) - abs(r)
    yaw_limite  = _clamp(marge, 0.0, throttle * RATIO_YAW_MAX)
    y           = _clamp(cmd_yaw, -yaw_limite, yaw_limite)
    # CW  (M2, M4) boostés → yaw horaire
    yaw_cw  =  RATIO_HAUT * y
    yaw_ccw = -RATIO_BAS  * y

    # ── Mixer final ───────────────────────────────────────────────────
    # M1 = AV-G-CCW
    m1 = throttle + pitch_av + roll_g + yaw_ccw
    # M2 = AV-D-CW
    m2 = throttle + pitch_av + roll_d + yaw_cw
    # M3 = AR-D-CCW
    m3 = throttle + pitch_ar + roll_d + yaw_ccw
    # M4 = AR-G-CW
    m4 = throttle + pitch_ar + roll_g + yaw_cw

    # ── Normalisation : aucun moteur > 1.0 ───────────────────────────
    max_val = max(m1, m2, m3, m4, 1.0)
    m1 /= max_val; m2 /= max_val
    m3 /= max_val; m4 /= max_val

    motors = [_clamp(m1, 0.0, 1.0),
              _clamp(m2, 0.0, 1.0),
              _clamp(m3, 0.0, 1.0),
              _clamp(m4, 0.0, 1.0)]

    # ── Désaturation thrust ───────────────────────────────────────────
    somme_cible    = 4.0 * _clamp(throttle, 0.0, 1.0)
    somme_actuelle = sum(motors)
    if somme_actuelle > somme_cible + 1e-6:
        facteur = somme_cible / somme_actuelle
        motors  = [m * facteur for m in motors]

    return motors


# ---------------------------------------------------------------------------
# Boucle physique principale — 50 Hz, dt = 0.02 s
# ---------------------------------------------------------------------------
def update(state, dt: float):
    if not state.moteurs_armes:
        _sol_sans_moteurs(state)
        return

    # 1. Mixer
    state.moteurs = mixer_moteurs(
        state.cmd_throttle,
        state.cmd_roll,
        state.cmd_pitch,
        state.cmd_yaw
    )

    # 2a. Poussée totale (N)
    somme        = sum(state.moteurs)
    thrust_total = (somme / 4.0) * THRUST_MAX

    # 2b. Couples (N·m)
    # Roll  : G(M1+M4) vs D(M2+M3)
    couple_roll  = ((state.moteurs[0] + state.moteurs[3])
                  - (state.moteurs[1] + state.moteurs[2])) * THRUST_MAX * BRAS_LEVIER

    # Pitch : AV(M1+M2) vs AR(M3+M4)
    # En avançant : AR > AV → couple_pitch < 0 → pitch < 0 (nez bas vers avant)
    couple_pitch = ((state.moteurs[0] + state.moteurs[1])
                  - (state.moteurs[2] + state.moteurs[3])) * THRUST_MAX * BRAS_LEVIER

    # Yaw   : CW(M2+M4) vs CCW(M1+M3)
    couple_yaw   = ((state.moteurs[1] + state.moteurs[3])
                  - (state.moteurs[0] + state.moteurs[2])) * THRUST_MAX * COEFF_YAW

    # 2c. Accélérations angulaires
    alpha_roll  = (couple_roll  - DRAG_ANG * state.roll_rate)  / INERTIE_ROLL
    alpha_pitch = (couple_pitch - DRAG_ANG * state.pitch_rate) / INERTIE_PITCH
    alpha_yaw   = (couple_yaw   - DRAG_ANG * state.yaw_rate)   / INERTIE_YAW

    # 3. Euler angulaire
    state.roll_rate  += alpha_roll  * dt
    state.pitch_rate += alpha_pitch * dt
    state.yaw_rate   += alpha_yaw   * dt

    state.roll  += state.roll_rate  * dt
    state.pitch += state.pitch_rate * dt
    state.yaw   += state.yaw_rate   * dt

    state.roll  = _clamp(state.roll,  -ROLL_MAX,  ROLL_MAX)
    state.pitch = _clamp(state.pitch, -PITCH_MAX, PITCH_MAX)
    state.yaw   = ((state.yaw + math.pi) % (2 * math.pi)) - math.pi

    # 4a. Forces dans le repère monde
    # +X = droite,  +Y = avant,  +Z = haut
    #
    # force_y = -thrust * sin(pitch)
    #   pitch < 0 (nez vers avant) → -thrust * sin(pitch) > 0 → avance ✓
    #   pitch > 0 (nez vers arrière) → force_y < 0 → recule ✓
    force_x =  thrust_total * math.sin(state.roll)
    force_y = -thrust_total * math.sin(state.pitch)
    force_z =  thrust_total * math.cos(state.roll) * math.cos(state.pitch) - MASSE * G

    frot_x = -DRAG_LIN * state.vitesse.x
    frot_y = -DRAG_LIN * state.vitesse.y
    frot_z = -DRAG_LIN * state.vitesse.z

    # 4b. Accélérations linéaires
    state.acceleration.x = (force_x + frot_x) / MASSE
    state.acceleration.y = (force_y + frot_y) / MASSE
    state.acceleration.z = (force_z + frot_z) / MASSE

    # 4c. Euler linéaire
    state.vitesse.x += state.acceleration.x * dt
    state.vitesse.y += state.acceleration.y * dt
    state.vitesse.z += state.acceleration.z * dt

    state.vitesse.x = _clamp(state.vitesse.x, -VITESSE_MAX_XY, VITESSE_MAX_XY)
    state.vitesse.y = _clamp(state.vitesse.y, -VITESSE_MAX_XY, VITESSE_MAX_XY)
    state.vitesse.z = _clamp(state.vitesse.z, -VITESSE_MAX_Z,  VITESSE_MAX_Z)

    state.position.x += state.vitesse.x * dt
    state.position.y += state.vitesse.y * dt
    state.position.z += state.vitesse.z * dt

    # 5. Contraintes sol
    if state.position.z <= ALTITUDE_SOL:
        state.position.z  = ALTITUDE_SOL
        if state.vitesse.z < 0:
            state.vitesse.z = 0.0
        state.acceleration.z = 0.0

    # 6. Batterie
    _update_batterie(state, dt)

    # 7. Métriques
    state.push_historique()
    state.update_distance()
    if state.mode_vol != "SOL":
        state.temps_vol += dt


def _sol_sans_moteurs(state):
    state.vitesse.reset()
    state.acceleration.reset()
    state.roll_rate  *= 0.0
    state.pitch_rate *= 0.0
    state.yaw_rate   *= 0.0
    state.roll       *= 0.85
    state.pitch      *= 0.85
    if state.position.z < ALTITUDE_SOL:
        state.position.z = ALTITUDE_SOL


def _update_batterie(state, dt):
    CAPACITE_MAH   = 5000.0
    TENSION_PLEINE = 12.6
    TENSION_VIDE   = 9.9
    COURANT_MAX    = 40.0

    puissance_norm         = sum(state.moteurs) / 4.0
    state.batterie_courant = puissance_norm * COURANT_MAX
    state.batterie_mah    += state.batterie_courant * (dt / 3600.0) * 1000.0

    state.batterie_pct = _clamp(
        100.0 * (1.0 - state.batterie_mah / CAPACITE_MAH), 0.0, 100.0)

    state.batterie_tension = (TENSION_VIDE
        + (TENSION_PLEINE - TENSION_VIDE) * (state.batterie_pct / 100.0))
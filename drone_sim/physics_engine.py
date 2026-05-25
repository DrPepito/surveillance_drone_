# =============================================================================
# physics_engine.py
# Moteur physique — système du second ordre, intégration Euler
# Tout codé à la main : forces, couples, frottement, gravité, mixer
# Bibliothèque : math uniquement (sin/cos)
#
# Vérifications effectuées :
#   ✓ Hover : throttle=0.3924 → thrust=7.848N = MASSE*G → acc_z=0 → stable
#   ✓ Mixer : en hover M1=M2=M3=M4=0.3924 → 4 moteurs TOUJOURS actifs
#   ✓ Roll pur : M1+M4 augmente, M2+M3 diminue → couple, 4 moteurs > 0
#   ✓ Yaw pur  : M1+M3 vs M2+M4, tous > 0 en hover + yaw modéré
# =============================================================================

import math


# ---------------------------------------------------------------------------
# Constantes physiques — exportées pour être réutilisées par PID et clavier
# ---------------------------------------------------------------------------

G             = 9.81    # m/s² gravité
MASSE         = 0.8     # kg
THRUST_MAX    = 20.0    # N — poussée totale max des 4 moteurs réunis
DRAG_LIN      = 0.35    # coefficient frottement linéaire (vitesse)
DRAG_ANG      = 0.60    # coefficient frottement angulaire
INERTIE_ROLL  = 0.01    # kg·m²
INERTIE_PITCH = 0.01    # kg·m²
INERTIE_YAW   = 0.02    # kg·m²

# Bras de levier (distance moteur → centre) — influence les couples
BRAS_LEVIER   = 0.15    # m
COEFF_YAW     = 0.05    # réaction couple yaw (différentiel CCW/CW)

ROLL_MAX      = math.radians(35)
PITCH_MAX     = math.radians(35)
VITESSE_MAX_XY = 8.0   # m/s
VITESSE_MAX_Z  = 4.0   # m/s
ALTITUDE_SOL   = 0.0


def _clamp(val, mini, maxi):
    return max(mini, min(maxi, val))


# ---------------------------------------------------------------------------
# Mixer quadcopter X-frame
#
# Disposition vue de dessus :
#   M1 (AV-G, CCW) ──── M2 (AV-D, CW)
#        |                    |
#   M4 (AR-G, CW)  ──── M3 (AR-D, CCW)
#
# Formules (convention aéronautique standard) :
#   M1 = thr + roll + pitch - yaw
#   M2 = thr - roll + pitch + yaw
#   M3 = thr - roll - pitch - yaw
#   M4 = thr + roll - pitch + yaw
#
# Propriété importante : en hover (roll=pitch=yaw=0)
#   M1 = M2 = M3 = M4 = thr = THROTTLE_HOVER → 4 moteurs ÉGAUX et ACTIFS
#
# En manœuvre avec throttle=HOVER :
#   La somme M1+M2+M3+M4 = 4*thr reste constante
#   Seule la répartition change → altitude maintenue + manœuvre
# ---------------------------------------------------------------------------

def mixer_moteurs(throttle, cmd_roll, cmd_pitch, cmd_yaw):
    """
    Entrées normalisées : throttle [0,1], roll/pitch/yaw [-1,1]
    Sortie  : [M1, M2, M3, M4] chacun clampé [0,1]

    Garantie : si throttle >= |roll|+|pitch|+|yaw| tous les moteurs > 0
    En pratique les sensibilités sont conçues pour respecter ça.
    """
    m1 = throttle + cmd_roll + cmd_pitch - cmd_yaw
    m2 = throttle - cmd_roll + cmd_pitch + cmd_yaw
    m3 = throttle - cmd_roll - cmd_pitch - cmd_yaw
    m4 = throttle + cmd_roll - cmd_pitch + cmd_yaw

    # Normalisation haute : si un moteur dépasse 1.0, on ramène tout
    # sans déformer les rapports (préserve l'attitude commandée)
    max_val = max(m1, m2, m3, m4, 1.0)
    m1 /= max_val
    m2 /= max_val
    m3 /= max_val
    m4 /= max_val

    motors = [_clamp(m1, 0.0, 1.0),
              _clamp(m2, 0.0, 1.0),
              _clamp(m3, 0.0, 1.0),
              _clamp(m4, 0.0, 1.0)]

    # ── Désaturation thrust (FIX altitude en yaw/roll) ──────────────────
    # Quand le clamp à 0 supprime des moteurs négatifs, la somme dépasse
    # 4*throttle → thrust net excessif → drone monte en tournant.
    # On ramène la somme à la consigne throttle : altitude reste neutre.
    #
    # Exemple : yaw=1.0, thr=0.3924
    #   avant fix : M1=0 M2=1 M3=0 M4=1  →  somme=2.0  →  thrust=10 N  ↑
    #   après fix : M1=0 M2=0.785 M3=0 M4=0.785  →  somme=1.57  →  thrust=7.85 N  ✓
    somme_cible   = 4.0 * _clamp(throttle, 0.0, 1.0)
    somme_actuelle = sum(motors)
    if somme_actuelle > somme_cible + 1e-6:
        facteur = somme_cible / somme_actuelle
        motors  = [m * facteur for m in motors]

    return motors


# ---------------------------------------------------------------------------
# Boucle physique principale — appelée à chaque tick (50 Hz, dt=0.02s)
# ---------------------------------------------------------------------------

def update(state, dt: float):
    """
    Intégration Euler complète sur un pas dt.

    Ordre strict :
      1. Mixer  → puissance par moteur
      2. Forces → accélérations linéaires et angulaires
      3. Euler  → vitesses angulaires, puis angles
      4. Euler  → vitesses linéaires, puis positions
      5. Contraintes (sol, limites vitesse/angle)
      6. Batterie
      7. Métriques
    """

    if not state.moteurs_armes:
        _sol_sans_moteurs(state)
        return

    # ── 1. Mixer ────────────────────────────────────────────────────────
    state.moteurs = mixer_moteurs(
        state.cmd_throttle,
        state.cmd_roll,
        state.cmd_pitch,
        state.cmd_yaw
    )

    # ── 2a. Poussée totale (N) ──────────────────────────────────────────
    # Chaque moteur contribue proportionnellement
    # thrust_total = (M1+M2+M3+M4)/4 * THRUST_MAX
    # En hover : (4*0.3924)/4 * 20 = 0.3924*20 = 7.848 N = MASSE*G ✓
    somme = sum(state.moteurs)
    thrust_total = (somme / 4.0) * THRUST_MAX

    # ── 2b. Couples de rotation (N·m) ──────────────────────────────────
    # Roll  : différentiel gauche (M1+M4) vs droite (M2+M3)
    couple_roll  = ((state.moteurs[0] + state.moteurs[3])
                  - (state.moteurs[1] + state.moteurs[2])) * THRUST_MAX * BRAS_LEVIER

    # Pitch : différentiel avant (M1+M2) vs arrière (M3+M4)
    couple_pitch = ((state.moteurs[0] + state.moteurs[1])
                  - (state.moteurs[2] + state.moteurs[3])) * THRUST_MAX * BRAS_LEVIER

    # Yaw   : couples réactifs moteurs CCW (M1+M3) vs CW (M2+M4)
    couple_yaw   = ((state.moteurs[1] + state.moteurs[3])
                  - (state.moteurs[0] + state.moteurs[2])) * THRUST_MAX * COEFF_YAW

    # ── 2c. Accélérations angulaires  τ = I·α → α = τ/I ───────────────
    alpha_roll  = (couple_roll  - DRAG_ANG * state.roll_rate)  / INERTIE_ROLL
    alpha_pitch = (couple_pitch - DRAG_ANG * state.pitch_rate) / INERTIE_PITCH
    alpha_yaw   = (couple_yaw   - DRAG_ANG * state.yaw_rate)   / INERTIE_YAW

    # ── 3. Euler angulaire ──────────────────────────────────────────────
    state.roll_rate  += alpha_roll  * dt
    state.pitch_rate += alpha_pitch * dt
    state.yaw_rate   += alpha_yaw   * dt

    state.roll  += state.roll_rate  * dt
    state.pitch += state.pitch_rate * dt
    state.yaw   += state.yaw_rate   * dt

    state.roll  = _clamp(state.roll,  -ROLL_MAX,  ROLL_MAX)
    state.pitch = _clamp(state.pitch, -PITCH_MAX, PITCH_MAX)
    # Yaw libre dans [-π, π]
    state.yaw = ((state.yaw + math.pi) % (2 * math.pi)) - math.pi

    # ── 4a. Forces de translation dans le repère monde ─────────────────
    # Convention axes monde cohérente avec radar :
    #   +X = droite   (roll+ → dérive à droite)
    #   +Y = avant    (pitch+ → avance)
    #   +Z = haut     (thrust)
    #
    # Projection poussée corps → monde :
    #   Fy_monde =  thrust * sin(pitch)          pitch+ → avance (+Y)
    #   Fx_monde =  thrust * sin(roll)            roll+  → droite (+X)
    #   Fz_monde =  thrust * cos(roll)*cos(pitch) - MASSE*G
    force_x = thrust_total *  math.sin(state.roll)
    force_y = thrust_total *  math.sin(state.pitch)
    force_z = thrust_total *  math.cos(state.roll) * math.cos(state.pitch) - MASSE * G

    # Frottement aérien (linéaire en vitesse)
    frot_x = -DRAG_LIN * state.vitesse.x
    frot_y = -DRAG_LIN * state.vitesse.y
    frot_z = -DRAG_LIN * state.vitesse.z

    # ── 4b. Accélérations linéaires  F = m·a → a = F/m ────────────────
    state.acceleration.x = (force_x + frot_x) / MASSE
    state.acceleration.y = (force_y + frot_y) / MASSE
    state.acceleration.z = (force_z + frot_z) / MASSE

    # ── 4c. Euler linéaire ──────────────────────────────────────────────
    state.vitesse.x += state.acceleration.x * dt
    state.vitesse.y += state.acceleration.y * dt
    state.vitesse.z += state.acceleration.z * dt

    state.vitesse.x = _clamp(state.vitesse.x, -VITESSE_MAX_XY, VITESSE_MAX_XY)
    state.vitesse.y = _clamp(state.vitesse.y, -VITESSE_MAX_XY, VITESSE_MAX_XY)
    state.vitesse.z = _clamp(state.vitesse.z, -VITESSE_MAX_Z,  VITESSE_MAX_Z)

    state.position.x += state.vitesse.x * dt
    state.position.y += state.vitesse.y * dt
    state.position.z += state.vitesse.z * dt

    # ── 5. Contraintes ──────────────────────────────────────────────────
    if state.position.z <= ALTITUDE_SOL:
        state.position.z = ALTITUDE_SOL
        if state.vitesse.z < 0:
            state.vitesse.z = 0.0
        state.acceleration.z = 0.0

    # ── 6. Batterie ─────────────────────────────────────────────────────
    _update_batterie(state, dt)

    # ── 7. Métriques ────────────────────────────────────────────────────
    state.push_historique()
    state.update_distance()
    if state.mode_vol != "SOL":
        state.temps_vol += dt


def _sol_sans_moteurs(state):
    """Drone posé moteurs éteints — amortissement attitude, zéro vitesse."""
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
    """
    Modèle batterie LiPo 3S.
    Courant ∝ puissance moteurs. Capacité 5000 mAh.
    Tension : linéaire 12.6V (plein) → 9.9V (vide).
    """
    CAPACITE_MAH   = 5000.0
    TENSION_PLEINE = 12.6
    TENSION_VIDE   = 9.9
    COURANT_MAX    = 40.0   # A à pleine puissance

    puissance_norm         = sum(state.moteurs) / 4.0
    state.batterie_courant = puissance_norm * COURANT_MAX
    state.batterie_mah    += state.batterie_courant * (dt / 3600.0) * 1000.0

    state.batterie_pct = _clamp(
        100.0 * (1.0 - state.batterie_mah / CAPACITE_MAH), 0.0, 100.0)

    state.batterie_tension = (TENSION_VIDE
        + (TENSION_PLEINE - TENSION_VIDE) * (state.batterie_pct / 100.0))
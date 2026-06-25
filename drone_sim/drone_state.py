# drone_state.py  (version HIL v2)
# État complet du drone à chaque instant T
#
# Corrections v2 :
#   ✓ MODE_HOME documenté dans exporter_telemetrie()
#   ✓ HIST_SIZE constante de classe (plus de N local)
#   ✓ cible_yaw supprimé (champ mort)
#   ✓ mode_vol protégé par property + setter validant
#   ✓ distance_origine 2D documentée explicitement
#   ✓ MODES_VALIDES comme ensemble de classe

import math


# vecteur 3D minimaliste, on s'en sert pour position, vitesse et accélération
# pas de numpy ici pour rester léger et sans dépendance lourde
class Vec3:

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


# cette classe est le "bus de données" central du simulateur
# tout le monde y lit et y écrit : physics_engine, pid_controller, HUD, HIL bridge
# on garde tout dans des unités SI : mètres, m/s, radians
#
# organisation interne en blocs :
#   cinématique  : position, vitesse, accélération
#   attitude     : roll, pitch, yaw + vitesses angulaires
#   commandes    : ce que le PID ou le pilote demande aux moteurs
#   moteurs      : puissance effective de chaque moteur
#   batterie     : charge, tension, consommation
#   systèmes     : mode de vol, temps, distance à l'origine
#   PID          : consigne altitude
#   historique   : buffers pour les graphes

class DroneState:

    # les modes sont des constantes de classe pour éviter les typos partout dans le code
    # on valide les écritures via la property mode_vol donc une faute de frappe lève une erreur
    MODE_SOL       = "SOL"
    MODE_DECOLLAGE = "DECOLLAGE"
    MODE_VOL       = "VOL"
    MODE_ATTERRO   = "ATTERRO"
    MODE_URGENCE   = "URGENCE"
    MODE_HOME      = "HOME"

    MODES_VALIDES  = {
        MODE_SOL, MODE_DECOLLAGE, MODE_VOL,
        MODE_ATTERRO, MODE_URGENCE, MODE_HOME
    }

    # taille unique des buffers historiques, référencée par les GraphsWidget aussi
    HIST_SIZE = 200

    def __init__(self):

        # cinématique : repère monde, x=droite, y=avant, z=altitude
        self.position     = Vec3()
        self.vitesse      = Vec3()   # m/s dans le repère monde
        self.acceleration = Vec3()   # m/s², calculée par physics_engine

        # attitude en radians, convention NED
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        # vitesses angulaires corps (rad/s), issues de physics_engine
        self.roll_rate  = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate   = 0.0

        # commandes normalisées -1.0 à +1.0 (throttle : 0.0 à 1.0)
        # c'est ce que le PID écrit, physics_engine les lit pour calculer les moteurs
        self.cmd_roll     = 0.0
        self.cmd_pitch    = 0.0
        self.cmd_yaw      = 0.0
        self.cmd_throttle = 0.0

        # puissance effective de chaque moteur, normalisée 0.0 → 1.0
        # ordre X-frame : M1=avant-droit, M2=arrière-gauche, M3=avant-gauche, M4=arrière-droit
        self.moteurs       = [0.0, 0.0, 0.0, 0.0]
        self.moteurs_armes = False   # tant que False, physics_engine ignore les commandes

        # batterie : on suit le pourcentage, la tension, la capacité consommée et le courant
        # bat_mah et bat_courant sont calculés par physics_engine à chaque tick
        self.batterie_pct     = 100.0
        self.batterie_tension = 12.6   # tension LiPo 3S pleine charge
        self.batterie_mah     = 0.0
        self.batterie_courant = 0.0

        # mode de vol stocké dans _mode_vol, accès via la property qui valide
        self._mode_vol        = DroneState.MODE_SOL
        self.temps_vol        = 0.0
        # distance 2D depuis (0,0) : Z ignoré volontairement
        # c'est ce qu'affiche le radar et ce que testent les alertes de zone
        # si on voulait une distance 3D : sqrt(x²+y²+z²)
        self.distance_origine = 0.0

        # consigne altitude pour le PID, mise à jour par MainWindow selon le mode
        self.cible_altitude = 0.0
        # note : cible_yaw a été supprimé en v2, aucun PID ne l'utilisait

        # buffers circulaires pour les graphes du panneau droit
        # on pop(0) et append() à chaque tick, c'est pas ultra-efficace
        # mais pour 200 points à 50 Hz c'est largement suffisant
        N = DroneState.HIST_SIZE
        self.hist_altitude  = [0.0] * N
        self.hist_vitesse_z = [0.0] * N
        self.hist_roll      = [0.0] * N
        self.hist_pitch     = [0.0] * N

        # compteur de trame pour le bridge HIL, se wrap à 65535 comme un uint16
        self._frame_cnt = 0

    # on passe par une property pour valider les écritures sur mode_vol
    # comme ça une faute de frappe dans le code lève immédiatement une ValueError
    # plutôt que de planter silencieusement plus loin

    @property
    def mode_vol(self) -> str:
        return self._mode_vol

    @mode_vol.setter
    def mode_vol(self, val: str):
        if val not in DroneState.MODES_VALIDES:
            raise ValueError(
                f"Mode de vol invalide : '{val}'. "
                f"Valeurs acceptées : {DroneState.MODES_VALIDES}"
            )
        self._mode_vol = val

    def push_historique(self):
        # appelé par GraphsWidget à chaque tick pour faire avancer les buffers
        # on convertit roll et pitch en degrés ici, les graphes affichent en degrés
        def push(lst, val):
            lst.pop(0)
            lst.append(val)
        push(self.hist_altitude,  self.position.z)
        push(self.hist_vitesse_z, self.vitesse.z)
        push(self.hist_roll,      math.degrees(self.roll))
        push(self.hist_pitch,     math.degrees(self.pitch))

    def update_distance(self):
        # recalcule la distance 2D horizontale, appelée par physics_engine à chaque tick
        # Z volontairement ignoré : le radar et les alertes de zone ne se soucient pas de l'altitude
        self.distance_origine = math.sqrt(
            self.position.x**2 + self.position.y**2)

    def exporter_telemetrie(self) -> dict:
        # snapshot complet de l'état, appelé à 50 Hz par HilBridge.envoyer()
        # tout est arrondi pour réduire la taille des trames JSON sur l'UDP
        # on garde plus de décimales sur l'attitude (att_rad) car c'est critique pour la physique
        #
        # champs exportés :
        #   t          — temps de vol (s)
        #   pos        — [x, y, z] position monde (m)
        #   vel        — [vx, vy, vz] vitesse monde (m/s)
        #   att_rad    — [roll, pitch, yaw] radians (6 décimales)
        #   att_deg    — [roll, pitch, yaw] degrés (3 décimales)
        #   att_rate   — [roll_rate, pitch_rate, yaw_rate] rad/s
        #   acc        — [ax, ay, az] accélération m/s²
        #   moteurs    — [M1, M2, M3, M4] puissance normalisée [0-1]
        #   cmd        — {throttle, roll, pitch, yaw} normalisés
        #   cible_alt  — consigne altitude PID (m)
        #   bat_pct    — batterie (%)
        #   bat_v      — tension batterie (V)
        #   bat_mah    — capacité consommée (mAh)
        #   mode       — "SOL"|"DECOLLAGE"|"VOL"|"ATTERRO"|"URGENCE"|"HOME"
        #   arme       — bool moteurs armés
        #   _cnt       — compteur de trame uint16, pour détecter les pertes de paquets côté récepteur

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

    def reset(self):
        # touche R dans MainWindow : remet tout à zéro sauf le cap
        # on garde le yaw pour que le drone reparte dans la même direction qu'avant le reset
        # le keyboard_controller gèle ses consignes au tick suivant via le guard MODE_SOL
        yaw_sauvegarde = self.yaw
        self.__init__()
        self.yaw = yaw_sauvegarde

    def __repr__(self):
        # affichage rapide en console pour le debug, montre ce qui change souvent
        return (f"[{self.mode_vol}] "
                f"pos={self.position} "
                f"roll={math.degrees(self.roll):.1f}° "
                f"pitch={math.degrees(self.pitch):.1f}° "
                f"bat={self.batterie_pct:.0f}%")
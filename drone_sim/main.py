# =============================================================================
# main.py
# Point d'entrée — boucle 50 Hz — gestion modes de vol
# PyQt6 pour fenêtre/timer/layout, tout le reste à la main
# =============================================================================

import sys
import math

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget,
                              QVBoxLayout, QHBoxLayout)
from PyQt6.QtCore    import QTimer, Qt
from PyQt6.QtGui     import QPainter, QColor, QPen, QFont, QPolygon, QKeyEvent
from PyQt6.QtCore    import QPoint

from drone_state         import DroneState
from physics_engine      import update as physics_update
from pid_controller      import FlightPIDs
from keyboard_controller import KeyboardController
from pid_controller import THROTTLE_HOVER
from orientation_widget  import OrientationWidget
from graph_widget       import GraphsWidget


FPS                = 50
DT                 = 1.0 / FPS
ALTITUDE_DECOLLAGE = 1.5    # m
THROTTLE_OFFSET_HOVER = 0.5 # offset PID altitude (≈ hovering)


# ---------------------------------------------------------------------------
# HUD principal — QPainter
# ---------------------------------------------------------------------------

class HUDWidget(QWidget):

    C_FOND   = QColor("#EC9C9CB0")
    C_CYAN   = QColor("#00D9FF")
    C_VERT   = QColor("#00FF88")
    C_ROUGE  = QColor("#FF3B3B")
    C_ORANGE = QColor("#FF8800")
    C_GRIS   = QColor("#444444")
    C_BLANC  = QColor("#DDDDDD")

    def __init__(self, state: DroneState):
        super().__init__()
        self.state = state
        self.setMinimumSize(800, 680)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, self.C_FOND)

        self._dessiner_radar(p, cx=w//2, cy=h//2 - 40, rayon=150)
        self._dessiner_horizon_artificiel(p, x=w-300, y=70, larg=260, haut=170)
        self._dessiner_telemetrie(p, x=16, y=20)
        self._dessiner_jauge_altitude(p, x=w-52, y=70, haut=h-160)
        self._dessiner_jauge_throttle(p, x=w-100, y=70, haut=h-160)
        self._dessiner_moteurs(p, x=16, y=h-155)
        self._dessiner_batterie(p, x=w//2-100, y=h-54, larg=200)
        self._dessiner_mode(p, x=w//2, y=28)
        self._dessiner_vitesse(p, x=w//2, y=h-100)
        self._dessiner_legende_clavier(p, x=w-300, y=h-180)

    # ── Radar ────────────────────────────────────────────────────────

    def _dessiner_radar(self, p, cx, cy, rayon):
        s = self.state
        p.setPen(QPen(self.C_GRIS, 0.5))
        for r in [rayon*0.33, rayon*0.66, rayon]:
            p.drawEllipse(int(cx-r), int(cy-r), int(r*2), int(r*2))
        p.drawLine(cx-rayon, cy, cx+rayon, cy)
        p.drawLine(cx, cy-rayon, cx, cy+rayon)

        echelle = rayon / 30.0
        dx = int(_clamp(s.position.x * echelle, -rayon+8, rayon-8))
        dy = int(_clamp(-s.position.y * echelle, -rayon+8, rayon-8))

        # Trace
        p.setPen(QPen(self.C_CYAN, 0.5, Qt.PenStyle.DotLine))
        p.drawLine(cx, cy, cx+dx, cy+dy)

        # Drone triangle orienté cap
        self._drone_triangle(p, cx+dx, cy+dy, s.yaw)

        p.setPen(QPen(self.C_CYAN))
        p.setFont(QFont("Monospace", 9))
        p.drawText(cx-rayon, cy+rayon+16,
                   f"Dist origine : {s.distance_origine:.1f} m")

    def _drone_triangle(self, p, x, y, yaw):
        t = 8
        p.save()
        p.translate(x, y)
        p.rotate(math.degrees(yaw))
        p.setBrush(self.C_CYAN)
        p.setPen(Qt.PenStyle.NoPen)
        tri = QPolygon([QPoint(0,-t), QPoint(t//2,t//2), QPoint(-t//2,t//2)])
        p.drawPolygon(tri)
        p.restore()

    # ── Horizon artificiel ───────────────────────────────────────────

    def _dessiner_horizon_artificiel(self, p, x, y, larg, haut):
        s = self.state
        cx, cy = x + larg//2, y + haut//2
        offset_pitch = int(math.degrees(s.pitch) * 2)

        p.save()
        p.setClipRect(x, y, larg, haut)
        p.translate(cx, cy + offset_pitch)
        p.rotate(math.degrees(-s.roll))
        p.fillRect(-larg, -haut*2, larg*2, haut*2, QColor("#003355"))
        p.fillRect(-larg, 0, larg*2, haut*2, QColor("#3D2B1F"))
        p.setPen(QPen(self.C_BLANC, 1))
        p.drawLine(-larg, 0, larg, 0)
        p.setPen(QPen(self.C_BLANC, 0.5))
        p.setFont(QFont("Monospace", 7))
        for deg in range(-30, 31, 10):
            if deg == 0: continue
            py_ = int(-deg*2)
            ll = 40 if deg % 20 == 0 else 20
            p.drawLine(-ll, py_, ll, py_)
            p.drawText(ll+3, py_+4, f"{deg}°")
        p.restore()

        # Chevron fixe
        p.setPen(QPen(self.C_ORANGE, 2))
        p.drawLine(cx-30, cy, cx-10, cy)
        p.drawLine(cx-10, cy, cx, cy-8)
        p.drawLine(cx, cy-8, cx+10, cy)
        p.drawLine(cx+10, cy, cx+30, cy)

        p.setPen(QPen(self.C_GRIS, 0.5))
        p.drawRect(x, y, larg, haut)
        p.setPen(QPen(self.C_GRIS))
        p.setFont(QFont("Monospace", 8))
        p.drawText(x, y-4, "HORIZON")

    # ── Télémétrie ───────────────────────────────────────────────────

    def _dessiner_telemetrie(self, p, x, y):
        s = self.state
        p.setFont(QFont("Monospace", 10))
        lh = 20

        def row(label, val, c=None):
            nonlocal y
            p.setPen(QPen(self.C_GRIS)); p.drawText(x, y, label)
            p.setPen(QPen(c or self.C_CYAN)); p.drawText(x+136, y, val)
            y += lh

        row("POS X",   f"{s.position.x:+7.2f} m")
        row("POS Y",   f"{s.position.y:+7.2f} m")
        row("ALT  Z",  f"{s.position.z:+7.2f} m")
        y += 5
        row("ROLL",    f"{math.degrees(s.roll):+7.1f} °")
        row("PITCH",   f"{math.degrees(s.pitch):+7.1f} °")
        row("YAW",     f"{math.degrees(s.yaw):+7.1f} °")
        y += 5
        row("VIT X",   f"{s.vitesse.x:+6.2f} m/s")
        row("VIT Y",   f"{s.vitesse.y:+6.2f} m/s")
        row("VIT Z",   f"{s.vitesse.z:+6.2f} m/s")
        y += 5
        # Consignes vitesse (ce que commande le pilote)
        kb = getattr(self, '_keyboard_ref', None)
        if kb and hasattr(kb, 'consigne_vx'):
            row("CMD VX",  f"{kb.consigne_vx:+5.1f} m/s", self.C_ORANGE)
            row("CMD VY",  f"{kb.consigne_vy:+5.1f} m/s", self.C_ORANGE)
        y += 5
        row("ALT CIB", f"{s.cible_altitude:+6.2f} m",   self.C_VERT)
        row("THR ACT", f"{s.cmd_throttle*100:.1f} %",    self.C_CYAN)
        y += 5
        mn = int(s.temps_vol//60); sc = int(s.temps_vol%60)
        row("TEMPS",   f"{mn:02d}:{sc:02d}")

    # ── Jauges verticales ────────────────────────────────────────────

    def _dessiner_jauge_altitude(self, p, x, y, haut):
        s = self.state
        ALT_MAX = 20.0
        p.setPen(QPen(self.C_GRIS, 0.5)); p.drawRect(x, y, 24, haut)
        rem = int(haut * min(s.position.z/ALT_MAX, 1.0))
        c = self.C_VERT if s.position.z < 15 else self.C_ORANGE
        p.fillRect(x+1, y+haut-rem, 22, rem, c)
        p.setPen(QPen(self.C_BLANC)); p.setFont(QFont("Monospace", 8))
        p.drawText(x-4, y+haut+14, "ALT")
        p.drawText(x-2, y+haut+26, f"{s.position.z:.1f}")

    def _dessiner_jauge_throttle(self, p, x, y, haut):
        s = self.state
        p.setPen(QPen(self.C_GRIS, 0.5)); p.drawRect(x, y, 24, haut)
        rem = int(haut * s.cmd_throttle)
        p.fillRect(x+1, y+haut-rem, 22, rem, self.C_CYAN)
        # Ligne hover
        hover_y = y + haut - int(haut * THROTTLE_HOVER)
        p.setPen(QPen(self.C_VERT, 1, Qt.PenStyle.DashLine))
        p.drawLine(x-4, hover_y, x+28, hover_y)
        p.setPen(QPen(self.C_BLANC)); p.setFont(QFont("Monospace", 8))
        p.drawText(x-4, y+haut+14, "THR")
        p.drawText(x-2, y+haut+26, f"{int(s.cmd_throttle*100)}%")

    # ── Moteurs ──────────────────────────────────────────────────────

    def _dessiner_moteurs(self, p, x, y):
        s = self.state
        for i, m in enumerate(s.moteurs):
            mx = x + i * 66
            hb = 55
            rem = int(hb * m)
            p.setPen(QPen(self.C_GRIS, 0.5)); p.drawRect(mx, y, 38, hb)
            c = self.C_VERT if m < 0.7 else self.C_ORANGE
            p.fillRect(mx+1, y+hb-rem, 36, rem, c)
            p.setPen(QPen(self.C_BLANC)); p.setFont(QFont("Monospace", 8))
            p.drawText(mx+8, y+hb+13, f"M{i+1}")
            p.drawText(mx+2, y+hb+25, f"{int(m*100)}%")

    # ── Batterie ─────────────────────────────────────────────────────

    def _dessiner_batterie(self, p, x, y, larg):
        s = self.state
        pct = s.batterie_pct
        c = self.C_VERT if pct > 40 else (self.C_ORANGE if pct > 20 else self.C_ROUGE)
        p.setPen(QPen(self.C_GRIS, 0.5)); p.drawRect(x, y, larg, 18)
        p.fillRect(x+1, y+1, int((larg-2)*pct/100), 16, c)
        p.setPen(QPen(self.C_BLANC)); p.setFont(QFont("Monospace", 9))
        p.drawText(x+larg//2-60, y+13, f"BAT {pct:.0f}%  {s.batterie_tension:.1f}V")

    # ── Mode ─────────────────────────────────────────────────────────

    def _dessiner_mode(self, p, x, y):
        couleurs = {DroneState.MODE_SOL: self.C_GRIS,
                    DroneState.MODE_DECOLLAGE: self.C_VERT,
                    DroneState.MODE_VOL: self.C_CYAN,
                    DroneState.MODE_ATTERRO: self.C_ORANGE,
                    DroneState.MODE_URGENCE: self.C_ROUGE}
        p.setPen(QPen(couleurs.get(self.state.mode_vol, self.C_BLANC)))
        p.setFont(QFont("Monospace", 14, QFont.Weight.Bold))
        p.drawText(x-60, y, f"[ {self.state.mode_vol} ]")

    # ── Vitesse sol ──────────────────────────────────────────────────

    def _dessiner_vitesse(self, p, x, y):
        s = self.state
        vxy = math.sqrt(s.vitesse.x**2 + s.vitesse.y**2)
        p.setPen(QPen(self.C_CYAN)); p.setFont(QFont("Monospace", 11))
        p.drawText(x-100, y,
                   f"VIT SOL : {vxy:.2f} m/s    VIT Z : {s.vitesse.z:+.2f} m/s")

    # ── Légende clavier ──────────────────────────────────────────────

    def _dessiner_legende_clavier(self, p, x, y):
        p.setFont(QFont("Monospace", 8))
        touches = [
            ("↑↓",    "Avancer / Reculer"),
            ("←→",    "Gauche / Droite"),
            ("Z / S",  "Monter / Descendre"),
            ("Q / D",  "Yaw G / D"),
            ("T / L",  "Décollage / Atterro"),
            ("ESPACE", "URGENCE"),
            ("R",      "Reset"),
        ]
        for i, (k, desc) in enumerate(touches):
            p.setPen(QPen(self.C_CYAN))
            p.drawText(x, y + i*15, k)
            p.setPen(QPen(self.C_GRIS))
            p.drawText(x+60, y + i*15, desc)


# ---------------------------------------------------------------------------
# Fenêtre principale
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Flight Simulator — AZERTY")
        self.setMinimumSize(1200, 800)
        self.setStyleSheet("background-color: #111111;")

        self.state    = DroneState()
        self.pids     = FlightPIDs()
        self.keyboard = KeyboardController()

        # Layout : HUD à gauche, widget orientation à droite
        self.hud         = HUDWidget(self.state)
        self.hud._keyboard_ref = self.keyboard
        self.orientation = OrientationWidget(self.state)
        self.graphs      = GraphsWidget(self.state)

        conteneur = QWidget()
        layout    = QHBoxLayout(conteneur)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.hud, stretch=1)

        # Colonne droite : orientation + espace vide
        col_droite = QWidget()
        col_droite.setStyleSheet("background-color: #0A0A0A;")
        col_droite.setFixedWidth(240)
        vl = QVBoxLayout(col_droite)
        vl.setContentsMargins(4, 4, 4, 4)
        vl.addWidget(self.orientation)
        vl.addWidget(self.graphs, stretch=1)
        layout.addWidget(col_droite)

        self.setCentralWidget(conteneur)

        # Timer 50 Hz
        self.timer = QTimer()
        self.timer.setInterval(int(1000 / FPS))
        self.timer.timeout.connect(self._tick)
        self.timer.start()

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # ------------------------------------------------------------------
    # Boucle principale
    # ------------------------------------------------------------------

    def _tick(self):
        s = self.state

        self.keyboard.update(s)

        if self.keyboard.consommer_urgence():
            self._urgence()
        if self.keyboard.consommer_reset():
            s.reset()
            self.pids.reset_all()
            self.keyboard.set_throttle_hover()
        if self.keyboard.consommer_decollage():
            self._decollage()
        if self.keyboard.consommer_atterrissage():
            self._atterrissage()
        if self.keyboard.consommer_home():
            self._retour_home()

        self._update_mode()
        self._appliquer_pids()

        physics_update(s, DT)

        self.hud.update()
        self.graphs.push()
        self.graphs.update()
        self.orientation.update()

    # ------------------------------------------------------------------
    # Modes
    # ------------------------------------------------------------------

    def _decollage(self):
        if self.state.mode_vol == DroneState.MODE_SOL:
            self.state.moteurs_armes  = True
            self.state.mode_vol       = DroneState.MODE_DECOLLAGE
            self.state.cible_altitude = ALTITUDE_DECOLLAGE
            self.keyboard.set_throttle_hover()
            self.pids.reset_all()

    def _atterrissage(self):
        if self.state.mode_vol in (DroneState.MODE_VOL, DroneState.MODE_DECOLLAGE):
            self.state.mode_vol       = DroneState.MODE_ATTERRO
            self.state.cible_altitude = 0.0

    def _retour_home(self):
        """Touche H — mémorise la consigne home pour le PID position."""
        if self.state.mode_vol == DroneState.MODE_VOL:
            self.state.cible_altitude = self.state.position.z   # garde altitude
            # Les PID pos_x/y ramèneront vers 0,0 (futur mode autonome)
            # En phase 1 : juste un atterrissage propre à la position actuelle
            self.state.mode_vol = DroneState.MODE_ATTERRO
            self.state.cible_altitude = 0.0

    def _urgence(self):
        self.state.mode_vol      = DroneState.MODE_URGENCE
        self.state.moteurs_armes = False
        self.state.moteurs       = [0.0, 0.0, 0.0, 0.0]
        self.state.cmd_throttle  = 0.0
        self.keyboard.reset_commandes()
        self.pids.reset_all()

    def _update_mode(self):
        s = self.state
        if s.mode_vol == DroneState.MODE_DECOLLAGE:
            if s.position.z >= ALTITUDE_DECOLLAGE * 0.95:
                s.mode_vol = DroneState.MODE_VOL
                # Verrouille la cible altitude à la hauteur atteinte
                s.cible_altitude = s.position.z
        elif s.mode_vol == DroneState.MODE_ATTERRO:
            if s.position.z <= 0.05 and abs(s.vitesse.z) < 0.1:
                s.mode_vol      = DroneState.MODE_SOL
                s.moteurs_armes = False
                s.moteurs       = [0.0, 0.0, 0.0, 0.0]
                s.cmd_throttle  = 0.0

    def _appliquer_pids(self):
        """
        Architecture DJI Position Hold — cascade vitesse → angle :

        1. Altitude  : PID(cible_altitude, z)            → cmd_throttle
        2. Rotation des consignes vitesse dans le repère monde selon yaw :
             vx_monde =  cmd_vx*cos(yaw) + cmd_vy*sin(yaw)
             vy_monde = -cmd_vx*sin(yaw) + cmd_vy*cos(yaw)
           → ↑ avance toujours dans la direction où pointe le nez ✓
        3. PID vitesse monde → angle cible → cmd normalisée → mixer
        4. Yaw : taux direct
        """
        s = self.state
        if s.mode_vol in (DroneState.MODE_SOL, DroneState.MODE_URGENCE):
            return

        # ── 1. Altitude ───────────────────────────────────────────────────
        correction_alt = self.pids.altitude.calculer(
            s.cible_altitude, s.position.z, DT)
        s.cmd_throttle = _clamp(THROTTLE_HOVER + correction_alt, 0.0, 1.0)

        if s.mode_vol not in (DroneState.MODE_VOL, DroneState.MODE_DECOLLAGE):
            return

        # ── 2. Projection des consignes dans le repère monde (via yaw) ───
        # Les consignes clavier sont en repère DRONE (↑ = nez, → = droite)
        # On les tourne par le yaw pour obtenir les consignes en repère MONDE
        cy = math.cos(s.yaw)
        sy = math.sin(s.yaw)
        cvx = self.keyboard.consigne_vx
        cvy = self.keyboard.consigne_vy

        consigne_vx_monde =  cvx * cy + cvy * sy
        consigne_vy_monde = -cvx * sy + cvy * cy

        # ── 3. PID vitesse monde → angle cible ───────────────────────────
        angle_pitch_cible = self.pids.vel_y.calculer(
            consigne_vy_monde, s.vitesse.y, DT)

        angle_roll_cible = self.pids.vel_x.calculer(
            consigne_vx_monde, s.vitesse.x, DT)

        # ── 3. Conversion angle (rad) → commande normalisée [-1,1] ───────
        from physics_engine import PITCH_MAX, ROLL_MAX
        s.cmd_pitch = _clamp(angle_pitch_cible / PITCH_MAX, -1.0, 1.0)
        s.cmd_roll  = _clamp(angle_roll_cible  / ROLL_MAX,  -1.0, 1.0)

        # ── 4. Yaw — taux direct ──────────────────────────────────────────
        from keyboard_controller import YAW_RATE_MAX
        s.cmd_yaw = _clamp(
            self.keyboard.consigne_yaw_rate / YAW_RATE_MAX,
            -1.0, 1.0)

    # ------------------------------------------------------------------
    # Clavier
    # ------------------------------------------------------------------

    def keyPressEvent(self, event: QKeyEvent):
        self.keyboard.key_press(event.key())

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.keyboard.key_release(event.key())


def _clamp(v, a, b):
    return max(a, min(b, v))


# ---------------------------------------------------------------------------
# Lancement
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
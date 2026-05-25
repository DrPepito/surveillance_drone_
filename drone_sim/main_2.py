# =============================================================================
# main.py
# Point d'entrée — boucle 50 Hz — gestion modes de vol
# PyQt6 pour fenêtre/timer/layout, tout le reste à la main
# =============================================================================

import sys
import math

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget,
                              QVBoxLayout, QHBoxLayout)
from PyQt6.QtCore    import QTimer, Qt, QPoint, QPointF, QRectF
from PyQt6.QtGui     import (QPainter, QColor, QPen, QFont,
                              QPolygon, QPolygonF, QKeyEvent)

from drone_state         import DroneState
from physics_engine      import update as physics_update
from pid_controller      import FlightPIDs
from keyboard_controller import KeyboardController
from pid_controller import THROTTLE_HOVER
from orientation_widget  import OrientationWidget


FPS                = 50
DT                 = 1.0 / FPS
ALTITUDE_DECOLLAGE = 1.5    # m
THROTTLE_OFFSET_HOVER = 0.5 # offset PID altitude (≈ hovering)


# ---------------------------------------------------------------------------
# HUD principal v2 — style cockpit DJI / FPV
# ---------------------------------------------------------------------------
#
# Layout (3 colonnes) :
#   LEFT  (145px) — 6 métriques clés + légende clavier
#   CENTER (flex) — barre header / horizon (58%) / radar (42%)
#   RIGHT  (82px) — jauge ALT + jauge THR avec zone hover
#
# Nouveautés vs v1 :
#   ✔ Flight Path Marker (FPM) — vecteur vitesse réel sur l'horizon
#   ✔ Trail de trajectoire sur le radar
#   ✔ Indicateur de roll arc (haut horizon)
#   ✔ Codage couleur attitude : vert/orange/rouge
#   ✔ Radar : marqueurs N/S/E/W, vecteur vitesse, ligne vers origine
#   ✔ Jauge hover avec zone verte ±5%
#   ✔ Télémétrie simplifiée — 6 valeurs max
# ---------------------------------------------------------------------------

class HUDWidget(QWidget):

    # ── Palette ──────────────────────────────────────────────────────
    C_BG     = QColor("#0C0C0C")
    C_PANEL  = QColor("#101010")
    C_BORDER = QColor("#222222")
    C_DGRAY  = QColor("#1E1E1E")
    C_GRAY   = QColor("#555555")
    C_WHITE  = QColor("#DDDDDD")
    C_CYAN   = QColor("#00D9FF")
    C_GREEN  = QColor("#00FF88")
    C_ORANGE = QColor("#FF8800")
    C_RED    = QColor("#FF3D3D")
    C_YELLOW = QColor("#FFD700")
    C_SKY    = QColor("#082030")
    C_GROUND = QColor("#261A0A")

    TRAIL_MAX = 120   # points de trajectoire stockés

    def __init__(self, state: DroneState):
        super().__init__()
        self.state        = state
        self._trail       = []          # [(x_monde, y_monde), ...]
        self._keyboard_ref = None
        self.setMinimumSize(800, 680)

    # ------------------------------------------------------------------
    # API tick
    # ------------------------------------------------------------------

    def push_trail(self):
        """Appelé depuis _tick() pour enregistrer la trajectoire."""
        s = self.state
        if s.moteurs_armes:
            self._trail.append((s.position.x, s.position.y))
            if len(self._trail) > self.TRAIL_MAX:
                self._trail.pop(0)

    # ------------------------------------------------------------------
    # Couleur attitude (stability feedback)
    # ------------------------------------------------------------------

    def _att_color(self):
        a = max(abs(math.degrees(self.state.roll)),
                abs(math.degrees(self.state.pitch)))
        if a < 10:  return self.C_GREEN
        if a < 22:  return self.C_ORANGE
        return self.C_RED

    # ------------------------------------------------------------------
    # paintEvent
    # ------------------------------------------------------------------

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        p.fillRect(0, 0, w, h, self.C_BG)

        # ── Constantes layout ──
        HDR_H   = 38
        LEFT_W  = 148
        RIGHT_W = 82
        CX      = LEFT_W
        CW      = w - LEFT_W - RIGHT_W
        BODY_H  = h - HDR_H
        HOR_H   = int(BODY_H * 0.58)
        RAD_H   = BODY_H - HOR_H

        self._draw_header (p, 0,       0,       w,       HDR_H)
        self._draw_left   (p, 0,       HDR_H,   LEFT_W,  BODY_H)
        self._draw_horizon(p, CX,      HDR_H,   CW,      HOR_H)
        self._draw_radar  (p, CX,      HDR_H + HOR_H, CW, RAD_H)
        self._draw_gauges (p, w-RIGHT_W, HDR_H, RIGHT_W, BODY_H)

    # ==================================================================
    # Header — mode / cap / batterie
    # ==================================================================

    def _draw_header(self, p, x, y, w, h):
        s = self.state
        p.fillRect(x, y, w, h, self.C_PANEL)
        p.setPen(QPen(self.C_BORDER, 1))
        p.drawLine(x, y+h-1, x+w, y+h-1)

        # Badge mode (gauche)
        MODE_COL = {
            "SOL":      self.C_GRAY,
            "DECOLLAGE":self.C_YELLOW,
            "VOL":      self.C_GREEN,
            "ATTERRO":  self.C_ORANGE,
            "URGENCE":  self.C_RED,
        }
        mc = MODE_COL.get(s.mode_vol, self.C_GRAY)
        bw = 96
        p.fillRect(x+4, y+4, bw, h-8,
                   QColor(mc.red(), mc.green(), mc.blue(), 35))
        p.setPen(QPen(mc, 1.5))
        p.drawRect(x+4, y+4, bw, h-8)
        p.setFont(QFont("Monospace", 11, QFont.Weight.Bold))
        p.drawText(x+4, y+4, bw, h-8,
                   Qt.AlignmentFlag.AlignCenter, s.mode_vol)

        # Dot stabilité
        ac = self._att_color()
        p.setBrush(ac)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(x+106, y+h//2-5, 10, 10)

        # Cap + temps (centre)
        yaw_deg = math.degrees(s.yaw) % 360
        mn = int(s.temps_vol//60); sc_t = int(s.temps_vol%60)
        cap_str = f"CAP {yaw_deg:05.1f}°    {mn:02d}:{sc_t:02d}"
        p.setPen(QPen(self.C_WHITE))
        p.setFont(QFont("Monospace", 10))
        p.drawText(x, y, w, h, Qt.AlignmentFlag.AlignCenter, cap_str)

        # Batterie (droite)
        bat  = s.batterie_pct
        bc   = self.C_GREEN if bat > 30 else self.C_ORANGE if bat > 15 else self.C_RED
        p.setPen(QPen(bc))
        p.setFont(QFont("Monospace", 9))
        bstr = f"{bat:.0f}%  {s.batterie_tension:.1f}V"
        p.drawText(x, y, w-8, h,
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                   bstr)

    # ==================================================================
    # Left panel — métriques clés + légende
    # ==================================================================

    def _draw_left(self, p, x, y, w, h):
        s  = self.state
        ac = self._att_color()
        p.fillRect(x, y, w, h, self.C_PANEL)
        p.setPen(QPen(self.C_BORDER, 1))
        p.drawLine(x+w-1, y, x+w-1, y+h)

        spd_xy = math.sqrt(s.vitesse.x**2 + s.vitesse.y**2)

        metrics = [
            ("ALT",   f"{s.position.z:5.1f}",         "m",    self.C_CYAN),
            ("VIT",   f"{spd_xy:5.1f}",                "m/s",  self.C_YELLOW),
            ("VZ",    f"{s.vitesse.z:+5.1f}",          "m/s",  self.C_ORANGE),
            ("DST",   f"{s.distance_origine:5.1f}",    "m",    self.C_WHITE),
            ("ROLL",  f"{math.degrees(s.roll):+5.1f}", "°",    ac),
            ("PITCH", f"{math.degrees(s.pitch):+5.1f}","°",    ac),
        ]

        p.setPen(QPen(self.C_GRAY))
        p.setFont(QFont("Monospace", 7))
        p.drawText(x+5, y+13, "TÉLÉMÉTRIE")

        n = len(metrics)
        # Répartir sur la moitié haute moins marge titre
        bloc_zone_h = h - 130
        bloc_h = bloc_zone_h // n

        for i, (lbl, val, unit, col) in enumerate(metrics):
            yy = y + 20 + i * bloc_h

            # Label
            p.setPen(QPen(self.C_GRAY))
            p.setFont(QFont("Monospace", 7))
            p.drawText(x+6, yy+2, lbl)

            # Valeur (grande)
            p.setPen(QPen(col))
            p.setFont(QFont("Monospace", 12, QFont.Weight.Bold))
            p.drawText(x+2, yy+14, w-10, 18,
                       Qt.AlignmentFlag.AlignRight, val)

            # Unité
            p.setPen(QPen(self.C_GRAY))
            p.setFont(QFont("Monospace", 7))
            p.drawText(x+2, yy+30, w-6, 10,
                       Qt.AlignmentFlag.AlignRight, unit)

            # Séparateur
            p.setPen(QPen(self.C_DGRAY, 0.5))
            p.drawLine(x+6, yy+38, x+w-6, yy+38)

        # Consigne altitude (visuel cible)
        ya = y + 20 + n * bloc_h + 8
        p.setPen(QPen(self.C_GRAY))
        p.setFont(QFont("Monospace", 7))
        p.drawText(x+5, ya, "ALT CIBLE")
        p.setPen(QPen(self.C_GREEN))
        p.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        p.drawText(x+2, ya+13, w-8, 16,
                   Qt.AlignmentFlag.AlignRight,
                   f"{s.cible_altitude:.1f} m")

        # Légende clavier
        self._draw_key_hints(p, x, y+h-110, w, 108)

    def _draw_key_hints(self, p, x, y, w, h):
        p.setPen(QPen(self.C_DGRAY, 0.5))
        p.drawLine(x+6, y, x+w-6, y)
        p.setPen(QPen(self.C_GRAY))
        p.setFont(QFont("Monospace", 7))
        p.drawText(x+5, y+11, "COMMANDES")
        hints = [("T/L","Déco/Atterro"), ("Z/S","Altitude"),
                 ("↑↓←→","Déplacement"), ("Q/D","Yaw"),
                 ("ESPACE","URGENCE"), ("R","Reset")]
        for i, (k, a) in enumerate(hints):
            yy = y + 22 + i*14
            p.setPen(QPen(self.C_CYAN));  p.setFont(QFont("Monospace",7))
            p.drawText(x+4, yy, k)
            p.setPen(QPen(self.C_GRAY))
            p.drawText(x+38, yy, a)

    # ==================================================================
    # Horizon artificiel — avec Flight Path Marker
    # ==================================================================

    def _draw_horizon(self, p, x, y, w, h):
        s  = self.state
        cx = x + w//2
        cy = y + h//2
        ac = self._att_color()

        p.save()
        p.setClipRect(x, y, w, h)
        p.fillRect(x, y, w, h, self.C_BG)

        # ── Boule horizon (pitch offset + roll) ──────────────────────
        offset_pitch = int(math.degrees(s.pitch) * 3.5)

        p.save()
        p.setClipRect(x+1, y+1, w-2, h-2)
        p.translate(cx, cy + offset_pitch)
        p.rotate(math.degrees(-s.roll))

        p.fillRect(-w, -h*3, w*2, h*3, self.C_SKY)
        p.fillRect(-w, 0,    w*2, h*3, self.C_GROUND)
        p.setPen(QPen(self.C_WHITE, 1.5))
        p.drawLine(-w, 0, w, 0)

        # Échelle de pitch
        p.setFont(QFont("Monospace", 7))
        for deg in range(-30, 31, 5):
            if deg == 0: continue
            py_ = int(-deg * 3.5)
            ll  = 55 if deg % 10 == 0 else 28
            col = QColor("#AAAAAA") if deg % 10 == 0 else QColor("#555555")
            p.setPen(QPen(col, 0.8 if deg % 10 == 0 else 0.5))
            p.drawLine(-ll, py_, ll, py_)
            if deg % 10 == 0:
                p.setPen(QPen(QColor("#888888")))
                p.drawText( ll+4, py_+4, f"{deg}°")
                p.drawText(-ll-22, py_+4, f"{deg}°")
        p.restore()  # roll/translate

        # ── Flight Path Marker (FPM) ──────────────────────────────────
        # Calcul avionique : composantes vitesse dans repère drone
        vx, vy, vz = s.vitesse.x, s.vitesse.y, s.vitesse.z
        cy_yaw, sy_yaw = math.cos(s.yaw), math.sin(s.yaw)
        v_fwd   =  vx * sy_yaw + vy * cy_yaw   # vitesse vers avant drone
        v_right =  vx * cy_yaw - vy * sy_yaw   # vitesse vers droite drone
        v_horiz = math.sqrt(vx**2 + vy**2)

        if v_horiz > 0.4 or abs(vz) > 0.3:
            # Angles : beta = glissement latéral, gamma = pente vol
            beta  = math.atan2(v_right, max(abs(v_fwd), 0.1)) \
                    * (1 if v_fwd >= 0 else -1)
            gamma = math.atan2(vz, max(v_horiz, 0.1))

            # Position écran (même échelle que pitch : 3.5 px/°)
            fpm_x = cx + int(math.degrees(beta) * 3.5)
            # gamma est en repère monde — on soustrait le pitch actuel visuel
            fpm_y = cy - int(math.degrees(gamma) * 3.5) + offset_pitch
            fpm_y = max(y+10, min(y+h-10, fpm_y))
            fpm_x = max(x+10, min(x+w-10, fpm_x))

            # Symbole FPM : cercle + 3 branches (standard avionique)
            R = 7
            p.setPen(QPen(self.C_YELLOW, 1.8))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(fpm_x-R, fpm_y-R, R*2, R*2)
            p.drawLine(fpm_x,    fpm_y-R, fpm_x,    fpm_y-R-8)  # haut
            p.drawLine(fpm_x-R,  fpm_y,   fpm_x-R-8, fpm_y)     # gauche
            p.drawLine(fpm_x+R,  fpm_y,   fpm_x+R+8, fpm_y)     # droite

        # ── Symbole avion fixe (chevrons) ────────────────────────────
        p.setPen(QPen(self.C_ORANGE, 2.5))
        p.drawLine(cx-55, cy, cx-18, cy)
        p.drawLine(cx-18, cy, cx-6,  cy+10)
        p.drawLine(cx+55, cy, cx+18, cy)
        p.drawLine(cx+18, cy, cx+6,  cy+10)
        p.setBrush(self.C_ORANGE)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(cx-4, cy-4, 8, 8)

        # ── Arc indicateur de roll ────────────────────────────────────
        self._draw_roll_arc(p, cx, y+22, 58, s.roll)

        # ── Cadre couleur attitude ────────────────────────────────────
        p.setPen(QPen(ac, 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(x+2, y+2, w-4, h-4)

        # ── Labels overlays ──────────────────────────────────────────
        p.setPen(QPen(self.C_GRAY)); p.setFont(QFont("Monospace", 7))
        p.drawText(x+5, y+12, "HORIZON")

        # Vitesse horizontale (gauche)
        p.setPen(QPen(self.C_YELLOW))
        p.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        p.drawText(x+5, y+2, 80, 20, Qt.AlignmentFlag.AlignLeft,
                   f"{v_horiz:.1f} m/s")

        # Altitude (droite)
        p.setPen(QPen(self.C_CYAN))
        p.drawText(x+w-85, y+2, 80, 20, Qt.AlignmentFlag.AlignRight,
                   f"{s.position.z:.1f} m")

        # FPM légende (si actif)
        if v_horiz > 0.4 or abs(vz) > 0.3:
            p.setPen(QPen(self.C_YELLOW))
            p.setFont(QFont("Monospace", 7))
            p.drawText(x+5, y+h-6, "● FPM actif")

        p.restore()  # clip total

    def _draw_roll_arc(self, p, cx, cy, r, roll):
        """Arc de roll en haut de l'horizon — triangle mobile."""
        p.save()
        p.translate(cx, cy)

        # Arc de fond
        p.setPen(QPen(self.C_GRAY, 0.8))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawArc(QRectF(-r, -r, r*2, r*2), 30*16, 120*16)

        # Marqueurs ±10, ±20, ±30
        for deg in [-30, -20, -10, 0, 10, 20, 30]:
            rad = math.radians(90 - deg)
            x1  = r * math.cos(rad)
            y1  = -r * math.sin(rad)
            ll  = 9 if abs(deg) % 30 == 0 else 5
            x2  = (r-ll) * math.cos(rad)
            y2  = -(r-ll) * math.sin(rad)
            p.setPen(QPen(self.C_GRAY, 0.8))
            p.drawLine(int(x2), int(y2), int(x1), int(y1))

        # Triangle qui suit le roll
        roll_deg = math.degrees(roll)
        rad = math.radians(90 - roll_deg)
        tx  = r * math.cos(rad)
        ty  = -r * math.sin(rad)
        p.translate(int(tx), int(ty))
        p.rotate(-roll_deg)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(self.C_ORANGE)
        p.drawPolygon(QPolygon([QPoint(0,0), QPoint(-5,-8), QPoint(5,-8)]))
        p.restore()

    # ==================================================================
    # Radar — mini-map avec trail, vecteur vitesse, N/S/E/W
    # ==================================================================

    def _draw_radar(self, p, x, y, w, h):
        s     = self.state
        cx    = x + w//2
        cy    = y + h//2
        rayon = min(w, h)//2 - 10
        SCALE = rayon / 25.0   # 25 m = bord du radar

        p.save()
        p.setClipRect(x, y, w, h)
        p.fillRect(x, y, w, h, QColor("#070707"))

        # ── Cercles de distance ───────────────────────────────────────
        for dist_m, lbl in [(8,"8m"), (16,"16m"), (25,"25m")]:
            r = int(dist_m * SCALE)
            p.setPen(QPen(QColor("#1A1A1A"), 0.8))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(cx-r, cy-r, r*2, r*2)
            p.setPen(QPen(QColor("#333333")))
            p.setFont(QFont("Monospace", 6))
            p.drawText(cx+r+2, cy+4, lbl)

        # ── Axes cardinaux (repère monde fixe) ───────────────────────
        p.setPen(QPen(QColor("#202020"), 0.6))
        p.drawLine(cx, cy-rayon, cx, cy+rayon)
        p.drawLine(cx-rayon, cy, cx+rayon, cy)

        p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
        for txt, px_, py_ in [("N", cx-4, y+14),
                                ("S", cx-4, y+h-4),
                                ("W", x+4,  cy+5),
                                ("E", x+w-11, cy+5)]:
            p.setPen(QPen(QColor("#3A3A3A")))
            p.drawText(px_, py_, txt)

        # ── Trail de trajectoire ──────────────────────────────────────
        if len(self._trail) > 1:
            for i in range(1, len(self._trail)):
                t     = i / len(self._trail)
                alpha = int(30 + t * 180)
                p.setPen(QPen(QColor(0, 210, 130, alpha), 0.8 + t))
                ax = cx + int(self._trail[i-1][0] * SCALE)
                ay = cy - int(self._trail[i-1][1] * SCALE)
                bx = cx + int(self._trail[i][0] * SCALE)
                by = cy - int(self._trail[i][1] * SCALE)
                p.drawLine(ax, ay, bx, by)

        # ── Croix origine (home) ──────────────────────────────────────
        p.setPen(QPen(self.C_GREEN, 1.5))
        p.drawLine(cx-5, cy, cx+5, cy)
        p.drawLine(cx, cy-5, cx, cy+5)

        # ── Position drone ────────────────────────────────────────────
        dx = cx + int(_clamp(s.position.x * SCALE, -(rayon-8), rayon-8))
        dy = cy - int(_clamp(s.position.y * SCALE, -(rayon-8), rayon-8))

        # Ligne origine → drone
        if s.distance_origine > 0.8:
            p.setPen(QPen(QColor(0, 210, 255, 50), 0.8,
                          Qt.PenStyle.DotLine))
            p.drawLine(cx, cy, dx, dy)

        # ── Vecteur vitesse sur le radar ──────────────────────────────
        spd_xy = math.sqrt(s.vitesse.x**2 + s.vitesse.y**2)
        if spd_xy > 0.3:
            SCALE_VEL = SCALE * 2.5
            evx = dx + int(s.vitesse.x * SCALE_VEL)
            evy = dy - int(s.vitesse.y * SCALE_VEL)
            p.setPen(QPen(self.C_YELLOW, 1.8))
            p.drawLine(dx, dy, evx, evy)
            # Pointe de flèche
            angle = math.atan2(evx - dx, dy - evy)
            self._arrow_head(p, evx, evy, angle, 6)

        # ── Triangle drone (orienté yaw) ──────────────────────────────
        t = 9
        p.save()
        p.translate(dx, dy)
        p.rotate(math.degrees(s.yaw))
        p.setBrush(self.C_CYAN)
        p.setPen(QPen(QColor("#000000"), 1))
        p.drawPolygon(QPolygon([QPoint(0,-t), QPoint(t//2+1,t//2),
                                QPoint(-t//2-1,t//2)]))
        p.setPen(QPen(self.C_WHITE, 1.5))
        p.drawLine(0, -t, 0, -t-5)
        p.restore()

        # ── Infos overlay ─────────────────────────────────────────────
        p.setPen(QPen(self.C_GRAY)); p.setFont(QFont("Monospace", 7))
        p.drawText(x+4, y+12, "CARTE")
        p.setPen(QPen(self.C_CYAN))
        p.drawText(x+4, y+h-5, f"DST  {s.distance_origine:.1f} m")

        p.setPen(QPen(self.C_BORDER, 0.5))
        p.drawRect(x, y, w-1, h-1)
        p.restore()

    def _arrow_head(self, p, x, y, angle, size):
        a1 = angle + math.pi * 0.82
        a2 = angle - math.pi * 0.82
        tri = QPolygon([
            QPoint(x, y),
            QPoint(x + int(size * math.sin(a1)), y - int(size * math.cos(a1))),
            QPoint(x + int(size * math.sin(a2)), y - int(size * math.cos(a2))),
        ])
        p.setBrush(self.C_YELLOW)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(tri)

    # ==================================================================
    # Jauges droite — ALT + THR (avec zone hover)
    # ==================================================================

    def _draw_gauges(self, p, x, y, w, h):
        s = self.state
        p.fillRect(x, y, w, h, self.C_PANEL)
        p.setPen(QPen(self.C_BORDER, 1))
        p.drawLine(x, y, x, y+h)

        mid   = h // 2
        pad   = 10
        bar_w = w - pad*2

        # ── Jauge ALT ────────────────────────────────────────────────
        ax, ay, aw, ah = x+pad, y+22, bar_w, mid-36
        self._gauge_bar(p, ax, ay, aw, ah,
                        value    = s.position.z,
                        val_max  = 20.0,
                        target   = s.cible_altitude,
                        color    = self.C_CYAN,
                        label    = "ALT",
                        unit     = "m",
                        grads    = [5,10,15,20])

        # ── Jauge THR ────────────────────────────────────────────────
        tx, ty, tw, th_ = x+pad, y+mid+14, bar_w, mid-36
        self._gauge_bar(p, tx, ty, tw, th_,
                        value    = s.cmd_throttle,
                        val_max  = 1.0,
                        target   = THROTTLE_HOVER,   # ligne hover
                        color    = self.C_ORANGE,
                        label    = "THR",
                        unit     = "%",
                        grads    = [0.25, 0.5, 0.75, 1.0],
                        hover_zone = True,
                        fmt_fn   = lambda v: f"{int(v*100)}%")

    def _gauge_bar(self, p, x, y, w, h, value, val_max,
                   target=None, color=None, label="", unit="",
                   grads=None, hover_zone=False, fmt_fn=None):
        """Barre verticale réutilisable avec cible et zone hover."""
        color  = color or self.C_CYAN
        grads  = grads or []
        fmt_fn = fmt_fn or (lambda v: f"{v:.1f}")

        ratio = _clamp(value / val_max, 0.0, 1.0)

        # Fond
        p.setPen(QPen(self.C_GRAY, 0.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(x, y, w, h)

        # Remplissage
        fill_h = int(h * ratio)
        p.fillRect(x+1, y+h-fill_h, w-1, fill_h, color)

        # Zone hover (plage ±4% autour de la cible)
        if hover_zone and target is not None:
            zone = 0.04
            yt   = y + int(h * (1 - _clamp(target/val_max, 0, 1)))
            zh   = int(h * zone * 2)
            p.fillRect(x+1, yt-zh//2, w-1, zh,
                       QColor(0, 255, 136, 28))

        # Ligne cible
        if target is not None:
            yt = y + int(h * (1 - _clamp(target/val_max, 0, 1)))
            p.setPen(QPen(self.C_GREEN, 1.5,
                          Qt.PenStyle.DashLine if not hover_zone
                          else Qt.PenStyle.SolidLine))
            p.drawLine(x-3, yt, x+w+3, yt)
            if hover_zone:
                p.setPen(QPen(self.C_GREEN))
                p.setFont(QFont("Monospace", 5))
                p.drawText(x, yt-2, "HVR")

        # Graduations
        p.setFont(QFont("Monospace", 6))
        for v in grads:
            yy = y + int(h * (1 - v/val_max))
            p.setPen(QPen(self.C_DGRAY, 0.5))
            p.drawLine(x, yy, x+w, yy)
            p.setPen(QPen(self.C_GRAY))
            p.drawText(x, yy-2, str(int(v)) if v == int(v) else f"{v:.2f}")

        # Label titre
        p.setPen(QPen(self.C_GRAY))
        p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
        p.drawText(x, y-10, label)

        # Valeur courante
        p.setPen(QPen(color))
        p.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        p.drawText(x, y+h+14, fmt_fn(value))

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
        vl.addStretch()
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
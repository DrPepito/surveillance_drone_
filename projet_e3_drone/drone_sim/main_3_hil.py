# =============================================================================
# main_combined.py  —  version "feature-complete v3"
#
# Nouveautés v3 :
#   • YAW_SCALE = 0.45 — rotation yaw moins agressive
#   • Vario (VZ) : barre verticale graduée sur le côté droit de l'horizon
#   • Indicateur de glide path (5°) en mode atterrissage
#   • Scanlines légères sur l'horizon (effet HUD militaire)
#   • Radar : cercles 10/20/30 m avec labels, vecteur cap (yaw), zone sécurité rouge
#   • Alertes : batterie critique (<15%) clignotante, alerte VZ fort, badge HOLD
#   • Chrono de vol plus grand + badge mode vol en header
#   • Vitesse maximale atteinte (PEAK) affichée
#   • Estimation du temps de vol restant (basée sur consommation batterie)
#   • Mode jour / nuit toggle (touche J)
#   • DT dynamique mesuré à chaque tick (correction jitter OS)
# =============================================================================

import sys
import math
import time      # ← ajout pour DT dynamique
import cv2
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget,
                              QVBoxLayout, QHBoxLayout)
from PyQt6.QtCore    import QTimer, Qt, QPoint, QRectF
from PyQt6.QtGui     import (QPainter, QColor, QPen, QFont,
                              QPolygon, QKeyEvent, QLinearGradient)

from drone_state         import DroneState
from physics_engine      import update as physics_update
from pid_controller      import FlightPIDs
from keyboard_controller import KeyboardController
from pid_controller      import THROTTLE_HOVER

from orientation_widget  import OrientationWidget
from graph_widget        import GraphsWidget
from test import image_Comparateur

from hil_bridge_serial import HilBridgeSerial

from video_feed import VideoFeed
from PyQt6.QtGui import QImage 

try:
    from hil_bridge import HilBridge, HilConfig
    HIL_DISPONIBLE = True
except ImportError:
    HIL_DISPONIBLE = False
    HilBridge = None
    HilConfig  = None


FPS                = 50
DT                 = 1.0 / FPS   # utilisé uniquement pour timer.setInterval()
ALTITUDE_DECOLLAGE = 1.5

# ── Nouveaux paramètres v3 ─────────────────────────────────────────────────
YAW_SCALE          = 0.45    # Réduction sensibilité yaw (0.45 = 55% plus lent)
SAFETY_DIST        = 25.0    # m — distance max avant zone rouge sur radar
VARIO_RANGE        = 5.0     # m/s — plage max de l'échelle vario
GLIDE_SLOPE_DEG    = 5.0     # degrés — angle de référence glide path
VZ_HARD_LANDING    = 2.5     # m/s — seuil alerte atterrissage dur
HOVER_VXY_THR      = 0.15    # m/s — seuil détection hovering
HOVER_VZ_THR       = 0.08    # m/s — seuil vertical hovering
BAT_CRIT_PCT       = 15.0    # % — seuil batterie critique
BAT_DRAIN_ALPHA    = 0.004   # EMA très lente pour drain batterie


# ---------------------------------------------------------------------------
# Utilitaires
# ---------------------------------------------------------------------------

def _clamp(v, a, b):
    return max(a, min(b, v))

def _lerp(a, b, t):
    return a + (b - a) * t

def _lerp_color(ca: QColor, cb: QColor, t: float) -> QColor:
    t = _clamp(t, 0.0, 1.0)
    return QColor(
        int(ca.red()   + (cb.red()   - ca.red())   * t),
        int(ca.green() + (cb.green() - ca.green()) * t),
        int(ca.blue()  + (cb.blue()  - ca.blue())  * t),
        int(ca.alpha() + (cb.alpha() - ca.alpha()) * t),
    )

def _ema(prev, cur, alpha=0.12):
    return prev + alpha * (cur - prev)

def _ema_angle(prev, cur, alpha=0.18):
    diff = cur - prev
    while diff >  math.pi: diff -= 2*math.pi
    while diff < -math.pi: diff += 2*math.pi
    return prev + alpha * diff


# ---------------------------------------------------------------------------
# Smoothed
# ---------------------------------------------------------------------------

class Smoothed:
    A_FAST  = 0.22
    A_MED   = 0.20
    A_ANG   = 0.18
    A_SLOW  = 0.07
    A_COLOR = 0.08

    def __init__(self):
        self.alt      = 0.0
        self.vz       = 0.0
        self.vx       = 0.0
        self.vy       = 0.0
        self.vxy      = 0.0
        self.dist     = 0.0
        self.roll     = 0.0
        self.pitch    = 0.0
        self.yaw      = 0.0
        self.throttle = 0.0
        self.moteurs  = [0.0, 0.0, 0.0, 0.0]
        self.bat_pct  = 100.0
        self.bat_v    = 12.6
        self.cible_alt = 0.0
        self.fpm_x    = 0.0
        self.fpm_y    = 0.0
        self.fpm_vis  = 0.0
        self.att_t    = 0.0
        self.alt_t    = 0.0

    def update(self, s: DroneState):
        a   = self.A_MED
        af  = self.A_FAST
        aa  = self.A_ANG
        as_ = self.A_SLOW
        ac  = self.A_COLOR

        self.alt      = _ema(self.alt,      s.position.z,    a)
        self.vz       = _ema(self.vz,       s.vitesse.z,     a)
        self.vx       = _ema(self.vx,       s.vitesse.x,     a)
        self.vy       = _ema(self.vy,       s.vitesse.y,     a)
        self.vxy      = _ema(self.vxy,      math.sqrt(s.vitesse.x**2 + s.vitesse.y**2), a)
        self.dist     = _ema(self.dist,     s.distance_origine, as_)
        self.roll     = _ema_angle(self.roll,  s.roll,  aa)
        self.pitch    = _ema_angle(self.pitch, s.pitch, aa)
        self.yaw      = _ema_angle(self.yaw,   s.yaw,   aa)
        self.throttle = _ema(self.throttle, s.cmd_throttle, af)
        self.bat_pct  = _ema(self.bat_pct,  s.batterie_pct,     as_)
        self.bat_v    = _ema(self.bat_v,    s.batterie_tension,  as_)
        self.cible_alt = _ema(self.cible_alt, s.cible_altitude, a)

        for i in range(4):
            self.moteurs[i] = _ema(self.moteurs[i], s.moteurs[i], af)

        ang = max(abs(math.degrees(s.roll)), abs(math.degrees(s.pitch)))
        target_att_t = _clamp((ang - 8.0) / 20.0, 0.0, 1.0)
        self.att_t = _ema(self.att_t, target_att_t, ac)

        target_alt_t = _clamp((s.position.z - 8.0) / 8.0, 0.0, 1.0)
        self.alt_t = _ema(self.alt_t, target_alt_t, ac)

        vx, vy, vz = s.vitesse.x, s.vitesse.y, s.vitesse.z
        cy_y, sy_y = math.cos(s.yaw), math.sin(s.yaw)
        v_fwd   =  vx * sy_y + vy * cy_y
        v_right = -(vx * cy_y - vy * sy_y)
        v_horiz = math.sqrt(vx**2 + vy**2)
        self.fpm_vis = 1.0

        if v_horiz > 0.05 or abs(vz) > 0.05:
            beta  = math.atan2(v_right, v_fwd)
            gamma = math.atan2(vz, max(v_horiz, 0.01))
            self.fpm_x = _ema(self.fpm_x, math.degrees(beta)  * 3.5, 0.03)
            self.fpm_y = _ema(self.fpm_y, math.degrees(gamma) * 3.5, 0.03)
        else:
            self.fpm_x = _ema(self.fpm_x, 0.0, 0.03)
            self.fpm_y = _ema(self.fpm_y, 0.0, 0.03)


# ---------------------------------------------------------------------------
# HUD principal
# ---------------------------------------------------------------------------

class HUDWidget(QWidget):
# ── Palette nuit (défaut) — plus douce, moins agressive ───────────────
    C_BG_NIGHT     = QColor("#2A2D35")
    C_PANEL_NIGHT  = QColor("#1A1D24")
    C_BORDER_NIGHT = QColor("#2C3245")
    C_DGRAY_NIGHT  = QColor("#1E2230")
    C_GRAY_NIGHT   = QColor("#52566A")
    C_WHITE_NIGHT  = QColor("#DCE2EE")
    C_MUTED_NIGHT  = QColor("#7880A0")
    C_SKY_NIGHT    = QColor("#0D2240")
    C_GROUND_NIGHT = QColor("#2A1E08")

    # ── Palette jour — beige chaud cockpit civil ───────────────────────────
    C_BG_DAY     = QColor("#E8E0D0")
    C_PANEL_DAY  = QColor("#D4C8B4")
    C_BORDER_DAY = QColor("#A89880")
    C_DGRAY_DAY  = QColor("#C4B8A4")
    C_GRAY_DAY   = QColor("#887868")
    C_WHITE_DAY  = QColor("#1C1810")
    C_MUTED_DAY  = QColor("#5C5044")
    C_SKY_DAY    = QColor("#4A8EC2")
    C_GROUND_DAY = QColor("#7A6245")

    # ── Couleurs invariantes (mode indépendant) ────────────────────────────
    C_CYAN   = QColor("#2EC4D8")
    C_GREEN  = QColor("#3AB870")
    C_ORANGE = QColor("#D4823C")
    C_RED    = QColor("#D44040")
    C_YELLOW = QColor("#D4B828")
    C_GRAY   = QColor("#52566A")

    C_ATT_OK  = QColor("#48C47A")
    C_ATT_MED = QColor("#A0B830")
    C_ATT_BAD = QColor("#C84040")

    C_HIL_BG   = QColor(10, 12, 20, 215)
    C_HIL_OK   = QColor("#38D878")
    C_HIL_ERR  = QColor("#D44040")
    C_HIL_WARN = QColor("#D08030")
    C_HIL_TXT  = QColor("#8898B0")

    C_DRONE_CURSOR = QColor("#7006FA")

    TRAIL_MAX = 150
    BOT_H     = 80

    def __init__(self, state: DroneState):
        super().__init__()
        self.state         = state
        self.smooth        = Smoothed()
        self._video        = None 
        
        
        self._personne_detectee  = False   # mis à jour depuis boucle_vision
        self._personne_confiance = 0.0
 
 
        
        self._trail        = []
        self._keyboard_ref = None
        self._hil_ref      = None
        self._mode_ing     = False
        self.setMinimumSize(800, 700)

        # ── État v3 ───────────────────────────────────────────────────────
        self._day_mode       = False
        self._blink_tick     = 0
        self._peak_speed     = 0.0
        self._bat_prev_pct   = None
        self._bat_drain_rate = 0.0
        self._bat_time_rem   = None

        # ── Trame UDP — snapshot de la dernière trame envoyée ─────────────
        self._last_telem      : dict  = {}
        self._last_telem_json : str   = ""
        self._last_telem_bytes: int   = 0
        self._last_telem_cnt  : int   = 0

        self._apply_palette_night()

    # ------------------------------------------------------------------
    # Palettes jour/nuit
    # ------------------------------------------------------------------

    def _apply_palette_night(self):
        self.C_BG     = self.C_BG_NIGHT
        self.C_PANEL  = self.C_PANEL_NIGHT
        self.C_BORDER = self.C_BORDER_NIGHT
        self.C_DGRAY  = self.C_DGRAY_NIGHT
        self.C_WHITE  = self.C_WHITE_NIGHT
        self.C_MUTED  = self.C_MUTED_NIGHT
        self.C_SKY    = self.C_SKY_NIGHT
        self.C_GROUND = self.C_GROUND_NIGHT

    def _apply_palette_day(self):
        self.C_BG     = self.C_BG_DAY
        self.C_PANEL  = self.C_PANEL_DAY
        self.C_BORDER = self.C_BORDER_DAY
        self.C_DGRAY  = self.C_DGRAY_DAY
        self.C_WHITE  = self.C_WHITE_DAY
        self.C_MUTED  = self.C_MUTED_DAY
        self.C_SKY    = self.C_SKY_DAY
        self.C_GROUND = self.C_GROUND_DAY

    def toggle_day_mode(self):
        self._day_mode = not self._day_mode
        if self._day_mode:
            self._apply_palette_day()
        else:
            self._apply_palette_night()

    def set_hil(self, bridge):
        self._hil_ref = bridge
        
    def set_video(self, feed: VideoFeed):
        """Connecte le flux vidéo au HUD."""
        self._video = feed
        
    def set_detection(self, trouve: bool, confiance: float):
        """Met à jour l'état de détection personne (thread-safe : atomique)."""
        self._personne_detectee  = trouve
        self._personne_confiance = confiance

    def toggle_mode_ingenieur(self):
        self._mode_ing = not self._mode_ing

    def set_last_telem(self, tel: dict):
        import json as _json
        self._last_telem       = tel
        raw                    = _json.dumps(tel, separators=(',', ':'))
        self._last_telem_json  = raw
        self._last_telem_bytes = len(raw.encode())
        self._last_telem_cnt   = tel.get("_cnt", 0)

    # ------------------------------------------------------------------
    # Helpers v3
    # ------------------------------------------------------------------

    def _blink_on(self, period_ticks=20) -> bool:
        return (self._blink_tick % period_ticks) < (period_ticks // 2)

    def _is_hovering(self) -> bool:
        s  = self.state
        sm = self.smooth
        return (s.mode_vol == DroneState.MODE_VOL
                and sm.vxy < HOVER_VXY_THR
                and abs(sm.vz) < HOVER_VZ_THR
                and s.moteurs_armes)

    def reset_peak(self):
        self._peak_speed = 0.0

    # ------------------------------------------------------------------
    # Tick & lissage
    # ------------------------------------------------------------------

    def tick_smooth(self):
        self.smooth.update(self.state)
        s  = self.state
        sm = self.smooth

        if s.moteurs_armes:
            self._trail.append((s.position.x, s.position.y))
            if len(self._trail) > self.TRAIL_MAX:
                self._trail.pop(0)

        self._blink_tick = (self._blink_tick + 1) % 100

        if sm.vxy > self._peak_speed:
            self._peak_speed = sm.vxy

        if self._bat_prev_pct is not None:
            tick_drain  = self._bat_prev_pct - sm.bat_pct
            drain_per_s = tick_drain * FPS
            self._bat_drain_rate = _ema(
                self._bat_drain_rate, max(0.0, drain_per_s), BAT_DRAIN_ALPHA)
            if self._bat_drain_rate > 1e-4:
                reserve = 10.0
                rem_pct = max(0.0, sm.bat_pct - reserve)
                self._bat_time_rem = rem_pct / self._bat_drain_rate
            else:
                self._bat_time_rem = None
        self._bat_prev_pct = sm.bat_pct

    # ------------------------------------------------------------------
    # Couleurs lissées
    # ------------------------------------------------------------------

    def _att_color(self) -> QColor:
        t = self.smooth.att_t
        if t < 0.5:
            return _lerp_color(self.C_ATT_OK, self.C_ATT_MED, t * 2)
        else:
            return _lerp_color(self.C_ATT_MED, self.C_ATT_BAD, (t - 0.5) * 2)

    def _alt_gauge_color(self) -> QColor:
        return _lerp_color(self.C_CYAN, self.C_ORANGE, self.smooth.alt_t)

    def _bat_color(self) -> QColor:
        pct = self.smooth.bat_pct
        if pct > 45:
            return self.C_GREEN
        elif pct > 20:
            return _lerp_color(self.C_GREEN, self.C_ORANGE,
                               _clamp((45 - pct) / 25.0, 0, 1))
        else:
            return _lerp_color(self.C_ORANGE, self.C_RED,
                               _clamp((20 - pct) / 15.0, 0, 1))

    def _motor_color(self, val: float) -> QColor:
        t = _clamp((val - 0.45) / 0.40, 0.0, 1.0)
        if t < 0.5:
            return _lerp_color(self.C_CYAN, self.C_YELLOW, t * 2)
        else:
            return _lerp_color(self.C_YELLOW, self.C_RED, (t - 0.5) * 2)

    # ------------------------------------------------------------------
    # paintEvent
    # ------------------------------------------------------------------

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        p.fillRect(0, 0, w, h, self.C_BG)

        HDR_H   = 44
        BOT_H   = self.BOT_H
        LEFT_W  = 156
        RIGHT_W = 86
        CX      = LEFT_W
        CW      = w - LEFT_W - RIGHT_W
        BODY_H  = h - HDR_H - BOT_H
        HOR_H   = int(BODY_H * 0.60)
        RAD_H   = BODY_H - HOR_H

        self._draw_header  (p, 0,         0,              w,       HDR_H)
        self._draw_left    (p, 0,         HDR_H,          LEFT_W,  BODY_H)
        self._draw_horizon (p, CX,        HDR_H,          CW,      HOR_H)
        self._draw_radar   (p, CX,        HDR_H + HOR_H,  CW,      RAD_H)
        self._draw_gauges  (p, w-RIGHT_W, HDR_H,          RIGHT_W, BODY_H)
        self._draw_bottom  (p, 0,         h - BOT_H,      w,       BOT_H)
        self._draw_alerts  (p, 0,         0,              w,       h)

        if self._mode_ing:
            self._draw_hil_overlay(p, w, h)

    # ==================================================================
    # Header
    # ==================================================================

    def _draw_header(self, p, x, y, w, h):
        s  = self.state
        sm = self.smooth

        p.fillRect(x, y, w, h, self.C_PANEL)
        p.setPen(QPen(self.C_BORDER, 1))
        p.drawLine(x, y+h-1, x+w, y+h-1)

        MODE_COL = {
            DroneState.MODE_SOL:       QColor("#464A58"),
            DroneState.MODE_DECOLLAGE: QColor("#9E8030"),
            DroneState.MODE_VOL:       QColor("#2E8C5A"),
            DroneState.MODE_ATTERRO:   QColor("#8C5A28"),
            DroneState.MODE_URGENCE:   QColor("#8C3030"),
            DroneState.MODE_HOME:      QColor("#2060A0"),  # bleu navigation

        }
        mc  = MODE_COL.get(s.mode_vol, self.C_GRAY)
        bw  = 104
        p.fillRect(x+4, y+4, bw, h-8, QColor(mc.red(), mc.green(), mc.blue(), 28))
        p.setPen(QPen(mc, 1))
        p.drawRect(x+4, y+4, bw, h-8)
        p.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        p.setPen(QPen(mc))
        p.drawText(x+4, y+4, bw, h-8, Qt.AlignmentFlag.AlignCenter, s.mode_vol)

        if self._is_hovering():
            hx = x + bw + 12
            p.fillRect(hx, y+10, 44, h-20, QColor(64, 200, 128, 40))
            p.setPen(QPen(self.C_GREEN, 1))
            p.drawRect(hx, y+10, 44, h-20)
            p.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
            p.setPen(QPen(self.C_GREEN))
            p.drawText(hx, y+10, 44, h-20, Qt.AlignmentFlag.AlignCenter, "HOLD")

        ac = self._att_color()
        p.setBrush(QColor(ac.red(), ac.green(), ac.blue(), 180))
        p.setPen(QPen(ac, 0.5))
        p.drawEllipse(x+166, y+h//2-5, 10, 10)

        mn  = int(s.temps_vol // 60)
        sc_ = int(s.temps_vol % 60)
        yaw_deg = math.degrees(sm.yaw) % 360
        p.setPen(QPen(self.C_WHITE))
        p.setFont(QFont("Monospace", 14, QFont.Weight.Bold))
        p.drawText(x, y, w, h, Qt.AlignmentFlag.AlignCenter,
                   f"{mn:02d}:{sc_:02d}")
        p.setFont(QFont("Monospace", 8))
        p.setPen(QPen(self.C_MUTED))
        p.drawText(x, y+h-13, w, 12, Qt.AlignmentFlag.AlignCenter,
                   f"CAP  {yaw_deg:05.1f}°")

        hil_txt = ""
        if self._hil_ref and HIL_DISPONIBLE and self._hil_ref.actif:
            st = self._hil_ref.stats
            hil_txt = "  HIL " + ("U✓" if st["udp_ok"] else "U✗") + \
                      (" S✓" if st["serial_ok"] else " S✗")

        bc = self._bat_color()

        p.setPen(QPen(self.C_YELLOW))
        p.setFont(QFont("Monospace", 8))
        p.drawText(x, y+4, w-8, 12,
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignTop,
                   f"PEAK {self._peak_speed:.1f} m/s")

        p.setPen(QPen(bc))
        p.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        p.drawText(x, y+h//2-6, w-8, 14,
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                   f"{sm.bat_pct:.0f}%  {sm.bat_v:.1f}V{hil_txt}")

        if self._bat_time_rem is not None and s.moteurs_armes:
            rem_m = int(self._bat_time_rem // 60)
            rem_s = int(self._bat_time_rem % 60)
            tr_col = self.C_GREEN if self._bat_time_rem > 300 else \
                     (self.C_YELLOW if self._bat_time_rem > 120 else self.C_ORANGE)
            p.setPen(QPen(tr_col))
            p.setFont(QFont("Monospace", 8))
            p.drawText(x, y+h-14, w-8, 12,
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignBottom,
                       f"~{rem_m:02d}:{rem_s:02d} restant")

        day_lbl = "☀ JOUR" if self._day_mode else "☾ NUIT"
        p.setPen(QPen(QColor(180, 180, 140, 120)))
        p.setFont(QFont("Monospace", 7))
        p.drawText(x+4, y+h-14, 60, 12,
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom,
                   day_lbl)

    # ==================================================================
    # Panneau gauche
    # ==================================================================

    def _draw_left(self, p, x, y, w, h):
        sm = self.smooth
        ac = self._att_color()

        p.fillRect(x, y, w, h, self.C_PANEL)
        p.setPen(QPen(self.C_BORDER, 1))
        p.drawLine(x+w-1, y, x+w-1, y+h)

        ag    = self._alt_gauge_color()
        vz_col = _lerp_color(self.C_CYAN, self.C_ORANGE,
                             _clamp(abs(sm.vz) / 3.0, 0, 1))

        metrics = [
            ("ALT",   f"{sm.alt:5.1f}",               "m",    ag),
            ("VIT",   f"{sm.vxy:5.1f}",                "m/s",  self.C_YELLOW),
            ("VZ",    f"{sm.vz:+5.2f}",                "m/s",  vz_col),
            ("DST",   f"{sm.dist:5.1f}",               "m",    self.C_MUTED),
            ("ROLL",  f"{math.degrees(sm.roll):+5.1f}", "°",    ac),
            ("PITCH", f"{math.degrees(sm.pitch):+5.1f}","°",    ac),
        ]

        kb = self._keyboard_ref
        if kb and hasattr(kb, 'consigne_vx'):
            metrics += [
                ("CMD VX", f"{kb.consigne_vx:+5.1f}", "m/s", self.C_ORANGE),
                ("CMD VY", f"{kb.consigne_vy:+5.1f}", "m/s", self.C_ORANGE),
            ]

        p.setPen(QPen(self.C_MUTED))
        p.setFont(QFont("Monospace", 7))
        p.drawText(x+6, y+13, "TÉLÉMÉTRIE")

        n      = len(metrics)
        zone_h = h - 145
        bloc_h = max(30, zone_h // max(n, 1))

        for i, (lbl, val, unit, col) in enumerate(metrics):
            yy = y + 20 + i * bloc_h
            p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7))
            p.drawText(x+6, yy+2, lbl)
            p.setPen(QPen(col)); p.setFont(QFont("Monospace", 12, QFont.Weight.Bold))
            p.drawText(x+2, yy+14, w-10, 18, Qt.AlignmentFlag.AlignRight, val)
            p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7))
            p.drawText(x+2, yy+30, w-6, 10, Qt.AlignmentFlag.AlignRight, unit)
            p.setPen(QPen(self.C_DGRAY, 0.5))
            p.drawLine(x+8, yy+38, x+w-8, yy+38)

        ya = y + 20 + n * bloc_h + 8
        p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7))
        p.drawText(x+5, ya, "ALT CIBLE")
        p.setPen(QPen(self.C_GREEN)); p.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        p.drawText(x+2, ya+13, w-8, 16, Qt.AlignmentFlag.AlignRight,
                   f"{sm.cible_alt:.1f} m")

        self._draw_key_hints(p, x, y+h-130, w, 128)

    def _draw_key_hints(self, p, x, y, w, h):
        p.setPen(QPen(self.C_DGRAY, 0.5)); p.drawLine(x+6, y, x+w-6, y)
        p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7))
        p.drawText(x+5, y+11, "COMMANDES")
        hints = [("T/L","Déco/Atterro"), ("Z/S","Altitude"),
                 ("↑↓←→","Déplacement"), ("Q/D","Yaw"),
                 ("ESPACE","URGENCE"), ("R","Reset"),
                 ("H","Home"), ("I","Overlay HIL"),
                 ("J","Jour/Nuit")]
        for i, (k, a) in enumerate(hints):
            yy = y + 22 + i * 13
            p.setPen(QPen(self.C_CYAN)); p.setFont(QFont("Monospace", 7))
            p.drawText(x+4, yy, k)
            p.setPen(QPen(self.C_MUTED)); p.drawText(x+40, yy, a)

    # ==================================================================
    # Horizon artificiel
    # ==================================================================


    def _draw_horizon(self, p, x, y, w, h):
        sm = self.smooth
        s  = self.state
        cx = x + w // 2
        cy = y + h // 2
        ac = self._att_color()
 
        VARIO_W    = 26
        VARIO_MARG = 4
        horiz_w    = w - VARIO_W - VARIO_MARG
 
        offset_pitch = int(math.degrees(sm.pitch) * 3.5)
 
        p.save()
        p.setClipRect(x, y, w, h)
        p.fillRect(x, y, w, h, self.C_BG)
 
        # ══════════════════════════════════════════════════════════════════
        # COUCHE 1 — Vidéo brute (fond)
        # ══════════════════════════════════════════════════════════════════
        p.save()
        p.setClipRect(x + 2, y + 2, horiz_w - 4, h - 4)
 
        qimg = self._video.derniere_qimage() if self._video else None
 
        if qimg is not None:
            scaled = qimg.scaled(
                horiz_w - 4, h - 4,
                Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                Qt.TransformationMode.SmoothTransformation,
            )
            ox = (scaled.width()  - (horiz_w - 4)) // 2
            oy = (scaled.height() - (h - 4))        // 2
            p.drawImage(x + 2, y + 2, scaled, ox, oy, horiz_w - 4, h - 4)
        else:
            p.fillRect(x + 2, y + 2, horiz_w - 4, h - 4, QColor("#04C5FFDF"))
            p.setPen(QPen(QColor(80, 85, 100, 180)))
            p.setFont(QFont("Monospace", 10))
            p.drawText(x + 2, y + 2, horiz_w - 4, h - 4,
                       Qt.AlignmentFlag.AlignCenter, "⬛  PAS DE SIGNAL")
 
        p.restore()
 
        # ══════════════════════════════════════════════════════════════════
        # COUCHE 2 — Horizon synthétique semi-transparent par-dessus
        # ══════════════════════════════════════════════════════════════════
        p.save()
        p.setClipRect(x + 2, y + 2, horiz_w - 4, h - 4)
        
        p.setOpacity(0.60)   #  opacité réduite : horizon visible sans masquer la vidéo
 
        p.translate(cx - (VARIO_W + VARIO_MARG) // 2, cy + offset_pitch)
        p.rotate(math.degrees(-sm.roll))
 
        # Dégradé ciel semi-transparent
        sky_grad = QLinearGradient(0, -h * 2, 0, 0)
        sky_grad.setColorAt(0.0, QColor(10,  40,  80,  180))
        sky_grad.setColorAt(1.0, QColor(20,  80, 140,  90))
        p.fillRect(-w * 2, -h * 4, w * 4, h * 4, sky_grad)
 
        # Dégradé sol semi-transparent
        gnd_grad = QLinearGradient(0, 0, 0, h * 2)
        gnd_grad.setColorAt(0.0, QColor(60,  40,  10,  100))
        gnd_grad.setColorAt(1.0, QColor(40,  25,   5,  180))
        p.fillRect(-w * 2, 0, w * 4, h * 4, gnd_grad)
 
        # Ligne d'horizon
        p.setOpacity(0.75)
        p.setPen(QPen(QColor("#9C7E1B"), 1.5))
        p.drawLine(-w * 2, 0, w * 2, 0)
 
        # Graduations de tangage
        
        p.setFont(QFont("Monospace", 7))
        for deg in range(-30, 31, 5):
            if deg == 0:
                continue
            py_  = int(-deg * 3.5)
            ll   = 52 if deg % 10 == 0 else 26
            alp  = 220 if deg % 10 == 0 else 130
            p.setOpacity(alp / 255 * 0.85)
            p.setPen(QPen(QColor(0, 0, 0, alp), 0.9))  #COULEUR 
            p.drawLine(-ll, py_, ll, py_)
            if deg % 10 == 0:
                p.setOpacity(1)
                p.setPen(QPen(QColor(0, 0, 0, 200))) #COULEUR
                p.drawText( ll + 4,  py_ + 4, f"{deg:+}°")
                p.drawText(-ll - 24, py_ + 4, f"{deg:+}°")
 
        p.restore()   # fin horizon synthétique
 
        # ══════════════════════════════════════════════════════════════════
        # COUCHE 3 — Scanlines HUD
        # ══════════════════════════════════════════════════════════════════
        sl_alpha = 10 if not self._day_mode else 6
        p.save()
        p.setClipRect(x + 2, y + 2, horiz_w - 4, h - 4)
        p.setOpacity(1.0)
        p.setPen(QPen(QColor(0, 0, 0, sl_alpha), 1))
        for sy in range(y, y + h, 4):
            p.drawLine(x + 2, sy, x + horiz_w - 2, sy)
        p.restore()
 
        # ══════════════════════════════════════════════════════════════════
        # COUCHE 4 — Instruments HUD (opacité pleine)
        # ══════════════════════════════════════════════════════════════════
        p.setOpacity(1.0)
        cx_h = x + horiz_w // 2
 
        # FPM
        fpm_px = cx_h + int(sm.fpm_x)
        fpm_py = cy - int(sm.fpm_y) + offset_pitch
        fpm_py = max(y + 12, min(y + h - 12, fpm_py))
        fpm_px = max(x + 12, min(x + horiz_w - 12, fpm_px))
        R  = 7
        fc = QColor(220, 200, 80, 220)
        p.setPen(QPen(fc, 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(fpm_px - R, fpm_py - R, R * 2, R * 2)
        p.drawLine(fpm_px,     fpm_py - R,     fpm_px,     fpm_py - R - 5)
        p.drawLine(fpm_px - R, fpm_py,         fpm_px - R - 5, fpm_py)
        p.drawLine(fpm_px + R, fpm_py,         fpm_px + R + 5, fpm_py)
 
        # Chevron central
        chevron_c = QColor(255, 225, 80, 230)
        p.setPen(QPen(chevron_c, 2.0))
        p.drawLine(cx_h - 52, cy, cx_h - 16, cy)
        p.drawLine(cx_h - 16, cy, cx_h - 5,  cy + 9)
        p.drawLine(cx_h + 52, cy, cx_h + 16, cy)
        p.drawLine(cx_h + 16, cy, cx_h + 5,  cy + 9)
        p.setBrush(QColor(chevron_c.red(), chevron_c.green(), chevron_c.blue(), 190))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(cx_h - 3, cy - 3, 6, 6)
 
        # Arc de roulis
        self._draw_roll_arc(p, cx_h, y + 22, 58, sm.roll, chevron_c)
 
        # Glide path
        if s.mode_vol in (DroneState.MODE_ATTERRO, DroneState.MODE_DECOLLAGE):
            gp_col  = QColor(100, 255, 130, 210)
            gp_px   = int(GLIDE_SLOPE_DEG * 3.5)
            glide_y = cy + gp_px
            p.setPen(QPen(gp_col, 1.2, Qt.PenStyle.DashLine))
            p.drawLine(x + 30, glide_y, x + horiz_w - 60, glide_y)
            p.setPen(QPen(gp_col, 1.4))
            gd = 5
            p.drawLine(cx_h - gd, glide_y, cx_h,      glide_y - gd)
            p.drawLine(cx_h,      glide_y - gd, cx_h + gd, glide_y)
            p.drawLine(cx_h + gd, glide_y, cx_h,      glide_y + gd)
            p.drawLine(cx_h,      glide_y + gd, cx_h - gd, glide_y)
            p.setPen(QPen(gp_col))
            p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
            p.drawText(cx_h + gd + 4, glide_y + 4, f"GS {GLIDE_SLOPE_DEG:.0f}°")
 
        # Bordure attitude
        border_alpha = int(20 + sm.att_t * 160)
        border_c = QColor(ac.red(), ac.green(), ac.blue(), border_alpha)
        p.setPen(QPen(border_c, 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(x + 2, y + 2, horiz_w - 4, h - 4)
 
        # Heading tape
        self._draw_heading_tape(p, x, y, horiz_w, h, sm.yaw)
 
        # Labels vitesse / altitude
        p.setPen(QPen(self.C_MUTED))
        p.setFont(QFont("Monospace", 7))
        p.drawText(x + 6, y + 12, "HORIZON  ● FPM")
 
        p.setPen(QPen(QColor(255, 220, 80, 230)))
        p.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        p.drawText(x + 6, y + 2, 80, 20, Qt.AlignmentFlag.AlignLeft,
                   f"{sm.vxy:.1f} m/s")
 
        p.setPen(QPen(QColor(self.C_CYAN.red(), self.C_CYAN.green(),
                             self.C_CYAN.blue(), 230)))
        p.drawText(x + horiz_w - 86, y + 2, 80, 20, Qt.AlignmentFlag.AlignRight,
                   f"{sm.alt:.1f} m")
 
        # Badge CAM LIVE
        if self._video and self._video.actif:
            p.setPen(QPen(QColor(58, 210, 140, 190)))
            p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
            p.drawText(x + horiz_w - 80, y + h - 30, 74, 12,
                       Qt.AlignmentFlag.AlignRight, "● CAM LIVE")
 
        # ══════════════════════════════════════════════════════════════════
        # COUCHE 5 — WARNING PERSONNE DÉTECTÉE (clignotant)
        # ══════════════════════════════════════════════════════════════════
        if self._personne_detectee :
            WARN_W = 280
            WARN_H = 32
            wx = cx_h - WARN_W // 2
            wy = y + h - 52   # juste au-dessus du heading tape
 
            # Fond rouge semi-transparent
            p.fillRect(wx, wy, WARN_W, WARN_H,
                       QColor(200, 30, 30, 80))
            # Bordure rouge vive
            p.setPen(QPen(QColor(255, 60, 60, 230), 1.5))
            p.drawRect(wx, wy, WARN_W, WARN_H)
            # Texte
            p.setPen(QPen(QColor(255, 100, 100, 255)))
            p.setFont(QFont("Monospace", 11, QFont.Weight.Bold))
            p.drawText(wx, wy, WARN_W, WARN_H,
                       Qt.AlignmentFlag.AlignCenter,
                       f"⚠  PERSONNE  {self._personne_confiance:.0f}%")
 
        p.restore()   # fin clip global
 
        # Vario
        self._draw_vario(p, x + horiz_w + VARIO_MARG // 2, y, VARIO_W, h, sm.vz)
 
 

    # ------------------------------------------------------------------
    # Vario
    # ------------------------------------------------------------------

    def _draw_vario(self, p, x, y, w, h, vz: float):
        MARG_V = 20
        bar_h  = h - MARG_V * 2
        bar_x  = x + (w - 16) // 2
        bar_w  = 16
        cy     = y + h // 2

        p.fillRect(x, y, w, h, QColor(self.C_BG.red(), self.C_BG.green(),
                                       self.C_BG.blue(), 220))
        p.setPen(QPen(self.C_BORDER, 0.5))
        p.drawRect(bar_x, y+MARG_V, bar_w, bar_h)

        vz_clamped = _clamp(vz, -VARIO_RANGE, VARIO_RANGE)
        fill_ratio  = vz_clamped / VARIO_RANGE
        fill_px     = int(abs(fill_ratio) * bar_h / 2)

        if fill_ratio >= 0:
            bar_top = cy - fill_px
            vz_bar_col = self.C_GREEN
        else:
            bar_top = cy
            vz_bar_col = _lerp_color(self.C_ORANGE, self.C_RED,
                                     _clamp(abs(fill_ratio) - 0.5, 0, 1) * 2)

        if fill_px > 0:
            p.fillRect(bar_x+1, bar_top, bar_w-2, fill_px,
                       QColor(vz_bar_col.red(), vz_bar_col.green(),
                              vz_bar_col.blue(), 200))

        p.setPen(QPen(QColor(self.C_WHITE.red(), self.C_WHITE.green(),
                             self.C_WHITE.blue(), 160), 1.0))
        p.drawLine(bar_x-2, cy, bar_x+bar_w+2, cy)

        p.setFont(QFont("Monospace", 6))
        for grad in range(1, int(VARIO_RANGE)+1):
            gy_up   = cy - int(grad / VARIO_RANGE * bar_h / 2)
            gy_down = cy + int(grad / VARIO_RANGE * bar_h / 2)
            tick_w  = 4 if grad % 2 == 0 else 2
            tick_col = QColor(self.C_MUTED.red(), self.C_MUTED.green(),
                              self.C_MUTED.blue(), 140)
            p.setPen(QPen(tick_col, 0.7))
            p.drawLine(bar_x, gy_up,   bar_x+tick_w, gy_up)
            p.drawLine(bar_x, gy_down, bar_x+tick_w, gy_down)
            if grad % 2 == 0:
                p.setPen(QPen(self.C_MUTED))
                p.drawText(bar_x+bar_w+2, gy_up+3, f"{grad}")
                p.drawText(bar_x+bar_w+2, gy_down+3, f"-{grad}")

        ind_y = _clamp(cy - int(fill_ratio * bar_h / 2), y+MARG_V+1, y+h-MARG_V-1)
        ind_col = vz_bar_col
        p.setBrush(QColor(ind_col.red(), ind_col.green(), ind_col.blue(), 230))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygon([
            QPoint(bar_x-6, ind_y),
            QPoint(bar_x,   ind_y-4),
            QPoint(bar_x,   ind_y+4),
        ]))

        p.setPen(QPen(ind_col))
        p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
        p.drawText(x, y+h-14, w, 12, Qt.AlignmentFlag.AlignCenter,
                   f"{vz:+.1f}")
        p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 6))
        p.drawText(x, y+2, w, 12, Qt.AlignmentFlag.AlignCenter, "VZ")

    # ------------------------------------------------------------------
    # Heading tape & roll arc
    # ------------------------------------------------------------------

    def _draw_heading_tape(self, p, x, y, w, h, yaw):
        TAPE_H  = 28
        TAPE_Y  = y + h - TAPE_H - 2
        cx      = x + w // 2

        p.fillRect(x + 2, TAPE_Y, w - 4, TAPE_H, QColor(10, 12, 18, 180))
        p.setPen(QPen(QColor(60, 70, 90, 160), 0.5))
        p.drawRect(x + 2, TAPE_Y, w - 4, TAPE_H)

        PX_PER_DEG = 4.5
        yaw_deg    = math.degrees(yaw) % 360
        p.setClipRect(x + 2, TAPE_Y, w - 4, TAPE_H)

        CARDINALS = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
        for step_deg in range(-180, 181, 10):
            deg_int = int(yaw_deg / 10) * 10 + step_deg
            deg_int = deg_int % 360
            delta   = step_deg - (yaw_deg % 10)
            px = cx + int(delta * PX_PER_DEG)
            if px < x + 4 or px > x + w - 4:
                continue

            is_20   = (deg_int % 20 == 0)
            is_card = (deg_int % 90 == 0)
            tick_h  = 11 if is_card else (8 if is_20 else 5)
            alpha   = 230 if is_card else (170 if is_20 else 100)

            col = QColor(self.C_CYAN.red(), self.C_CYAN.green(),
                         self.C_CYAN.blue(), alpha)
            p.setPen(QPen(col, 0.8 if not is_card else 1.2))
            p.drawLine(px, TAPE_Y + 1, px, TAPE_Y + 1 + tick_h)

            if is_20 or is_card:
                label = CARDINALS.get(deg_int, f"{deg_int:03d}")
                f_sz  = 8 if is_card else 7
                bold  = QFont.Weight.Bold if is_card else QFont.Weight.Normal
                lc    = self.C_WHITE if is_card else QColor(180, 190, 200, 180)
                p.setPen(QPen(lc))
                p.setFont(QFont("Monospace", f_sz, bold))
                p.drawText(px - 12, TAPE_Y + 12, 24, 12,
                           Qt.AlignmentFlag.AlignCenter, label)

        p.setClipping(False)
        tri_c = QColor(self.C_YELLOW.red(), self.C_YELLOW.green(),
                       self.C_YELLOW.blue(), 200)
        p.setBrush(tri_c); p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygon([
            QPoint(cx - 5, TAPE_Y),
            QPoint(cx + 5, TAPE_Y),
            QPoint(cx,     TAPE_Y + 6),
        ]))
        p.setPen(QPen(tri_c))
        p.setFont(QFont("Monospace", 8, QFont.Weight.Bold))
        p.drawText(cx - 18, TAPE_Y + TAPE_H - 5, 36, 10,
                   Qt.AlignmentFlag.AlignCenter, f"{yaw_deg:05.1f}°")

    def _draw_roll_arc(self, p, cx, cy, r, roll, indicator_color):
        p.save(); p.translate(cx, cy)
        p.setPen(QPen(QColor(90, 105, 120, 140), 0.8))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawArc(QRectF(-r, -r, r*2, r*2), 30*16, 120*16)
        for deg in [-30, -20, -10, 0, 10, 20, 30]:
            rad = math.radians(90 - deg)
            x1 = r * math.cos(rad); y1 = -r * math.sin(rad)
            ll = 8 if abs(deg) % 30 == 0 else 4
            x2 = (r-ll) * math.cos(rad); y2 = -(r-ll) * math.sin(rad)
            p.setPen(QPen(QColor(90, 105, 120, 140), 0.7))
            p.drawLine(int(x2), int(y2), int(x1), int(y1))
        roll_deg = math.degrees(roll)
        rad = math.radians(90 - roll_deg)
        tx = r * math.cos(rad); ty = -r * math.sin(rad)
        p.translate(int(tx), int(ty)); p.rotate(-roll_deg)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(indicator_color)
        p.drawPolygon(QPolygon([QPoint(0, 0), QPoint(-4, -8), QPoint(4, -8)]))
        p.restore()

    # ==================================================================
    # Radar
    # ==================================================================

    def _draw_radar(self, p, x, y, w, h):
        sm = self.smooth
        s  = self.state
        p.fillRect(x, y, w, h, QColor("#08090C"))
        p.setPen(QPen(self.C_BORDER, 1)); p.drawRect(x, y, w, h)

        cx = x + w//2; cy = y + h//2
        rayon = min(w, h)//2 - 14
        echelle = rayon / 30.0

        for dist_m in [10, 20, 30]:
            r_px = int(dist_m * echelle)
            if dist_m * echelle >= SAFETY_DIST * echelle:
                ring_col = QColor(180, 50, 50, 80)
                p.setPen(QPen(ring_col, 1.0, Qt.PenStyle.DashLine))
            else:
                ring_col = QColor(38, 42, 55, 180)
                p.setPen(QPen(ring_col, 0.6))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(cx-r_px, cy-r_px, r_px*2, r_px*2)
            lbl_col = QColor(180, 50, 50, 160) if dist_m >= SAFETY_DIST else \
                      QColor(self.C_MUTED.red(), self.C_MUTED.green(),
                             self.C_MUTED.blue(), 130)
            p.setPen(QPen(lbl_col))
            p.setFont(QFont("Monospace", 6))
            p.drawText(cx + r_px + 3, cy - 4, f"{dist_m}m")

        p.setPen(QPen(QColor(38, 42, 55, 180), 0.5))
        p.drawLine(cx-rayon, cy, cx+rayon, cy)
        p.drawLine(cx, cy-rayon, cx, cy+rayon)

        p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7))
        p.drawText(cx-4, y+h-rayon-2, "N")
        p.drawText(cx-4, y+rayon+13,  "S")
        p.drawText(cx+rayon+3, cy+4,  "E")
        p.drawText(cx-rayon-9, cy+4,  "W")

        dist_actual = s.distance_origine
        if dist_actual > SAFETY_DIST:
            safe_r = int(SAFETY_DIST * echelle)
            p.setPen(QPen(QColor(200, 50, 50, 100), 1.5, Qt.PenStyle.DashLine))
            p.setBrush(QColor(180, 40, 40, 18))
            p.drawEllipse(cx-safe_r, cy-safe_r, safe_r*2, safe_r*2)
            p.setPen(QPen(QColor(200, 60, 60, 180)))
            p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
            p.drawText(cx-30, cy-safe_r-14, 60, 12,
                       Qt.AlignmentFlag.AlignCenter, " !!! HORS ZONE")

        if len(self._trail) > 1:
            for i in range(1, len(self._trail)):
                ax, ay = self._trail[i-1]; bx, by = self._trail[i]
                t = i / len(self._trail)
                alpha = int(12 + t * 130)
                
                epaisseur = 0.5 + t * 2.0
                p.setPen(QPen(QColor(58, 191, 216, alpha), epaisseur))
                p.drawLine(
                    cx + int(_clamp(ax*echelle, -rayon+8, rayon-8)),
                    cy - int(_clamp(ay*echelle, -rayon+8, rayon-8)),
                    cx + int(_clamp(bx*echelle, -rayon+8, rayon-8)),
                    cy - int(_clamp(by*echelle, -rayon+8, rayon-8)),
                )

        dx = int(_clamp(s.position.x * echelle, -rayon+8, rayon-8))
        dy = int(_clamp(-s.position.y * echelle, -rayon+8, rayon-8))

        p.setPen(QPen(QColor(91, 207, 234, 35), 0.5, Qt.PenStyle.DotLine))
        p.drawLine(cx, cy, cx+dx, cy+dy)

        vscale = echelle * 1.0
        vdx = int(_clamp(sm.vx * vscale, -rayon, rayon))
        vdy = int(_clamp(-sm.vy * vscale, -rayon, rayon))
        if abs(vdx) + abs(vdy) > 2:
            p.setPen(QPen(QColor(self.C_YELLOW.red(), self.C_YELLOW.green(),
                                 self.C_YELLOW.blue(), 170), 2.5))
            p.drawLine(cx+dx, cy+dy, cx+dx+vdx, cy+dy+vdy)

        yaw_len = 18
        yaw_vx  = int(yaw_len * math.sin(s.yaw))
        yaw_vy  = int(-yaw_len * math.cos(s.yaw))
        p.setPen(QPen(QColor(160, 120, 220, 190), 1.2))
        p.drawLine(cx+dx, cy+dy, cx+dx+yaw_vx, cy+dy+yaw_vy)
        tip_x = cx + dx + yaw_vx
        tip_y = cy + dy + yaw_vy
        p.setBrush(QColor(160, 120, 220, 180))
        p.setPen(Qt.PenStyle.NoPen)
        p.save()
        p.translate(tip_x, tip_y)
        p.rotate(math.degrees(s.yaw))
        p.drawPolygon(QPolygon([QPoint(0,-4), QPoint(-3,3), QPoint(3,3)]))
        p.restore()

        self._drone_triangle(p, cx+dx, cy+dy, sm.yaw)

        p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 8))
        p.drawText(x+5, y+h-5, f"Dist {sm.dist:.1f} m  ▷ cap  ⟶ vit")

    def _drone_triangle(self, p, x, y, yaw):
        t  = 14
        c  = self.C_DRONE_CURSOR
        p.save()
        p.translate(x, y)
        p.rotate(math.degrees(yaw))

        for r, a in [(t*2, 18), (t+5, 35)]:
            p.setBrush(QColor(c.red(), c.green(), c.blue(), a))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(-r, -r, r*2, r*2)

        tri = QPolygon([QPoint(0, -t), QPoint(t//2+3, t//2+2), QPoint(-(t//2+3), t//2+2)])
        p.setBrush(QColor(c.red(), c.green(), c.blue(), 190))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(tri)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor(195, 160, 240, 200), 1.0))
        p.drawPolygon(tri)
        p.setBrush(QColor(255, 255, 255, 200))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(-2, -2, 4, 4)
        p.restore()

    # ==================================================================
    # Jauges droites
    # ==================================================================

    def _draw_gauges(self, p, x, y, w, h):
        sm = self.smooth
        p.fillRect(x, y, w, h, self.C_PANEL)
        p.setPen(QPen(self.C_BORDER, 1)); p.drawLine(x, y, x, y+h)

        ALT_MAX = 20.0
        JAU_W   = 26
        MARG_T  = 22
        JAU_H   = h - 44

        ax = x + 8
        p.setPen(QPen(QColor(48, 54, 68, 200), 0.5))
        p.drawRect(ax, y+MARG_T, JAU_W, JAU_H)
        rem = int(JAU_H * _clamp(sm.alt / ALT_MAX, 0, 1))
        ac_g = self._alt_gauge_color()
        if rem > 0:
            p.fillRect(ax+1, y+MARG_T+JAU_H-rem, JAU_W-2, rem,
                       QColor(ac_g.red(), ac_g.green(), ac_g.blue(), 210))
        cib_y = y+MARG_T + int(JAU_H * (1.0 - _clamp(sm.cible_alt / ALT_MAX, 0, 1)))
        p.setPen(QPen(QColor(self.C_GREEN.red(), self.C_GREEN.green(),
                             self.C_GREEN.blue(), 170), 1, Qt.PenStyle.DashLine))
        p.drawLine(ax-3, cib_y, ax+JAU_W+3, cib_y)
        p.setPen(QPen(ac_g)); p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
        p.drawText(ax-2, y+MARG_T+JAU_H+12, "ALT")
        p.setPen(QPen(self.C_WHITE)); p.setFont(QFont("Monospace", 7))
        p.drawText(ax-2, y+MARG_T+JAU_H+22, f"{sm.alt:.1f}")

        tx = x + 8 + JAU_W + 10
        if tx + JAU_W < x + w - 2:
            p.setPen(QPen(QColor(48, 54, 68, 200), 0.5))
            p.drawRect(tx, y+MARG_T, JAU_W, JAU_H)
            rem2 = int(JAU_H * sm.throttle)
            thr_t = _clamp((sm.throttle - THROTTLE_HOVER) / 0.25, 0, 1)
            tc = _lerp_color(self.C_CYAN, self.C_ORANGE, abs(thr_t))
            if rem2 > 0:
                p.fillRect(tx+1, y+MARG_T+JAU_H-rem2, JAU_W-2, rem2,
                           QColor(tc.red(), tc.green(), tc.blue(), 210))
            hover_y = y+MARG_T + int(JAU_H * (1.0 - THROTTLE_HOVER))
            p.setPen(QPen(QColor(self.C_GREEN.red(), self.C_GREEN.green(),
                                 self.C_GREEN.blue(), 130), 1, Qt.PenStyle.DashLine))
            p.drawLine(tx-3, hover_y, tx+JAU_W+3, hover_y)
            p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
            p.drawText(tx-2, y+MARG_T+JAU_H+12, "THR")
            p.setPen(QPen(self.C_WHITE)); p.setFont(QFont("Monospace", 7))
            p.drawText(tx-2, y+MARG_T+JAU_H+22, f"{int(sm.throttle*100)}%")

    # ==================================================================
    # Barre inférieure
    # ==================================================================

    def _draw_bottom(self, p, x, y, w, h):
        sm = self.smooth
        s  = self.state

        p.fillRect(x, y, w, h, self.C_PANEL)
        p.setPen(QPen(self.C_BORDER, 1))
        p.drawLine(x, y, x+w, y)

        MARG    = 8
        BAT_W   = 210
        MOT_W   = w - BAT_W - MARG * 3

        mot_x  = x + MARG
        mot_y  = y + MARG
        mot_h  = h - MARG * 2
        slot_w = MOT_W // 4
        bar_h  = mot_h - 32
        bar_w  = max(20, slot_w - 12)

        p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7))
        p.drawText(mot_x, y+6, "MOTEURS")

        for i, m in enumerate(sm.moteurs):
            bx = mot_x + i * slot_w + (slot_w - bar_w) // 2
            by = mot_y + 14

            p.setPen(QPen(QColor(38, 44, 58, 200), 0.5))
            p.drawRect(bx, by, bar_w, bar_h)
            mc = self._motor_color(m)
            filled = int(bar_h * m)
            if filled > 0:
                p.fillRect(bx+1, by+bar_h-filled, bar_w-2, filled,
                           QColor(mc.red(), mc.green(), mc.blue(), 220))
            hover_y_m = by + int(bar_h * 0.50)
            p.setPen(QPen(QColor(100, 110, 128, 100), 0.7, Qt.PenStyle.DotLine))
            p.drawLine(bx, hover_y_m, bx+bar_w, hover_y_m)
            p.setPen(QPen(mc)); p.setFont(QFont("Monospace", 8, QFont.Weight.Bold))
            p.drawText(bx, by+bar_h+12, bar_w, 12,
                       Qt.AlignmentFlag.AlignCenter, f"{int(m*100):3d}%")
            p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 6))
            p.drawText(bx, by+bar_h+22, bar_w, 10,
                       Qt.AlignmentFlag.AlignCenter, f"M{i+1}")

        sep_x = mot_x + MOT_W + MARG
        p.setPen(QPen(self.C_BORDER, 1))
        p.drawLine(sep_x, y+4, sep_x, y+h-4)

        bat_x = sep_x + MARG
        bat_y = y + MARG
        bc    = self._bat_color()
        pct   = sm.bat_pct
        volt  = sm.bat_v

        bat_visible = True
        if pct < BAT_CRIT_PCT:
            bat_visible = self._blink_on(period_ticks=25)

        p.setPen(QPen(self.C_MUTED)); p.setFont(QFont("Monospace", 7))
        p.drawText(bat_x, bat_y+8, "BATTERIE")

        jau_y  = bat_y + 12
        jau_h2 = 18
        jau_w2 = BAT_W - MARG * 2
        p.setPen(QPen(QColor(38, 44, 58, 200), 0.5))
        p.drawRect(bat_x, jau_y, jau_w2, jau_h2)
        fill_w = int(jau_w2 * _clamp(pct / 100.0, 0, 1))
        if fill_w > 2 and bat_visible:
            p.fillRect(bat_x+1, jau_y+1, fill_w-2, jau_h2-2,
                       QColor(bc.red(), bc.green(), bc.blue(), 210))

        if bat_visible:
            p.setPen(QPen(bc))
            p.setFont(QFont("Monospace", 22, QFont.Weight.Bold))
            p.drawText(bat_x, jau_y+jau_h2+3, jau_w2//2, 28,
                       Qt.AlignmentFlag.AlignLeft, f"{pct:.0f}%")
            p.setFont(QFont("Monospace", 14, QFont.Weight.Bold))
            p.setPen(QPen(QColor(bc.red(), bc.green(), bc.blue(), 180)))
            p.drawText(bat_x + jau_w2//2, jau_y+jau_h2+8, jau_w2//2, 22,
                       Qt.AlignmentFlag.AlignRight, f"{volt:.2f}V")

        if pct > 50:
            bat_status, status_col = "✓ OK", self.C_GREEN
        elif pct > 20:
            bat_status, status_col = "⚠ FAIBLE", self.C_ORANGE
        elif pct > BAT_CRIT_PCT:
            bat_status, status_col = "⚠ CRITIQUE", self.C_RED
        else:
            bat_status = "!!! URGENCE BAT" if self._blink_on(20) else ""
            status_col = self.C_RED
        p.setPen(QPen(status_col)); p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
        p.drawText(bat_x, jau_y+jau_h2+32, jau_w2, 10,
                   Qt.AlignmentFlag.AlignRight, bat_status)

        if self._bat_time_rem is not None and s.moteurs_armes:
            rem_m = int(self._bat_time_rem // 60)
            rem_s = int(self._bat_time_rem % 60)
            tr_str = f"~{rem_m:02d}:{rem_s:02d} vol restant"
            tr_col = self.C_GREEN if self._bat_time_rem > 300 else \
                     (self.C_YELLOW if self._bat_time_rem > 120 else self.C_ORANGE)
            p.setPen(QPen(tr_col)); p.setFont(QFont("Monospace", 7))
            p.drawText(bat_x, jau_y+jau_h2+44, jau_w2, 10,
                       Qt.AlignmentFlag.AlignLeft, tr_str)

    # ==================================================================
    # Alertes overlay
    # ==================================================================

    def _draw_alerts(self, p, x, y, w, h):
        sm = self.smooth
        s  = self.state
        alerts = []

        if sm.bat_pct < BAT_CRIT_PCT and self._blink_on(20):
            alerts.append(("⛔ BATTERIE CRITIQUE", self.C_RED))

        if (s.mode_vol in (DroneState.MODE_ATTERRO, DroneState.MODE_VOL)
                and sm.vz < -VZ_HARD_LANDING
                and self._blink_on(15)):
            alerts.append((f"⚠ DESCENTE RAPIDE  {abs(sm.vz):.1f} m/s", self.C_ORANGE))

        if s.distance_origine > SAFETY_DIST and self._blink_on(25):
            alerts.append((f"⚠ HORS ZONE  {s.distance_origine:.0f} m", QColor("#D4823C")))

        if not alerts:
            return

        ALERT_H = 24
        ALERT_W = 320
        ax = w // 2 - ALERT_W // 2

        for idx, (msg, col) in enumerate(alerts):
            ay = y + 52 + idx * (ALERT_H + 4)
            p.fillRect(ax, ay, ALERT_W, ALERT_H,
                       QColor(col.red(), col.green(), col.blue(), 55))
            p.setPen(QPen(col, 1.2))
            p.drawRect(ax, ay, ALERT_W, ALERT_H)
            p.setPen(QPen(col))
            p.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
            p.drawText(ax, ay, ALERT_W, ALERT_H,
                       Qt.AlignmentFlag.AlignCenter, msg)

    # ==================================================================
    # Overlay ingénieur HIL
    # ==================================================================

    def _draw_hil_overlay(self, p, w, h):
        s   = self.state
        OW  = 420; OH = h - 20
        OX  = w - OW - 10; OY = 10

        p.fillRect(OX, OY, OW, OH, self.C_HIL_BG)
        p.setPen(QPen(QColor(self.C_CYAN.red(), self.C_CYAN.green(),
                              self.C_CYAN.blue(), 160), 1))
        p.drawRect(OX, OY, OW, OH)

        p.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
        p.setPen(QPen(self.C_CYAN))
        p.drawText(OX+8, OY+16, "◈  MODE INGÉNIEUR — HIL BRIDGE")
        p.setPen(QPen(QColor("#242830"), 0.5))
        p.drawLine(OX+6, OY+22, OX+OW-6, OY+22)

        y_o = OY + 36; lh = 14
        fn  = QFont("Monospace", 8)
        fb  = QFont("Monospace", 8, QFont.Weight.Bold)

        def sep():
            nonlocal y_o
            y_o += 4
            p.setPen(QPen(QColor("#1E2230"), 0.5))
            p.drawLine(OX+6, y_o, OX+OW-6, y_o)
            y_o += 8

        def titre_section(txt):
            nonlocal y_o
            p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
            p.setPen(QPen(self.C_HIL_TXT))
            p.drawText(OX+8, y_o, txt)
            y_o += lh

        def ligne(label, val, col=None):
            nonlocal y_o
            p.setFont(fn); p.setPen(QPen(self.C_HIL_TXT))
            p.drawText(OX+10, y_o, label)
            p.setFont(fb); p.setPen(QPen(col or self.C_WHITE))
            p.drawText(OX+160, y_o, val)
            y_o += lh

        titre_section("BRIDGE")
        if self._hil_ref and HIL_DISPONIBLE:
            st = self._hil_ref.stats
            ligne("UDP",
                  f"{'CONNECTÉ' if st['udp_ok'] else 'ABSENT'}  "
                  f"envois={st['udp_sent']}  err={st['udp_errors']}",
                  self.C_HIL_OK if st["udp_ok"] else self.C_HIL_ERR)
            ligne("Serial",
                  f"{'CONNECTÉ' if st['serial_ok'] else 'ABSENT'}  "
                  f"envois={st['serial_sent']}  drops={st['serial_drops']}",
                  self.C_HIL_OK if st["serial_ok"] else self.C_HIL_ERR)
            ligne("Latence UDP", f"{st['latence_udp_ms']:.2f} ms",
                  self.C_HIL_OK if st['latence_udp_ms'] < 2.0 else self.C_HIL_WARN)
            ligne("Trames totales", str(st["frames"]))
        else:
            ligne("HIL Bridge",
                  "non initialisé" if not HIL_DISPONIBLE else "inactif",
                  self.C_HIL_ERR)

        sep()
        titre_section("PAYLOAD — CHAMPS DÉCODÉS")

        tel = self._last_telem if self._last_telem else {}

        if not tel:
            if hasattr(s, 'exporter_telemetrie'):
                tel = s.exporter_telemetrie()
            else:
                tel = {
                    'pos': [s.position.x, s.position.y, s.position.z],
                    'vel': [s.vitesse.x, s.vitesse.y, s.vitesse.z],
                    'att_deg': [math.degrees(s.roll), math.degrees(s.pitch), math.degrees(s.yaw)],
                    'cmd': {'throttle': s.cmd_throttle, 'roll': s.cmd_roll,
                            'pitch': s.cmd_pitch, 'yaw': s.cmd_yaw},
                    'moteurs': s.moteurs, 'mode': s.mode_vol, 'arme': s.moteurs_armes,
                    'bat_pct': s.batterie_pct, 'bat_v': s.batterie_tension,
                    't': s.temps_vol, '_cnt': 0,
                }

        sections = [
            ("Position",  [f"x={tel['pos'][0]:+.3f} m",
                           f"y={tel['pos'][1]:+.3f} m",
                           f"z={tel['pos'][2]:+.3f} m"]),
            ("Vitesse",   [f"vx={tel['vel'][0]:+.3f} m/s",
                           f"vy={tel['vel'][1]:+.3f} m/s",
                           f"vz={tel['vel'][2]:+.3f} m/s"]),
            ("Attitude",  [f"roll ={tel.get('att_deg',[0,0,0])[0]:+.2f}°",
                           f"pitch={tel.get('att_deg',[0,0,0])[1]:+.2f}°",
                           f"yaw  ={tel.get('att_deg',[0,0,0])[2]:+.2f}°"]),
            ("Commandes", [f"thr={tel['cmd']['throttle']:.3f}",
                           f"roll={tel['cmd']['roll']:+.3f}",
                           f"pitch={tel['cmd']['pitch']:+.3f}",
                           f"yaw={tel['cmd']['yaw']:+.3f}"]),
            ("Moteurs",   [f"M1={tel['moteurs'][0]:.3f}", f"M2={tel['moteurs'][1]:.3f}",
                           f"M3={tel['moteurs'][2]:.3f}", f"M4={tel['moteurs'][3]:.3f}"]),
            ("Système",   [f"mode={tel['mode']}  armé={tel['arme']}",
                           f"bat={tel.get('bat_pct',0):.1f}%  {tel.get('bat_v',0):.2f}V",
                           f"t={tel['t']:.2f}s"]),
        ]

        col_w = (OW - 20) // 2; col = 0; y_col = [y_o, y_o]
        for titre, vals in sections:
            yy = y_col[col]; ox = OX + 10 + col * col_w
            p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
            p.setPen(QPen(self.C_CYAN)); p.drawText(ox, yy, titre); yy += 12
            p.setFont(QFont("Monospace", 7))
            for v in vals:
                p.setPen(QPen(self.C_WHITE)); p.drawText(ox+4, yy, v); yy += 11                      
            yy += 4; y_col[col] = yy; col = 1 - col
        y_o = max(y_col[0], y_col[1])

        sep()

        hil_cfg = self._hil_ref.cfg if self._hil_ref else None
        host_str = (f"{hil_cfg.udp_host}:{hil_cfg.udp_port}"
                    if hil_cfg else "127.0.0.1:5005")

        p.setFont(QFont("Monospace", 7, QFont.Weight.Bold))
        p.setPen(QPen(self.C_CYAN))
        frame_lbl = (f"TRAME UDP BRUTE  →  {host_str}  "
                     f"#{self._last_telem_cnt:05d}  "
                     f"[{self._last_telem_bytes} B]")
        p.drawText(OX+8, y_o, frame_lbl); y_o += 13

        JSON_ZONE_H = OH - (y_o - OY) - 18
        if JSON_ZONE_H > 20:
            p.fillRect(OX+6, y_o, OW-12, JSON_ZONE_H, QColor(4, 6, 10, 220))
            p.setPen(QPen(QColor(30, 36, 48, 200), 0.5))
            p.drawRect(OX+6, y_o, OW-12, JSON_ZONE_H)

            raw = self._last_telem_json if self._last_telem_json else \
                  '{"status":"en attente de la première trame…"}'
            CHARS_PER_LINE = 52
            lines = []
            while len(raw) > CHARS_PER_LINE:
                cut = raw.rfind(',', 0, CHARS_PER_LINE)
                if cut < 10: cut = CHARS_PER_LINE
                else: cut += 1
                lines.append(raw[:cut])
                raw = raw[cut:]
            if raw:
                lines.append(raw)

            p.setFont(QFont("Monospace", 6))
            max_lines = (JSON_ZONE_H - 6) // 10
            y_txt = y_o + 9
            for i, ln in enumerate(lines[:max_lines]):
                p.setPen(QPen(QColor(60, 180, 200, 200)))
                p.drawText(OX+10, y_txt, ln)
                y_txt += 10

            if len(lines) > max_lines:
                p.setPen(QPen(QColor("#363840")))
                p.drawText(OX+10, y_txt, f"… +{len(lines)-max_lines} lignes")

        p.setFont(QFont("Monospace", 7)); p.setPen(QPen(QColor("#363840")))
        p.drawText(OX+8, OY+OH-8, "[ I ] fermer cet overlay")


# ---------------------------------------------------------------------------
# Fenêtre principale
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Flight Simulator — AZERTY")
        self.setMinimumSize(1200, 820)
        self.setStyleSheet("background-color: #0C0D10;")

        self.state    = DroneState()
        self.pids     = FlightPIDs()
        self.keyboard = KeyboardController()
        self._last_tick_time = time.monotonic()   # ← DT dynamique init

        self.hud         = HUDWidget(self.state)
        
        #PARTIE VIDEO
        self.video_feed = VideoFeed(source=0)   # 0 = webcam par défaut
        self.video_feed.demarrer()
        self.hud.set_video(self.video_feed)
        ####
        self.hud._keyboard_ref = self.keyboard
        self.orientation = OrientationWidget(self.state)
        self.graphs      = GraphsWidget(self.state)

        self.hil = None

        self.hil_serial = HilBridgeSerial(port="COM7")
        self.hil_serial.demarrer()

        if HIL_DISPONIBLE:
            cfg = HilConfig(
                udp_actif    = True,
                udp_host     = "127.0.0.1",
                udp_port     = 5005,
                serial_actif = False,
                serial_port  = "/dev/ttyUSB0",
                serial_baud  = 115200,
            )
            self.hil = HilBridge(cfg)
            self.hil.demarrer()
            self.hud.set_hil(self.hil)

        conteneur = QWidget()
        layout    = QHBoxLayout(conteneur)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.hud, stretch=1)

        col_droite = QWidget()
        col_droite.setStyleSheet("background-color: #08090D;")
        col_droite.setFixedWidth(240)
        vl = QVBoxLayout(col_droite)
        vl.setContentsMargins(4, 4, 4, 4)
        vl.addWidget(self.orientation)
        vl.addWidget(self.graphs, stretch=1)
        layout.addWidget(col_droite)

        self.setCentralWidget(conteneur)

        self.timer = QTimer()
        self.timer.setInterval(int(1000 / FPS))
        self.timer.timeout.connect(self._tick)
        self.timer.start()

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def closeEvent(self, event):
        if self.hil:
            self.hil.arreter()
            
            
        if self.hil_serial:
            
            self.hil_serial.arreter()
            self.video_feed.arreter() 
            
        super().closeEvent(event)

    # ------------------------------------------------------------------
    # Boucle principale
    # ------------------------------------------------------------------

    def _tick(self):
        # ── DT dynamique : temps réel écoulé depuis le dernier tick ──
        now = time.monotonic()
        dt  = now - self._last_tick_time
        dt  = max(0.005, min(dt, 0.05))   # clamp 5 ms … 50 ms
        self._last_tick_time = now

        s = self.state
        self.keyboard.update(s,dt)

        if self.keyboard.consommer_urgence():      
            self._urgence()
        if self.keyboard.consommer_reset():
            s.reset()
            self.pids.reset_all()
            self.keyboard.set_throttle_hover()
            self.hud.reset_peak()
        if self.keyboard.consommer_decollage():    
            self._decollage()
        if self.keyboard.consommer_atterrissage(): 
            self._atterrissage()
        if self.keyboard.consommer_home():        
            self._retour_home()

        self._update_mode()
        self._appliquer_pids(dt)      # ← dt réel
        physics_update(s, dt)         # ← dt réel

        if self.hil and HIL_DISPONIBLE and hasattr(s, 'exporter_telemetrie'):
            tel = s.exporter_telemetrie()
            self.hud.set_last_telem(tel)
            self.hil.envoyer(tel)
        if self.hil_serial:
            self.hil_serial.envoyer(s)

        self.hud.tick_smooth()
        self.hud.update()
        self.graphs.push()
        self.graphs.update()
        self.orientation.update()

    # ------------------------------------------------------------------
    # Modes de vol
    # ------------------------------------------------------------------

    def _decollage(self):
        if self.state.mode_vol == DroneState.MODE_SOL:
            self.state.moteurs_armes  = True
            self.state.mode_vol       = DroneState.MODE_DECOLLAGE
            self.state.cible_altitude = ALTITUDE_DECOLLAGE
            self.keyboard.set_throttle_hover()
            self.pids.reset_all()

    def _atterrissage(self):
        if self.state.mode_vol in (DroneState.MODE_VOL,
                                DroneState.MODE_DECOLLAGE,
                                DroneState.MODE_HOME):
            self.state.mode_vol       = DroneState.MODE_ATTERRO
            self.state.cible_altitude = 0.0
    def _retour_home(self):
        if self.state.mode_vol == DroneState.MODE_VOL:
            self.state.mode_vol = DroneState.MODE_HOME


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
                s.mode_vol       = DroneState.MODE_VOL
                s.cible_altitude = s.position.z

        elif s.mode_vol == DroneState.MODE_ATTERRO:
            if s.position.z <= 0.05 and abs(s.vitesse.z) < 0.1:
                s.mode_vol      = DroneState.MODE_SOL
                s.moteurs_armes = False
                s.moteurs       = [0.0, 0.0, 0.0, 0.0]
                s.cmd_throttle  = 0.0

        elif s.mode_vol == DroneState.MODE_HOME:
            # Sécurité : si le drone touche le sol pendant HOME → MODE_SOL
            if s.position.z <= 0.05 and abs(s.vitesse.z) < 0.1:
                s.mode_vol      = DroneState.MODE_SOL
                s.moteurs_armes = False
                s.moteurs       = [0.0, 0.0, 0.0, 0.0]
                s.cmd_throttle  = 0.0
                
            
                
    def _appliquer_pids(self, dt: float):
        s = self.state
        if s.mode_vol in (DroneState.MODE_SOL, DroneState.MODE_URGENCE):
            return

        correction_alt = self.pids.altitude.calculer(s.cible_altitude, s.position.z, dt)
        s.cmd_throttle = _clamp(THROTTLE_HOVER + correction_alt, 0.0, 1.0)

        # PIDs XY actifs en VOL, DECOLLAGE et HOME
        if s.mode_vol not in (DroneState.MODE_VOL, DroneState.MODE_DECOLLAGE,
                            DroneState.MODE_HOME):
            return

        cy_v = math.cos(s.yaw); sy_v = math.sin(s.yaw)
        cvx  = self.keyboard.consigne_vx
        cvy  = self.keyboard.consigne_vy

        consigne_vx_monde =  cvx * cy_v + cvy * sy_v
        consigne_vy_monde = -cvx * sy_v + cvy * cy_v

        angle_pitch_cible = self.pids.vel_y.calculer(consigne_vy_monde, s.vitesse.y, dt)
        angle_roll_cible  = self.pids.vel_x.calculer(consigne_vx_monde, s.vitesse.x, dt)

        from physics_engine import PITCH_MAX, ROLL_MAX
        s.cmd_pitch = _clamp(angle_pitch_cible / PITCH_MAX, -1.0, 1.0)
        s.cmd_roll  = _clamp(angle_roll_cible  / ROLL_MAX,  -1.0, 1.0)

        from keyboard_controller import YAW_RATE_MAX
        s.cmd_yaw = _clamp(
            self.keyboard.consigne_yaw_rate / YAW_RATE_MAX * YAW_SCALE,
            -1.0, 1.0)
        # ------------------------------------------------------------------
        # Clavier
        # ------------------------------------------------------------------

    def keyPressEvent(self, event: QKeyEvent):
        self.keyboard.key_press(event.key())
        key = event.key()
        if key == Qt.Key.Key_I:
            self.hud.toggle_mode_ingenieur()
            self.hud.update()
        elif key == Qt.Key.Key_J:
            self.hud.toggle_day_mode()
            if self.hud._day_mode:
                self.setStyleSheet("background-color: #C8C4BC;")
            else:
                self.setStyleSheet("background-color: #0C0D10;")
            self.hud.update()

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.keyboard.key_release(event.key())


## Modifications à appliquer dans main_combined.py ###

# ─────────────────────────────────────────────────────────────────────────
# 1) Remplacer le bloc final (lecture webcam locale) par ceci :
# ─────────────────────────────────────────────────────────────────────────

# AVANT (webcam locale, ne reflète pas la vue du drone) :
#
#   video = cv2.VideoCapture(0)
#   while True:
#       ret, img1 = video.read()
#       comparateur = image_Comparateur(img1)
#       teste = comparateur.detect_personne()
#       comparateur.affiche()

# APRÈS (flux caméra du drone, reçu via UART multiplexé) :

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()

    # Boucle de traitement IA sur le flux caméra du drone.
    # w.hil_serial est l'instance HilBridgeSerial déjà créée dans MainWindow,
    # qui reçoit en parallèle (thread dédié) les images relayées par ESP32 #1.
   # def boucle_vision():
     #   while True:
    #        img = w.hil_serial.derniere_image()
  #          if img is not None:
   #             comparateur = image_Comparateur(img)
 #               comparateur.detect_personne()
#                comparateur.affiche()
            # Pas de sleep agressif : derniere_image() est non-bloquant,
            # mais on évite de tourner à vide en boucle serrée si pas d'image.
#            else:
#                cv2.waitKey(10)


#TEST AVEC CAMERA ORDI 


    def boucle_vision():
        from test import image_Comparateur
        while True:
            frame = w.video_feed.derniere_frame()
            if frame is not None:
                comparateur = image_Comparateur(frame)
                trouve, conf = comparateur.detect_personne()
                # Transmet le résultat au HUD (pas d'affichage cv2 séparé)
                w.hud.set_detection(trouve, conf)
            else:
                cv2.waitKey(10)



    import threading
    thread_vision = threading.Thread(target=boucle_vision, daemon=True)
    thread_vision.start()

    sys.exit(app.exec())


# ─────────────────────────────────────────────────────────────────────────
# 2) Pourquoi un thread séparé pour la vision ?
# ─────────────────────────────────────────────────────────────────────────
#
# L'ancienne boucle "while True: video.read()" était BLOQUANTE et tournait
# dans le thread principal, ce qui empêchait Qt (app.exec()) de jamais être
# atteint — la fenêtre ne s'affichait jamais réellement avec ce design.
#
# En la déplaçant dans un thread dédié, Qt peut gérer sa propre boucle
# d'événements (rendu HUD à 50 Hz) en parallèle de la détection IA,
# sans que l'un bloque l'autre.
#
# Le HilBridgeSerial lui-même a déjà son propre thread de lecture UART
# (voir hil_bridge_serial.py), donc derniere_image() est juste une lecture
# rapide protégée par lock — aucun risque de bloquer la boucle vision
# en attendant des données série.


# ─────────────────────────────────────────────────────────────────────────
# 3) Rien à changer dans MainWindow.__init__ : 
#    self.hil_serial = HilBridgeSerial(port="COM7") existe déjà
#    et gère maintenant AUSSI la réception (image + RC) automatiquement.
# ─────────────────────────────────────────────────────────────────────────
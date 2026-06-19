# =============================================================================
# graphs_widget.py
# Oscilloscope engineering temps réel — 4 courbes superposées
# 100% QPainter, zéro matplotlib, zéro bibliothèque externe
#
# Courbes affichées :
#   ① Altitude     — position Z (cyan) + consigne (vert tiret)
#   ② Vitesse Z    — vz (orange), ligne zéro en blanc
#   ③ Attitude     — roll (rouge) + pitch (cyan)
#   ④ Moteurs      — M1/M2/M3/M4 + throttle moyen
#
# Architecture :
#   - Chaque canal = ring-buffer Python list de N valeurs
#   - push() appelé depuis le tick 50 Hz (main.py)
#   - paintEvent dessine les N derniers points en coordonnées normalisées
#   - Grid, axes, labels, unités — style Bode / MATLAB dark
# =============================================================================

import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QPainter, QColor, QPen, QFont, QPolygonF
from PyQt6.QtCore    import QPointF


# ---------------------------------------------------------------------------
# Palette
# ---------------------------------------------------------------------------

C_BG       = QColor("#070707")
C_GRID     = QColor("#1C1C1C")
C_ZERO     = QColor("#333333")
C_BORDER   = QColor("#2A2A2A")
C_LABEL    = QColor("#555555")
C_WHITE    = QColor("#CCCCCC")

# Courbes
C_CYAN     = QColor("#00D9FF")
C_GREEN    = QColor("#00FF88")
C_ORANGE   = QColor("#FF8800")
C_RED      = QColor("#FF3B3B")
C_PURPLE   = QColor("#CC88FF")
C_YELLOW   = QColor("#FFD700")
C_PINK     = QColor("#FF69B4")


# ---------------------------------------------------------------------------
# Définition des 4 panneaux — chacun contient 1 à 4 courbes
# ---------------------------------------------------------------------------

PANNEAUX = [
    {
        "titre"  : "ALTITUDE",
        "unite"  : "m",
        "y_min"  : -0.5,
        "y_max"  : 20.0,
        "zero"   : 0.0,
        "courbes": [
            {"cle": "alt",      "couleur": C_CYAN,   "epaisseur": 1.8, "label": "Z reel"},
            {"cle": "alt_cib",  "couleur": C_GREEN,  "epaisseur": 1.0, "label": "cible",
             "style": Qt.PenStyle.DashLine},
        ],
        "grad_y": [0, 5, 10, 15, 20],
    },
    {
        "titre"  : "VITESSE Z",
        "unite"  : "m/s",
        "y_min"  : -4.5,
        "y_max"  :  4.5,
        "zero"   :  0.0,
        "courbes": [
            {"cle": "vz",  "couleur": C_ORANGE,  "epaisseur": 1.8, "label": "vz"},
        ],
        "grad_y": [-4, -2, 0, 2, 4],
    },
    {
        "titre"  : "ATTITUDE",
        "unite"  : "deg",
        "y_min"  : -40.0,
        "y_max"  :  40.0,
        "zero"   :   0.0,
        "courbes": [
            {"cle": "roll",  "couleur": C_RED,   "epaisseur": 1.5, "label": "roll"},
            {"cle": "pitch", "couleur": C_CYAN,  "epaisseur": 1.5, "label": "pitch"},
        ],
        "grad_y": [-35, 0, 35],
    },
    {
        "titre"  : "MOTEURS",
        "unite"  : "%",
        "y_min"  : -0.02,
        "y_max"  :  1.05,
        "zero"   :  0.0,
        "courbes": [
            {"cle": "m1",  "couleur": C_CYAN,   "epaisseur": 1.0, "label": "M1"},
            {"cle": "m2",  "couleur": C_RED,    "epaisseur": 1.0, "label": "M2"},
            {"cle": "m3",  "couleur": C_GREEN,  "epaisseur": 1.0, "label": "M3"},
            {"cle": "m4",  "couleur": C_YELLOW, "epaisseur": 1.0, "label": "M4"},
            {"cle": "thr", "couleur": C_WHITE,  "epaisseur": 1.6, "label": "THR",
             "style": Qt.PenStyle.DashLine},
        ],
        "grad_y": [0, 0.25, 0.5, 0.75, 1.0],
    },
]

N_POINTS = 300   # ~6 secondes à 50 Hz


# ===========================================================================
# GraphsWidget
# ===========================================================================

class GraphsWidget(QWidget):
    """
    Widget oscilloscope multi-canaux.

    Utilisation dans main.py :
        self.graphs = GraphsWidget(self.state)
        # dans _tick() :
        self.graphs.push()
        self.graphs.update()
    """

    PADDING_G  = 38   # marge gauche (graduation Y)
    PADDING_D  = 8    # marge droite
    PADDING_H  = 18   # marge haut (titre)
    PADDING_B  = 6    # marge bas
    INTER      = 6    # espace inter-panneaux

    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        self.setMinimumSize(240, 380)
        self.setMaximumWidth(300)

        # Ring-buffers : un dict de listes
        self._bufs = {k: [0.0] * N_POINTS for k in [
            "alt", "alt_cib", "vz",
            "roll", "pitch",
            "m1", "m2", "m3", "m4", "thr",
        ]}

        self._tick_count = 0
        self._font_titre = QFont("Monospace", 7, QFont.Weight.Bold)
        self._font_grad  = QFont("Monospace", 7)
        self._font_leg   = QFont("Monospace", 6)

    # ------------------------------------------------------------------
    # API publique
    # ------------------------------------------------------------------

    def push(self):
        """Appelé depuis le tick 50 Hz — enregistre l'état courant."""
        s = self.state
        b = self._bufs

        def _push(lst, val):
            lst.pop(0)
            lst.append(float(val))

        _push(b["alt"],     s.position.z)
        _push(b["alt_cib"], s.cible_altitude)
        _push(b["vz"],      s.vitesse.z)
        _push(b["roll"],    math.degrees(s.roll))
        _push(b["pitch"],   math.degrees(s.pitch))
        _push(b["m1"],      s.moteurs[0])
        _push(b["m2"],      s.moteurs[1])
        _push(b["m3"],      s.moteurs[2])
        _push(b["m4"],      s.moteurs[3])
        _push(b["thr"],     s.cmd_throttle)

        self._tick_count += 1

    # ------------------------------------------------------------------
    # Rendu
    # ------------------------------------------------------------------

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # Fond général
        p.fillRect(0, 0, w, h, C_BG)

        n = len(PANNEAUX)
        hauteur_totale = h - self.INTER * (n - 1)
        hauteur_pan    = hauteur_totale // n

        for i, pan in enumerate(PANNEAUX):
            y_pan = i * (hauteur_pan + self.INTER)
            self._dessiner_panneau(p, pan, 0, y_pan, w, hauteur_pan)

    # ------------------------------------------------------------------
    # Dessin d'un panneau
    # ------------------------------------------------------------------

    def _dessiner_panneau(self, p, pan, px, py, pw, ph):
        """Dessine un panneau oscilloscope complet."""

        # Zone de tracé (intérieure, sans marges)
        gx = px + self.PADDING_G
        gy = py + self.PADDING_H
        gw = pw - self.PADDING_G - self.PADDING_D
        gh = ph - self.PADDING_H - self.PADDING_B

        if gw < 10 or gh < 10:
            return

        y_min = pan["y_min"]
        y_max = pan["y_max"]
        span  = y_max - y_min if (y_max - y_min) != 0 else 1.0

        # ── Fond zone trace ──────────────────────────────────────────
        p.fillRect(gx, gy, gw, gh, QColor("#0B0B0B"))

        # ── Grille horizontale ───────────────────────────────────────
        p.setPen(QPen(C_GRID, 0.5))
        n_lignes = 5
        for i in range(n_lignes + 1):
            yy = gy + int(i * gh / n_lignes)
            p.drawLine(gx, yy, gx + gw, yy)

        # ── Ligne zéro ──────────────────────────────────────────────
        zero = pan.get("zero", 0.0)
        if y_min <= zero <= y_max:
            yz = gy + int(gh * (1.0 - (zero - y_min) / span))
            p.setPen(QPen(C_ZERO, 0.8, Qt.PenStyle.DashLine))
            p.drawLine(gx, yz, gx + gw, yz)

        # ── Grille verticale (4 traits) ──────────────────────────────
        p.setPen(QPen(C_GRID, 0.5))
        for i in range(1, 4):
            xx = gx + int(i * gw / 4)
            p.drawLine(xx, gy, xx, gy + gh)

        # ── Graduations Y ────────────────────────────────────────────
        p.setFont(self._font_grad)
        p.setPen(QPen(C_LABEL))
        for val in pan.get("grad_y", []):
            if y_min <= val <= y_max:
                yy = gy + int(gh * (1.0 - (val - y_min) / span))
                # Trait graduation
                p.setPen(QPen(C_LABEL, 0.5))
                p.drawLine(gx - 3, yy, gx, yy)
                # Texte
                txt = _format_val(val)
                p.setPen(QPen(C_LABEL))
                p.drawText(px, yy + 4, self.PADDING_G - 5, 10,
                           Qt.AlignmentFlag.AlignRight, txt)

        # ── Courbes ──────────────────────────────────────────────────
        for courbe in pan["courbes"]:
            cle     = courbe["cle"]
            couleur = courbe["couleur"]
            ep      = courbe.get("epaisseur", 1.5)
            style   = courbe.get("style", Qt.PenStyle.SolidLine)

            buf = self._bufs.get(cle)
            if buf is None:
                continue

            pen = QPen(couleur, ep, style)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            p.setPen(pen)

            poly = QPolygonF()
            for j, val in enumerate(buf):
                xp = gx + j * gw / (N_POINTS - 1)
                # Clamp visuel sans modifier buffer
                val_c = max(y_min, min(y_max, val))
                yp = gy + gh * (1.0 - (val_c - y_min) / span)
                poly.append(QPointF(xp, yp))

            if len(poly) > 1:
                p.drawPolyline(poly)

        # ── Bordure panneau ──────────────────────────────────────────
        p.setPen(QPen(C_BORDER, 0.7))
        p.drawRect(gx, gy, gw, gh)

        # ── Titre + unité ────────────────────────────────────────────
        p.setFont(self._font_titre)
        p.setPen(QPen(C_WHITE))
        p.drawText(gx + 3, py + 12, pan["titre"])
        p.setPen(QPen(C_LABEL))
        p.setFont(self._font_grad)
        p.drawText(gx + 3 + 60, py + 12, f"[{pan['unite']}]")

        # ── Légende courbes ──────────────────────────────────────────
        p.setFont(self._font_leg)
        leg_x = gx + gw - 4
        leg_y = py + 12
        for courbe in reversed(pan["courbes"]):
            lbl = courbe.get("label", "")
            if not lbl:
                continue
            p.setPen(QPen(courbe["couleur"]))
            fm_w = len(lbl) * 5 + 2
            leg_x -= fm_w
            p.drawText(leg_x, leg_y, lbl)
            leg_x -= 4

        # ── Valeur live (dernier échantillon) ─────────────────────────
        # Afficher la valeur actuelle de la première courbe en haut à droite
        first_cle = pan["courbes"][0]["cle"]
        buf_first = self._bufs.get(first_cle)
        if buf_first:
            last_val = buf_first[-1]
            p.setFont(self._font_grad)
            p.setPen(QPen(pan["courbes"][0]["couleur"]))
            txt = _format_val(last_val, decimales=2)
            p.drawText(gx + gw - 52, py + 12, txt + " " + pan["unite"])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _format_val(val, decimales=1):
    """Formate un float pour l'affichage en graduation."""
    if val == int(val):
        return str(int(val))
    fmt = f"{{:+.{decimales}f}}" if val < 0 else f"{{:.{decimales}f}}"
    return fmt.format(val)
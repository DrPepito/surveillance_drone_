# =============================================================================
# orientation_widget.py
# Flèche 3D temps réel — indique l'orientation du drone (roll, pitch, yaw)
# 100% QPainter, zéro OpenGL, zéro bibliothèque externe
#
# Principe : on dessine une flèche 3D en projetant ses points 3D
# sur le plan 2D de l'écran avec une projection isométrique simple.
# La flèche pointe dans la direction "avant" du drone.
# Deux axes secondaires (droite = rouge, haut = vert) complètent le repère.
# =============================================================================

import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore    import Qt
from PyQt6.QtGui     import QPainter, QColor, QPen, QFont, QPolygon
from PyQt6.QtCore    import QPoint


# ---------------------------------------------------------------------------
# Projection isométrique manuelle
# Angle de vue fixe : az=30°, el=20° — donne une bonne lisibilité
# ---------------------------------------------------------------------------

AZ_VUE = math.radians(30)    # azimut caméra
EL_VUE = math.radians(20)    # élévation caméra

def _projeter(x3, y3, z3, echelle=60):
    """
    Projette un point 3D (x=droite, y=avant, z=haut) en 2D écran.
    Rotation autour de Z (azimut) puis inclinaison (élévation).
    Retourne (px, py) en pixels, centré sur (0,0) — à translater ensuite.
    """
    # Rotation azimut autour de l'axe Z
    xa = x3 * math.cos(AZ_VUE) - y3 * math.sin(AZ_VUE)
    ya = x3 * math.sin(AZ_VUE) + y3 * math.cos(AZ_VUE)
    za = z3

    # Projection élévation (inclinaison de la caméra)
    px = xa * echelle
    py = -(za * math.cos(EL_VUE) - ya * math.sin(EL_VUE)) * echelle

    return int(px), int(py)


def _rotation_3d(vecteur, roll, pitch, yaw):
    """
    Applique roll, pitch, yaw (en radians) à un vecteur 3D (x, y, z).
    Ordre : yaw → pitch → roll (convention aéronautique).
    Tout codé à la main avec sin/cos.
    """
    x, y, z = vecteur

    # Yaw — rotation autour de Z
    x1 = x * math.cos(yaw) - y * math.sin(yaw)
    y1 = x * math.sin(yaw) + y * math.cos(yaw)
    z1 = z

    # Pitch — rotation autour de X
    x2 = x1
    y2 = y1 * math.cos(pitch) - z1 * math.sin(pitch)
    z2 = y1 * math.sin(pitch) + z1 * math.cos(pitch)

    # Roll — rotation autour de Y
    x3 = x2 * math.cos(roll) + z2 * math.sin(roll)
    y3 = y2
    z3 = -x2 * math.sin(roll) + z2 * math.cos(roll)

    return x3, y3, z3


class OrientationWidget(QWidget):
    """
    Widget compact affichant une flèche 3D représentant l'orientation du drone.

    Repère couleurs :
      Bleu cyan  → axe avant  (direction de vol)
      Rouge      → axe droite
      Vert       → axe haut   (poussée)

    La flèche tourne en temps réel selon roll/pitch/yaw du DroneState.
    """

    C_FOND   = QColor("#0A0A0A")
    C_AVANT  = QColor("#00D9FF")   # cyan — axe avant
    C_DROITE = QColor("#FF3B3B")   # rouge — axe droite
    C_HAUT   = QColor("#00FF88")   # vert  — axe haut
    C_GRILLE = QColor("#1A1A1A")
    C_TEXTE  = QColor("#888888")
    C_BLANC  = QColor("#CCCCCC")

    def __init__(self, state, parent=None):
        super().__init__(parent)
        self.state = state
        self.setMinimumSize(200, 220)
        self.setMaximumSize(240, 260)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx   = w // 2
        cy   = h // 2 - 10   # léger décalage vers le haut pour les labels

        # Fond
        p.fillRect(0, 0, w, h, self.C_FOND)

        # Cercle de fond (sphère de référence)
        p.setPen(QPen(self.C_GRILLE, 0.5))
        rayon_sphere = min(w, h) // 2 - 20
        p.drawEllipse(cx - rayon_sphere, cy - rayon_sphere,
                      rayon_sphere * 2, rayon_sphere * 2)

        # Croix de référence (axes monde fixes)
        p.setPen(QPen(self.C_GRILLE, 0.5))
        p.drawLine(cx - rayon_sphere, cy, cx + rayon_sphere, cy)
        p.drawLine(cx, cy - rayon_sphere, cx, cy + rayon_sphere)

        # Récupération attitude actuelle
        roll  = self.state.roll
        pitch = self.state.pitch
        yaw   = self.state.yaw

        echelle = rayon_sphere * 0.85

        # ── Axe droite (X) — rouge ──────────────────────────────────
        self._dessiner_axe(p, cx, cy,
                           vecteur_base=(1, 0, 0),
                           roll=roll, pitch=pitch, yaw=yaw,
                           echelle=echelle,
                           couleur=self.C_DROITE,
                           label="X")

        # ── Axe haut (Z) — vert ─────────────────────────────────────
        self._dessiner_axe(p, cx, cy,
                           vecteur_base=(0, 0, 1),
                           roll=roll, pitch=pitch, yaw=yaw,
                           echelle=echelle,
                           couleur=self.C_HAUT,
                           label="Z")

        # ── Axe avant (Y) — cyan, dessiné en dernier (au-dessus) ────
        self._dessiner_fleche_avant(p, cx, cy,
                                    roll=roll, pitch=pitch, yaw=yaw,
                                    echelle=echelle)

        # Labels valeurs numériques en bas
        self._dessiner_labels(p, w, h, roll, pitch, yaw)

        # Titre
        p.setPen(QPen(self.C_TEXTE))
        p.setFont(QFont("Monospace", 8))
        p.drawText(4, 12, "ORIENTATION")

    # ------------------------------------------------------------------
    # Dessin d'un axe simple (ligne + petit carré à l'extrémité)
    # ------------------------------------------------------------------

    def _dessiner_axe(self, p, cx, cy, vecteur_base,
                      roll, pitch, yaw, echelle, couleur, label):

        # Point de départ : origine (0,0,0)
        ox, oy = _projeter(0, 0, 0, echelle)

        # Extrémité de l'axe après rotation
        vr = _rotation_3d(vecteur_base, roll, pitch, yaw)
        ex, ey = _projeter(*vr, echelle)

        # Demi-transparence : axe négatif en pointillé
        vm = _rotation_3d((-vecteur_base[0], -vecteur_base[1], -vecteur_base[2]),
                          roll, pitch, yaw)
        mx, my = _projeter(*vm, echelle)

        couleur_faible = QColor(couleur)
        couleur_faible.setAlpha(60)
        p.setPen(QPen(couleur_faible, 1, Qt.PenStyle.DotLine))
        p.drawLine(cx + ox, cy + oy, cx + mx, cy + my)

        # Axe positif
        p.setPen(QPen(couleur, 1.5))
        p.drawLine(cx + ox, cy + oy, cx + ex, cy + ey)

        # Petit carré à l'extrémité
        p.setBrush(couleur)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(cx + ex - 3, cy + ey - 3, 6, 6)

        # Label
        p.setPen(QPen(couleur))
        p.setFont(QFont("Monospace", 8, QFont.Weight.Bold))
        p.drawText(cx + ex + 4, cy + ey + 4, label)

    # ------------------------------------------------------------------
    # Flèche avant — la principale, avec une vraie tête de flèche 3D
    # ------------------------------------------------------------------

    def _dessiner_fleche_avant(self, p, cx, cy, roll, pitch, yaw, echelle):
        """
        La flèche avant est dessinée comme un corps + une tête triangulaire.
        On calcule 5 points 3D et on les projette.
        """

        # Corps de la flèche : de -0.3 à 0.75 sur l'axe avant
        base_3d  = (0.0,  -0.25, 0.0)
        tip_3d   = (0.0,   0.75, 0.0)   # pointe

        # Ailerons de la tête (triangle dans le plan XY)
        ail_g_3d = (-0.18, 0.45, 0.0)
        ail_d_3d = ( 0.18, 0.45, 0.0)

        # Application rotation
        br = _rotation_3d(base_3d,  roll, pitch, yaw)
        tr = _rotation_3d(tip_3d,   roll, pitch, yaw)
        gr = _rotation_3d(ail_g_3d, roll, pitch, yaw)
        dr = _rotation_3d(ail_d_3d, roll, pitch, yaw)

        # Projection 2D
        bx, by = _projeter(*br, echelle)
        tx, ty = _projeter(*tr, echelle)
        gx, gy = _projeter(*gr, echelle)
        dx, dy = _projeter(*dr, echelle)

        # Corps
        p.setPen(QPen(self.C_AVANT, 2.5))
        p.drawLine(cx + bx, cy + by, cx + tx, cy + ty)

        # Tête triangulaire remplie
        p.setPen(QPen(self.C_AVANT, 1))
        p.setBrush(self.C_AVANT)
        triangle = QPolygon([
            QPoint(cx + tx, cy + ty),
            QPoint(cx + gx, cy + gy),
            QPoint(cx + dx, cy + dy),
        ])
        p.drawPolygon(triangle)

        # Label "Y (avant)"
        p.setPen(QPen(self.C_AVANT))
        p.setFont(QFont("Monospace", 8, QFont.Weight.Bold))
        p.drawText(cx + tx + 5, cy + ty - 2, "Y")

    # ------------------------------------------------------------------
    # Labels numériques en bas du widget
    # ------------------------------------------------------------------

    def _dessiner_labels(self, p, w, h, roll, pitch, yaw):
        p.setFont(QFont("Monospace", 8))
        y_base = h - 44

        infos = [
            (self.C_DROITE, f"R {math.degrees(roll):+6.1f}°"),
            (self.C_AVANT,  f"P {math.degrees(pitch):+6.1f}°"),
            (self.C_HAUT,   f"Y {math.degrees(yaw):+6.1f}°"),
        ]

        for i, (couleur, texte) in enumerate(infos):
            p.setPen(QPen(couleur))
            p.drawText(6, y_base + i * 14, texte)
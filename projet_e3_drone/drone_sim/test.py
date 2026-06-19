import cv2
import numpy as np
from ultralytics import YOLO

# Chargement du modèle UNE SEULE FOIS au niveau module
# (évite de recharger yolov8n.pt à chaque frame → gain énorme de perf)
_model = YOLO("yolov8n.pt")


class image_Comparateur:
    def __init__(self, img):
        self.img2  = img
        self.gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)

    def detect_personne(self):
        """
        Détecte les personnes dans l'image.
        Retourne (True, confiance_max_%) si au moins une personne est trouvée,
        (False, 0.0) sinon.
        """
        confiance_max = 0.0
        personne_trouvee = False

        results = _model(self.img2)
        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0:          # classe 0 = personne
                    conf = box.conf[0].item()
                    if conf > confiance_max:
                        confiance_max = conf
                    personne_trouvee = True

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(self.img2, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"Personne {conf*100:.1f}%"
                    cv2.putText(self.img2, label, (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return personne_trouvee, confiance_max * 100

    def affiche(self):
        cv2.imshow("Camera drone", self.img2)
        cv2.waitKey(1)


# --- Test standalone avec webcam locale ---
if __name__ == "__main__":
    video = cv2.VideoCapture(0)
    if not video.isOpened():
        print("Erreur : impossible d'ouvrir la webcam.")
    else:
        while True:
            ret, img = video.read()
            if not ret:
                print("Erreur lecture webcam.")
                break

            comparateur = image_Comparateur(img)
            trouve, conf = comparateur.detect_personne()

            if trouve:
                print(f"Personne détectée — confiance max : {conf:.1f}%")

            comparateur.affiche()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    video.release()
    cv2.destroyAllWindows()
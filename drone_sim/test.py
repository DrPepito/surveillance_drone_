import cv2
import numpy as np
import matplotlib.pyplot as plt
from   ultralytics import YOLO


class image_Comparateur:
    def __init__(self, img2_path):

        self.img2 = img2_path
        self.gray2 = cv2.cvtColor(self.img2, cv2.COLOR_BGR2GRAY)


    def detect_personne(self):

    # Charger la base de donné de personne pré-entraîné
        model = YOLO("yolov8n.pt")

    # Détecter les visages
      
        results = model(self.img2)
        for result in results:
            for box in result.boxes:
                if box.cls == 0:  # Classe 0 = personne dans YOLO
                    confiance = box.conf[0].item()
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(self.img2, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return True , confiance*100

    def affiche(self) :
        cv2.imshow(" image", self.img2)
        cv2.waitKey(1)
        


# --- Exemple d'utilisation ---
if __name__ == "__main__":
    video=cv2.VideoCapture(0)
    while True :
        ret, img1= video.read()
        comparateur = image_Comparateur( img1)

        teste = comparateur.detect_personne()
    
        comparateur.affiche()
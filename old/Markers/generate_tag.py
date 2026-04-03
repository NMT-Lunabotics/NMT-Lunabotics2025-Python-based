#!/usr/bin/env python3
import cv2
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import letter
from reportlab.lib.utils import ImageReader
import tempfile

def generate_aruco_pdf(tag_id=70, marker_size=500, border_size=50, output_file=f"tag.pdf"):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    tag_image = cv2.aruco.drawMarker(aruco_dict, tag_id, marker_size)

    tag_image = cv2.copyMakeBorder(
        tag_image,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255
    )

    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
        temp_path = tmp.name
        cv2.imwrite(temp_path, tag_image)

    c = canvas.Canvas(output_file, pagesize=letter)
    width, height = letter

    img = ImageReader(temp_path)
    img_width = width * 0.8
    img_height = img_width

    x = (width - img_width) / 2
    y = (height - img_height) / 2

    c.drawImage(img, x, y, img_width, img_height)
    c.save()

if __name__ == "__main__":
    generate_aruco_pdf(tag_id=77, output_file=f"tag77.pdf", marker_size=500, border_size=50)
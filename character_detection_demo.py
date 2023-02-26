# importing the libraries
import cv2
import pytesseract

# setting the path of pytesseract exe
# you have to write the location of
# on which your tesseract was installed
pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

# Now we will read the image in our program
# you have to put your image path in place of photo.jpg
img = cv2.imread('IMG_7602.jpg')

# Our image will read as BGR format,
# So we will convert in RGB format because
# tesseract can only read in RGB format
ancho = img.shape[1] #columnas
alto = img.shape[0] # filas
# Rotación
M = cv2.getRotationMatrix2D((ancho//2,alto//2),83,1)
img = cv2.warpAffine(img,M,(ancho,alto))
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,2))
# img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel,iterations=1)
# img = 255 - img
# For getting the text and number from image
ocr_config = r'--psm 10 _char_whitelist=H'
# ocr_config = r'-c tessedit _char_whitelist=H --psm 10 -l osd'
print(pytesseract.image_to_string(img, config=ocr_config))
print("H detected") if pytesseract.image_to_string(img, config=ocr_config) else print("no H detected")

# For displaying the original image
cv2.imshow("result", img)
cv2.waitKey(0)

# importing the libraries
import cv2
import pytesseract

def rotate_image(image, angle):
    ancho = img.shape[1] #columnas
    alto = img.shape[0] # filas
    M = cv2.getRotationMatrix2D((ancho//2,alto//2),angle,1)
    return cv2.warpAffine(img,M,(ancho,alto))

def char_finder(img, char):
    ocr_config = r'--psm 10 -c tessedit_char_whitelist=' + char + ' -c min_orientation_margin=180'
    print(ocr_config)
    res = pytesseract.image_to_string(img, config=ocr_config)
    if res:
        print(res)
        return True
    else:
        return False

# setting the path of pytesseract exe
# you have to write the location of
# on which your tesseract was installed
pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

# Now we will read the image in our program
# you have to put your image path in place of photo.jpg
#img = cv2.imread('IMG_7602.jpg')
img = cv2.imread('Captura de pantalla_2023-02-26_15-51-29.png')

# Our image will read as BGR format,
# So we will convert in RGB format because
# tesseract can only read in RGB format

# Rotación
rotation_angle = 15

vid = cv2.VideoCapture(0)

while(True):

    # Capture the video frame
    # by frame
    ret, frame = vid.read()

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for i in range(int(90/rotation_angle)):
        # img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,2))
        # img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel,iterations=1)
        # img = 255 - img
        # For getting the text and number from image
        ocr_config = r'-c tessedit_char_whitelist=H --psm 10'
        if char_finder(frame, "H"):
            print("H detected")
            # For displaying the original image
        else:
            img = rotate_image(img, rotation_angle)
            print("no H detected")

    # Display the resulting frame
    cv2.imshow('frame', frame)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
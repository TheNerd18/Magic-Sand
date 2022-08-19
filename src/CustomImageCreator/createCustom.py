from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QFileDialog
import sys
import os

#had to change some restrictions sudo mv /etc/ImageMagick-6/policy.xml /etc/ImageMagick-6/policy.xmlout removed restrictions
# call sudo mv /etc/ImageMagick-6/policy.xmlout /etc/ImageMagick-6/policy.xml to put policy backconvert image.pdf image_%0d.tiff
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.convert_file()

    def convert_file(self):
        file_filter = 'PowerPoint File (*.ppt *pptx)'
        
        response = QFileDialog.getOpenFileName(
            parent = self,
            caption = "Select ppt or pptx file",
            directory = os.getcwd(),
            filter=file_filter,
            initialFilter= file_filter
        )

        print(response[0])
        ppt_pdf = 'soffice --headless --convert-to pdf '+response[0]
        os.system(ppt_pdf)
        
        pdf_tif = 'convert -density 100 '+response[0]+' bin/data/layerImages/custom/100_%02d.tif'
        os.system(pdf_tif)

if __name__ =='__main__':
    app = QApplication([])
    window = MainWindow()

    
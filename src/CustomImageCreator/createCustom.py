from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QFileDialog
import sys
import os

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

        pdf_tif = 'convert -density 100 '+response[0]+' bin/data/layerImages/custom/100_%02d.tif'
        os.system(pdf_tif)

if __name__ =='__main__':
    app = QApplication([])
    window = MainWindow()

    

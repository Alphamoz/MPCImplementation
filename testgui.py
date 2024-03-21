import sys

from PyQt5.QtCore import *
from PyQt5.QtGui  import *
from PyQt5.QtWidgets import *
 
class MainGui(object):
    def setupUi(self, MainWindow):
        MainWindow.resize(218, 379)
        self.centralwidget = QWidget(MainWindow)
        self.layout = QVBoxLayout(self.centralwidget)
        self.p_text = QPlainTextEdit(self.centralwidget)
        self.layout.addWidget(self.p_text)
        MainWindow.setCentralWidget(self.centralwidget)

        self.p_text.keyPressEvent = self.keyPressEvent

    def keyPressEvent(self, e):
        print("event", e)
        if e.key()  == Qt.Key_Return :
            print(' return')
        elif e.key() == Qt.Key_Enter :   
            print(' enter')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = MainGui()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
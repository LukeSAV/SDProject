import sys
from PySide.QtGui import *
import GenQtGui
from MainWindow import *
from BluetoothManager import *


class MainGui(QMainWindow, Ui_MainWindow):
    def __init__(self, bluetoothManager, parent=None):
        super(MainGui, self).__init__(parent)
        self.setupUi(self)
        self.bluetoothManager = bluetoothManager



        self.motorSlider.sliderReleased.connect(self.driveSliderReleased)
        self.motorSlider.valueChanged.connect(lambda value: self.driveLabel.setText(str(value)))
        self.motorSlider.mouseDoubleClickEvent =self.stopButtonPress


        self.stopButton.clicked.connect(self.stopButtonPress)
        self.pSlider.valueChanged.connect( lambda x:self.pEdit.setText(
            str(x/100.0 * (float(self.pMaxEdit.text())- float(self.pMinText.text())) + float(self.pMinText.text()))))

        self.iSlider.valueChanged.connect( lambda x:self.iEdit.setText(
            str(x/100.0 * (float(self.iMaxEdit.text())- float(self.iMinText.text())) + float(self.iMinText.text()))))

        self.dSlider.valueChanged.connect( lambda x:self.dEdit.setText(
            str(x/100.0 * (float(self.dMaxEdit.text())- float(self.dMinText.text())) + float(self.dMinText.text()))))

        self.pEdit.returnPressed = lambda: self.pSlider.setSliderPosition(int(self.pEdit.text()))
        self.iEdit.returnPressed = lambda: self.iSlider.setSliderPosition(int(self.iEdit.text()))
        self.dEdit.returnPressed = lambda: self.dSlider.setSliderPosition(int(self.dEdit.text()))

        # Broadcast new value to vehicle
        self.pSlider.sliderReleased.connect(self.pSliderReleased)
        self.iSlider.sliderReleased.connect(self.iSliderReleased)
        self.dSlider.sliderReleased.connect(self.dSliderReleased)


    def printH(self):
        print("hello")
    def stopButtonPress(self, _=""):
        self.motorSlider.setSliderPosition(0)
        self.driveLabel.setText("0")

    def pSliderReleased(self):
        self.bluetoothManager.send("P:"+self.pEdit.text() + ";")

    def iSliderReleased(self):
        pass
    def dSliderReleased(self):
        pass
    def driveSliderReleased(self):
        #TODO send drive command
        pass


if __name__ == "__main__":
    manager = BluetoothManager("20:16:01:20:58:33")
    currentApp = QApplication(sys.argv)
    currentForm = MainGui(manager)
    currentForm.show()
    currentApp.exec_()
    manager.close()
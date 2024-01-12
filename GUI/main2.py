# This is a sample Python script.
# @section author_main Author(s)
# - Created by mohamed hatem /sara said / mohamed maged /emad hamdy /salma swillam /kirollos nashaat/abdelrahman gamal/hasan said / on 15/11/2023.
import sys
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication,QLCDNumber,QPushButton,QLabel
from PyQt5.QtGui import QFont,QFontDatabase
from threading import *
import serial
import RPi.GPIO as GPIO
ser = serial.Serial('/dev/ttyAMA0', 9600) 
################################################################################################
# Import the generated UI
################################################################################################
from interface_ui import Ui_MainWindow as interface_ui
class MainWindow(QMainWindow,interface_ui):
    def __init__(self, parent=None):
        QMainWindow.__init__(self)
        ################################################################################################
        # Setup the UI main window
        ################################################################################################
        #self. = Ui_MainWindow()
        self.setupUi(self)

        ################################################################################################
        # Show window
        ################################################################################################
        self.show()
        #set background-color
        self.label_3.setStyleSheet("\n"
                                   "background-color: rgb(26, 15, 36);")
        #hide all alarms 
        self.rightwarning.hide()
        self.left_warning.hide()
        self.right_blind_spot.hide()
        self.left_blind_spot.hide()
        self.right_lane.hide()
        self.left_lane.hide()
        self.rainicon1.hide()
        self.rain_icon2.hide()
        self.ACC.hide()
        self.widget.setEnableBarGraph(False)   #?
        self.widget_2.setEnableBarGraph(False)
        # set Gauge units
        self.widget.units = "m/min" 
        self.widget_2.units = "CM"
        # set  gaugae scale
        self.widget.scalacount = 5  # scale
         # set minimum gaugae value
        self.widget.minValue = 0 # 
        # set max gaugae value
        self.widget.maxValue = 200#distance max value
        self.widget_2.maxValue = 300# speed max value 

        self.widget.setEnableScaleText(True) # enable for text on graph
        self.widget.setMouseTracking(False)
        self.widget_2.setMouseTracking(False)
        
        self.widget_2.setGaugeTheme(24) #from  1 to 24
        self.widget_2.setScalePolygonColor(
            color3="#555",
            color2="#080001",
            color1="#FF0022"

        )
        
        self.widget.setGaugeTheme(24)  # from  1 to 24
        self.widget.setScalePolygonColor(
            color3="#555",
            color2="#080001",
            color1="#FF0022"

        )
        self.widget_2.setNeedleColor(255,0,0)  
        self.widget.setNeedleColor(255, 0, 0)  

         #display current speed value
        def change(value):
            self.spped_lcd.display(value)
         #display current speed distance 
        def change2(value):
            self.distance_lcd.display(value)
        self.widget.valueChanged.connect(change)
        self.widget_2.valueChanged.connect(change2)
        self.widget.setEnableCenterPoint(True)
         #display current mode
        def line_input(text):
            self.lineEdit.clear()
            self.lineEdit.setText(text)

        line_input("Normal mode")
        #self.label_10.raise_()


    def threadRead(self):
            t12 = Thread(target=self.SerUpdate)
            t12.start()
    def SerUpdate(self):
            while (1):
                counter1=0
                counter2=0
                #gui array 
                m=ser.read(14)
                print(m)
                m=list(m)
                i=0
                while i<14:
                        m[i]=m[i]-48
                        i=i+1
                # Extrat mode from array 
                mode=m[0:2]
                mode=((int(mode[0])*10)+int(mode[1]))
                # Extrat distance from array 
                distance=m[2:5]
                distance=((int(distance[0])*100)+(int(distance[1]*10))+((int(distance[2]))))
                self.widget_2.updateValue(distance)
                 # Extrat Speed from array 
                speed=m[5:8]
                speed=((int(speed[0])*100)+int(speed[1]*10)+int(speed[2]))
                self.widget.updateValue(speed)
                # Extract Rain from array
                rain=m[8:9]
                rain=int(rain[0])
                #  Extract lane from array
                lane=m[9:11]
                lane=((int(lane[0])*10)+int(lane[1]))
                #  Extract lane from array
                spot=m[11:13]
                spot=((int(spot[0])*10)+int(spot[1]))
                match mode:
                    case 0:
                        self.lineEdit.clear()
                        self.ACC.setVisible(False)
                        self.lineEdit.setText("normal mode")
                    case 1:
                        self.lineEdit.clear()
                        self.ACC.setVisible(True)
                        self.lineEdit.setText("ACC mode")
                    case 10:
                        self.lineEdit.clear()
                        self.ACC.setVisible(False)
                        self.lineEdit.setText("self drive")  
                match rain:
                    case 0:
                        self.rainicon1.setVisible(False)
                        self.rain_icon2.setVisible(False)
                    case 1:
                        counter1=counter1+1
                        counter2=counter2+1
                        self.rainicon1.setVisible(True)
                        self.rain_icon2.setVisible(True)
                match lane:
                    case 0:
                        self.left_lane.setVisible(False)
                        self.right_lane.setVisible(False)
                    case 1:
                        counter2=counter2+1;
                        self.left_lane.setVisible(False)
                        self.right_lane.setVisible(True)
                        
                    case 10:
                        counter1=counter1+1;
                        self.left_lane.setVisible(True)
                        self.right_lane.setVisible(False)           
                    case 11:
                        counter2=counter2+1;
                        counter1=counter1+1;
                        self.left_lane.setVisible(True)
                        self.right_lane.setVisible(True)
                match spot:
                    case 0:
                        self.left_blind_spot.setVisible(False)
                        self.right_blind_spot.setVisible(False)
                    case 1:
                        counter2=counter2+1;
                        self.left_blind_spot.setVisible(False)
                        self.right_blind_spot.setVisible(True)
                        
                    case 10:
                        counter1=counter1+1;
                        self.left_blind_spot.setVisible(True)
                        self.right_blind_spot.setVisible(False)           
                    case 11:
                        counter2=counter2+1;
                        counter1=counter1+1;
                        self.left_blind_spot.setVisible(True)
                        self.right_blind_spot.setVisible(True)        
                if(counter1>0):
                     self.left_warning.setVisible(True)
                else:
                     self.left_warning.setVisible(False)
                if(counter2>0):
                     self.rightwarning.setVisible(True)
                else:
                     self.rightwarning.setVisible(False)     
                
            

if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = MainWindow()
    window.threadRead()
    window.show()
    sys.exit(app.exec_())

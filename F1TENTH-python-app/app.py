from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import sys


class Color(QWidget):

    def __init__(self, color, *args, **kwargs):
        super(Color, self).__init__(*args, **kwargs)
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)


##setting the window
class MainWindow(QMainWindow):
    my_signal = pyqtSignal(str)
    part = 1  # for instruction changing
    selectedButt = 10
    keyPressed = pyqtSignal(QEvent)

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, *kwargs)

        self.setWindowTitle("F10 control app v 0.3")
        ##
        self.menusManager = QStackedLayout()

        self.createLogoButton()
        self.createBlankButton()
        self.createGoBackButton()

        self.createMainMenu()  # 0
        self.createConnectionMenu()  # 1
        self.createControlMenu()  # 2
        self.createHowToMenu()  # 3

        self.unFocusAll()

        self.menusManager.addWidget(self.mainMenuWidget)
        self.menusManager.addWidget(self.connectionMenuWidget)
        self.menusManager.addWidget(self.controlMenuWidget)
        self.menusManager.addWidget(self.howToMenuWidget)

        self.menusManager.setCurrentIndex(0)  # main menu as a start point

        self.menusWidget = QWidget()
        self.menusWidget.setLayout(
            self.menusManager)  # to put a layout inside another layout we have to put it into a widget

        self.windowLayout = QVBoxLayout()

        self.blankLayout = QHBoxLayout()
        self.blankLayout.addWidget(self.buttBlank, Qt.AlignHCenter)
        self.blankLayout.setAlignment(Qt.AlignCenter)
        self.blankWidget = QWidget()
        self.blankWidget.setLayout(self.blankLayout)

        self.logoLayout = QHBoxLayout()
        self.logoLayout.addWidget(self.buttLogo, Qt.AlignHCenter)
        self.logoLayout.setAlignment(Qt.AlignCenter)
        self.logoWidget = QWidget()
        self.logoWidget.setLayout(self.logoLayout)

        self.exitLayout = QHBoxLayout()
        self.exitLayout.addWidget(self.buttGoBack, Qt.AlignHCenter)
        self.exitLayout.setAlignment(Qt.AlignCenter)
        self.exitWidget = QWidget()
        self.exitWidget.setLayout(self.exitLayout)

        self.windowLayout.addWidget(self.blankWidget)
        self.windowLayout.addWidget(self.logoWidget)
        self.windowLayout.addWidget(self.menusWidget)
        self.windowLayout.addWidget(self.exitWidget)
        self.windowLayout.setAlignment(Qt.AlignCenter)

        self.baseWidget = QWidget()
        self.baseWidget.setLayout(self.windowLayout)

        self.selectedButt = 0
        self.allButtonsEventInstall()  # for HOVER events
        # self.keyPressed.connect(self.on_key)

        self.setCentralWidget(self.baseWidget)

    ##      MENUS

    def createMainMenu(self):
        print("Main Menu Created")

        self.createConnectionButton()
        self.createControlMenuButton()
        self.createHowToMenuButton()
        self.createRandomGif()

        self.mainMenuLayout = QVBoxLayout()
        self.mainMenuLayout.addWidget(self.buttConnection)
        self.mainMenuLayout.addWidget(self.buttControl)
        self.mainMenuLayout.addWidget(self.buttHowTo)
        # self.mainMenuLayout.addWidget(self.movie_screen)
        self.mainMenuLayout.setAlignment(Qt.AlignCenter)

        self.mainMenuWidget = QWidget()

        self.mainMenuWidget.setLayout(self.mainMenuLayout)

    def createConnectionMenu(self):
        print("Connection Test Menu Created")
        # self.createGoBackButton()

        self.connectionMenuLayout = QVBoxLayout()
        # self.connectionMenuLayout.addWidget(self.buttGoBack)

        self.connectionMenuLayout.setAlignment(Qt.AlignCenter)

        self.connectionMenuWidget = QWidget()
        self.connectionMenuWidget.setLayout(self.connectionMenuLayout)

    def createControlMenu(self):
        print("Control Menu Created")

        self.controlMenuLayout = QVBoxLayout()

        self.arrowUPLayout = QHBoxLayout()
        self.createUPbutton()
        self.arrowUPLayout.addWidget((self.buttUP))
        self.arrowUPLayout.setAlignment(Qt.AlignCenter)
        self.arrowUPWidget = QWidget()
        self.arrowUPWidget.setLayout(self.arrowUPLayout)
        self.controlMenuLayout.addWidget(self.arrowUPWidget)
        self.controlMenuLayout.setAlignment(Qt.AlignCenter)

        self.arrowsLayout = QHBoxLayout()
        self.createLEFTbutton()
        self.arrowsLayout.addWidget(self.buttLEFT)
        self.createDOWNbutton()
        self.arrowsLayout.addWidget(self.buttDOWN)
        self.createRIGHTbutton()
        self.arrowsLayout.addWidget(self.buttRIGHT)
        self.arrowsLayout.setAlignment(Qt.AlignCenter)
        self.arrowsWidget = QWidget()
        self.arrowsWidget.setLayout(self.arrowsLayout)
        self.controlMenuLayout.addWidget(self.arrowsWidget)

        self.AUTOLayout = QHBoxLayout()
        self.createAutonomousButton()
        self.AUTOLayout.addWidget((self.buttAuto))
        self.AUTOLayout.setAlignment(Qt.AlignCenter)
        self.AUTOWidget = QWidget()
        self.AUTOWidget.setLayout(self.AUTOLayout)
        self.controlMenuLayout.addWidget(self.AUTOWidget)
        self.controlMenuLayout.setAlignment(Qt.AlignCenter)

        self.SQUARELayout = QHBoxLayout()
        self.createSQUAREButton()
        self.SQUARELayout.addWidget((self.buttSQUARE))
        self.SQUARELayout.setAlignment(Qt.AlignCenter)
        self.SQUAREWidget = QWidget()
        self.SQUAREWidget.setLayout(self.SQUARELayout)
        self.controlMenuLayout.addWidget(self.SQUAREWidget)
        self.controlMenuLayout.setAlignment(Qt.AlignCenter)

        # self.goBackLayout = QHBoxLayout()
        # #self.createGoBackButton()
        # self.goBackLayout.addWidget((self.buttGoBack))
        # self.goBackLayout.setAlignment(Qt.AlignCenter)
        # self.goBackWidget = QWidget()
        # self.goBackWidget.setLayout(self.goBackLayout)
        # self.controlMenuLayout.addWidget(self.goBackWidget)
        # self.controlMenuLayout.setAlignment(Qt.AlignCenter)

        self.controlMenuWidget = QWidget()
        self.controlMenuWidget.setLayout(self.controlMenuLayout)

    def createHowToMenu(self):
        print("HowTo Menu Created")

        self.createIns1Button()
        self.createIns2Button()
        # self.createGoBackButton()

        self.howToMenuLayout = QVBoxLayout()

        self.instructionsLayout = QStackedLayout()
        self.instructionsLayout.addWidget(self.buttIns1)
        self.instructionsLayout.addWidget(self.buttIns2)
        self.instructionsLayout.setCurrentIndex(0)
        self.instructionsLayout.setAlignment(Qt.AlignVCenter)
        self.instructionsWidget = QWidget()
        self.instructionsWidget.setLayout(self.instructionsLayout)

        self.howToMenuLayout.addWidget(self.instructionsWidget)
        # self.howToMenuLayout.addWidget(self.buttGoBack)

        self.howToMenuLayout.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)

        self.howToMenuWidget = QWidget()
        self.howToMenuWidget.setLayout(self.howToMenuLayout)

    ## BUTTONS

    def createRandomGif(self):
        print("random GIF created")
        # Load the file into a QMovie
        self.movie = QMovie("resources/random.gif", QByteArray(), self)

        size = self.movie.scaledSize()
        self.setGeometry(200, 200, size.width(), size.height())

        self.movie_screen = QLabel()
        # Make label fit the gif
        self.movie_screen.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.movie_screen.setAlignment(Qt.AlignCenter)

        # Add the QMovie object to the label
        self.movie.setCacheMode(QMovie.CacheAll)
        self.movie.setSpeed(100)
        self.movie_screen.setMovie(self.movie)
        self.movie.start()

    def createBlankButton(self):
        print("Blank Button created")
        self.buttBlank = QPushButton()
        self.buttBlank.setIconSize(QSize(400, 125))
        self.buttBlank.setFixedSize(QSize(1, 1))
        self.buttBlank.setStyleSheet("border: none")

    def createLogoButton(self):
        print("Logo Button created")
        self.buttLogo = QPushButton()
        self.buttLogo.setIcon(QIcon(QPixmap("resources/logo.png")))
        self.buttLogo.setIconSize(QSize(400, 125))
        self.buttLogo.setFixedSize(QSize(400, 125))
        self.buttLogo.setStyleSheet("border: none")

        self.buttLogo.clicked.connect(self.logoClicked)

    def createConnectionButton(self):
        print("Connection Button Created")
        self.buttConnection = QPushButton()
        self.buttConnection.setIcon(QIcon(QPixmap("resources/ConnectionTest.png")))
        self.buttConnection.setIconSize(QSize(300, 75))
        self.buttConnection.setFixedSize(QSize(300, 75))
        self.buttConnection.setStyleSheet("border: none")
        self.buttConnection.clicked.connect(self.connectionClicked)

    def createControlMenuButton(self):
        print("Control Button Created")
        self.buttControl = QPushButton()
        self.buttControl.setIcon(QIcon(QPixmap("resources/ControlPanel.png")))
        self.buttControl.setIconSize(QSize(300, 75))
        self.buttControl.setFixedSize(QSize(300, 75))
        self.buttControl.setStyleSheet("border: none")
        self.buttControl.clicked.connect(self.controlClicked)

    def createHowToMenuButton(self):
        print("How To Button Created")
        self.buttHowTo = QPushButton()
        self.buttHowTo.setIcon(QIcon(QPixmap("resources/HowTo.png")))
        self.buttHowTo.setIconSize(QSize(300, 75))
        self.buttHowTo.setFixedSize(QSize(300, 75))
        self.buttHowTo.setStyleSheet("border: none")
        self.buttHowTo.clicked.connect(self.howToClicked)

    def createGoBackButton(self):
        print("Go Back Button Created")
        self.buttGoBack = QPushButton()
        self.buttGoBack.setIcon(QIcon(QPixmap("resources/Exit.png")))
        self.buttGoBack.setIconSize(QSize(300, 75))
        self.buttGoBack.setFixedSize(QSize(300, 75))
        self.buttGoBack.setStyleSheet("border: none")
        self.buttGoBack.clicked.connect(self.goBackClicked)

    def createUPbutton(self):
        print("UP Button Created")
        self.buttUP = QPushButton()
        self.buttUP.setIcon(QIcon(QPixmap("resources/UP.png")))
        self.buttUP.setIconSize(QSize(120, 75))
        self.buttUP.setFixedSize(QSize(120, 75))
        self.buttUP.setStyleSheet("border: none")
        self.buttUP.clicked.connect(self.UPclicked)

    def createDOWNbutton(self):
        print("DOWN Button Created")
        self.buttDOWN = QPushButton()
        self.buttDOWN.setIcon(QIcon(QPixmap("resources/DOWN.png")))
        self.buttDOWN.setIconSize(QSize(120, 75))
        self.buttDOWN.setFixedSize(QSize(120, 75))
        self.buttDOWN.setStyleSheet("border: none")
        self.buttDOWN.clicked.connect(self.DOWNclicked)

    def createLEFTbutton(self):
        print("LEFT Button Created")
        self.buttLEFT = QPushButton()
        self.buttLEFT.setIcon(QIcon(QPixmap("resources/LEFT.png")))
        self.buttLEFT.setIconSize(QSize(120, 75))
        self.buttLEFT.setFixedSize(QSize(120, 75))
        self.buttLEFT.setStyleSheet("border: none")
        self.buttLEFT.clicked.connect(self.LEFTclicked)

    def createRIGHTbutton(self):
        print("RIGHT Button Created")
        self.buttRIGHT = QPushButton()
        self.buttRIGHT.setIcon(QIcon(QPixmap("resources/RIGHT.png")))
        self.buttRIGHT.setIconSize(QSize(120, 75))
        self.buttRIGHT.setFixedSize(QSize(120, 75))
        self.buttRIGHT.setStyleSheet("border: none")
        self.buttRIGHT.clicked.connect(self.RIGHTclicked)

    def createAutonomousButton(self):
        print("Autonomous Button Created")
        self.buttAuto = QPushButton()
        self.buttAuto.setIcon(QIcon(QPixmap("resources/Autonomous.png")))
        self.buttAuto.setIconSize(QSize(300, 75))
        self.buttAuto.setFixedSize(QSize(300, 75))
        self.buttAuto.setStyleSheet("border: none")
        # self.buttAuto.enterEvent() #begin the hover
        # self.buttAuto.leaveEvent() #end the hover
        self.buttAuto.clicked.connect(self.AutonomousClicked)

    def createSQUAREButton(self):
        print("Square Template Button Created")
        self.buttSQUARE = QPushButton()
        self.buttSQUARE.setIcon(QIcon(QPixmap("resources/Square.png")))
        self.buttSQUARE.setIconSize(QSize(300, 75))
        self.buttSQUARE.setFixedSize(QSize(300, 75))
        self.buttSQUARE.setStyleSheet("border: none")
        # self.buttSQUARE.enterEvent() #begin the hover
        # self.buttSQUARE.leaveEvent() #end the hover
        self.buttSQUARE.clicked.connect(self.SQUAREClicked)

    def createIns1Button(self):
        print("Instruction Button Created")

        self.buttIns1 = QPushButton()
        self.buttIns1.setIcon(QIcon(QPixmap("resources/instruction1.png")))
        self.buttIns1.setIconSize(QSize(530, 300))
        self.buttIns1.setFixedSize(QSize(530, 300))
        self.buttIns1.setStyleSheet("border: none")
        # self.buttSQUARE.enterEvent() #begin the hover
        # self.buttSQUARE.leaveEvent() #end the hover
        self.buttIns1.clicked.connect(self.insClicked)

    def createIns2Button(self):
        print("Instruction2 Button Created")

        self.buttIns2 = QPushButton()
        self.buttIns2.setIcon(QIcon(QPixmap("resources/instruction2.png")))
        self.buttIns2.setIconSize(QSize(530, 210))
        self.buttIns2.setFixedSize(QSize(530, 210))
        self.buttIns2.setStyleSheet("border: none")
        # self.buttSQUARE.enterEvent() #begin the hover
        # self.buttSQUARE.leaveEvent() #end the hover
        self.buttIns2.clicked.connect(self.insClicked)

    ###     CONNECTIONS & EVENTS

    def eventFilter(self, obj, event):
        # logo button
        if obj == self.buttLogo and event.type() == QEvent.HoverEnter:
            self.buttLogoOnHovered()
            self.selectedButt = 1
        if obj == self.buttLogo and event.type() == QEvent.HoverLeave:
            self.buttLogoOffHovered()
            self.selectedButt = 0

        # ConnectionTest button
        if obj == self.buttConnection and event.type() == QEvent.HoverEnter:
            self.buttConnectionOnHovered()
            self.selectedButt = 2
        if obj == self.buttConnection and event.type() == QEvent.HoverLeave:
            self.buttConnectionOffHovered()
            self.selectedButt = 0

        # Control button
        if obj == self.buttControl and event.type() == QEvent.HoverEnter:
            self.buttControlOnHovered()
            self.selectedButt = 3
        if obj == self.buttControl and event.type() == QEvent.HoverLeave:
            self.buttControlOffHovered()
            self.selectedButt = 0

        # HowTo button
        if obj == self.buttHowTo and event.type() == QEvent.HoverEnter:
            self.buttHowToOnHovered()
            self.selectedButt = 4
        if obj == self.buttHowTo and event.type() == QEvent.HoverLeave:
            self.buttHowToOffHovered()
            self.selectedButt = 0

        # GoBack button
        if obj == self.buttGoBack and event.type() == QEvent.HoverEnter:
            self.buttGoBackOnHovered()
            self.selectedButt = 5
        if obj == self.buttGoBack and event.type() == QEvent.HoverLeave:
            self.buttGoBackOffHovered()
            self.selectedButt = 0

        # Autonomous button
        if obj == self.buttAuto and event.type() == QEvent.HoverEnter:
            self.buttAutonomousOnHovered()
            self.selectedButt = 6
        if obj == self.buttAuto and event.type() == QEvent.HoverLeave:
            self.buttAutonomousOffHovered()
            self.selectedButt = 0

        # SQUARE button
        if obj == self.buttSQUARE and event.type() == QEvent.HoverEnter:
            self.buttSQUAREOnHovered()
            self.selectedButt = 7
        if obj == self.buttSQUARE and event.type() == QEvent.HoverLeave:
            self.buttSQUAREOffHovered()
            self.selectedButt = 0

        # UP button
        if obj == self.buttUP and event.type() == QEvent.HoverEnter:
            self.buttUPOnHovered()
            self.selectedButt = 8
        if obj == self.buttUP and event.type() == QEvent.HoverLeave:
            self.buttUPOffHovered()
            self.selectedButt = 0

        # DOWN button
        if obj == self.buttDOWN and event.type() == QEvent.HoverEnter:
            self.buttDOWNOnHovered()
            self.selectedButt = 9
        if obj == self.buttDOWN and event.type() == QEvent.HoverLeave:
            self.buttDOWNOffHovered()
            self.selectedButt = 0

        # LEFT button
        if obj == self.buttLEFT and event.type() == QEvent.HoverEnter:
            self.buttLEFTOnHovered()
            self.selectedButt = 10
        if obj == self.buttLEFT and event.type() == QEvent.HoverLeave:
            self.buttLEFTOffHovered()
            self.selectedButt = 0

        # RIGHT button
        if obj == self.buttRIGHT and event.type() == QEvent.HoverEnter:
            self.buttRIGHTOnHovered()
            self.selectedButt = 11
        if obj == self.buttRIGHT and event.type() == QEvent.HoverLeave:
            self.buttRIGHTOffHovered()
            self.selectedButt = 0

        # KEYBOARD

        if event.type() == QEvent.KeyPress:
            print("start")
            print("current window" + str(self.menusManager.currentIndex()))
            if event.key() == Qt.Key_Up:
                print("Up arrow")
                self.UPclicked()
            elif event.key() == Qt.Key_Down:
                print("Down arrow")
                self.DOWNclicked()
            elif event.key() == Qt.Key_Left:
                print("Left arrow")
                self.LEFTclicked()
            elif event.key() == Qt.Key_Right:
                print("Right arrow")
                self.RIGHTclicked()
            elif event.key() == Qt.Key_Space:
                print("spacja")
                self.enterHandler(self.selectedButt)


            elif event.key() == Qt.Key_W:
                print("W")
                if (self.selectedButt >= 0 and self.selectedButt <= 7 and self.selectedButt != 5 and self.selectedButt != 1):
                    if(self.selectedButt == 0):
                        if(self.menusManager.currentIndex() == 0):
                            self.hoverHandler(2)
                        elif(self.menusManager.currentIndex() == 2):
                            self.hoverHandler(6)
                    elif(self.selectedButt == 2):
                        self.hoverHandler(4)
                    elif(self.selectedButt == 6):
                        self.hoverHandler(7)
                    else:
                        self.hoverHandler(self.selectedButt - 1)
                if (self.selectedButt >=8 and self.selectedButt <=11):
                    self.hoverHandler(6)


            elif event.key() == Qt.Key_S:
                print("S")
                if (self.selectedButt >= 0 and self.selectedButt <= 7 and self.selectedButt != 5 and self.selectedButt != 1):
                    if(self.selectedButt == 0):
                        if(self.menusManager.currentIndex() == 0):
                            self.hoverHandler(4)
                        elif(self.menusManager.currentIndex() == 2):
                            self.hoverHandler(7)
                    elif(self.selectedButt == 4):
                        self.hoverHandler(2)
                    elif(self.selectedButt == 7):
                        self.hoverHandler(6)
                    else:
                        self.hoverHandler(self.selectedButt + 1)
                if (self.selectedButt >=8 and self.selectedButt <=11):
                    self.hoverHandler(7)

            elif event.key() == Qt.Key_A:
                print("A")
            elif event.key() == Qt.Key_D:
                print("D")

            elif event.key() == Qt.Key_Escape:
                self.goBackClicked()
                self.unHoverAll()
                self.selectedButt = 0
                print("escape")


            elif event.key() == Qt.Key_Enter:
                print("enter")
            else:
                print("inne")
                pass
                # return super().keyPressEvent(self, e)
            print("stop" + str(self.selectedButt))

        if event.type() == QEvent.KeyRelease:
            if event.key() == Qt.Key_Up:
                print("Up arrow")
                self.buttUPOffHovered()
            elif event.key() == Qt.Key_Down:
                print("Down arrow")
                self.buttDOWNOffHovered()
            elif event.key() == Qt.Key_Left:
                print("Left arrow")
                self.buttLEFTOffHovered()
            elif event.key() == Qt.Key_Right:
                print("Right arrow")
                self.buttRIGHTOffHovered()


        return super(MainWindow, self).eventFilter(obj, event)

    def allButtonsEventInstall(self):
        self.buttBlank.installEventFilter(self)
        self.buttLogo.installEventFilter(self)
        self.buttConnection.installEventFilter(self)
        self.buttControl.installEventFilter(self)
        self.buttHowTo.installEventFilter(self)
        self.buttAuto.installEventFilter(self)
        self.buttGoBack.installEventFilter(self)
        self.buttUP.installEventFilter(self)
        self.buttDOWN.installEventFilter(self)
        self.buttLEFT.installEventFilter(self)
        self.buttRIGHT.installEventFilter(self)
        self.buttSQUARE.installEventFilter(self)



    def logoClicked(self):
        print("logo klik!")

    def buttLogoOnHovered(self):
        self.buttLogo.setIcon(QIcon(QPixmap("resources/HOVER/logo.png")))

    def buttLogoOffHovered(self):
        self.buttLogo.setIcon(QIcon(QPixmap("resources/logo.png")))



    def connectionClicked(self):
        print("conncection test klik!")
        # self.createConnectionMenu()
        self.menusManager.setCurrentIndex(1)
        self.buttGoBack.setIcon(QIcon(QPixmap("resources/GoBack.png")))
        # self.menusManager.addWidget(self.connectionMenuWidget)

    def buttConnectionOnHovered(self):
        self.buttConnection.setIcon(QIcon(QPixmap("resources/HOVER/ConnectionTest.png")))

    def buttConnectionOffHovered(self):
        self.buttConnection.setIcon(QIcon(QPixmap("resources/ConnectionTest.png")))



    def controlClicked(self):
        print("control klik!")
        # self.createControlMenu()
        self.menusManager.setCurrentIndex(2)
        self.buttGoBack.setIcon(QIcon(QPixmap("resources/GoBack.png")))
        # self.menusManager.addWidget(self.connectionMenuWidget)

    def buttControlOnHovered(self):
        self.buttControl.setIcon(QIcon(QPixmap("resources/HOVER/ControlPanel.png")))

    def buttControlOffHovered(self):
        self.buttControl.setIcon(QIcon(QPixmap("resources/ControlPanel.png")))



    def howToClicked(self):
        print("howTo klik!")
        # self.createConnectionMenu()
        self.menusManager.setCurrentIndex(3)
        self.buttGoBack.setIcon(QIcon(QPixmap("resources/GoBack.png")))

    def buttHowToOnHovered(self):
        self.buttHowTo.setIcon(QIcon(QPixmap("resources/HOVER/HowTo.png")))

    def buttHowToOffHovered(self):
        self.buttHowTo.setIcon(QIcon(QPixmap("resources/HowTo.png")))




    def goBackClicked(self):
        print("go back klik!")
        # self.createMainMenu()
        if self.menusManager.currentIndex() == 0:
            QCoreApplication.instance().quit()
        else:
            self.menusManager.setCurrentIndex(0)
            self.buttGoBack.setIcon(QIcon(QPixmap("resources/Exit.png")))
        print(self.menusManager.count())
        # self.menusManager.addWidget(self.connectionMenuWidget)

    def buttGoBackOnHovered(self):
        if self.menusManager.currentIndex() != 0:
            self.buttGoBack.setIcon(QIcon(QPixmap("resources/HOVER/GoBack.png")))
        else:
            self.buttGoBack.setIcon(QIcon(QPixmap("resources/HOVER/Exit.png")))

    def buttGoBackOffHovered(self):
        if self.menusManager.currentIndex() != 0:
            self.buttGoBack.setIcon(QIcon(QPixmap("resources/GoBack.png")))
        else:
            self.buttGoBack.setIcon(QIcon(QPixmap("resources/Exit.png")))



    def AutonomousClicked(self):
        print("Autonomous klik!")

    def buttAutonomousOnHovered(self):
        self.buttAuto.setIcon(QIcon(QPixmap("resources/HOVER/Autonomous.png")))

    def buttAutonomousOffHovered(self):
        self.buttAuto.setIcon(QIcon(QPixmap("resources/Autonomous.png")))



    def SQUAREClicked(self):
        print("SQUARE klik!")

    def buttSQUAREOnHovered(self):
        self.buttSQUARE.setIcon(QIcon(QPixmap("resources/HOVER/Square.png")))

    def buttSQUAREOffHovered(self):
        self.buttSQUARE.setIcon(QIcon(QPixmap("resources/Square.png")))



    def UPclicked(self):
        print("UP klik!")
        self.buttUP.setIcon(QIcon(QPixmap("resources/PRESS/UP.png")))

    def buttUPOnHovered(self):
        self.buttUP.setIcon(QIcon(QPixmap("resources/HOVER/UP.png")))

    def buttUPOffHovered(self):
        self.buttUP.setIcon(QIcon(QPixmap("resources/UP.png")))



    def DOWNclicked(self):
        print("DOWN klik!")
        self.buttDOWN.setIcon(QIcon(QPixmap("resources/PRESS/DOWN.png")))

    def buttDOWNOnHovered(self):
        self.buttDOWN.setIcon(QIcon(QPixmap("resources/HOVER/DOWN.png")))

    def buttDOWNOffHovered(self):
        self.buttDOWN.setIcon(QIcon(QPixmap("resources/DOWN.png")))



    def LEFTclicked(self):
        print("LEFT klik!")
        self.buttLEFT.setIcon(QIcon(QPixmap("resources/PRESS/LEFT.png")))

    def buttLEFTOnHovered(self):
        self.buttLEFT.setIcon(QIcon(QPixmap("resources/HOVER/LEFT.png")))

    def buttLEFTOffHovered(self):
        self.buttLEFT.setIcon(QIcon(QPixmap("resources/LEFT.png")))


    def RIGHTclicked(self):
        print("RIGHT klik!")
        self.buttRIGHT.setIcon(QIcon(QPixmap("resources/PRESS/RIGHT.png")))

    def buttRIGHTOnHovered(self):
        self.buttRIGHT.setIcon(QIcon(QPixmap("resources/HOVER/RIGHT.png")))

    def buttRIGHTOffHovered(self):
        self.buttRIGHT.setIcon(QIcon(QPixmap("resources/RIGHT.png")))

    def insClicked(self):
        print("instruction klik! ")

        if self.instructionsLayout.currentIndex() == 0:
            self.instructionsLayout.setCurrentIndex(1)
        elif self.instructionsLayout.currentIndex() == 1:
            self.instructionsLayout.setCurrentIndex(0)
        else:
            self.instructionsLayout.setCurrentIndex(0)
        # self.updateHowToMenu() #for HowToMenu update

    ### KEYBOARD-SUPPORTED SELECTION

    def unHoverAll(self):
        self.buttLogoOffHovered()  # 1
        self.buttConnectionOffHovered()  # 2
        self.buttControlOffHovered()  # 3
        self.buttHowToOffHovered()  # 4
        self.buttGoBackOffHovered()  # 5
        self.buttAutonomousOffHovered()  # 6
        self.buttSQUAREOffHovered()  # 7
        self.buttUPOffHovered()  # 8
        self.buttDOWNOffHovered()  # 9
        self.buttLEFTOffHovered()  # 10
        self.buttRIGHTOffHovered()  # 11

    def unFocusAll(self):
        self.buttLogo.setFocusPolicy(Qt.NoFocus)
        self.buttConnection.setFocusPolicy(Qt.NoFocus)
        self.buttControl.setFocusPolicy(Qt.NoFocus)
        self.buttHowTo.setFocusPolicy(Qt.NoFocus)
        self.buttGoBack.setFocusPolicy(Qt.NoFocus)
        self.buttAuto.setFocusPolicy(Qt.NoFocus)
        self.buttSQUARE.setFocusPolicy(Qt.NoFocus)
        self.buttUP.setFocusPolicy(Qt.NoFocus)
        self.buttDOWN.setFocusPolicy(Qt.NoFocus)
        self.buttLEFT.setFocusPolicy(Qt.NoFocus)
        self.buttRIGHT.setFocusPolicy(Qt.NoFocus)

    def hoverHandler(self, numb):
        self.unHoverAll()
        self.selectedButt = numb

        if numb is 0:
            pass
        elif numb is 1:
            self.buttLogoOnHovered()  # 1
        elif numb is 2:
            self.buttConnectionOnHovered()  # 2
        elif numb is 3:
            self.buttControlOnHovered()  # 3
        elif numb is 4:
            self.buttHowToOnHovered()  # 4
        elif numb is 5:
            self.buttGoBackOnHovered()  # 5


        elif numb is 6:
            self.buttAutonomousOnHovered()  # 6
        elif numb is 7:
            self.buttSQUAREOnHovered()  # 7

        elif numb is 8:
            self.buttUPOnHovered()  # 8
        elif numb is 9:
            self.buttDOWNOnHovered()  # 9
        elif numb is 10:
            self.buttLEFTOnHovered()  # 10
        elif numb is 11:
            self.buttRIGHTOnHovered()  # 11
        pass




    def enterHandler(self, numb):
        #self.unHoverAll()
        #self.selectedButt = numb

        print("hi1")

        if numb is 0:
            pass
        elif numb is 1:
            self.logoClicked()  # 1
        elif numb is 2:
            self.connectionClicked()  # 2
        elif numb is 3:
            print("clik333")
            self.controlClicked() # 3
        elif numb is 4:
            self.howToClicked()  # 4
        elif numb is 5:
            self.goBackClicked()  # 5


        elif numb is 6:
            self.AutonomousClicked()  # 6
        elif numb is 7:
            self.SQUAREClicked()  # 7

        elif numb is 8:
            self.UPClicked()  # 8
        elif numb is 9:
            self.DOWNClicked()  # 9
        elif numb is 10:
            self.LEFTClicked()  # 10
        elif numb is 11:
            self.RIGHTClicked()  # 11
        self.unHoverAll()
        self.selectedButt = 0
        print("hi2")

        pass


##################      MAIN
app = QApplication(sys.argv)
app.setStyleSheet("QWidget {background-image: url(./resources/background2.png) }")
window = MainWindow()
window.setFixedSize(600, 800)
window.show()

app.exec_()


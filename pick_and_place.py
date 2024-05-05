import sys
import os
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel

cube_red = {"x": 0.55, "y": 0.3, "z": 0.0125}
cube_green = {"x": 0.55, "y": 0.0, "z": 0.0125}
cube_brown = {"x": 0.55, "y": -0.3, "z": 0.0125}
cube_yellow = {"x": 0.75, "y": 0.0, "z": 0.0125}

cubes = [cube_red, cube_green, cube_brown, cube_yellow]
cube_size = 0.025

class NumberSubmitter(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Pick and place')
        self.setGeometry(100, 100, 1000, 1000)

        # Create labels and line edits for coords
        self.pick_label_x = QLabel('x', self)
        self.pick_edit_x = QLineEdit(self)
        self.pick_edit_x.setMaximumWidth(50)  # Set maximum width
        self.pick_label_y = QLabel('y', self)
        self.pick_edit_y = QLineEdit(self)
        self.pick_edit_y.setMaximumWidth(50) 
        self.pick_label_z = QLabel('z', self)
        self.pick_edit_z = QLineEdit(self)
        self.pick_edit_z.setMaximumWidth(50) 

        self.place_label_x = QLabel('x', self)
        self.place_edit_x = QLineEdit(self)
        self.place_edit_x.setMaximumWidth(50)
        self.place_label_y = QLabel('y', self)
        self.place_edit_y = QLineEdit(self)
        self.place_edit_y.setMaximumWidth(50) 
        self.place_label_z = QLabel('z', self)
        self.place_edit_z = QLineEdit(self)
        self.place_edit_z.setMaximumWidth(50)

        self.tower_label_x = QLabel('x', self)
        self.tower_edit_x = QLineEdit(self)
        self.tower_edit_x.setMaximumWidth(50)
        self.tower_label_y = QLabel('y', self)
        self.tower_edit_y = QLineEdit(self)
        self.tower_edit_y.setMaximumWidth(50)



        # Create submit button
        self.pick_button = QPushButton('Pick', self)
        self.pick_button.clicked.connect(self.pick)
        self.place_button = QPushButton('Place', self)
        self.place_button.clicked.connect(self.place)
        self.tower_button = QPushButton('Build tower', self)
        self.tower_button.clicked.connect(self.build_tower)


        self.pick_label_x.move(50, 100)
        self.pick_edit_x.move(30, 130)
        self.pick_label_y.move(120, 100)
        self.pick_edit_y.move(100, 130)
        self.pick_label_z.move(190, 100)
        self.pick_edit_z.move(170, 130)
        self.pick_button.move(250, 130)

        self.place_label_x.move(50, 100+200)
        self.place_edit_x.move(30, 130+200)
        self.place_label_y.move(120, 100+200)
        self.place_edit_y.move(100, 130+200)
        self.place_label_z.move(190, 100+200)
        self.place_edit_z.move(170, 130+200)
        self.place_button.move(250, 130+200)

        self.tower_label_x.move(50, 100+400)
        self.tower_edit_x.move(30, 130+400)
        self.tower_label_y.move(120, 100+400)
        self.tower_edit_y.move(100, 130+400)
        self.tower_button.move(250, 130+400)

    def pick(self):
        x = self.pick_edit_x.text()
        y = self.pick_edit_y.text()
        z = self.pick_edit_z.text()
        command = "rosrun ltl_project pick _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(x, y, z, "false")
        subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'))

    def place(self):
        x = self.place_edit_x.text()
        y = self.place_edit_y.text()
        z = self.place_edit_z.text()
        command = "rosrun ltl_project place _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(x, y, z, "false")
        subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'))

    def build_tower(self):
        place_x = self.tower_edit_x.text()
        place_y = self.tower_edit_y.text()

        i = 0
        for cube in cubes:
            pick_x = cube["x"]
            pick_y = cube["y"]
            pick_z = cube["z"]
            pick_command = "rosrun ltl_project pick _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(pick_x, pick_y, pick_z, "false")
            subprocess.call(pick_command, shell=True, cwd=os.path.expanduser('~/ros'))
            place_z = (i + 1) * cube_size
            place_command = "rosrun ltl_project place _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(place_x, place_y, place_z, "false")
            subprocess.call(place_command, shell=True, cwd=os.path.expanduser('~/ros'))
            i += 1






if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = NumberSubmitter()
    window.show()
    sys.exit(app.exec_())


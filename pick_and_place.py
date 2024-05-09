import sys
import os
import subprocess
from functools import partial
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel


cube_size = 0.025
NUMBER_OF_CUBES = 4
cube_positions = []


def listen_to_rostopic():
    try:
        i = 1
        while i <= NUMBER_OF_CUBES:     # For every cube
            command = "rostopic echo -n 1 /cube{}".format(i)    # Command for listening to 1 rostopic value
            process = subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'), stdout=subprocess.PIPE, stderr=subprocess.PIPE)  # Execute the command
            output, error = process.communicate(timeout=3)    # Save the data from the command, timeout after 3 seconds if no rostopic found
            output_str = output.decode("utf-8")    # Decode the rostopic message
            output_list = output_str.split()    # Split the message to x, y, z, color
            cube_coords = [output_list[2], output_list[3], output_list[4], output_list[5]]  # Save the data
            cube_positions.append(cube_coords)  
            i += 1
    except:
        print("No rostopic found!")    



class NumberSubmitter(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.last_picked = -1
        self.cube_positions = cube_positions.copy()
    
        self.setWindowTitle('Pick and place')
        self.setGeometry(100, 100, 1000, 1000)
        
        # Create labels and line edits for pick, place, and tower
        self.pick_label_x = QLabel('x', self)
        self.pick_edit_x = QLineEdit(self)
        self.pick_edit_x.setMaximumWidth(50)
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



        # Create submit buttons
        self.pick_button = QPushButton('Pick', self)
        self.pick_button.clicked.connect(self.pick)
        self.place_button = QPushButton('Place', self)
        self.place_button.clicked.connect(self.place)
        self.tower_button = QPushButton('Build tower', self)
        self.tower_button.clicked.connect(self.build_tower)
        
        # Define pick buttons for each cube found in rostopic
        i = 0
        self.pick_color_buttons = []
        for cube in self.cube_positions:
            pick_color_button = QPushButton("Pick {} cube".format(cube[3]), self)
            pick_color_button.setStyleSheet("background-color : {}".format(cube[3])) 
            pick_color_button.clicked.connect(partial(self.pick_color, i))
            self.pick_color_buttons.append(pick_color_button)
            self.pick_color_buttons[i].move(600, 100+40*i)
            i += 1

        # Move all the widgets to their places
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

    # Used to pick up cube from coordinates defined in the text input field
    def pick(self):
        x = self.pick_edit_x.text()     # Save the input of text field
        self.pick_edit_x.clear()    # Clear the field
        y = self.pick_edit_y.text() 
        self.pick_edit_y.clear()
        z = self.pick_edit_z.text()
        self.pick_edit_z.clear()
        command = "rosrun ltl_project pick _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(x, y, z, "false")
        subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'))
        self.last_picked = -1   # So far coordinates are not updated after manual pick-command
        
        
    # Used to pick up a cube from certain coordinates
    def pick_color(self, cube_index):
        cube = self.cube_positions[cube_index]
        command = "rosrun ltl_project pick _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(cube[0], cube[1], cube[2], "false")
        subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'))
        self.last_picked = cube_index   # Update previously picked up cube so place can update its coords
        

    def place(self):
        x = self.place_edit_x.text()
        self.place_edit_x.clear()   
        y = self.place_edit_y.text()
        self.place_edit_y.clear()
        z = self.place_edit_z.text()
        self.place_edit_z.clear()
        command = "rosrun ltl_project place _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(x, y, z, "false")
        subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'))
        if self.last_picked != -1:
            self.cube_positions[self.last_picked] = [x, y, z, self.cube_positions[self.last_picked]]    # Update cube position after placing

    def build_tower(self):
        place_x = self.tower_edit_x.text()
        self.tower_edit_x.clear()
        place_y = self.tower_edit_y.text()
        self.tower_edit_y.clear()

        i = 0
        for cube in self.cube_positions:
            self.last_picked = i
            pick_x = float(cube[0])
            pick_y = float(cube[1])
            pick_z = float(cube[2])
            pick_command = "rosrun ltl_project pick _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(pick_x, pick_y, pick_z, "false")
            subprocess.call(pick_command, shell=True, cwd=os.path.expanduser('~/ros'))
            place_z = (i + 1) * cube_size - cube_size / 2
            place_command = "rosrun ltl_project place _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(place_x, place_y, place_z, "false")
            subprocess.call(place_command, shell=True, cwd=os.path.expanduser('~/ros'))
            self.cube_positions[i] = [place_x, place_y, place_z, self.cube_positions[i]]    # Update cube position after placing
            i += 1






if __name__ == '__main__':
    listen_to_rostopic()
    app = QApplication(sys.argv)
    window = NumberSubmitter()
    window.show()
    sys.exit(app.exec_())


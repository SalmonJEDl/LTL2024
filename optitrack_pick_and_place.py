import sys
import os
import subprocess
import time
from functools import partial
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLineEdit, QLabel, QCheckBox
from PyQt5.QtCore import QTimer
import threading

cube_size = 0.025
number_of_cubes = 0
cube_positions = []

def listen_to_optitrack():  # This function updates cube positions from OptiTrack every second in a separate thread
    while True:
        try:
            command = "rostopic echo -n 1 /natnet_ros/pointcloud"    # Command for listening to 1 rostopic value
            process = subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'), stdout=subprocess.PIPE, stderr=subprocess.PIPE)  # Execute the command
            output, error = process.communicate(timeout=3)    # Save the data from the command, timeout after 3 seconds if no rostopic found
            output_str = output.decode("utf-8")    # Decode the rostopic message
            output_list = output_str.split()    # Split the message to x, y, z, color
            number_of_cubes = output_list.count("x:")
            cube_positions.clear()
            for i in range(number_of_cubes):
                cube_coords = [float(output_list[13+i*7]), float(output_list[15+i*7]), float(output_list[17+i*7])]  # Parse the coordinates from rostopic
                cube_positions.append(cube_coords)
            time.sleep(1)
        except:
            print("No rostopic found!")
            break;

        
        
            
        
        
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
        self.swap_button = QPushButton("Swap 2 cubes", self)
        self.swap_button.clicked.connect(self.swap_cubes)
        
        
        # Define coordinate labels for the cubes
        i = 0
        self.cube_pos_labels = []
        for cube in cube_positions:
            cube_pos_label = QLabel("Cube {} location: {}, {}, {}".format(i+1, cube[0], cube[1], cube[2]), self)
            self.cube_pos_labels.append(cube_pos_label)
            self.cube_pos_labels[i].move(500, 600+30*i)
            i += 1
            
        def update_pos_labels():
            i = 0
            for label in self.cube_pos_labels:
                label.setText("Cube {} location: {}, {}, {}".format(i+1, cube_positions[i][0], cube_positions[i][1], cube_positions[i][2]))
                i += 1
            
        self.timer = QTimer(self)
        self.timer.timeout.connect(update_pos_labels)
        self.timer.start(1000)
        
        
        # Define pick buttons for each cube found in rostopic
        i = 0
        self.pick_color_buttons = []
        for cube in self.cube_positions:
            pick_color_button = QPushButton("Pick cube {}".format(i+1), self)
            pick_color_button.clicked.connect(partial(self.pick_color, i))
            self.pick_color_buttons.append(pick_color_button)
            self.pick_color_buttons[i].move(600, 100+40*i)
            i += 1

        
        # Define checkboxes for swapping two cubes
        i = 0
        self.checkboxes = []
        for cube in self.cube_positions:
            checkbox = QCheckBox("Cube {}".format(i+1), self)
            self.checkboxes.append(checkbox)
            self.checkboxes[i].move(600, 400+40*i)
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
        self.swap_button.move(750, 400)

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
            self.cube_positions[self.last_picked] = [x, y, z]    # Update cube position after placing
            
            
    def place_to_coords(self, x, y, z):
        command = "rosrun ltl_project place _x:={} _y:={} _z:={} _reset:={} > /dev/null".format(x, y, z, "false")
        subprocess.Popen(command, shell=True, cwd=os.path.expanduser('~/ros'))
        if self.last_picked != -1:
            self.cube_positions[self.last_picked] = [x, y, z]    # Update cube position after placing

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
            self.cube_positions[i] = [place_x, place_y, place_z]    # Update cube position after placing
            i += 1
            
    def swap_cubes(self):
        boxes_ticked = []
        for i in range(NUMBER_OF_CUBES):
            if self.checkboxes[i].checkState() == 2:    # Box is checked if its state is 2
                boxes_ticked.append(i)
        if len(boxes_ticked) == 2:  # Proceed to swap if 2 boxes are ticked
            cube_1 = self.cube_positions[boxes_ticked[0]]
            cube_2 = self.cube_positions[boxes_ticked[1]]
            pick1_x, pick1_y, pick1_z = cube_1[0], cube_1[1], cube_1[2]
            pick2_x, pick2_y, pick2_z = cube_2[0], cube_2[1], cube_2[2]
            self.pick_color(boxes_ticked[0])    # Pick cube 1
            time.sleep(15)  # Using timers for now since the rosnodes overlapped
            self.place_to_coords(pick1_x, pick1_y+0.1, pick1_z)     # Put cube 1 0.1m aside
            time.sleep(15)
            self.pick_color(boxes_ticked[1])    # Pick cube 2
            time.sleep(15)
            self.place_to_coords(pick1_x, pick1_y, pick1_z)     # Place cube 2 to cube 1's original spot
            time.sleep(15)
            self.pick_color(boxes_ticked[0])    # Pick up cube 1 from its temporary spot
            time.sleep(15)
            self.place_to_coords(pick2_x, pick2_y, pick2_z)     # Place cube 1 to the original spot of cube 2
        else:
            print("Exactly 2 boxes have to be ticked")
            
            


if __name__ == '__main__':
    thread1 = threading.Thread(target=listen_to_optitrack)  # Asyncronously read OptiTrack data
    thread1.start()
    time.sleep(1)   # Wait for the OptiTrack data before opening the interface
    app = QApplication(sys.argv)
    window = NumberSubmitter()
    window.show()
    sys.exit(app.exec_())

























if __name__ == '__main__':
    listen_to_optitrack()
    #app = QApplication(sys.argv)
    #window = NumberSubmitter()
    #window.show()
    #sys.exit(app.exec_())


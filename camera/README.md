# People Counter Camera Application

The People Counter camera application uses various OpenCV methods, mainly the OpenCV implementation of watershed, to track the movement of people detected by the camera and thus detecting room entrance/exit. The camera then stores all entrance/exit history in a local MySQL database. When more than 200 history entries has been stored or when backend requests for the camera information, all history entries will be sent to the backend using mqtt and all local history deleted.

## Libraries Used

- Paho MQTT
- OpenCV
- MySQL (Python Driver for MySQL)
- NumPy
- ROS

# How to Setup The Camera

1. Clone the repository.
2. Ensure ros is correctly installed and configured
3. Run the ```dbsetup.py``` file to set up a database where local history records will be stored. Currently the database setup porition of the code is commented out so be sure to uncomment it and make corresponding changes to MySQL cridentials in accordance to your local MySQL setup. 
4. Go to the ```people_counter/camera``` directory and run ```source ./devel/setup.bash```. You should run this everytime you open a new command prompt/terminal that you wish to run people counter camera code on.
5. In the same directory, run ```catkin_make```
6. In the same directory, run ```roslaunch ./launch/app.launch``` to launch counting algorithm.


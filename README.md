
# QUADCOPTER SIMULATION USING ROS
ARIES Project

Welcome to the repository,
In the below lines we have tried to give a  detailed instruction set on how the drone simulation in Gazebo9 using Robotic Operating System (ROS) happens.

                                       Quadcopter simulation in ROS

The follwoing steps will help you to develop the project starting in a basic windows machine :

1. Firstly if you are windows user dual boot your machine to Ubuntu Linux 18.04. (Do not use WSL or Virtual Machine as simulation requires a lot of power which cannot be provided by them).
2. Understand basic linux commands (ls,cd,mkdir etc.)
3. Developing the newly linux machine with some tools so that it functions better.

Updating & Upgrading Ubuntu

                sudo apt-get update && sudo apt-get upgrade


Solving Time Issue After Dual Boot

        timedatectl set-local-rtc 1 --adjust-system-clock


Install Terminator

          sudo add-apt-repository ppa:gnome-terminator

        sudo apt-get install terminator


Install Sublime Text

      wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -

       echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list

     sudo apt-get update

      sudo apt-get install sublime-text

        subl


Important Linux Packages

        sudo apt-get install libavcodec-dev libsdl1.2-dev xsltproc libbullet-dev libsdl1.2-dev libgoogle-glog-dev protobuf-compiler python-wstool


Install Gazebo 9

        sudo apt-get install gazebo9 libgazebo9-*


Install Arduino IDE

Open up a terminal in arduino folder and then type this command

            sudo ./install.sh


Install GIT

         sudo apt-get install git


Downloading ROS melodic version and setup your catkin workspace. Copy paste below commands in your linux terminal to develop that.


Setup your sources.list

                   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /


Set up your keys

                       sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BAD


Update packages and install ROS

                                 sudo apt update


            
                           sudo apt install ros-melodic-desktop-full


Setup the environment

                echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
                          
                          
                             source ~/.bashrc


Dependencies

                        sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-w

Rosdep

                                      sudo apt install python-rosdep
                           
                                        sudo rosdep init 
                             
                                         rosdep update

Creating a catkin workspace

                                  mkdir catkin_ws
                         
                         
                                    cd catkin_ws
                         
                        
                        
                                    mkdir -p src
                         
                       
                       
                                    cd src
                        
                        
                        
                           catkin_init_workspace
                
                
                
                    printf "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
                  
                    
                    
                                  cd ~/catkin_ws

                           
                           
                                  catkin_make
                            
                 
                 
                            source ~/catkin_ws/devel/setup.bash


Checking If the thing is correctly installed.

                           rosversion -d

                               gazebo -v


Now with the above steps the Ubuntu machine is ready with ROS installed for simulations. We will proceed by installing Ardupilot and MAVProxy


Installing Ardupilot and MAVProxy


Clone ArduPilot


In home directory:

         cd ~
        
         
         sudo apt install git 
        
        
        git clone https://github.com/ArduPilot/ardupilot.git
        
        
        cd ardupilot
        
        
        git checkout Copter-3.6
        
        
        git submodule update --init --recursive


Install dependencies:

         sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect


Use pip (Python package installer) to install mavproxy:

           sudo pip install future pymavlink MAVProxy


Open ~/.bashrc for editing:

            gedit ~/.bashrc


Add these lines to end of ~/.bashrc (the file open in the text editor):

    export PATH=$PATH:$HOME/ardupilot/Tools/autotest
    
    
    
    export PATH=/usr/lib/ccache:$PATH


Save and close the text editor.

     Reload ~/.bashrc:


         
         . ~/.bashrc


Run SITL (Software In The Loop) once to set params:

         cd ~/ardupilot/ArduCopter

               
          
          sim_vehicle.py -w



  

Intalling QGroundControl which provides full flight control and vehicle setup for ardupilot powered vehicles



Installing QGroundControl for Ubuntu Linux 18.04 LTS :

Add current user accout to dialout group and remove modemmanager

           sudo usermod -a -G dialout $USER

           
           sudo apt-get remove modemmanager


Download QGroundControl.AppImage

      wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage


Change permissions and run

           chmod +x ./QGroundControl.AppImage 

          
          
          ./QGroundControl.AppImage  (or double click)


Run SITL and connect with Q Ground

            cd ~/ardupilot/ArduCopter/
            
            
            
            
            sim_vehicle.py
            
 
 
Dependencies installation

Install mavros and mavlink from source:

                      cd ~/catkin_ws


                 wstool init ~/catkin_ws/src


             rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall


               rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall


                 wstool merge -t src /tmp/mavros.rosinstall


                   wstool update -t src


            rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y


catkin build Add a line to end of ~/.bashrc by running the following command:

             echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


update global variables

                   source ~/.bashrc


install geographiclib dependancy

            sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh



GroundController


Mission Planning through python code in 2D



Install the mono file which will help us to run few windows applications in linux

                                              sudo pip install mono 


Now go to this website and download the version 1.3.33 which gets saved in downloads and unzip it.

                           //firmware.us.ardupilot.org/Tools/MissionPlanner/archive/


On addition to this get these installed as well and also get the move.py file attached above in your home directory.

                                                sudo pip install mavproxy

                                        sudo pip install pymavlink==2.4.8


Now open a terminal and run :

                      dronekit-sitl copter
In another terminal run

              mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14549 --out 127.0.0.1:14450


Now cd to the location where you have unzipped the downloaded file and write this cmd in the third terminal

                               sudo mono MissionPlanner.exe


Now cd to the location where this move.py file got downloaded and run this cmd in the 4th terminal.

                                     python move.py


Wait for sometime the python file first gets executed in the terminal and then in the GroundController tab.
            
            
Now The procedure to download the necessary plugins for autonomous drone simulation in arduipilot



Install Gazebo plugin for APM (ArduPilot Master) :

                      cd ~
                      
                  git clone https://github.com/khancyr/ardupilot_gazebo.git

                  cd ardupilot_gazebo

Ubuntu 18.04 only checkout dev

                   git checkout dev


build and install plugin

          mkdir build
          
          cd build
           
           cmake ..
           
           make -j4

        sudo make install



                 echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc


Set paths for models:

       echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc


      . ~/.bashrc




NOw you are all set to simulate the drone we have added two python files in this repo one is the waypoint.py and another is the mov.py . The mov.py has pre defined waypoints and the quadcopter follows the direction while the waypoints.py asks the user to enter the desired no. of waypoints as well as their values.


Run Simulator

NOTE the iris_arducopter_runway is not currently working in gazebo11. The iq_sim worlds DO work


In one Terminal (Terminal 1), run Gazebo:


                        gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world


In another Terminal (Terminal 2), run SITL:

                    cd ~/ardupilot/ArduCopter/


                     sim_vehicle.py -v ArduCopter -f gazebo-iris --console
Fly drone through this command in another terminal with this the drone will follow the waypoints as instructed in this mov.py code.

                                             python mov.py 
                                             
Or you can fly drone through this command in another terminal with this the drone will follow the waypoints as instructed in this waypoints.py code.

                                             python waypoint.py                                            
  
  
  The detailed documentation of process of writting this python file is available at http://dronekit.io/
  
  Thanks :)

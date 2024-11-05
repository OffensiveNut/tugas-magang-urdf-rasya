# tugas-magang-urdf-rasya

### Cara Run Program tugas 1
1. cd _direktoriAnda_/tugas-magang-urdf-rasya
2. catkin_make
3. source devel/setup.bash #ATAU
   source devel/setup.zsh #(jika pake zsh)
4. roscore
5. roslaunch robot_description display.launch
6. rosrun arm_controller arm_controller

   ### Cara Run Program tugas 3
1. cd _direktoriAnda_/tugas-magang-urdf-rasya
2. catkin_make
3. source devel/setup.bash #ATAU
   source devel/setup.zsh #(jika pake zsh)
4. roscore #jika belum
5. roslaunch writer_robot_description display.launch
6. rosrun arm_controller penulis

alias alone='kill $(pgrep bash)'
alias falone='kill -9 $(pgrep bash)'

alias cbuild='colcon build --packages-select '  

alias roscd='cd ~/ASV_Loyola_US'
alias s='source ~/ASV_Loyola_US/install/setup.bash'

alias hearall='ros2 topic echo -f '
alias status='ros2 topic echo /ASV/status'

alias submoduloscd='cd /home/xavier/ASV_Loyola_US/src/asv_loyola_us/asv_loyola_us/submodulos'
alias mqttssh='ssh -i ~/AzureDronesKey.pem -fN -L 1883:127.0.0.1:1883 azuredrones@dronesloyolaus.eastus.cloudapp.azure.com'
alias nodered='ssh -i ~/AzureDronesKey.pem -fN -L 1880:127.0.0.1:1880 azuredrones@dronesloyolaus.eastus.cloudapp.azure.com'

alias check_ssh_running='ps -aux | grep "ssh -i"'

buall()
{
roscd;
colcon build --symlink-install;
}

sbuall()
{
buall;
s;
}

function roslaunch(){
ros2 launch sibiu_nano_p system.launch.py;
}

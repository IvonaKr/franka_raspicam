# paket franka_raspicam

Služi za povezivanje paketa `raspicam_node` i  `franka_gazebo_moveit`, te spremanje `.avi` datoteka i `.txt`.


Wired connection:
Na računalu je potrebno najprije stvoriti Wired Connection koje omogućuje povezivanje RaspberryPi s računalom uz pomoć ethernet veze.

Add new connection --> IPv4 --> Manual :

upisati ip adresu koja se razlikuje od ip  adrese Raspberry Pi-ja (192.168.21.114) :

npr : Adress :  192.168.21.118 Netmask : 255.255.255.0

Povezivanje na Rpi, potrebno unijeti lozinku :
```
ssh ivona-rpi@192.168.21.114 
```


Pokrenuti `roscore` na računalu.
 
Povezati Rpi s `ROS_MASTER` na računalu:

export ROS_MASTER_URI=http://192.168.21.118:11311/ && export ROS_IP=rpi && export ROS_HOSTNAME=rpi

Pokretanje `raspicam_noda` na Rpiju :
```
roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true
```
Skripte za kontrolu i skupljanje dataseta :
```
rosrun franka_raspicam collection.py
rosrun Franka_raspicam franka_images.py
```


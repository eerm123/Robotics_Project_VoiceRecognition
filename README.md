# Toomas/Ultron #

Voice-controlled robot


### Project members ###

Oliver Kane

Christopher Robin Subbi

Emil Eensaar

Eero Jürgenson


## Project Overview ##

Ülesande eesmärk on luua robot, kes käitub vastavalt suuliselt saadud käsklustele (vähemalt seitse, nt otse ja tagurpidi sõitmine jne). Robot on autonoomne, see tähendab, et on valmis täitma vastavaid käsklusi nii kaua, kuni robot on töös. Käsklused otsustame hiljem. Samas lõpetab robot viimase käskluse tegemise, kui sellele öelda “Stop!”, ja jääb seisma. Jagame neljase grupi pooleks: kaks tegelevad alamülesandega A ja teised kaks alamülesandega B (vastutatavad nähtaval Trellos). Alamülesanne A seisneb ROSi õppimises ja Robotondi liikuma panemises (vastavalt käsklustele). Alamülesande B sisu on Speech to Text API Pythoniga ühendamine, õppimine ja selle kasutamine, ning hääletuvastuskoodi kirjutamine. Oluline on, et öeldud käsklus tõlgendatakse korrektselt – robot saab sellest samasuguselt aru, nagu käskluse andnud inimene seda ütles. Kui mõlemad alamülesanded on tehtud, on tähtis kaks alamülesannet ühildada, et vastav robot täidaks püstitatud eesmärki. Esmalt tutvustavad kaks paari teineteisele, mis on Robotondi ja Text to Speech võimalused, seejärel teeme ajurünnaku ning mõtleme, kuidas riistvara ühildada tarkvaraga. Kirjutame koos ühise koodi ning teeme katsetusi. Kui arvame, et oleme suutelised looma ka hääletuvastamise ja selle tõlgendamise eesti keeles pärast ingliskeelset versiooni, proovime ka seda teostada. Lisaks seitsmele töötavale käsklusele võime neile veel uusi lisada.

## How to run the code ##

- Get a laptop with Ubuntu 20.04
- Download ROS Noetic
- Connect the laptop and robotont to the same hotspot (the hotspot has to have access to the internet for the code to run)
- Go into the ```/etc/hosts``` file and add the IP of the robotont to the laptop and the IP of the laptop to the robotont
- Establish a ROS_MASTER_URI on the laptop with the robotont and add it to the ```~/.bashrc``` file
```
export ROS_MASTER_URI=http://192.168.200.roboti_nr:11311/
```
- Create a catkin workspace (https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Add the following packages from https://github.com/robotont to the catkin workspace
```sh
robotont_laserscan_to_distance
robotont_nuc_description
robotont_description
robotont_navigation
robotont_gazebo
robotont_driver
robotont_demos
robotont_msgs
```
- Also add the ```robotont_laserscan_to_distance``` package to the catkin workspace on the robotont
- Beware of and fix any errors from the file ```laserscan_to_distance.py``` which is in the ```/scripts``` folder of the ```robotont_laserscan_to_distance``` package
- Add ```Toomas_Ultron.py``` & ```Toomas_SA.json``` to the ```/scripts``` folder in the ```/robotont_demos``` package
- Correct the file path for ```Toomas_SA.json``` on line 76 and the file path for the audio files on line 84 in ```Toomas_Ultron.py```
- Download pytorch on the laptop with ```pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu```
- Download pyglet on the laptop with ```pip install pyglet```
- Connect to the robotont and run the line ```roslaunch robotont_laserscan_to_distance distance_from_depth_image.launch```
- If the previous was done correctly then the code should work by opening a terminal on your laptop and running ``` rosrun robotont_demos Toomas_Ultron.py ```

## Table of Components ##

| Item | Link to the item | We will provide | Need from instructors | 3D print | Total |
| ---- | ---------------- | --------------: | --------------------: | -------: | :---: |
| ClearBot | https://clearbot.eu | 0 | 1 | 0 | 1 |
| Wireless Headphones |  | 1 | 0 | 0 | 1 |


## Trello ##

Link to Trello: https://trello.com/b/6teOlyng/toomas-ultron-beepboop


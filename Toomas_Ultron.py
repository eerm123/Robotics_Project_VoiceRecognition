#!/usr/bin/env python3
import speech_recognition as sr
import pyglet
import pyaudio
import time
import word2num
import numpy as np
from google.cloud import speech_v1p1beta1 as speech
from pydub import AudioSegment
import rospy
from geometry_msgs.msg import Twist
from robotont_laserscan_to_distance.msg import LaserScanSplit
import math
import threading
import random

##########################################################################################################################################################################################
distances = LaserScanSplit()
depth_sensor = [1,1,1]
avoid_obstacle = ""

#GLOBAL MUUTUJAD#


meter_words = ["minute","m","meter","meters","min","easter","minutes","letters","letter"]
move_words = ["movie","mo","moves"]
forward_words = ["farah","faraday","forwards","photo","photos","try 4 word"]
backward_words = ["backwards","bastards","bastard","backboard"]
degrees_words=["deg","degree","°"]
turn_words=["turner","turing","thorn","turned","stanton"]
turn_right_words=["android","staton"]
and_words=["then","standard","hand"]
zero = ["zero","sierra","cedar"]
one = ["one","juan","on","wand"]
two = ["two","too","do","to","duo"]
three = ["three","free","dream","tree"]
four = ["four","for","door","floor","flour"]
five = ["five","fine","fin","file","spy"]
six = ["six","sex","sax"]
seven = ["seven","evan"]
eight = ["eight","ate"]
nine = ["nine","ninth"]
stopp = ["sup"]
point=[" point "]
#turn_left_words=
triangle = ["drying","try and go","trying"]
square = ["squid","squad"]
holonomic = ["anomic","halo","palindromic","palindromic"]
sideways = ["suicide","suicideboys"]
diagonally = []
koik = {"meters":meter_words,
        "move":move_words,
        "forward":forward_words,
        "backward": backward_words,
        "degrees":degrees_words,
        "turn right":turn_right_words,
        "and":and_words,
        0:zero,
        1:one,
        2:two,
        3:three,
        4:four,
        5:five,
        6:six,
        7:seven,
        8:eight,
        9:nine,
        "stop":stopp,
        "triangle":triangle,
        "square":square,
        "sideways":sideways,
        "holonomic":holonomic,
        ".":point}

keel = "en-US"  # state#

state = "ON"

tasks=["move","drive","turn","triangle","square","circle"]

phrase_set = ["$ADDRESSNUM"]
phrase_set2 = ["move forward meters and turn degrees","move forward", "move backwards","meters","meter","degrees","turn right","turn left","then"]

key_path = "/home/toomas-admin/toomas-ultron/Toomas_SA.json"

liikumine = [[90,"right"],[90,"left"],[1,"forward"],[1,"backward"]]

stop = False

autonoomne = True

audio_failid = [
    "/home/toomas-admin/toomas-ultron/robotont_music/circle.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/forward.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/ready.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/speak_now.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/square.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/stopped.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/automatic.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/turning.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/backward.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/escape.mp3",
    "/home/toomas-admin/toomas-ultron/robotont_music/spinner.mp3"
    ]

toomas_audio = pyglet.media.Player()
toomas_audio.queue(pyglet.media.load(audio_failid[2]))
toomas_audio.play()

########################################################################################################################################################################################

velocity_publisher = rospy.Publisher(
    '/cmd_vel', Twist, queue_size=10)

vel_msg = Twist()
k2sud = ""

#delta kivi p6rand = 0.075

#Tyti libe p6rand = 0.06

#delta vaip = 0.1

#physicum kivi p6rand = 0.05

#Tamme gymna saali p6rand = 0.08

speed = 0.075


left_depth = []
center_depth = []
right_depth = []
left_average = 0
center_average = 0
right_average = 0

def scan_callback(data):
    global distances
    global depth_sensor
    global left_depth
    global center_depth
    global right_depth
    global left_average
    global center_average
    global right_average
    
    if len(left_depth) < 5:
        (left_depth).append(data.left_min)
    else:
        (left_depth).remove(left_depth[0])
        (left_depth).append(data.left_min)
    
    if len(center_depth) < 5:
        (center_depth).append(data.center_min)
    else:
        (center_depth).remove(center_depth[0])
        (center_depth).append(data.center_min)
        
    if len(right_depth) < 5:
        (right_depth).append(data.right_min)
    else:
        (right_depth).remove(right_depth[0])
        (right_depth).append(data.right_min)
    
    if len(center_depth) < 5:
        depth_sensor = [data.left_min, data.center_min, data.right_min]
    else:
        left_average = sum(left_depth)/len(left_depth)
        center_average = sum(center_depth)/len(center_depth)
        right_average = sum(right_depth)/len(right_depth)
        depth_sensor = [left_average, center_average, right_average]
    
    

def closing():

    # After the loop, stops the robot by setting all inputs to 0
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)

def spinner():
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[10]))
    toomas_audio.play()
    
    for i in range(6):
        for i in range(5):
            if stop == True:
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
                break
            else:
                vel_msg.angular.z = 5
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
        
        for i in range(5):
            if stop == True:
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
                break
            else:
                vel_msg.angular.z = -5
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
 
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    toomas_audio.delete()
    

def escape():
    global speed
    global depth_sensor
    global avoid_obstacle
    
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[9]))
    toomas_audio.play()
    
    for i in range(65):
        if stop == True:
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            break
        elif (depth_sensor[0] < 0.4) or (depth_sensor[1] < 0.4) or (depth_sensor[2] < 0.4):
            #print(depth_sensor)
            vel_msg.linear.x = 0
            if depth_sensor[0] > depth_sensor[2]:
                if avoid_obstacle == "left":
                    turn_degrees(90, "left")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "left"
                turn_degrees(90, "left")
            else:
                if avoid_obstacle == "right":
                    turn_degrees(90, "right")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "right"
                turn_degrees(90, "right")
        else:
            vel_msg.linear.x = speed
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    
    previous_speed = 0
    previous_speed = speed
    speed = 5
    
    for i in range(int(12000*(1/speed)*10)):
        if stop == True:
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            break
        elif (depth_sensor[0] < 0.4) or (depth_sensor[1] < 0.4) or (depth_sensor[2] < 0.4):
            #print(depth_sensor)
            vel_msg.linear.x = 0
            if depth_sensor[0] > depth_sensor[2]:
                if avoid_obstacle == "left":
                    turn_degrees(90, "left")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "left"
                turn_degrees(90, "left")
            else:
                if avoid_obstacle == "right":
                    turn_degrees(90, "right")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "right"
                turn_degrees(90, "right")
        else:
            vel_msg.linear.x = speed
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    speed = previous_speed
    toomas_audio.delete()

def drive_forward(distance):
    global depth_sensor
    global avoid_obstacle
    
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[1]))
    toomas_audio.play()
    
    for i in range(int(distance*(1/speed)*10)):
        if stop == True:
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            break
        elif (depth_sensor[0] < 0.4) or (depth_sensor[1] < 0.4) or (depth_sensor[2] < 0.4):
            #print(depth_sensor)
            vel_msg.linear.x = 0
            if depth_sensor[0] > depth_sensor[2]:
                if avoid_obstacle == "left":
                    turn_degrees(90, "left")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "left"
                turn_degrees(90, "left")
            else:
                if avoid_obstacle == "right":
                    turn_degrees(90, "right")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "right"
                turn_degrees(90, "right")
        else:
            vel_msg.linear.x = speed
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    toomas_audio.delete()

def drive_backward(distance):
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[8]))
    toomas_audio.play()
    
    for i in range(int(distance*(1/speed)*10)):
        if stop == True:
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            break
        else:
            vel_msg.linear.x = -speed
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    toomas_audio.delete()

def drive_sideways(distance, direction):
    global depth_sensor
    global avoid_obstacle
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[4]))
    toomas_audio.play()
    if direction == "left":
        direction = speed
    elif direction == "right":
        direction = -speed
    for i in range(int(distance*(1/speed)*10)):
        if stop == True:
            vel_msg.linear.y = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            break
        elif (depth_sensor[0] < 0.4) or (depth_sensor[1] < 0.4) or (depth_sensor[2] < 0.4):
            vel_msg.linear.y = 0
            if depth_sensor[0] > depth_sensor[2]:
                if avoid_obstacle == "left":
                    turn_degrees(90, "left")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "left"
                turn_degrees(90, "left")
            else:
                if avoid_obstacle == "right":
                    turn_degrees(90, "right")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "right"
                turn_degrees(90, "right")
        else:
            vel_msg.linear.y = direction
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
    vel_msg.linear.y = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    toomas_audio.delete()

def drive_diagonal(distance, direction, angle):
    global depth_sensor
    global avoid_obstacle
    distance_x = 0
    distance_y = 0
    
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[6]))
    toomas_audio.play()
    
    if distance == abs(distance):
        distance_x = speed
    else:
        distance_x = -speed
    if direction == "left":
        direction_y = speed
    elif direction == "right":
        direction_y = -speed

    for i in range(int(abs(distance)*(1/speed)*10)):
        if stop == True:
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            break
        elif (depth_sensor[0] < 0.4) or (depth_sensor[1] < 0.4) or (depth_sensor[2] < 0.4):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            if depth_sensor[0] > depth_sensor[2]:
                if avoid_obstacle == "left":
                    turn_degrees(90, "left")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "left"
                turn_degrees(90, "left")
            else:
                if avoid_obstacle == "right":
                    turn_degrees(90, "right")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "right"
                turn_degrees(90, "right")
        else:
            if distance_x > 0:
                vel_msg.linear.x = distance_x - angle*direction_y
            else:
                vel_msg.linear.x = distance_x + angle*direction_y
            vel_msg.linear.y = angle*direction_y
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    toomas_audio.delete()
    
def turn_degrees(degrees, direction):
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[7]))
    toomas_audio.play()
    degrees = int(degrees)
    if direction == "left":
        for i in range(int(degrees/5)):
            if stop == True:
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
                break
            else:
                vel_msg.angular.z = 0.1 + (speed/0.1)
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
        
    elif direction == "right":
        for i in range(int(degrees/5)):
            if stop == True:
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
                break
            else:
                vel_msg.angular.z = -0.1 - (speed/0.1)
                velocity_publisher.publish(vel_msg)
                rospy.sleep(0.1)
    
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    toomas_audio.delete()

def holonomic_square(distance, direction):
    drive_forward(distance)
    drive_sideways(distance, direction)
    drive_backward(distance)
    if direction == "left":
        direction = "right"
    elif direction == "right":
        direction = "left"
    drive_sideways(distance, direction)
    
def square(distance, direction):
    for i in range(4):
        drive_forward(distance)
        turn_degrees(90, direction)

def circle(length):
    global depth_sensor
    global avoid_obstacle
    
    
    toomas_audio = pyglet.media.Player()
    toomas_audio.queue(pyglet.media.load(audio_failid[0]))
    toomas_audio.play()
    
    offset = 4*(abs(1-length))/0.1
    for i in range(int((2.7*3.14*(2*length))*(1/speed))+int(offset)):
        if stop == True:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
            toomas_audio.delete()
            break
        elif (depth_sensor[0] < 0.4) or (depth_sensor[1] < 0.4) or (depth_sensor[2] < 0.4):
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            if depth_sensor[0] > depth_sensor[2]:
                if avoid_obstacle == "left":
                    turn_degrees(90, "left")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "left"
                turn_degrees(90, "left")
            else:
                if avoid_obstacle == "right":
                    turn_degrees(90, "right")
                    avoid_obstacle = ""
                else:
                    avoid_obstacle = "right"
                turn_degrees(90, "right")
        else:
            vel_msg.linear.x = speed*4
            vel_msg.angular.z = (1/length)*speed*4
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(0.1)
    toomas_audio.delete()
        
def triangle(length):
    drive_diagonal(length, "left", 30/90)
    drive_sideways(length*0.75, "right")
    drive_diagonal(-length, "left", 30/90)
    
# LISTEN GOOGLE-CLOUD-SPEECH-DETECT 
def google():
    global keel
    while True:
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("Speak now")
            
            toomas_audio = pyglet.media.Player()
            toomas_audio.queue(pyglet.media.load(audio_failid[3]))
            toomas_audio.play()
            
            audio = r.listen(source)
            try:
                #Kasutan AudioSegment libraryt (kuna cloud-speech tahab kasutada andmeid kindlas vormingus, kasutan default settinguid)
                audio_segment = AudioSegment(
                    audio.frame_data,
                    sample_width=audio.sample_width,
                    frame_rate=audio.sample_rate,
                    channels=1
                )                 
                
                #Convertin audiot raw_dataks
                audio_content = audio_segment.raw_data
                
                print("audio:", len(audio_content))
                #Saan lõpuks kasutada Google cloudi configi
                client = speech.SpeechClient.from_service_account_file(key_path)
                config = speech.RecognitionConfig(
                    encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                    sample_rate_hertz=audio.sample_rate,
                    language_code=keel,
                    speech_contexts=[{"phrases": phrase_set, "boost": 20},
                                     {"phrases": phrase_set2, "boost": 15}],
                )
                response = client.recognize(config=config, audio={"content": audio_content})
                for result in response.results:
                    print("Transcript: {}".format(result.alternatives[0].transcript))
                    return result.alternatives[0].transcript
            except sr.UnknownValueError:
                print("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))
                
            
#########################################################################################################################################################
#PANEB LISTI KOKKU SONEKS
def tee_sone(hulk): 
    a=""
    for i in hulk:
       a+=f"{i}"+" "
    return a
#########################################################################################################################################################
#ASENDAB SONU LISTIS
def kontrolli_sona(loetud_lause,control_group,key): #asendab error sonad kasus
    for i in range(len(loetud_lause)):
        for j in range(len(control_group)):
            #print(control_group[j],loetud_lause[i])
            if control_group[j] == loetud_lause[i]:
                loetud_lause[i]=key
    return loetud_lause 
########################################################################################################################################################
#KAIB KOIK ERRORI LISTID LABI JA ASENDAB ERROR SONAD
def kogu_kontroll(lause_list,koik): 
    keys = list(koik.keys())
    n=0
    for i in range(len(keys)):
        replace = kontrolli_sona(lause_list,koik[keys[i]],keys[i])
        n+=1
    replace = tee_sone(replace)
    print(f"kogu_kontroll : {replace}")
    return replace 
########################################################################################################################################################
#LEIAB MEETRITE JA KRAADIDE ARVULISED ARGUMENDID
def find_argument(puhas_sone): #leiab meetrite ja kraadide argumendid
    puhas_sone= puhas_sone.split() 
    number_arguments={}
    for i in ["meters","degrees"]:#otsib kasust meetri ja kraadi indexi ja kirjutab numbri argumendi dicti 
        for j in puhas_sone:
            if i == j:
                try:
                    number_arguments[i]= float(puhas_sone[puhas_sone.index(j)-1])
                except ValueError:
                    if i == "degrees":
                        number_arguments[i] = float(45.0)
                    elif i == "meters":
                        number_arguments[i] = float(1.0)
                    else:
                        pass
    print(f"find_argument : {number_arguments}")
    return number_arguments #tagastab dicti
#######################################################################################################################################################
#JAGAB TASKID LISTI                
def jaga_taskid(sone):# jagab kasud listi
    
    if " and " in sone:
        a= sone.split()
        index = a.index("and")
        b= sone.split(f" {a[index]}")
        print(f"jaga_taskid : {b}")
        return b
    else:
        return [sone]   
#######################################################################################################################################################
#LEIAB ROBOTI KASUD
            
def robot_control(command):#otsib roboti kasklused
    global koik, stop, autonoomne, k2sud
    arv=command.split()# teeb saadud sone listiks
    final = kogu_kontroll(arv,koik)# vahetab error sonad oigeteks
    multiple=jaga_taskid(final)# jagab sone listiks kasu haaval
    orderid_robotile=[]
    for juhised in multiple:# votab taskide listist yhe taski korraga
        #print(juhised)
        numbers=find_argument(juhised)
        if "stop" in juhised:
                print("   stop    !")
                stop = True
                print(stop)
                break
        elif "auto" in juhised:
            autonoomne = True
            break
        elif 'diagonal' in juhised:
            if 'forward' in juhised:
                suund='forward'
            elif 'backward' in juhised:
                suund = 'backward'
            else:
                suund = 1.0
            if 'right' in juhised:
                pool = 'right'
            else:
                pool = 'left'
            if 'meters' in juhised:
                leng = numbers['meters']
            else:
                leng = 1.0
            if 'degrees' in juhised:
                kraad=numbers['degrees']
            else:
                kraad = 45.0
            if suund == 'backward':
                leng=leng*-1.0
            
            print(f'suund {suund} + pool{pool} +diagonal+ meter{leng} + deg{kraad}')
            orderid_robotile.append([leng,'diagonal',pool,kraad/90])
            suund = 0
            pool=0
            leng=0
            kraad=0
        elif "sideways" in juhised:
            if 'right' in juhised:
                pool = 'right'
            else:
                pool = 'left'
            if "meters" in list(numbers.keys()):
                print(f"Robot drives sideways {numbers['meters']}")
                orderid_robotile.append([numbers["meters"],"sideways", pool]) #lisab kasu argumendi listi kaskude listi
            else:
                orderid_robotile.append([1.0,"sideways", pool])
        elif "triangle" in juhised: 
            if "meters" in list(numbers.keys()):
                print(f"Robot makes a triangle border len {numbers['meters']}")
                orderid_robotile.append([numbers['meters'],"triangle"]) #lisab kasu argumendi listi kaskude listi
            else:
                orderid_robotile.append([1.0,"triangle"])
        elif "holonomic" in juhised:
            if 'right' in juhised:
                pool = 'right'
            else:
                pool = 'left'
            if "meters" in list(numbers.keys()):
                print(f"Robot makes a holonomic squre border len {numbers['meters']}")
                orderid_robotile.append([numbers['meters'],"holonomic", pool]) #lisab kasu argumendi listi kaskude listi
            else:
                orderid_robotile.append([1.0,"holonomic", "left"])
        elif "square" in juhised:
            if 'right' in juhised:
                pool = 'right'
            else:
                pool = 'left'
            if "meters" in list(numbers.keys()):
                print(f"Robot makes a squre border len {numbers['meters']}")
                orderid_robotile.append([numbers['meters'],"square", pool]) #lisab kasu argumendi listi kaskude listi
            else:
                orderid_robotile.append([1.0,"square", "left"])
        elif "circle" in juhised:
            if "meters" in list(numbers.keys()):
                print(f"Robot makes a circle border len {numbers['meters']}")
                orderid_robotile.append([numbers['meters'],"circle"]) #lisab kasu argumendi listi kaskude listi
            else:
                orderid_robotile.append([1.0,"circle"])
        elif "forward" in juhised:
            if "meters" in list(numbers.keys()):                
                print(f"Robot moves forward {numbers['meters']} meters")
                orderid_robotile.append([numbers["meters"],"forward"]) #lisab kasu argumendi listi kaskude listi
            else:
                orderid_robotile.append([1.0,"forward"])
        elif "backward" in juhised:
            if "meters" in list(numbers.keys()):
                print(f"Robot moves backward {numbers['meters']} meters")
                orderid_robotile.append([numbers["meters"],"backward"]) #lisab kasu argumendi listi kaskude listi
            else:
                orderid_robotile.append([1.0,"backward"])
        elif "right" in juhised:
            if "degrees" in list(numbers.keys()):# numbri kontroll
                print(f"Robot turn`s {numbers['degrees']} right")
                orderid_robotile.append([numbers['degrees'],"right"]) #lisab kasu argumendi listi kaskude listi
            else:
                print(f"Robot turn`s 90 right")
                orderid_robotile.append([90.0,"right"]) #lisab kasu argumendi listi kaskude listi
        elif "left" in juhised:
            if "degrees" in list(numbers.keys()):
                print(f"Robot turn`s {numbers['degrees']} left")
                orderid_robotile.append([numbers['degrees'],"left"]) #lisab kasu argumendi listi kaskude listi
            else:
                print(f"Robot turn`s 90 left")
                orderid_robotile.append([90.0,"left"]) #lisab kasu argumendi listi kaskude listi
        elif "turn" in juhised:
            if "degrees" in list(numbers.keys()):# numbri kontroll
                print(f"Robot turn`s {numbers['degrees']} right")
                orderid_robotile.append([numbers['degrees'],"right"]) #lisab kasu argumendi listi kaskude listi
        elif "escape" in juhised:
            print("run")
            orderid_robotile.append("escape")
        elif "spin" in juhised:
            print("chipi chipi chapa chapa")
            orderid_robotile.append("spin")

    
    print(orderid_robotile)
    return orderid_robotile #tagastab kaskude listi

            

def move():
    global k2sud, liikumine, autonoomne, stop

    # Start a new node
    rospy.init_node('robotont_velocity_publisher', anonymous=True)
    rospy.Subscriber('/scan_to_distance', LaserScanSplit, scan_callback)

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    

    while not rospy.is_shutdown():

        # For finding the ideal speed for different floor types
        """
        for i in range(20):
            vel_msg.linear.x = 1
            vel_msg.linear.y = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rospy.sleep(0.1)
        """
        if stop == False:
            if autonoomne == True:
                drive_forward(5)
                #spinner()
                #k2sud=[liikumine[random.randint(0,3)]]   
                #teeb autonoomset liikumist
            for k2sk in k2sud:
                print(k2sk)
                if "sideways" in k2sk:
                    drive_sideways((float(k2sk[0])), str(k2sk[2]))
                if "circle" in k2sk:
                    circle(float(k2sk[0]))
                if "diagonal" in k2sk:
                    drive_diagonal(float(k2sk[0]), str(k2sk[2]), float(k2sk[3]))
                if "holonomic" in k2sk:
                    holonomic_square((float(k2sk[0])), str(k2sk[2]))
                if "square" in k2sk:
                    square((float(k2sk[0])), str(k2sk[2]))
                if "triangle" in k2sk:
                    triangle(float(k2sk[0]))
                if "forward" in k2sk:
                    drive_forward(float(k2sk[0]))
                if "backward" in k2sk:
                    drive_backward(float(k2sk[0]))
                if "left" in k2sk:
                    turn_degrees(float(k2sk[0]), "left")
                if "right" in k2sk:
                    turn_degrees(float(k2sk[0]), "right")
                if "escape" in k2sk:
                    escape()
                if "spin" in k2sk:
                    spinner()
                
            k2sud = ""
        else:
            toomas_audio.delete()
            print("stop!")
            autonoomne = False
            stop = False
        

        # For testing all inputs
        """
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.1)
        """

        

def main():
    global k2sud
    while True:
        listen = google()
        käsklus = listen.lower()
        k2sud = robot_control(käsklus)

if __name__ == '__main__':
    threading.Thread(target=main).start()
    rospy.on_shutdown(closing)
    while True:
        move()
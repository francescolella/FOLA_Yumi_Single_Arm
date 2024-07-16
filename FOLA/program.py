#Implementazione della pagina dedicata alla programmazione.
#-Muovi fino a un punto
#-Registrazione movimento
#-Apri pinza
#-Chiudi pinza
#-Ripeti più volte
#Gestione scrittura su file, cambio ordine delle funzioni e aggiornamento

import kivy
kivy.require('1.0.5')

import rospy
from kivy.uix.floatlayout import FloatLayout
from kivy.properties import ObjectProperty
from std_msgs.msg import Int8, String, Bool
from kivy.lang import Builder
import roslaunch
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.popup import Popup
from kivy.uix.screenmanager import Screen
from kivy.clock import Clock
import math
import os

pub_free_drive=rospy.Publisher('freeDrive',Int8,queue_size=1)
pub_gripper=rospy.Publisher('gripper',Int8,queue_size=1)
pub_start_record=rospy.Publisher('/record/start_python', String, queue_size=1)
pub_stop_record=rospy.Publisher('/record/stop', Bool, queue_size=1)
pub_desiderate_joint = rospy.Publisher('desiderate_joint', JointState, queue_size=1)
pub_switch_controller = rospy.Publisher('switch_controller', Bool, queue_size=1)
pub_pose_des=rospy.Publisher('pose_des', Pose, queue_size=1)
pub_controller_cartesian=rospy.Publisher('controller_cartesian', Bool, queue_size=1)

Builder.load_file('program.kv')

def cancella_blocco(count, file_path, file_name):
    #cancella blocco
    with open(os.path.join(file_path, file_name), 'r') as original:
        with open(os.path.join(file_path, 'provvisorio.txt'), 'w') as new:
            text = original.readline()
            while(text!=str(count) +'_start\n'):
                new.write(text)
                text = original.readline()
            while(text!=str(count) +'_stop\n'):
                text = original.readline()
            text = original.readline()
            while(text!=''):
                new.write(text)
                text = original.readline()
            os.rename(file_path + '/provvisorio.txt' , file_path + '/' + file_name)

def callback_JointState(data):
    global JointState_pos_now
    JointState_pos_now = data.position
    for i in range(len(JointState_pos_now)):
        if((abs(JointState_pos_now[i] - min_Joint_pos[i]) < 2/180*math.pi) or (abs(JointState_pos_now[i] - max_Joint_pos[i]) < 2/180*math.pi)):
                global fine_corsa
                fine_corsa = True

def callback_move_result(data):
    global move_result
    move_result = data.data
    #print(move_result)  #debug
    #print('dato arrivato') #debug

def callback_bag_name(data):
    global bag_name
    bag_name = data.data
    #print("Nome intero del file bag") #debug
    #print(bag_name) #debug

def callback_xyz(data):
    global xyz_actual
    xyz_actual = data

sub_joint_state = rospy.Subscriber('joint_states', JointState, callback_JointState)
sub_name_bag = rospy.Subscriber('record/bag_name_python', String, callback_bag_name)
sub_move_result = rospy.Subscriber('/move_result', Int8, callback_move_result)
sub_xyz_actual = rospy.Subscriber('pose_cartesian', Pose, callback_xyz)

class ChooseDialog(FloatLayout):
    point_to_point = ObjectProperty(None)
    recorder_track = ObjectProperty(None)
    open_pinza = ObjectProperty(None)
    close_pinza = ObjectProperty(None)
    ripeti = ObjectProperty(None)

class PPDialog(FloatLayout):
    update = ObjectProperty(None)
    count = ObjectProperty(None)
    savePP = ObjectProperty(None)
    def freeDriver(self):
        #print('freeDrive') #debug
        pub_free_drive.publish(1)

    def allinea(self):
        pub_free_drive.publish(0)
        pub_switch_controller.publish(True)
        pub_controller_cartesian.publish(True)
        pose_des = Pose()
        #print(type(xyz_actual.orientation.y)) #debug
        while(not(abs(xyz_actual.orientation.z)<0.01 and abs(xyz_actual.orientation.w)<0.01)):
            pose_des.position.x = xyz_actual.position.x
            pose_des.position.y = xyz_actual.position.y
            pose_des.position.z = xyz_actual.position.z
            pose_des.orientation.x = xyz_actual.orientation.x
            pose_des.orientation.y = xyz_actual.orientation.y
            pose_des.orientation.z = 0.0
            pose_des.orientation.w = 0.0
            pub_pose_des.publish(pose_des)
        self.ids.save_button.disabled = False
        pub_switch_controller.publish(False)
        pub_controller_cartesian.publish(False)

    def disable_save_button(self):
        self.ids.save_button.disabled = True

class RTDialog(FloatLayout):
    saveRT = ObjectProperty(None)
    stopRT = ObjectProperty(None)
    file_name = ObjectProperty(None)
    update = ObjectProperty(None)
    count = ObjectProperty(None)
    RT_page = ObjectProperty(None)
    def freeDriver(self):
        #print('freeDrive') #debug
        pub_free_drive.publish(1)

    def start_recorder(self):
        self.ids.button_recorder_start.disabled = False
        Clock.schedule_interval(self.control_joint_callback, 0.1)

    def enable_stop_button(self):
        pub_start_record.publish(self.file_name)
        self.ids.button_recorder_stop.disabled = False

    def stop_fine_corsa_control(self):
        Clock.unschedule(self.control_joint_callback)

    def control_joint_callback(self, dt):
        global fine_corsa
        if(fine_corsa):
            #print('sono a fine corsa per cui devo rifare tutto') #debug
            self.stopRT(self.update, self.count)
            fine_corsa = False
            Clock.unschedule(self.control_joint_callback)
            content = ErrorDialog(close_popup_error = self.close_popup_error)
            self.popup_error = Popup(title= 'scelta operazione', content=content,
                size_hint=(1, 1))
            self.popup_error.open()
        #print('fine corsa = '+ str(fine_corsa)) #debug
        
    def close_popup_error(self):
        self.popup_error.dismiss()
        self.RT_page(True, self.count)
   
class ErrorDialog(FloatLayout):
    close_popup_error = ObjectProperty(None)

class STARTDialog(FloatLayout):
    saveSTART = ObjectProperty(None)
    update = ObjectProperty(None)
    def freeDriver(self):
        #print('freeDrive')
        pub_free_drive.publish(1)

class RIPETIDialog(FloatLayout):
    saveRIPETI = ObjectProperty(None)
    count = ObjectProperty(None)
    update = ObjectProperty()
    num_ripeti = 1 #default
      
class CustomwidgetPP(BoxLayout):
    PP_page = ObjectProperty(None)
    count = ObjectProperty(None)
    file_path = ObjectProperty(None)
    file_name = ObjectProperty(None)
    move_to_up = ObjectProperty(None)
    move_to_down = ObjectProperty(None)
    create_new_program_page = ObjectProperty(None)
    Test = ObjectProperty(None)
    def remove(self):
        cancella_blocco(self.count, self.file_path, self.file_name)
        #print('rimuovo il blocco numero' + str(self.count)) #sono riuscito a prendere il numero del blocco precedente

class CustomwidgetRT(BoxLayout):
    RT_page = ObjectProperty(None)
    count = ObjectProperty(None)
    file_path = ObjectProperty(None)
    file_name = ObjectProperty(None)
    move_to_up = ObjectProperty(None)
    move_to_down = ObjectProperty(None)
    create_new_program_page = ObjectProperty(None)
    Test = ObjectProperty(None)
    def remove(self):
        cancella_blocco(self.count, self.file_path, self.file_name)
        #print('rimuovo il blocco numero' + str(self.count)) #debug

class CustomwidgetOP(BoxLayout):
    count = ObjectProperty(None)
    file_path = ObjectProperty(None)
    file_name = ObjectProperty(None)
    move_to_up = ObjectProperty(None)
    move_to_down = ObjectProperty(None)
    create_new_program_page = ObjectProperty(None)
    Test = ObjectProperty(None)
    def remove(self):
        cancella_blocco(self.count, self.file_path, self.file_name)

class CustomwidgetCP(BoxLayout):
    count = ObjectProperty(None)
    file_path = ObjectProperty(None)
    file_name = ObjectProperty(None)
    move_to_up = ObjectProperty(None)
    move_to_down = ObjectProperty(None)
    create_new_program_page = ObjectProperty(None)
    Test = ObjectProperty(None)
    def remove(self):
        cancella_blocco(self.count, self.file_path, self.file_name)

class CustomwidgetRipetiStart(BoxLayout):
    ripeti_page = ObjectProperty(None)
    count = ObjectProperty(None)
    file_path = ObjectProperty(None)
    file_name = ObjectProperty(None)
    move_to_up = ObjectProperty(None)
    move_to_down = ObjectProperty(None)
    create_new_program_page = ObjectProperty(None)
    num_ripeti = ObjectProperty(None)
    def remove(self):
        cancella_blocco(self.count, self.file_path, self.file_name)
        cancella_blocco(self.count + 1, self.file_path, self.file_name)        

class CustomwidgetRipetiStop(BoxLayout):
    count = ObjectProperty(None)
    file_path = ObjectProperty(None)
    file_name = ObjectProperty(None)
    move_to_up = ObjectProperty(None)
    move_to_down = ObjectProperty(None)
    create_new_program_page = ObjectProperty(None)        

class New_program(Screen):
    #la variabile count conterà quante volte premo il bottone per inserire i widget in modo tale da dare un numero ad ogni widget e in caso di modifica poter cambiare ordine ecc.
    create_new_program_page = ObjectProperty(None)
    count = 0
    file_path = ''  
    file_name = ''  
    global max_Joint_pos
    global min_Joint_pos
    global fine_corsa
    fine_corsa = False
    max_Joint_pos = [2.94960643587, 0.76794487087, 2.94960643587, 1.3962634016, 5.06145483078, 2.40855436775, 3.99680398707]
    min_Joint_pos = [-2.94960643587,-2.51327412287,-2.94960643587,-2.16420827247,-5.06145483078, -1.53588974176, -3.99680398707]
   
    def corret_file(self):
        name=self.file_name.split('/')
        self.file_name=name[-1]
        #print(self.file_name) #debug

    def add_customwidget(self):
        content = ChooseDialog(point_to_point=self.point_to_point, recorder_track=self.recorder_track, 
                               open_pinza=self.open_pinza, close_pinza=self.close_pinza, ripeti=self.ripeti)
        self._popup = Popup(title= 'scelta operazione', content=content,
                            size_hint=(0.9, 0.9))
        self._popup.open()

    def point_to_point(self):
        self._popup.dismiss()
        self.PP_page(False, self.count)

    def recorder_track(self):
        #print('ok recorder track') #debug
        self._popup.dismiss()
        self.RT_page(False, self.count)

    def ripeti(self):
        self._popup.dismiss()
        self.ripeti_page(False, self.count)
    
    def ripeti_page(self, update, count):
        ripeti_page = RIPETIDialog(saveRIPETI = self.saveRIPETI, update=update, count=count)
        self.RIPETI_popup = Popup(title= 'RIPETI', content=ripeti_page,
                            size_hint=(0.9, 0.9))
        self.RIPETI_popup.open()
    
    def saveRIPETI(self, num_ripeti, update, count):
        #print("sono in ripeti e passo allo start il count: " + str(self.count)) #debug
        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write(str(count) + "_start\nFOR_start:\n" + str(num_ripeti) + "\n" + str(count) + "_stop\n"+ str(count + 1) + "_start\nFOR_stop:\n"+str(count + 1) + "_stop\n")
        if(update==True):
            self.sostituzione(count)
        else:
            customwidget = CustomwidgetRipetiStart(count = count,  file_path=self.file_path, file_name=self.file_name,
                                        move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, num_ripeti = num_ripeti,
                                        ripeti_page = self.ripeti_page)
            self.ids.container.add_widget(customwidget)
            customwidget = CustomwidgetRipetiStop(count = count + 1,  file_path=self.file_path, file_name=self.file_name,
                                        move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page)
            self.ids.container.add_widget(customwidget)
            self.count = self.count +1
            #print("print SAVE RIPETI") #debug
        self.RIPETI_popup.dismiss()  
    
    def START_page(self, update):
        START_page = STARTDialog(saveSTART = self.saveSTART, update=update)
        self.START_popup = Popup(title= 'scelta punto di START', content=START_page,
                            size_hint=(0.9, 0.9))
        self.START_popup.open()

    def saveSTART(self, update):
        pub_free_drive.publish(0)
        #print("print SAVE START") #debug
        self.write_JointState(JointState_pos_now)
        if(update):
            self.update_start()
        self.START_popup.dismiss()

    def PP_page(self, update, count):
        pp_page = PPDialog(savePP = self.savePP, update = update, count=count)
        self.PP_popup = Popup(title= 'scelta punto di arrivo', content=pp_page,
                            size_hint=(0.9, 0.9))
        self.PP_popup.open()
        #print('ok poin_to_point') #debug
    
    def savePP(self, update, count):
        pub_free_drive.publish(0)
        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write(str(count) + "_start\nPP:\n")
        self.write_JointState(JointState_pos_now)
        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write(str(count)+'_stop\n')
        self.PP_popup.dismiss()
        if(update==True):
            self.sostituzione(count)
        else:
            customwidget = CustomwidgetPP(PP_page = self.PP_page, count = self.count, file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
            self.ids.container.add_widget(customwidget)

    def RT_page(self, update, count):
        RT_page = RTDialog(saveRT = self.saveRT, stopRT = self.stopRT, file_name = self.file_name, update = update, count=count, RT_page = self.RT_page)
        self.RT_popup = Popup(title= 'scegli il percorso da ripetere', content=RT_page,
                            size_hint=(0.9, 0.9))
        self.RT_popup.open()
        #print('ok poin_to_point') #debug

    def saveRT(self):
        self.JointState_pos = JointState_pos_now

    def stopRT(self, update, count):
        pub_free_drive.publish(0)
        pub_stop_record.publish(True)
        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write(str(count) + "_start\nRT:\n") 
        self.write_JointState(self.JointState_pos)
        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write(bag_name + '\n')
            stream.write(str(count)+'_stop\n') 
        self.RT_popup.dismiss()
        if(update==True):
            self.sostituzione(count)
        else:
            customwidget = CustomwidgetRT(RT_page= self.RT_page, count = self.count, file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
            self.ids.container.add_widget(customwidget)
            
    def write_JointState(self, JointState_pos):
        dim = len(JointState_pos)
        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write('[')
            for i in range (dim - 1):
                stream.write(str(JointState_pos[i]) +  ', ')
            stream.write(str(JointState_pos[dim-1]))
            stream.write(']\n')
    
    def close_pinza(self):
        customwidget = CustomwidgetCP(count = self.count,  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
        self.ids.container.add_widget(customwidget)

        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write(str(self.count) + "_start\nCP:\n" + str(self.count) +'_stop\n')   
        self._popup.dismiss()

    def open_pinza(self):
        customwidget = CustomwidgetOP(count = self.count,  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
        self.ids.container.add_widget(customwidget)
        self._popup.dismiss()
        with open(os.path.join(self.file_path, self.file_name), 'a') as stream:
            stream.write(str(self.count) + "_start\nOP:\n" + str(self.count) +'_stop\n')   
    
    def sostituzione(self, count):
        #sostituzione dopo aver fatto update
        #print("il blocco da sostituire ha il numero "+ str(count) ) #debug
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo.txt'), 'a') as new:
                text = original.readline()
                while(text!=str(count)+'_start\n'):
                    new.write(text)
                    text = original.readline()
                text = original.readline()
                while(text!=str(count)+'_start\n'):
                    text = original.readline()
                new.write(text)
                text = original.readline()
                while(text!=str(count)+'_stop\n'):
                    new.write(text)
                    text = original.readline()
                new.write(text)
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo.txt'), 'a') as new:
                text = original.readline()
                while(text!=str(count)+'_stop\n'):
                    text = original.readline()
                text = original.readline()
                while(text!=str(count)+'_start\n'):
                    new.write(text)
                    text = original.readline()
        os.rename(self.file_path + '/temporaneo.txt',self.file_path + '/' + self.file_name)

    def read_program(self):
        #print("Ho iniziato la read del programma e count:" + str(self.count)) #debug
        with open(os.path.join(self.file_path, self.file_name), 'r') as stream:
            text=stream.readline()
            last_text=''
            max = 0
            while(text!=''):
                if(text=='PP:\n'):
                    #print('inserisco blocco PP') #debug
                    num=last_text.split('_')
                    #print('il blocco PP ha numero '+str(num[0])) #debug
                    customwidget = CustomwidgetPP(PP_page = self.PP_page, count = int(num[0]),  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
                    self.ids.container.add_widget(customwidget)
                    if(int(num[0])>max): max= int(num[0])
                elif(text=='RT:\n'):
                    num=last_text.split('_')
                    #print('il blocco RT ha numero '+str(num[0])) #debug
                    customwidget = CustomwidgetRT(RT_page= self.RT_page, count = int(num[0]),  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
                    self.ids.container.add_widget(customwidget)
                    if(int(num[0])>max): max= int(num[0])
                elif(text=='OP:\n'):
                    num=last_text.split('_')
                    #print('il blocco OP ha numero '+str(num[0])) #debug
                    customwidget = CustomwidgetOP(count = int(num[0]),  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
                    self.ids.container.add_widget(customwidget)
                    if(int(num[0])>max): max= int(num[0])
                elif(text=='CP:\n'):
                    num=last_text.split('_')
                    #print('il blocco CP ha numero '+str(num[0])) #debug
                    customwidget = CustomwidgetCP(count = int(num[0]),  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, Test = self.Test)
                    self.ids.container.add_widget(customwidget)
                    if(int(num[0])>max): max= int(num[0])
                elif(text=='FOR_start:\n'):
                    num=last_text.split('_')
                    #print('il blocco FOR_start ha numero '+str(num[0])) #debug
                    text=stream.readline()
                    rip = int(text)
                    customwidget = CustomwidgetRipetiStart(count = int(num[0]),  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page, num_ripeti=rip,
                                      ripeti_page = self.ripeti_page)
                    self.ids.container.add_widget(customwidget)
                    if(int(num[0])>max): max= int(num[0])
                elif(text=='FOR_stop:\n'):
                    num=last_text.split('_')
                    #print('il blocco FOR_stop ha numero '+str(num[0])) #debug
                    customwidget = CustomwidgetRipetiStop(count = int(num[0]),  file_path=self.file_path, file_name=self.file_name,
                                      move_to_up = self.move_to_up, move_to_down=self.move_to_down, create_new_program_page = self.create_new_program_page)
                    self.ids.container.add_widget(customwidget)
                    if(int(num[0])>max): max= int(num[0])
                last_text=text
                text=stream.readline()
            self.count = max
            #print('il massimo è: '+ str(self.count)) #debug
    
    def play(self, file_path, file_name):
        with open(os.path.join(file_path, file_name), 'r') as stream:
            text=stream.readline()
            while(text!=''):
                if(text=='Start:\n'):
                    text=stream.readline()
                    self.move_to_point(text)
                if(text=='PP:\n'):
                    text=stream.readline()
                    self.move_to_point(text)
                elif(text=='RT:\n'):
                    text=stream.readline()
                    self.move_to_point(text)
                    text=stream.readline()
                    pub_switch_controller.publish(True)
                    text = text.split('\n')
                    self.do_launch(text[0])
                    #print('fai partire il launch file: ' + text[0]) #debug
                    pub_switch_controller.publish(False)
                elif(text=='OP:\n'):
                    #print('Open pinza') #debug
                    pub_gripper.publish(False)
                elif(text=='CP:\n'):
                    #print('close Pinza') #debug
                    pub_gripper.publish(True)
                elif(text=='FOR_start:\n'):
                    text=stream.readline()
                    rip=int(text)
                    text=stream.readline()
                    num = text.split('_')
                    num_f = int(num[0])
                    with open(os.path.join(file_path, 'provvisorio'+ num[0] +'.txt'), 'w') as new:
                        text = stream.readline()
                        while(text != (str(num_f + 1)+'_start\n')):
                            new.write(text)
                            text=stream.readline()
                    for i in range(rip):
                        self.play(file_path, 'provvisorio'+num[0]+'.txt')
                        i= i+1
                    os.remove(file_path + '/provvisorio'+num[0]+'.txt')
                text=stream.readline()

    def move_to_point(self, text):
        joint_pub = JointState()
        joint_pub.position =eval(text)
        global move_result
        move_result = 0
        pub_desiderate_joint.publish(joint_pub)
        #print(joint_pub) #debug
        while(move_result==0):
            pass

    def do_launch(self, bag_name):
        cli_args = ['../tesi4_ws/src/bag_reader/bag_reader.launch','file:="'+ bag_name+'"']
        #print('file:="'+ bag_name+'"') #debug
        roslaunch_args = cli_args[1:]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()
        try:
            launch.spin()
        finally:
            launch.shutdown()     
        pass
    
    def move_to_up(self, count):
        #print("move_to_up") #debug
        #sposta blocco più sopra
        num_c =0
        #print("il blocco da spostare su, ha il numero "+ str(count) ) #debug
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo.txt'), 'w') as new:
                text = original.readline()
                while(text!=str(count) +'_start\n'):
                    text_prec = text
                    text = original.readline()
                #prendere il numero del blocco precedente
                num=text_prec.split('_')
                num_c = num[0]
                #print(num_c)  #debug
                while(text!= str(count) + '_stop\n'):
                    new.write(text)
                    text = original.readline()
                new.write(text)
                #ho scritto su un file la parte che va spostata
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo_1.txt'), 'w') as new:
                text = original.readline()
                while(text!=str(count)+ '_start\n'):
                    new.write(text)
                    text = original.readline()
                while(text!= str(count) + '_stop\n'):
                    text = original.readline()
                text = original.readline()
                while(text!=''):
                    new.write(text)
                    text = original.readline()

        with open(os.path.join(self.file_path, 'temporaneo_1.txt'), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo_2.txt'), 'w') as new:
                text = original.readline()
                while(text!=(num_c + '_start\n')):
                    new.write(text)
                    text = original.readline()
                with open(os.path.join(self.file_path, 'temporaneo.txt'), 'r') as temp:
                    text_tempo = temp.readline()
                    while(text_tempo!=''):
                        new.write(text_tempo)
                        text_tempo = temp.readline()
                while(text!=''):
                    new.write(text)
                    text = original.readline()
        os.rename(self.file_path + '/temporaneo_2.txt',self.file_path + '/' + self.file_name)
        os.remove(self.file_path + '/temporaneo_1.txt')
        os.remove(self.file_path + '/temporaneo.txt')
        self.create_new_program_page()
        
    def move_to_down(self, count):
        #print("move_to_down") #debug
        #sposta blocco più sotto
        num_c =0
        #print("il blocco da spostare giu, ha il numero "+ str(count) ) #debug
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo.txt'), 'w') as new:
                text = original.readline()
                while(text!=str(count)+'_start\n'):
                    text = original.readline()
                while(text!= str(count)+'_stop\n'):
                    new.write(text)
                    text = original.readline()
                new.write(text)
                #prendere il numero del blocco precedente
                text = original.readline()
                num=text.split('_')
                num_c = num[0]
                #print(num_c) 
                #ho scritto su un file la parte che va spostata
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo_1.txt'), 'w') as new:
                text = original.readline()
                while(text!=str(count)+ '_start\n'):
                    new.write(text)
                    text = original.readline()
                while(text!= str(count) + '_stop\n'):
                    text = original.readline()
                text = original.readline()
                while(text!=''):
                    new.write(text)
                    text = original.readline()

        with open(os.path.join(self.file_path, 'temporaneo_1.txt'), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo_2.txt'), 'w') as new:
                text = original.readline()
                while(text!=(num_c + '_stop\n')):
                    new.write(text)
                    text = original.readline()
                new.write(text)
                text = original.readline()
                with open(os.path.join(self.file_path, 'temporaneo.txt'), 'r') as temp:
                    text_tempo = temp.readline()
                    while(text_tempo!=''):
                        new.write(text_tempo)
                        text_tempo = temp.readline()
                while(text!=''):
                    new.write(text)
                    text = original.readline()
        os.rename(self.file_path + '/temporaneo_2.txt',self.file_path + '/' + self.file_name)
        os.remove(self.file_path + '/temporaneo_1.txt')
        os.remove(self.file_path + '/temporaneo.txt')
        self.create_new_program_page()
    
    def update_start(self):
        last_text = ''
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            text = original.readline()
            while(text!=''):
                last_text = text
                text = original.readline()
        with open(os.path.join(self.file_path, 'temporaneo.txt'), 'w') as new:
            new.write("Start:\n" + last_text)
            with open(os.path.join(self.file_path, self.file_name), 'r') as original:
                text = original.readline()
                text = original.readline()
                text = original.readline()
                while(text!=last_text):
                    new.write(text)
                    text = original.readline()
                os.rename(self.file_path + '/temporaneo.txt',self.file_path + '/' + self.file_name)  
    def Test(self, count):
        with open(os.path.join(self.file_path, self.file_name), 'r') as original:
            with open(os.path.join(self.file_path, 'temporaneo.txt'), 'w') as new:
                text = original.readline()
                while(text!=str(count)+'_start\n'):
                    text = original.readline()
                while(text!= str(count)+'_stop\n'):
                    new.write(text)
                    text = original.readline()
                new.write(text)
        self.play(self.file_path, 'temporaneo.txt')
        os.remove(self.file_path + '/temporaneo.txt')

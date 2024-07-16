#implementazione pagina del controllo manuale
#- Controllo per giunti
#- Controllo cartesiano
#- Apertura/Chiusura pinza
#- FreeDrive

import rospy
from kivy.uix.floatlayout import FloatLayout
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from kivy.properties import ObjectProperty, StringProperty
from std_msgs.msg import Int8, Bool
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen
from kivy.clock import Clock
import math

Builder.load_file('controller.kv')

def callback_JointState(data):
    global JointState_pos
    JointState_pos = data.position

def callback_xyz(data):
    global xyz_actual
    xyz_actual = data

#publisher
pub_free_drive=rospy.Publisher('freeDrive',Int8,queue_size=1)
pub_gripper=rospy.Publisher('gripper',Int8,queue_size=1)
pub_joint_des = rospy.Publisher('joint_des', JointState, queue_size=1)
pub_fine_corsa=rospy.Publisher('fine_corsa', Bool, queue_size=1)
pub_controller_cartesian=rospy.Publisher('controller_cartesian', Bool, queue_size=1)
pub_pose_des=rospy.Publisher('pose_des', Pose, queue_size=1)
#subscriber
sub_joint_state = rospy.Subscriber('joint_states', JointState, callback_JointState)
sub_xyz_actual = rospy.Subscriber('pose_cartesian', Pose, callback_xyz)

rospy.init_node('simple_gui', anonymous=True)

class Controller_joints(FloatLayout):
    upper = ObjectProperty(None)
    downer = ObjectProperty(None)
    stop_vel_modify =ObjectProperty(None)
    
    def __init__(self, **kwargs): #funzione eseguita alla creazione del widget
        super(Controller_joints, self).__init__(**kwargs)
        self.start_clock() #fa partire la funzione che inizializza la funzione periodica

    def my_callback(self, dt):
        global JointState_pos
        self.ids.joint_1.value = (JointState_pos[0] * 180/math.pi + 169) / 3.38  
        self.ids.joint_2.value = (JointState_pos[1] * 180/math.pi + 144) / 1.88
        self.ids.joint_3.value = (JointState_pos[2] * 180/math.pi + 169) / 3.38
        self.ids.joint_4.value = (JointState_pos[3] * 180/math.pi + 124) / 2.04
        self.ids.joint_5.value = (JointState_pos[4] * 180/math.pi + 290) / 5.80
        self.ids.joint_6.value = (JointState_pos[5] * 180/math.pi + 88 ) / 2.26
        self.ids.joint_7.value = (JointState_pos[6] * 180/math.pi + 229) / 4.58

    def start_clock(self):
        Clock.schedule_interval(self.my_callback, 0.1)
        #print('start clock') #debug

class Controller_xyz(Screen):
    pose_des = Pose()
    def __init__(self, **kwargs):
        super(Controller_xyz, self).__init__(**kwargs)
        self.start_clock()
        self.static_xyz()
    def my_callback(self, dt):
        global xyz_actual
        self.ids.x.text = 'x: '+ str(round(xyz_actual.position.x,3))
        self.ids.y.text = 'y: '+ str(round(xyz_actual.position.y,3))
        self.ids.z.text = 'z: '+ str(round(xyz_actual.position.z,3))
        self.ids.rx.text = 'rx: '+ str(round(xyz_actual.orientation.x,3))
        self.ids.ry.text = 'ry: '+ str(round(xyz_actual.orientation.y,3))
        self.ids.rz.text = 'rz: '+ str(round(xyz_actual.orientation.z,3))
        self.ids.rw.text = 'rw: '+ str(round(xyz_actual.orientation.w,3))

    def my_callback_upper_x(self, dt):
        self.pose_des.position.x = self.pose_des.position.x + 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_upper_y(self, dt):
        self.pose_des.position.y = self.pose_des.position.y + 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_upper_z(self, dt):
        self.pose_des.position.z = self.pose_des.position.z + 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_upper_rx(self, dt):
        self.pose_des.orientation.x = self.pose_des.orientation.x + 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_upper_ry(self, dt):
        self.pose_des.orientation.y = self.pose_des.orientation.y + 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_upper_rz(self, dt):
        self.pose_des.orientation.z = self.pose_des.orientation.z + 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_upper_rw(self, dt):
        self.pose_des.orientation.w = self.pose_des.orientation.w + 0.003
        pub_pose_des.publish(self.pose_des)

    def start_clock(self):
        Clock.schedule_interval(self.my_callback, 0.1)
        #print('start clock') #debug

    def copy_xyz(self):
        self.pose_des.position.x = xyz_actual.position.x
        self.pose_des.position.y = xyz_actual.position.y
        self.pose_des.position.z = xyz_actual.position.z
        self.pose_des.orientation.x = xyz_actual.orientation.x
        self.pose_des.orientation.y = xyz_actual.orientation.y
        self.pose_des.orientation.z = xyz_actual.orientation.z
        self.pose_des.orientation.w = xyz_actual.orientation.w
    def static_xyz(self):
        Clock.unschedule(self.my_callback_upper_x)
        Clock.unschedule(self.my_callback_upper_y)
        Clock.unschedule(self.my_callback_upper_z)
        Clock.unschedule(self.my_callback_upper_rx)
        Clock.unschedule(self.my_callback_upper_ry)
        Clock.unschedule(self.my_callback_upper_rz)
        Clock.unschedule(self.my_callback_upper_rw)
        Clock.unschedule(self.my_callback_downer_x)
        Clock.unschedule(self.my_callback_downer_y)
        Clock.unschedule(self.my_callback_downer_z)
        Clock.unschedule(self.my_callback_downer_rx)
        Clock.unschedule(self.my_callback_downer_ry)
        Clock.unschedule(self.my_callback_downer_rz)
        Clock.unschedule(self.my_callback_downer_rw)
        self.copy_xyz()
        pub_pose_des.publish(self.pose_des)
    def upper_xyz(self, id):
        self.copy_xyz()
        if(id == 'x'):
            Clock.schedule_interval(self.my_callback_upper_x, 0.1)
        elif(id == 'y'):
            Clock.schedule_interval(self.my_callback_upper_y, 0.1)
        elif(id == 'z'):
            Clock.schedule_interval(self.my_callback_upper_z, 0.1)
        elif(id == 'rx'):
            Clock.schedule_interval(self.my_callback_upper_rx, 0.1)
        elif(id == 'ry'):
            Clock.schedule_interval(self.my_callback_upper_ry, 0.1)
        elif(id == 'rz'):
            Clock.schedule_interval(self.my_callback_upper_rz, 0.1)
        elif(id == 'rw'):
            Clock.schedule_interval(self.my_callback_upper_rw, 0.1)

    def my_callback_downer_x(self, dt):
        self.pose_des.position.x = self.pose_des.position.x - 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_downer_y(self, dt):
        self.pose_des.position.y = self.pose_des.position.y - 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_downer_z(self, dt):
        self.pose_des.position.z = self.pose_des.position.z - 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_downer_rx(self, dt):
        self.pose_des.orientation.x = self.pose_des.orientation.x - 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_downer_ry(self, dt):
        self.pose_des.orientation.y = self.pose_des.orientation.y - 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_downer_rz(self, dt):
        self.pose_des.orientation.z = self.pose_des.orientation.z - 0.003
        pub_pose_des.publish(self.pose_des)
    def my_callback_downer_rw(self, dt):
        self.pose_des.orientation.w = self.pose_des.orientation.w - 0.003
        pub_pose_des.publish(self.pose_des)

    def downer_xyz(self,id):
        self.copy_xyz()
        if(id == 'x'):
            Clock.schedule_interval(self.my_callback_downer_x, 0.1)
        elif(id == 'y'):
            Clock.schedule_interval(self.my_callback_downer_y, 0.1)
        elif(id == 'z'):
            Clock.schedule_interval(self.my_callback_downer_z, 0.1)
        elif(id == 'rx'):
            Clock.schedule_interval(self.my_callback_downer_rx, 0.1)
        elif(id == 'ry'):
            Clock.schedule_interval(self.my_callback_downer_ry, 0.1)
        elif(id == 'rz'):
            Clock.schedule_interval(self.my_callback_downer_rz, 0.1)
        elif(id == 'rw'):
            Clock.schedule_interval(self.my_callback_downer_rw, 0.1)

class Controller(Screen):
    info = StringProperty()
    joint_des = JointState()
    joint_des.velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0]
    max_Joint_pos = [2.94960643587, 0.76794487087, 2.94960643587, 1.3962634016, 5.06145483078, 2.40855436775, 3.99680398707]
    min_Joint_pos = [-2.94960643587,-2.51327412287,-2.94960643587,-2.16420827247,-5.06145483078, -1.53588974176, -3.99680398707]
    stop = False
    actual_joint = 0

    def create_controller_joints(self):
        self.ids.container.clear_widgets()
        customwidget = Controller_joints(upper = self.upper, downer=self.downer, stop_vel_modify = self.stop_vel_modify)
        self.ids.container.add_widget(customwidget)
        pub_controller_cartesian.publish(False)

    def carica_controllo_xyz(self):
        self.ids.container.clear_widgets()
        customwidget = Controller_xyz()
        self.ids.container.add_widget(customwidget)
        pub_controller_cartesian.publish(True)

    def free_drive_fun(self, status):
        #verifica lo stato dello switch dedicato al free_drive
        if self.ids.switch_free_drive.active is True:
            msg=1
            pub_free_drive.publish(msg)
        else:
            msg=0
            pub_free_drive.publish(msg)

    def gripper_fun(self, status):
        #verifica lo stato dello switch dedicato al free_drive
        if self.ids.switch_gripper.active is True:
            msg=1
            pub_gripper.publish(msg)
        else:
            msg=0
            pub_gripper.publish(msg)

    def control_joint_callback_up(self, dt): #tramite questa callback vedo se sono vicino ai massimi, in quel caso non posso far crescere l'angolo
        #print(abs(JointState_pos[self.actual_joint] - self.max_Joint_pos[self.actual_joint])) #debug
        if(abs(JointState_pos[self.actual_joint] - self.max_Joint_pos[self.actual_joint]) < 2/180*math.pi):
            self.stop_vel_modify()
    
    def control_joint_callback_down(self, dt):
        #print(abs(JointState_pos[self.actual_joint] - self.max_Joint_pos[self.actual_joint])) #debug
        if(abs(JointState_pos[self.actual_joint] - self.min_Joint_pos[self.actual_joint]) < 2/180*math.pi):
            self.stop_vel_modify()

    def upper(self, joint):
        #self.switch_controller_velocity_joint() #debug
        self.actual_joint = joint
        self.joint_des.velocity[joint] = 0.1
        Clock.schedule_interval(self.control_joint_callback_up, 0.1)
        #print(abs(JointState_pos[self.actual_joint] - self.max_Joint_pos[self.actual_joint])) #debug
        #print('< ?'+ str(2/180*math.pi)) #debug
        if((- JointState_pos[self.actual_joint] + self.max_Joint_pos[self.actual_joint]) > 2.5/180*math.pi):
            print(self.joint_des) 
            pub_joint_des.publish(self.joint_des)
            
    def stop_vel_modify(self):
        Clock.unschedule(self.control_joint_callback_up)
        Clock.unschedule(self.control_joint_callback_down)
        self.zero_vel()
    
    def zero_vel(self):
        for i in range(len(self.joint_des.velocity)):
            self.joint_des.velocity[i] = 0.0
        pub_joint_des.publish(self.joint_des)
        #print(self.joint_des) #debug

    def downer(self, joint):
        self.actual_joint = joint
        self.joint_des.velocity[joint] = -0.1
        
        Clock.schedule_interval(self.control_joint_callback_down, 0.1)
        if((- JointState_pos[self.actual_joint] + self.min_Joint_pos[self.actual_joint]) < -2.5/180*math.pi ):
            pub_joint_des.publish(self.joint_des)
            #print(self.joint_des) #debug
              
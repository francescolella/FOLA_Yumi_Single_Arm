#implementazione pagina principale dell'applicazione
import rospy
import kivy
kivy.require('1.0.5')

from kivy.config import Config
kivy.config.Config.set('graphics', 'width', '600')
kivy.config.Config.set('graphics', 'height', '700')

from kivy.uix.gridlayout import GridLayout
from kivy.uix.screenmanager import ScreenManager, Screen, SlideTransition, NoTransition
from kivy.app import App
from std_msgs.msg import Bool, Int8
from controller import Controller
from programming import Programming
from program import New_program
pub_switch_controller = rospy.Publisher('switch_controller', Bool, queue_size=1)
pub_free_drive=rospy.Publisher('freeDrive',Int8,queue_size=1)
pub_controller_cartesian=rospy.Publisher('controller_cartesian', Bool, queue_size=1)

class MainMenu(GridLayout):
    pass
   
class YumiApp(App):
    new_program= Screen(name = 'new_program')
    def build(self):
        self.manager = ScreenManager(transition=SlideTransition(
                                    duration=.15))
        self.manager.add_widget(Controller())       
        self.manager.add_widget(Programming(info= 'programming'))
        self.manager.add_widget(New_program(create_new_program_page = self.create_new_program_page))
        layout = GridLayout(cols=1)
        layout.add_widget(self.manager)
        layout.add_widget(MainMenu())
        self.manager.get_screen('controller').create_controller_joints()
        self.switch_controller_velocity_joint()
        return layout
        
    def create_new_program_page(self):
        self.manager.transition = NoTransition()
        self.manager.remove_widget(self.manager.get_screen('new_program'))
        self.manager.add_widget(New_program(create_new_program_page = self.create_new_program_page))
        self.manager.current='new_program'
        self.manager.get_screen('new_program').file_path = self.manager.get_screen('programming').file_path
        self.manager.get_screen('new_program').file_name = self.manager.get_screen('programming').file_name
        self.manager.get_screen('new_program').corret_file()
        self.manager.get_screen('new_program').ids.label_name_program.text = 'crea il tuo programma\n' + self.manager.get_screen('new_program').file_name
        self.manager.transition = SlideTransition(duration=.15)

    def switch_controller_move_group(self):
        pub_free_drive.publish(0)
        pub_controller_cartesian.publish(False)
        pub_switch_controller.publish(False)
        
    def switch_controller_velocity_joint(self):
        pub_switch_controller.publish(True)
        self.manager.get_screen('controller').zero_vel()
                
if __name__ == '__main__':
    YumiApp().run()

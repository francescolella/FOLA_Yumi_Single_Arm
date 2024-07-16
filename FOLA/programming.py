#pagina di gestione file: nuovo programma o apri programma esistente
from kivy.uix.floatlayout import FloatLayout
from kivy.properties import ObjectProperty, StringProperty
from kivy.lang import Builder
from kivy.factory import Factory
from kivy.uix.popup import Popup
from kivy.uix.screenmanager import Screen
import os

Builder.load_file('programming.kv')

class LoadDialog(FloatLayout):
    load = ObjectProperty(None)
    cancel = ObjectProperty(None)
    prova = ObjectProperty(None)

class SaveDialog(FloatLayout):
    save = ObjectProperty(None)
    text_input = ObjectProperty(None)
    cancel = ObjectProperty(None)
    START_page = ObjectProperty(None)
    file_name = ObjectProperty(None)
    file_path = ObjectProperty(None)

class Programming(Screen):

    label_wid = ObjectProperty()
    info = StringProperty()
    loadfile = ObjectProperty(None)
    savefile = ObjectProperty(None)
    file_path = ''
    file_name = ''

    def dismiss_popup(self):
        self._popup.dismiss()

    def show_load(self):
        content = LoadDialog(load=self.load, cancel=self.dismiss_popup, prova=self.prova)
        self._popup = Popup(title="Load file", content=content,
                            size_hint=(0.9, 0.9))
        self._popup.open()

    def show_save(self):
        content = SaveDialog(save=self.save, cancel=self.dismiss_popup, file_path=self.file_path, file_name = self.file_name)
        self._popup = Popup(title="Save file", content=content,
                            size_hint=(0.9, 0.9))
        self._popup.open()

    def load(self, path, filename):
        print(path)
        print(filename)
        with open(os.path.join(path, filename[0]),'r') as stream:
            a = stream.readline()
            print(a)
        self.dismiss_popup()

    def save(self, path, filename):
        print(path)
        print(filename)
        with open(os.path.join(path, filename), 'w') as stream:
            stream.write("Start:\n")
        self.dismiss_popup() 

    def prova(self):
        print('file_name: '+ self.file_name)       

Factory.register('Root', cls=Programming)
Factory.register('LoadDialog', cls=LoadDialog)
Factory.register('SaveDialog', cls=SaveDialog)
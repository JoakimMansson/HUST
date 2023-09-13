import kivy
import os

from kivy.clock import Clock
from kivy.properties import ObjectProperty
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from kivy.app import App
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen 


from kivymd.app import MDApp
from kivymd.toast import toast
from kivymd.uix.button import MDIconButton, MDFillRoundFlatButton
from kivymd.uix.dialog import MDDialog
from kivymd.uix.menu import MDDropdownMenu


from kivy.config import Config



class StartScreen(Screen):


    def userExists(self):
        pass

kv = Builder.load_file(os.path.realpath("my.kv"))
class DataGUI(App):

    def build(self):
        Window.size = (350, 500)
        return kv


if __name__ == "__main__":
    DataGUI().run()
import sys,pygame, time
import threading
from controller_network import ControllerNetworkManager

class Controller(threading.Thread):
    def run(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            try:
                network = ControllerNetworkManager()
                Controller = pygame.joystick.Joystick(0)
                Controller.init()
                uiStatus = True
                controllerStatus = True
                print uiStatus
                print "Controller connected"
                print "Name:", Controller.get_name()
                print "Axes:", Controller.get_numaxes()
                print "Bottons:", Controller.get_numbuttons()
                print "Hats:", Controller.get_numhats()
            
                A = 0
                B = 0
                X = 0
                Y = 0
                Start = 0
                Back = 0
                LS = 0
                RS = 0
                LB = 0
                RB = 0
                LX = 0
                RX = 0
                LY = 0
                RY = 0
                LT = 0
                RT = 0
                
            
                clock = pygame.time.Clock()
                while (controllerStatus):
                    #print pygame.joystick.get_count()
                    clock.tick(5)
                    for event in pygame.event.get():
                        if event.type == pygame.JOYAXISMOTION:
                            if Controller.get_axis(0) != LY:
                                #print "LY ",Controller.get_axis(1)
                                network.sendControllerString("LY=%s;" % str(Controller.get_axis(1)))
                                LY = Controller.get_axis(1)
                            if Controller.get_axis(1) != LX:
                                #print "LX ",Controller.get_axis(0)
                                network.sendControllerString("LX=%s;" % str(Controller.get_axis(0)))
                                LX = Controller.get_axis(0)
                            if Controller.get_axis(2) != LT:
                                #print "LT ",Controller.get_axis(2)
                                network.sendControllerString("LT=%s;" % str(Controller.get_axis(2)))
                                LT = Controller.get_axis(2)
                            if Controller.get_axis(3) != RY:
                                #print "RY ",Controller.get_axis(4)
                                network.sendControllerString("RY=%s;" % str(Controller.get_axis(4)))
                                RY = Controller.get_axis(4)
                            if Controller.get_axis(4) != RX:
                                #print "RX ",Controller.get_axis(3)
                                network.sendControllerString("RX=%s;" % str(Controller.get_axis(3)))
                                RX = Controller.get_axis(3)
                            if Controller.get_axis(5) != RT:
                                #print "RT ",Controller.get_axis(5)
                                network.sendControllerString("RT=%s;" % str(Controller.get_axis(5)))
                                RT = Controller.get_axis(5)
                        if event.type == pygame.JOYBUTTONDOWN:
                            if Controller.get_button(0) != A:
                                #print "A Pressed"
                                network.sendControllerString("AA=1;")
                                A = 1
                            if Controller.get_button(1) != B:
                                #print "B Pressed"
                                network.sendControllerString("BB=1;")
                                B = 1
                            if Controller.get_button(2) != X:
                                #print "X Pressed"
                                network.sendControllerString("XX=1;")
                                X = 1
                            if Controller.get_button(3) != Y:
                                #print "Y Pressed"
                                network.sendControllerString("YY=1;")
                                Y = 1
                            if Controller.get_button(4) != LB:
                                #print "LB Pressed"
                                network.sendControllerString("LB=1;")
                                LB = 1
                            if Controller.get_button(5) != RB:
                                #print "RB Pressed"
                                network.sendControllerString("RB=1;")
                                RB = 1
                            if Controller.get_button(7) != Back:
                                #print "Back Pressed"
                                network.sendControllerString("Back=1;")
                                Back = 1
                            if Controller.get_button(6) != Start:
                                #print "Start Pressed"
                                network.sendControllerString("Start=1;")
                                Start = 1
                            if Controller.get_button(8) != LS:
                                #print "LS Pressed"
                                network.sendControllerString("LS=1;")
                                LS = 1
                            if Controller.get_button(9) != RS:
                                #print "RS Pressed"
                                network.sendControllerString("RS=1;")
                                RS = 1
                        if event.type == pygame.JOYBUTTONUP:
                            if Controller.get_button(0) != A:
                                #print "A Released"
                                network.sendControllerString("AA=0;")
                                A = 0
                            if Controller.get_button(1) != B:
                                #print "B Released"
                                network.sendControllerString("BB=0;")
                                B = 0
                            if Controller.get_button(2) != X:
                                #print "X Released"
                                network.sendControllerString("XX=0;")
                                X = 0
                            if Controller.get_button(3) != Y:
                                #print "Y Released"
                                network.sendControllerString("YY=0;")
                                Y = 0
                            if Controller.get_button(4) != LB:
                                #print "LB Released"
                                network.sendControllerString("LB=0;")
                                LB = 0
                            if Controller.get_button(5) != RB:
                                #print "RB Released"
                                network.sendControllerString("RB=0;")
                                RB = 0
                            if Controller.get_button(7) != Back:
                                #print "Back Released"
                                network.sendControllerString("Back=0;")
                                Back = 0
                            if Controller.get_button(6) != Start:
                                #print "Start Released"
                                network.sendControllerString("Start=0;")
                                Start = 0
                            if Controller.get_button(8) != LS:
                                #print "LS Released"
                                network.sendControllerString("LS=0;")
                                LS = 0
                            if Controller.get_button(9) != RS:
                                #print "RS Released"
                                network.sendControllerString("RS=0;")
                                RS = 0
                        if event.type == pygame.JOYHATMOTION:
                            #print "hat:",Controller.get_hat(0)
                            network.sendControllerString("D_pad=%s;" % str(Controller.get_hat(0)))
                """
                try:
                    pygame.joystick.quit()
                    Controller = pygame.joystick.Joystick(0)
                    Controller.init()
                except pygame.error:
                	print "Joystick Error"
                	controllerStatus = False
            	"""
            
            
            except KeyboardInterrupt:
                print "Caught the keyboard interrupt!"
                network.shutdown()                                
            except Exception as ex:
                print "Caught exception"
                print ex
        else:   
            print "No Controller found"
                
        pygame.quit()   

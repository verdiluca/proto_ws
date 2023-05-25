import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

#---------------------------------------------------------------DICHIARAZIONE_VARIABILI-------------------------------------------------------------------#

#lidar
n = 0.0
ne = 0.0
e = 0.0
no = 0.0
o = 0.0
#ultrasuoni
sas = 0.0
sac = 0.0
sad = 0.0
scs = 0.0
scc = 0.0
scd = 0.0
sbs = 0.0
sbc = 0.0
sbd = 0.0

Media_O = 0.0
Media_E = 0.0

#motori
Motori_avanti = 0.0
Motori_destra = 0.0
Motori_sinistra = 0.0
Motori_fermo = 0.0

#fasi
logica_proto = 1
logica_azzera_variabili = 1
logica_controllo_sensori_partenza = 1
logica_proto_avanti = 1
logica_sterzo_dx = 1
logica_sterzo_sx = 1
logica_sterzo_av = 1
logica_arresto_proto = 1

#timers
T1End = False

#generali
Chiave = False
Stop = False

motori_avviati = False
Proto_fermo = False
Proto_arrestato = False

Azzeramento_completato = False


#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#INIZIO CLASSE

class LidarNode(Node):

    def __init__(self):

        super().__init__('driver')

#-------------------------------------------------------------------------------------------------------------------#

        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.listener_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.cmd_vel_sub = self.create_subscription(
           TFMessage, '/VPsensori', self.sensori_callback, 10)

        self.vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 50)
        
        self.nord_pub = self.create_publisher(
            Float32, 'nord', 50)
        self.nordest_pub = self.create_publisher(
            Float32, 'nordest', 50)
        self.nordovest_pub = self.create_publisher(
            Float32, 'nordovest', 50)
        self.ovest_pub = self.create_publisher(
            Float32, 'ovest', 50)
        self.est_pub = self.create_publisher(
            Float32, 'est', 50)

#--------------------------------------------------------------------AVVIO_VOID---------------------------------------------------------------------------#

        timer_period2 = 0.101
        self.timer2 = self.create_timer(timer_period2, self.Assegna_Uscite)

        timer_period3 = 0.103
        self.timer3 = self.create_timer(timer_period3, self.Logica_Proto)

        timer_period4 = 0.107
        self.timer4 = self.create_timer(timer_period4, self.Logica_Azzera_Variabili)

        timer_period5 = 0.109
        self.timer5 = self.create_timer(timer_period5, self.Logica_Controllo_Sensori_Partenza)

        timer_period6 = 0.113
        self.timer6 = self.create_timer(timer_period6, self.Logica_Proto_avanti)

        timer_period7 = 0.127
        self.timer7 = self.create_timer(timer_period7, self.Logica_Sterzo_DX)

        timer_period8 = 0.131
        self.timer8 = self.create_timer(timer_period8, self.Logica_Sterzo_SX)

        timer_period9 = 0.137
        self.timer9 = self.create_timer(timer_period9, self.Logica_Sterzo_AV)

        timer_period10 = 0.139
        self.timer10 = self.create_timer(timer_period10, self.Logica_Arresto_Proto)
      

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

    def listener_callback(self, msg):

        global n
        global ne
        global e
        global no
        global o
        global Media_E
        global Media_O
        global dist_back


        an =  Float32()
        ane =  Float32()
        ano =  Float32()
        ao =  Float32()
        ae =  Float32()

        an.data = msg.ranges[240]
        ano.data = msg.ranges[285]
        ane.data = msg.ranges[195]
        ao.data = msg.ranges[331]
        ae.data = msg.ranges[149]
        # dist_back = format(msg.ranges[240], '.3g')
        # dist_left = format(msg.ranges[90], '.2f')
        # dist_right = format(msg.ranges[270], '.2f')
        # dist_head = format(msg.ranges[0], '.2f')
        # self.get_logger().info(f'{dist_back} {dist_left} {dist_right} {dist_head}')
        
        n = Float32()
        e = Float32()
        o = Float32()
        no = Float32()
        ne = Float32()
        
        n = float(format(msg.ranges[240], '.2f'))
        ne = float(format(msg.ranges[195], '.2f'))
        e = float(format(msg.ranges[149], '.2f'))
        no = float(format(msg.ranges[285], '.2f'))
        o = float(format(msg.ranges[331], '.2f'))

        Media_E = (e+ne)/2
        Media_O = (o+no)/2

        
        self.nord_pub.publish(an)
        self.nordest_pub.publish(ane)
        self.nordovest_pub.publish(ano)
        self.ovest_pub.publish(ao)
        self.est_pub.publish(ae)


        # print("NORD:", n)
        # print("NORD EST:", ne)
        # print("EST:", e)
        # print("NORD OVEST:", no)
        # print("OVEST:", o)
        # print("-------------------------------------------------")

        

#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
    def sensori_callback(self, msg2):

        

        print(msg2)
              
#tf2_msgs.msg.TFMessage(transforms=[geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=10, nanosec=20), frame_id=''), child_frame_id='', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=30.0, y=40.0, z=50.0), rotation=geometry_msgs.msg.Quaternion(x=60.0, y=70.0, z=80.0, w=90.0)))])


#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------TIMERS---------------------------------------------------------------------------#

    def T1(timer1):
        
        global T1End
        
        while timer1:
            time.sleep(1)
            timer1 -= 1
            if timer1 == 0:
                T1End = True
                


#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------ASSEGNA_USCITE-----------------------------------------------------------------------------#

    def Assegna_Uscite(self):
        
        #MOTORI
        global Motori_avanti
        global Motori_destra
        global Motori_sinistra
        global Motori_fermo
        global logica_proto
        global Azzeramento_completato
        global motori_avviati
        global logica_azzera_variabili
        global logica_controllo_sensori_partenza
        global logica_proto_avanti
        global logica_sterzo_dx
        global logica_sterzo_sx
        global logica_sterzo_av
        global logica_arresto_proto
        global Proto_fermo
        global Proto_arrestato

        Motori_avanti = Twist()
        Motori_destra = Twist()
        Motori_sinistra = Twist()
        Motori_fermo = Twist()

        Motori_avanti = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        Motori_destra = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        Motori_sinistra = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))
        Motori_fermo = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
         
        #self.vel_pub.publish(Motori_avanti)
        # geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
        
        #USCITE AZZERA VARIABILI
        if logica_azzera_variabili == 2:
            
            logica_controllo_sensori_partenza = 1
            logica_proto_avanti = 1
            logica_sterzo_dx = 1
            logica_sterzo_sx = 1
            logica_sterzo_av = 1
            logica_arresto_proto = 1

            Proto_fermo = False
            motori_avviati = False
            Proto_arrestato = False

            Azzeramento_completato = True
        
        if logica_proto_avanti == 2:
            self.vel_pub.publish(Motori_avanti)
            motori_avviati = True

        if logica_sterzo_dx == 2:
            self.vel_pub.publish(Motori_destra)

        if logica_sterzo_sx == 2:
            self.vel_pub.publish(Motori_sinistra)

        if logica_sterzo_av == 2:
            self.vel_pub.publish(Motori_fermo)
            Proto_fermo = True
            
        if logica_arresto_proto == 2:
            self.vel_pub.publish(Motori_fermo)
            Proto_arrestato = True

        
#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------------------LOGICA_PROTO-----------------------------------------------------------------------------#
    def Logica_Proto(self):
        
        global logica_proto
        global logica_azzera_variabili
        global logica_controllo_sensori_partenza
        global logica_sterzo_dx
        global logica_sterzo_sx
        global logica_arresto_proto
        global n
        global ne
        global e
        global no
        global o
        global sas
        global scs
        global sbs
        global sac
        global scc
        global sbc
        global sad
        global scd
        global sbd
        global Media_E
        global Media_O
        global Stop
        global Chiave

        if logica_proto == 1 and Chiave == True:
            logica_proto = 2


        if logica_proto == 2 and logica_azzera_variabili == 3:
            logica_proto = 3


        if logica_proto == 3 and logica_controllo_sensori_partenza == 4:
            logica_proto = 4


        if logica_proto == 4 and (o < 0.35 or no < 0.35 or sas < 0.35 or scs < 0.35 or sbs < 0.35):
            logica_proto = 5
        
        elif logica_proto == 4 and (e < 0.35 or ne < 0.35 or sad < 0.35 or scd < 0.35 or sbd < 0.35):
            logica_proto = 6

        elif logica_proto == 4 and (n < 0.35 or sac < 0.35 or scc < 0.35 or sbc < 0.35):
            logica_proto = 7


        if logica_proto == 5 and logica_sterzo_dx == 3 and n > 0.35 and ne > 0.35 and no > 0.35 and e > 0.35 and o > 0.35 and sas > 0.35 and scs > 0.35 and sbs > 0.35 and sac > 0.35 and scc > 0.35 and sbc > 0.35 and sad > 0.35 and scd > 0.35 and sbd > 0.35:
            logica_proto = 4
        
        elif logica_proto == 5 and logica_sterzo_dx == 3 and (e < 0.35 or ne < 0.35 or sad < 0.35 or scd < 0.35 or sbd < 0.35):
            logica_proto = 6
        
        elif logica_proto == 5 and logica_sterzo_dx == 3 and (n < 0.35 or sac < 0.35 or scc < 0.35 or sbc < 0.35):
            logica_proto = 7
        

        if logica_proto == 6 and logica_sterzo_sx == 3 and n > 0.35 and ne > 0.35 and no > 0.35 and e > 0.35 and o > 0.35 and sas > 0.35 and scs > 0.35 and sbs > 0.35 and sac > 0.35 and scc > 0.35 and sbc > 0.35 and sad > 0.35 and scd > 0.35 and sbd > 0.35:
            logica_proto = 4
        
        elif logica_proto == 6 and logica_sterzo_sx == 3 and (o < 0.35 or no < 0.35 or sas < 0.35 or scs < 0.35 or sbs < 0.35):
            logica_proto = 5
        
        elif logica_proto == 6 and logica_sterzo_sx == 3 and (n < 0.35 or sac < 0.35 or scc < 0.35 or sbc < 0.35):
            logica_proto = 7


        if logica_proto == 7 and (Media_O > Media_E) and logica_sterzo_av == 3:
            logica_proto = 6
        if logica_proto == 7 and (Media_E > Media_O) and logica_sterzo_av == 3:
            logica_proto = 5


        if logica_proto == 8 and logica_arresto_proto == 3:
            logica_proto = 1


        if Stop == True:
            logica_proto = 8
    

#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------------------LOGICA_AZZERA_VARIABILI------------------------------------------------------------------#

    def Logica_Azzera_Variabili(self):
        
        global logica_proto
        global logica_azzera_variabili
        global Azzeramento_completato

        if logica_azzera_variabili == 1 and logica_proto == 2:
            logica_azzera_variabili = 2

        if logica_azzera_variabili == 2 and Azzeramento_completato == True:
            logica_azzera_variabili = 3
        
        if logica_azzera_variabili == 3 and logica_proto == 3:
            Azzeramento_completato = False
            logica_azzera_variabili = 1

#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------LOGICA_CONTROLLO_SENSORI_PARTENZA------------------------------------------------------------------#

    def Logica_Controllo_Sensori_Partenza(self):

        global logica_proto
        global logica_controllo_sensori_partenza
        global Azzeramento_completato
        global T1End
        global T1
        global n
        global ne
        global e
        global no
        global o
        global sas
        global scs
        global sbs
        global sac
        global scc
        global sbc
        global sad
        global scd
        global sbd

        if logica_controllo_sensori_partenza == 1 and logica_proto == 3:
            logica_controllo_sensori_partenza = 2

        if logica_controllo_sensori_partenza == 2 and (n < 0.35 or o < 0.35 or no < 0.35 or e < 0.35 or ne < 0.35 or sas < 0.35 or scs < 0.35 or sbs < 0.35 or sad < 0.35 or scd < 0.35 or sbd < 0.35 or sac < 0.35 or scc < 0.35 or sbc < 0.35):
            T1(1)
            logica_controllo_sensori_partenza = 3

        elif logica_controllo_sensori_partenza == 2 and n > 0.35 and ne > 0.35 and no > 0.35 and e > 0.35 and o > 0.35 and sas > 0.35 and scs > 0.35 and sbs > 0.35 and sac > 0.35 and scc > 0.35 and sbc > 0.35 and sad > 0.35 and scd > 0.35 and sbd > 0.35:
            logica_controllo_sensori_partenza = 4

        if logica_controllo_sensori_partenza == 3 and T1End == True:
            T1End == False 
            logica_controllo_sensori_partenza = 2

        if logica_controllo_sensori_partenza == 4 and logica_proto == 4:
            logica_controllo_sensori_partenza == 1

#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------LOGICA_PROTO_AVANTI------------------------------------------------------------------#

    def Logica_Proto_avanti(self):

            global logica_proto
            global logica_proto_avanti
            global motori_avviati

            if logica_proto_avanti == 1 and logica_proto == 4:
                logica_proto_avanti = 2

            if logica_proto_avanti == 2 and motori_avviati == True:
                logica_proto_avanti = 3

            if logica_proto_avanti == 3 and (logica_proto == 5 or logica_proto == 6 or logica_proto == 7 or logica_proto == 8):
                motori_avviati == False 
                logica_proto_avanti = 1

#---------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------LOGICA_STERZO_DX----------------------------------------------------------------------#

    def Logica_Sterzo_DX(self):

            global logica_proto
            global logica_sterzo_dx
            global logica_arresto_proto
            global n
            global ne
            global e
            global no
            global o
            global sas
            global scs
            global sbs
            global sac
            global scc
            global sbc
            global sad
            global scd
            global sbd

            if logica_sterzo_dx == 1 and logica_proto == 5:
                logica_sterzo_dx = 2

            if logica_sterzo_dx == 2 and (o > 0.35 and no > 0.35 and sas > 0.35 and scs > 0.35 and sbs > 0.35):
                logica_sterzo_dx = 3

            elif logica_sterzo_dx == 2 and logica_proto == 8 and logica_arresto_proto == 2:
                logica_sterzo_dx == 1

            if logica_sterzo_dx == 3 and (logica_proto == 4 or logica_proto == 5 or logica_proto == 7 or logica_proto == 8):
                logica_sterzo_dx = 1

#---------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------LOGICA_STERZO_SX----------------------------------------------------------------------#

    def Logica_Sterzo_SX(self):

            global logica_proto
            global logica_sterzo_sx
            global logica_arresto_proto
            global n
            global ne
            global e
            global no
            global o
            global sas
            global scs
            global sbs
            global sac
            global scc
            global sbc
            global sad
            global scd
            global sbd

            if logica_sterzo_sx == 1 and logica_proto == 6:
                logica_sterzo_sx = 2

            if logica_sterzo_sx == 2 and (e > 0.35 and ne > 0.35 and sad > 0.35 and scd > 0.35 and sbd > 0.35):
                logica_sterzo_sx = 3

            elif logica_sterzo_sx == 2 and logica_proto == 8 and logica_arresto_proto == 2:
                logica_sterzo_sx == 1

            if logica_sterzo_sx == 3 and (logica_proto == 4 or logica_proto == 6 or logica_proto == 7 or logica_proto == 8):
                logica_sterzo_sx = 1


#---------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------LOGICA_STERZO_AV----------------------------------------------------------------------#

    def Logica_Sterzo_AV(self):

            global logica_proto
            global logica_sterzo_av
            global logica_arresto_proto
            global Proto_fermo

            if logica_sterzo_av == 1 and logica_proto == 7:
                logica_sterzo_av = 2

            if logica_sterzo_av == 2 and Proto_fermo == True:
                logica_sterzo_av = 3

            elif logica_sterzo_av == 2 and logica_proto == 8 and logica_arresto_proto == 2:
                logica_sterzo_av == 1

            if logica_sterzo_av == 3 and (logica_proto == 5 or logica_proto == 6 or logica_proto == 8):
                Proto_fermo == False
                logica_sterzo_av = 1


#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------LOGICA_ARRESTO_PROTO-----------------------------------------------------------------#

    def Logica_Arresto_Proto(self):

            global logica_proto
            global logica_arresto_proto
            global Proto_arrestato

            if logica_arresto_proto == 1 and logica_proto == 8:
                logica_arresto_proto = 2

            if logica_arresto_proto == 2 and Proto_arrestato == True and logica_sterzo_av == 1 and logica_sterzo_dx == 1 and logica_sterzo_sx == 1:
                logica_arresto_proto = 3

            if logica_arresto_proto == 3 and logica_proto == 1:
                Proto_arrestato = False
                logica_arresto_proto = 1

































def main(args=None):

    rclpy.init(args=args)

    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    lidar_node.destroy_node()
    rclpy.shutdown()
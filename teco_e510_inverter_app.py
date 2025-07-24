"""
2025 TECO E510 Inverter Driver
-----------------------------------------------------------------------------------
AC drives are generally known by many different names: Adjustable Frequency
Drives (AFD), Variable Frequency Drives (VFD), and Inverters. Drives are used
primarily to vary the speed of three phase AC induction motors, and they also
provide non-emergency start and stop control, acceleration and deceleration, and
overload protection. By gradually accelerating the motor, drives can reduce the
amount of motor startup inrush current.
AC drives function by converting incoming AC power to DC, which is then
synthesized back into single/three phase output power. The voltage and frequency of
this synthesized output power is directly varied by the drive, where the frequency
determines the speed of the single/three phase AC induction motor.

Model: E510-201-H1FN4S

CON2 RJ-45 (8P) Serial Comm Port Interface
Pin     | RS-485    
--------------------
1       | Data(A) + 
2       | Data(B) - 
3       | Data +    
4       | Rx        
5       | Tx        
6       | Data -    
7       | Vcc (5V)  
8       | GND       

-----------------------------------------------------------------------------------
MODBUS RTU OVER RS485 COMMUNICATIONS
MODBUS Protocol is a messaging structure, widely used to establish master-slave 
communication between intelligent devices. A MODBUS message sent from a master to a 
slave contains the address of the slave, the 'command' (e.g. 'read register' or 
'write register'), the data, and a check sum (LRC or CRC). Since Modbus protocol is 
just a messaging structure, it is independent of the underlying physical layer. It 
is traditionally implemented using RS232, RS422, or RS485. 

The E510 series inverter can be setup to communicate on standard Modbus networks 
using either ASCII or RTU transmission modes. With reference to the Group 09 
Communication Functions, you can select the desired mode (ASCII or RTU), data rate, 
data bits, parity, and stop bits. The mode and serial parameters must be the same 
for all devices on a Modbus network. 
-----------------------------------------------------------------------------------
DEFAULT RS485 SERIAL PORT PARAMETERS (Refer to Programmable Parameter Group 09)

Program No. | Description   | Value
-----------------------------------
09-00       | Slave Address | 1 
09-01       | RS485 Mode    | 0 : RTU
09-02       | Baud Rate     | 2 : 19200 
09-03       | Stop Bit      | 0 : 1 stop bit
09-04       | Parity        | 0 : None 
09-05       | Data Bits     | 0 : 8 bits

With reference to the TECO E510 Inverter Instruction Manual, the the hexadecimal 
value of Modbus register addresses for parameters is used.   
"""

##############################################################################
# LIBRARIES, MODULES AND PACKAGES
##############################################################################
from datetime import datetime, timedelta
from pathlib import Path
import minimalmodbus
import serial.tools.list_ports
import ttkbootstrap as tkb
from ttkbootstrap.constants import *
from ttkbootstrap.dialogs import Messagebox
from PIL import Image
import watermark

MEDIA = Path(__file__).parent / 'assets'
Image.CUBIC = Image.BICUBIC     # Workaround for PIL Image.CUBIC issue


##############################################################################
# INVERTER CLASS
##############################################################################

class TECO_E510_Inverter(minimalmodbus.Instrument):
    """
    Instrument class for TECO E510 Series Inverter using Modbus RTU protocol.

    Arguments:
        port (str): Serial port to connect to the inverter.
        slave_address (int): Modbus slave address of the inverter (default is 1).
    """

    def __init__(self, port, slaveaddress, mode = minimalmodbus.MODE_RTU, close_port_after_each_call = False, debug = False):
        super().__init__(port, slaveaddress, mode, close_port_after_each_call, debug)
        self.serial.baudrate = 19200
        self.serial.bytesize = 8
        self.serial.parity = serial.PARITY_NONE
        self.serial.stopbits = 1

    
    def get_inverter_details(self):
        """
        Read inverter parameters (model and version).

        Program     | Description   | Register Address
        ------------|---------------|-----------------
        13-00       | Model         | 0x0D00 
        13-01       | Version       | 0x0D01
        
        """
        inverter_details = self.read_registers(registeraddress=0x0D00, number_of_registers=2)
        self.model = str(inverter_details[0])
        self.version = str(inverter_details[1])
    
    
    def get_motor_parameters(self):
        """
        Read motor parameters.

        Program     | Description       | Register Address
        ------------|-------------------|-----------------
        02-01       | Motor Current     | 0x0201
        02-03       | Motor Speed       | 0x0203
        02-04       | Motor Voltage     | 0x0204
        02-05       | Motor Power       | 0x0205
        02-06       | Motor Frequency   | 0x0206
        02-07       | Motor Poles       | 0x0207

        """
        motor_parameters = self.read_registers(registeraddress=0x0200, number_of_registers=9)
        self.motor_voltage = float(motor_parameters[4]/10)
        self.motor_current = float(motor_parameters[1]/10)
        self.motor_frequency = float(motor_parameters[6]/10)
        self.motor_poles = int(motor_parameters[7])
        self.motor_power = float(motor_parameters[5]/10)
        self.motor_speed = int(motor_parameters[3])
    
    
    def get_status(self):
        """
        Read inverter status.

        Register No.    | Description 
        ----------------|------------
        0x2523          | Frequency Command
        0x2524          | Frequency Output  
        0x2525          | Output Voltage Command 
        0x2526          | DC Voltage Command 
        0x2527          | Output Current
        0x2528          | Temperature 
        0x2529          | Output Torque

        """
        status = self.read_registers(registeraddress=0x02523, number_of_registers=7) 
        self.command_frequency = float(status[0]/100)
        self.output_frequency = float(status[1]/100) 
        self.output_voltage = float(status[2]/10)
        self.command_dc_voltage = int(status[3])
        self.output_current = float(status[4]/10)
        self.temperature = float(status[5]/10) 
        self.output_torque = int(status[6])

        """
        Register No.: 0x2520
        --------|--------------------------
        Bit     | Value:Description
        --------|--------------------------
        0       | 0:Stop        ; 1:Run
        1       | 0:Forward     ; 1:Reverse
        2       | 0:Not Ready   ; 1:Ready 
        3       | 0:No Fault    ; 1:Fault
        """
        state = self.read_register(registeraddress=0x2520)
        self.operation = (state >> 0) & 1  # Extract the bit value (0 or 1)
        self.direction = (state >> 1) & 1  
        self.ready = (state >> 2) & 1  
        self.fault = (state >> 3) & 1  


    def set_frequency(self, frequency=0):
        """
        Set the operating frequency of the inverter.

        Register No.    | Description 
        ----------------|------------
        0x2502          | Frequency Command

        Arguments:
            frequency (int): Operating frequency 
        """
        self.write_register(registeraddress=0x2502, value=frequency, number_of_decimals=2, functioncode=6)


    def set_operation(self, operation=0):
        """
        Set the operation mode of the inverter.

        Arguments:
            operation (int): Operation mode (0 = stop forward, 1 = run forward, 2 = stop reverse, 3 = run reverse).
        """
        if operation not in [0, 1, 2, 3]:
            raise ValueError("Invalid operation mode. Must be 0, 1, 2, or 3.")
        self.write_register(registeraddress=0x2501, value=operation, functioncode=6)


##############################################################################
# APPLICATION
##############################################################################
class BallMillControl(tkb.Frame):
    """
    Ball Mill Control Application using TECO E510 Inverter.
    This class provides a GUI for controlling the ball mill via the TECO E510 inverter.
    """
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.pack()
        self.update_task = None  # To keep track of the update task
        self.define_variables()
        self.create_widgets()
        self.after(ms=100,func=self.find_inverter)
    

    def define_variables(self):
        """
        Initialise variables for addressing (set/get) widget values. 
        """
        self.mill_edit_var = tkb.BooleanVar(value=FALSE) 
        self.com_connect_var = tkb.BooleanVar(value=FALSE) 

        self.com_port_var = tkb.StringVar(value='')
        self.mill_rpm_var = tkb.StringVar(value='---')
        self.motor_rpm_var = tkb.StringVar(value='---') 
        self.drive_freq_var = tkb.StringVar(value='---') 
        self.time_remain_var = tkb.StringVar(value='---') 
        self.pause_var = tkb.StringVar(value='PAUSE') 
        self.time_elapsed_var = tkb.StringVar(value='---') 
        self.mill_percent_var = tkb.StringVar(value='---') 
        self.hour_var = tkb.StringVar(value='{:02}'.format(2))  # Zero padding of initial value 
        self.minute_var = tkb.StringVar(value='{:02}'.format(0)) 
        self.mill_total_time_var = tkb.StringVar() 
        self.vfd_model_var = tkb.StringVar(value='---') 
        self.vfd_firmware_var = tkb.StringVar(value='---') 
        self.motor_voltage_var = tkb.StringVar(value='---') 
        self.motor_amps_var = tkb.StringVar(value='---') 
        self.motor_freq_var = tkb.StringVar(value='---') 
        self.motor_maxrpm_var = tkb.StringVar(value='---') 
        self.message_var = tkb.StringVar(value="---") 

        self.mill_radio_var = tkb.IntVar()
        self.vessel_rpm_var = tkb.IntVar() 
        self.shaft_rpm_var = tkb.IntVar() 
        self.motor_rotation_var = tkb.IntVar(value=0)

        self.mill_progressbar_var = tkb.DoubleVar() 
        self.vfd_freq_var = tkb.DoubleVar() 
        self.mill_dia_var = tkb.DoubleVar(value=220.0) 
        self.shaft_dia_var = tkb.DoubleVar(value=45.4) 

        self.logo = tkb.PhotoImage(file=MEDIA/'logo_64.png')   # can resize with .subsample(x=2,y=2) OR .zoom(x=2,y=2)


    def create_widgets(self):
        """
        Create GUI widgets for the application.
        """
        self.create_banner()
        self.create_overview()
        self.create_notebook()
        self.create_footer()


    def create_banner(self):
        # Header frame (for logo and heading)
        header_frame = tkb.Frame(master=self, padding=10)
        header_frame.grid(row=0, column=0, sticky=tkb.W, columnspan=3)
        # Logo
        header_logo = tkb.Label(master=header_frame, image=self.logo)
        header_logo.grid(row=0, column=0, padx=(0, 5))
        # Heading text
        header_text = tkb.Label(master=header_frame, text="Ball Mill Controller", font='-size 24 -weight bold')
        header_text.grid(row=0, column=1, columnspan=2, sticky=tkb.W, padx=(5, 0))
        # Window Instructional label
        intro_text = tkb.Label(master=self, text="This application is used to control and monitor a Variable Frequency Drive (VFD) for ball milling operations.", padding=10)
        intro_text.grid(row=1, column=0, columnspan=3, sticky=tkb.W) 
    

    def create_overview(self):
        # Overview frame (for live values)
        overview_frame = tkb.Frame(master=self, padding=0) 
        overview_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky=(tkb.N, tkb.E, tkb.S, tkb.W))

        self.online_canvas = tkb.Canvas(master=overview_frame, width=20, height=20, highlightthickness=0)
        self.online_canvas.grid(row=0, column=0, padx=10, pady=5, sticky=tkb.E)
        self.online_indicator = self.online_canvas.create_oval(2, 2, 18, 18, fill="red")

        self.online_label = tkb.Label(master=overview_frame, text="OFFLINE")
        self.online_label.grid(row=0, column=1, padx=5, pady=5, sticky=(tkb.N, tkb.W))

        tkb.Separator(master=overview_frame, orient='horizontal', bootstyle="default").grid(row=1, column=0, columnspan=2, padx=5, pady=15, sticky=(tkb.W, tkb.E))
        
        tkb.Label(master=overview_frame, text="Mill (RPM):").grid(row=2, column=0, padx=5, sticky=tkb.E)
        
        self.mill_rpm_status = tkb.Entry(master=overview_frame, state='readonly', textvariable=self.mill_rpm_var, width=10, justify='right')
        self.mill_rpm_status.grid(row=2, column=1, padx=5, pady=5, sticky=tkb.E)

        tkb.Label(overview_frame, text="Motor (RPM):",).grid(row=3, column=0, padx=5, sticky=tkb.E)
        self.motor_rpm_status = tkb.Entry(master=overview_frame, state='readonly', textvariable=self.motor_rpm_var, width=10, justify='right')
        self.motor_rpm_status.grid(row=3, column=1, padx=5, pady=5, sticky=tkb.W)

        tkb.Label(overview_frame, text="Drive Freq. (Hz):").grid(row=4, column=0, padx=5, sticky=tkb.E)
        self.drive_freq_status = tkb.Entry(master=overview_frame, state='readonly', textvariable=self.drive_freq_var, width=10, justify='right')
        self.drive_freq_status.grid(row=4, column=1, padx=5, pady=5, sticky=tkb.W)

        tkb.Label(overview_frame, text="Remaining (hh:mm:ss):").grid(row=5, column=0, sticky=tkb.E)
        self.time_remain_status = tkb.Entry(master=overview_frame, 
            state='readonly', 
            textvariable=self.time_remain_var,
            width=10, 
            justify='right'
            ).grid(row=5, column=1, padx=5, pady=5, sticky=tkb.W)

        tkb.Separator(master=overview_frame, orient='horizontal', bootstyle="default").grid(row=6, column=0, columnspan=2, padx=5, pady=15, sticky=(tkb.W, tkb.E))

        self.run_button = tkb.Button(master=overview_frame, text="RUN", command=self.run_motor, state='disabled', bootstyle=SUCCESS)
        self.run_button.grid(row=7, column=0, columnspan=2, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        self.pause_button = tkb.Button(master=overview_frame, textvariable=self.pause_var, command=self.pause_motor, state='disabled', bootstyle=(WARNING, OUTLINE))
        self.pause_button.grid(row=8, column=0, columnspan=2, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        self.stop_button = tkb.Button(master=overview_frame, text="STOP", command=self.stop_motor, state='disabled', bootstyle=DANGER)
        self.stop_button.grid(row=9, column=0, columnspan=2, padx=5, pady=5, sticky=(tkb.W, tkb.E))

    
    def create_notebook(self):
        """
        Create a Notebook ewidget for displaying information in seperate tabs - about, serial COM, settings. 
        """
        self.notebook = tkb.Notebook(master=self)
        self.notebook.grid(row=2, column=2, padx=(0,10), pady=10, sticky=(tkb.W, tkb.E, tkb.N, tkb.S))

        # Status tab
        self.status_tab = tkb.Frame(master=self.notebook)
        self.notebook.add(self.status_tab, text="Status")
        
        # Milling tab
        self.milling_tab = tkb.Frame(master=self.notebook)
        self.notebook.add(self.milling_tab, text="Mill Settings")
        
        # Drive tab
        self.motor_tab = tkb.Frame(master=self.notebook)
        self.notebook.add(self.motor_tab, text="VFD Parameters")

        # About tab
        self.about_tab = tkb.Frame(master=self.notebook)
        self.notebook.add(self.about_tab, text="About")
        
        self.create_status()
        self.create_settings()
        self.create_VFD()
        self.create_about()


    def create_status(self):
        # Mill Speed Meter
        self.mill_rpm_meter = tkb.Meter(
            master=self.status_tab,
            bootstyle=SUCCESS,
            metertype='semi',
            metersize=175,
            meterthickness=15,
            amounttotal=150,
            stepsize=1,
            amountused=0,
            textright='rpm',
            textfont='-size 20 -weight bold',
            subtext='Mill Speed',
            subtextfont='-size 10',
            subtextstyle=LIGHT,
        )
        self.mill_rpm_meter.grid(row=0, column=0, padx=15, pady=15, sticky=(tkb.N, tkb.W))

        # Motor Speed Meter
        self.motor_rpm_meter = tkb.Meter(
            master=self.status_tab,
            bootstyle=WARNING,
            metertype='semi',
            metersize=175,
            meterthickness=15,
            amounttotal=750,
            stepsize=1,
            amountused=0,
            textright='rpm',
            textfont='-size 20 -weight bold',
            subtext='Motor Speed',
            subtextfont='-size 10',
            subtextstyle=LIGHT,
        )
        self.motor_rpm_meter.grid(row=0, column=1, padx=15, pady=15, sticky=(tkb.N, tkb.W))
        
        # VFD Frequency
        self.vfd_freq_meter = tkb.Meter(
            master=self.status_tab,
            bootstyle=LIGHT,
            metertype='semi',
            metersize=175,
            meterthickness=15,
            amounttotal=50,
            stepsize=1,
            amountused=0.0,
            textright='Hz',
            textfont='-size 20 -weight bold',
            subtext='VFD Frequency',
            subtextfont='-size 10',
            subtextstyle=LIGHT,
        )
        self.vfd_freq_meter.grid(row=0, column=2, padx=15, pady=15, sticky=(tkb.N, tkb.E))

        # Mill Progress Frame
        mill_progress_frame = tkb.Labelframe(master=self.status_tab, text="Mill Time", padding=5)
        mill_progress_frame.grid(row=1, column=0, columnspan=3, padx=5, sticky=(tkb.W, tkb.E))
        
        tkb.Label(master=mill_progress_frame, text="Elapsed (hh:mm:ss):", justify='right').grid(row=0, column=0, sticky=tkb.E)
        self.time_elapsed_value = tkb.Entry(mill_progress_frame, 
            state='readonly', 
            textvariable=self.time_elapsed_var,
            justify='right',
            width=10
            ).grid(row=0, column=1, padx=5, pady=5, sticky=tkb.W)
        
        tkb.Label(master=mill_progress_frame, text="Remaining (hh:mm:ss):", justify='right').grid(row=1, column=0, sticky=tkb.E)
        self.time_remain_value = tkb.Entry(mill_progress_frame, 
            state='readonly', 
            textvariable=self.time_remain_var,
            justify='right',
            width=10
            ).grid(row=1, column=1, padx=5, pady=5, sticky=tkb.W)

        # Progress Bar 
        tkb.Label(master=mill_progress_frame, text="Progress:").grid(row=2, column=0, sticky=tkb.E)
        self.mill_progressbar = tkb.Progressbar(
            master=mill_progress_frame,
            orient=HORIZONTAL,
            bootstyle=(PRIMARY,STRIPED),
            mode=DETERMINATE, 
            length=370,
            maximum=1,
            variable=self.mill_progressbar_var
        )
        self.mill_progressbar.grid(row=2, column=1, padx=5, pady=5, sticky=(tkb.W, tkb.E))
        
        self.mill_percent = tkb.Entry(master=mill_progress_frame,
            state='readonly',  
            textvariable=self.mill_percent_var,
            justify='right',
            width=10
            ).grid(row=2, column=2, padx=5, pady=5, sticky=(tkb.W, tkb.E))


    def create_settings(self):
        # Mill duration frame
        mill_time_frame = tkb.Labelframe(master=self.milling_tab, text="Mill Duration", padding=10)
        mill_time_frame.grid(row=0, column=0, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        tkb.Label(master=mill_time_frame, text="Hours (hh):", justify='right').grid(row=0, column=0, sticky=tkb.E)
        self.hour_spinbox = tkb.Spinbox(master=mill_time_frame, from_=0, to=23, justify='right', format="%02.0f", textvariable=self.hour_var, width=5, command=self.set_mill_time, state='disabled')   # Format with zero padding
        self.hour_spinbox.grid(row=0, column=1, padx=3, pady=5, sticky=(tkb.W, tkb.E))
        self.hour_spinbox.bind(sequence="<Return>",func=self.mill_time_spinbox_return)

        tkb.Label(master=mill_time_frame, text="Minutes (mm):", justify='right').grid(row=0, column=2, padx=(10,0), sticky=tkb.E)
        self.minute_spinbox = tkb.Spinbox(master=mill_time_frame, from_=0, to=59, justify='right', format="%02.0f", textvariable=self.minute_var, width=5, command=self.set_mill_time, state='disabled')   # Format with zero padding
        self.minute_spinbox.grid(row=0, column=3, padx=3, pady=5, sticky=(tkb.W, tkb.E))
        self.minute_spinbox.bind(sequence="<Return>", func=self.mill_time_spinbox_return)

        tkb.Label(master=mill_time_frame, text="Total Time (hh:mm):").grid(row=1, column=0, sticky=tkb.E)
        self.mill_total_time_value = tkb.Entry(master=mill_time_frame, 
            state='readonly', 
            textvariable=self.mill_total_time_var,
            width=10, 
            justify='right'
            ).grid(row=1, column=1, padx=5, pady=5, sticky=(tkb.W, tkb.E))
        self.set_mill_time()

        # Mill control frame
        mill_radio_frame = tkb.Labelframe(master=self.milling_tab, text="Mill Control", padding=10)
        mill_radio_frame.grid(row=1, column=0, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        # Radio Buttons
        # Must call .grid() independently to allow .configure() to be called 
        self.vessel_rpm_radio = tkb.Radiobutton(master=mill_radio_frame, text="Mill Vessel Speed (RPM):", variable=self.mill_radio_var, value=1, command=self.set_drive_frequency, state='disabled')
        self.vessel_rpm_radio.grid(row=0, column=0, padx=5, pady=5, sticky=tkb.W)
        self.shaft_rpm_radio = tkb.Radiobutton(master=mill_radio_frame, text="Drive Shaft Speed (RPM):", variable=self.mill_radio_var, value=2, command=self.set_drive_frequency, state='disabled')
        self.shaft_rpm_radio.grid(row=1, column=0, padx=5, pady=5, sticky=tkb.W)
        self.vfd_freq_radio = tkb.Radiobutton(master=mill_radio_frame, text="VFD Frequency (Hz):", variable=self.mill_radio_var, value=3, command=self.set_drive_frequency, state='disabled')
        self.vfd_freq_radio.grid(row=2, column=0, padx=5, pady=5, sticky=tkb.W)

        self.vessel_rpm_spinbox = tkb.Spinbox(master=mill_radio_frame, from_=0, to=100, textvariable=self.vessel_rpm_var, justify='right', width=5, command=self.set_drive_frequency, state='disabled')   # Format with zero padding
        self.vessel_rpm_spinbox.grid(row=0, column=1, padx=3, pady=5, sticky=(tkb.W, tkb.E))
        self.vessel_rpm_spinbox.bind("<Return>", self.mill_ctrl_spinbox_return)
        vessel_rpm_str = str(f"\nThe Mill Vessel Speed is a function of the ratio of the drive shaft to mill vessel diameters, multiplied by the drive shaft speed.")
        tkb.Label(master=mill_radio_frame, text=vessel_rpm_str, justify='left',wraplength=300).grid(row=0, column=2, padx=5, sticky=(tkb.N,tkb.W))
        
        self.shaft_rpm_spinbox = tkb.Spinbox(master=mill_radio_frame, from_=0, to=750, textvariable=self.shaft_rpm_var, justify='right', width=5, command=self.set_drive_frequency, state='disabled')   # Format with zero padding
        self.shaft_rpm_spinbox.grid(row=1, column=1, padx=3, pady=5, sticky=(tkb.W, tkb.E))
        self.shaft_rpm_spinbox.bind("<Return>", self.mill_ctrl_spinbox_return)
        shaft_rpm_str = str(f"\nFor direct coupling, the Drive Shaft Speed is equal to the motor speed.")
        tkb.Label(master=mill_radio_frame, text=shaft_rpm_str, justify='left',wraplength=300).grid(row=1, column=2, padx=5, sticky=(tkb.N,tkb.W))
        
        self.vfd_freq_spinbox = tkb.Spinbox(master=mill_radio_frame, from_=0, to=50, increment=0.1, format="%04.1f", textvariable=self.vfd_freq_var, justify='right', width=5, command=self.set_drive_frequency, state='disabled')   # Format with zero padding
        self.vfd_freq_spinbox.grid(row=2, column=1, padx=3, pady=5, sticky=(tkb.W, tkb.E))
        self.vfd_freq_spinbox.bind("<Return>", self.mill_ctrl_spinbox_return)
        self.vfd_freq_str = str(f"\nThe VFD Output Frequency is proportional to the motor speed.")
        tkb.Label(master=mill_radio_frame, text=self.vfd_freq_str, justify='left',wraplength=300).grid(row=2, column=2, padx=5, sticky=(tkb.N,tkb.W))

        tkb.Separator(master=mill_radio_frame, orient='horizontal', bootstyle="default").grid(row=3, columnspan=3, padx=5, pady=20, sticky=(tkb.E,tkb.W))

        tkb.Label(master=mill_radio_frame, text="Motor Rotation:", justify='right').grid(row=4, column=0, padx=5, pady=5, sticky=tkb.E)
        self.motor_forward = tkb.Radiobutton(master=mill_radio_frame, text="Forward (CCW)", variable=self.motor_rotation_var, value=0, state='disabled')
        self.motor_forward.grid(row=4, column=1, padx=5, pady=5, sticky=tkb.W)
        self.motor_reverse = tkb.Radiobutton(master=mill_radio_frame, text="Reverse (CW)", variable=self.motor_rotation_var, value=2, state='disabled')
        self.motor_reverse.grid(row=4, column=2, padx=5, pady=5, sticky=tkb.W)

        # Mill Dimensions Frame
        mill_dim_frame = tkb.Labelframe(master=self.milling_tab, text="Mill Dimensions", padding=10)
        mill_dim_frame.grid(row=3, column=0, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        self.mill_edit_toggle = tkb.Checkbutton(master=mill_dim_frame, 
            bootstyle=(SUCCESS),
            text="Edit Parameters",
            variable=self.mill_edit_var, 
            command=self.edit_mill_diameter,
            state='disabled'
            )
        self.mill_edit_toggle.grid(row=0, column=0, padx=3, pady=3, sticky=(tkb.N, tkb.W))

        tkb.Label(master=mill_dim_frame, text="Mill Vessel Diameter (mm):", justify='right').grid(row=1, column=0, sticky=tkb.W)
        self.vessel_dia_spinbox = tkb.Spinbox(master=mill_dim_frame, from_=50, to=300, increment=0.1, format="%.1f", textvariable=self.mill_dia_var, justify='right', width=5, command=self.set_drive_frequency, state='disabled')
        self.vessel_dia_spinbox.grid(row=1, column=1, padx=3, pady=5, sticky=(tkb.W, tkb.E))
        self.vessel_dia_spinbox.bind("<Return>", self.mill_ctrl_spinbox_return)
        
        tkb.Label(master=mill_dim_frame, text="Drive Shaft Diameter (mm):", justify='right').grid(row=2, column=0, sticky=tkb.W)
        self.shaft_dia_spinbox = tkb.Spinbox(master=mill_dim_frame, from_=25.4, to=50, increment=0.1, format="%.1f", textvariable=self.shaft_dia_var, justify='right', width=5, command=self.set_drive_frequency, state='disabled')
        self.shaft_dia_spinbox.grid(row=2, column=1, padx=3, pady=5, sticky=(tkb.W, tkb.E))
        self.vessel_dia_spinbox.bind("<Return>", self.mill_ctrl_spinbox_return)


    def create_VFD(self):
        # COM Port frame
        com_frame = tkb.Labelframe(master=self.motor_tab, text="Serial Communication", padding=10)
        com_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        tkb.Label(master=com_frame, text="COM Port:").grid(row=0, column=0, sticky=tkb.E)
        self.com_port = tkb.Combobox(master=com_frame, width=10, textvariable=self.com_port_var)
        self.com_port.grid(row=0, column=1, padx=3, pady=10, sticky=(tkb.W, tkb.E))

        #self.com_refresh_button = tkb.Button(master=com_frame, text="Refresh Ports", command=self.get_com_ports, bootstyle=(LIGHT, OUTLINE))
        self.com_refresh_button = tkb.Button(master=com_frame, text="Refresh Ports", command=self.find_inverter, bootstyle=(LIGHT, OUTLINE))
        self.com_refresh_button.grid(row=0, column=2, padx=10, pady=10, sticky=tkb.W)

        tkb.Label(master=com_frame, text="Disconnect").grid(row=1, column=0, pady=1, sticky=(tkb.N, tkb.E))
        self.com_connect_toggle = tkb.Checkbutton(master=com_frame,
            bootstyle=(SUCCESS, ROUND, TOGGLE),
            text="Connect",
            variable=self.com_connect_var, 
            command=self.connect_inverter,
            )
        self.com_connect_toggle.grid(row=1, column=1, padx=3, pady=3, sticky=(tkb.N, tkb.W))
        
        str_protocol = str(f"\nParameters for communication with the VFD\nare programmed directly into the unit.\n"
            f"\n[09-00]   Address\t=   1"
            f"\n[09-01]   Mode\t=   RTU"
            f"\n[09-02]   Baud\t=   19200"
            f"\n[09-03]   Stop Bits\t=   1"
            f"\n[09-04]   Parity\t=   None"
            f"\n[09-05]   Data Bits\t=   8"
            )
        tkb.Label(master=com_frame, text=str_protocol, justify='left').grid(row=0, column=4, rowspan=2, sticky=tkb.W)

        # VFD Information Frame
        drive_info_frame = tkb.Labelframe(master=self.motor_tab, text="VFD Information", padding=10)
        drive_info_frame.grid(row=1, column=0, padx=5, pady=5, sticky=(tkb.N, tkb.S, tkb.W, tkb.E))

        tkb.Label(master=drive_info_frame, text="[13-00]   Model:").grid(row=1, column=0, sticky=tkb.W)
        self.vfd_model_value = tkb.Entry(master=drive_info_frame, 
            state='readonly', 
            textvariable=self.vfd_model_var,
            width=10,
            justify='right'
            ).grid(row=1, column=1, padx=5, pady=5, sticky=tkb.W)

        tkb.Label(master=drive_info_frame, text="[13-01]   Firmware:").grid(row=2, column=0, sticky=tkb.W)
        self.vfd_firmware_value = tkb.Entry(master=drive_info_frame, 
            state='readonly', 
            textvariable=self.vfd_firmware_var,
            width=10,
            justify='right'
            ).grid(row=2, column=1, padx=5, pady=5, sticky=tkb.W)

        # Motor Nameplate Frame
        motor_info_frame = tkb.Labelframe(master=self.motor_tab, text="Motor Nameplate", padding=10)
        motor_info_frame.grid(row=1, column=1, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        tkb.Label(master=motor_info_frame, text="[02-04]   Motor Voltage (V):").grid(row=0, column=0, sticky=tkb.W)
        self.motor_voltage_value = tkb.Entry(master=motor_info_frame, 
            state='readonly', 
            textvariable=self.motor_voltage_var,
            width=10,
            justify='right'
            ).grid(row=0, column=1, padx=5, pady=5, sticky=tkb.W)
        
        tkb.Label(master=motor_info_frame, text="[02-01]   Motor Amps (A):").grid(row=1, column=0, sticky=tkb.W)
        self.motor_amps_value = tkb.Entry(master=motor_info_frame, 
            state='readonly', 
            textvariable=self.motor_amps_var,
            width=10,
            justify='right'
            ).grid(row=1, column=1, padx=5, pady=5, sticky=tkb.W)
        
        tkb.Label(master=motor_info_frame, text="[02-06]   Motor Frequency (Hz):").grid(row=2, column=0, sticky=tkb.W)
        self.motor_freq_value = tkb.Entry(master=motor_info_frame, 
            state='readonly', 
            textvariable=self.motor_freq_var,
            width=10,
            justify='right'
            ).grid(row=2, column=1, padx=5, pady=5, sticky=tkb.W)
        
        tkb.Label(master=motor_info_frame, text="[02-03]   Motor Speed (RPM):").grid(row=3, column=0, sticky=tkb.W)
        self.motor_maxrpm_value = tkb.Entry(master=motor_info_frame, 
            state='readonly', 
            textvariable=self.motor_maxrpm_var,
            width=10,
            justify='right'
            ).grid(row=3, column=1, padx=5, pady=5, sticky=tkb.W)
        
        # Frame (for images and heading)
        context_frame = tkb.Labelframe(master=self.motor_tab, text="Description", padding=5)
        context_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky=(tkb.W, tkb.E))

        # Description
        context_txt = str(
            f"The interface provides RS485 serial communication with an TECO E510-201-H1FN4S Inverter for control of a Techtop TAI-90S 8-pole 0.37kW single-phase motor to rotate a milling vessel at a prescribed speed (RPM)."
            f"\n\nAC drives are generally known by many different names: Adjustable Frequency Drives (AFD), Variable Frequency Drives (VFD), and Inverters. "
            f"AC drives function by converting incoming AC power to DC, which is then synthesized back into single/three phase output power. "
            f"The voltage and frequency of this synthesized output power is directly varied by the drive, where the frequency determines the speed of the single/three phase AC induction motor."
            )
        context_label = tkb.Label(master=context_frame, text=context_txt, wraplength=580, padding=5)
        context_label.grid(row=0, column=0, sticky=(tkb.W, tkb.E))


    def create_about(self):
        guide_frame = tkb.Labelframe(master=self.about_tab, text="Guide", padding=5)
        guide_frame.grid(row=0, column=0, padx=5, pady=5, sticky=(tkb.W, tkb.E))
        guide_txt = str(
            f"Follow these steps to initiate communication with the VFD."
            f"\n\n1. Switch to the 'VFD Parameters' tab:" 
            f"\n(a) Select the COM Port from the drop-down list"
            f"\n(b) Toggle the switch to establish a serial connectiuon. If successful, the VFD Information and Motor Nameplate fields are populated. If unsuccessful, try a different COM Port."
            f"\n\n2. Switch to the 'Mill Settings' tab:" 
            f"\n(a) Enter the milling hours and minutes using the spin-button fields. Default is 2 hours (02:00)"
            f"\n(b) Select the radio-button for the method of control. Enter the desired value in the corresponding spin-button field. Default is Mill Vessel Speed = 74 RPM"
            f"\n\n3. Switch to the 'Status' tab:" 
            f"\n(a) After confirming that mill vessel(s) are correctly loaded and balanced on the rollers, click the 'RUN' button."
            f"\n\nMeters display the ouput of the VFD and estimate of the mill vessel speed together with the elapsed (count-up) and remaining (count-down) time. The application will stop the rotation of the mill vessel(s) after the count-down timer reaches zero. "
            f"If necessary, the rotation of the mill vessel(s) can be stopped by clicking the 'STOP' button."
            )
        guide_label = tkb.Label(master=guide_frame, text=guide_txt, wraplength=580, padding=5)
        guide_label.grid(row=0, column=0, sticky=(tkb.W, tkb.E)) 

        watermark_frame = tkb.Labelframe(master=self.about_tab, text="Watermark", padding=5)
        watermark_frame.grid(row=1, column=0, padx=5, pady=5, sticky=(tkb.W, tkb.E))
        # https://github.com/rasbt/watermark 
        watermark_txt = watermark.watermark(
            author="Name (organisation)", 
            python=True,
            packages="minimalmodbus,ttkbootstrap", 
            updated=True, 
            current_date=True,
            )
        watermark_label = tkb.Label(master=watermark_frame, text=watermark_txt, wraplength=580, padding=5)
        watermark_label.grid(row=0, column=0, sticky=(tkb.W, tkb.E)) 


    def create_footer(self):
        """
        Create elements for establishing serial communications with each MFC.  
        """
        # Message frame
        message_frame = tkb.Labelframe(master=self, text="Messages", padding=5)
        message_frame.grid(row=3, column=0, columnspan=3, padx=10, pady=10, sticky=(tkb.W, tkb.E))

        self.message_label = tkb.Label(master=message_frame, textvariable=self.message_var, justify='left')
        self.message_label.grid(row=0, column=0, sticky=(tkb.W, tkb.E))

        # Quit button
        self.quit_button = tkb.Button(master=self, text="Quit", command=self.quit, bootstyle=(DANGER, OUTLINE))
        self.quit_button.grid(row=4, column=2, padx=10, pady=10, sticky=(tkb.E))
    

    ##############################################################################
    # APPLICATION FUNCTIONS
    ##############################################################################
    def get_com_ports(self):
        """
        Identify the available serial COM ports connected to the PC. 
        """
        all_comports = serial.tools.list_ports.comports()
        filtered_comports= [port.device for port in all_comports if port.manufacturer != "Intel"]    #!= is not
        filtered_comports.sort()    # Sort the lost in ascending order.
        self.message_var.set(value=f'Number of available COM ports : {len(filtered_comports)}')
        self.com_port.configure(values=filtered_comports)
        return filtered_comports


    def find_inverter(self):
        """
        Cycle through list of available COM ports and match to the TECO E510 inverter.
        """ 
        self.inverter = None    # Reset inverter
        self.message_var.set(value=f'Discovering TECO E510 Inverter ...')
        comports = self.get_com_ports()
        for port in comports:
            self.message_var.set(value=f'TECO E510 Inverter : attempting serial communication with {port} ...')
            self.inverter = TECO_E510_Inverter(port=port, slaveaddress=1)
            try: 
                self.inverter.get_inverter_details()
                if self.inverter.model is not None:
                    # SUCCESSFUL COMMUNICATION
                    self.message_var.set(value=f'TECO E510 Inverter : successful association with {port} ...')
                    break
            except Exception:
                self.inverter.serial.close
                self.inverter = None
                continue
        if self.inverter is None:
            self.message_var.set(value='ERROR : TECO E510 Inverter - no communication on any available COM port.')
            self.com_connect_toggle.configure(state='disabled')
            Messagebox.show_error(
                message=str(
                    f"ERROR : TECO E510 Inverter - no communication on any available COM port."
                    f"\nCheck device is powered and cables are connected."
                    f"\nClick the 'Refresh Ports' button within the 'Serial Communication' frame of the 'VFD Parameters' tab and then choose another COM port."
                ),
                title="ERROR : TECO E510 Inverter"
            )
            self.notebook.select(tab_id=self.motor_tab)
        else: 
            self.com_port_var.set(value=self.inverter.serial.port)
            self.com_connect_var.set(value=True)
            self.connect_inverter()
    

    def connect_inverter(self):
        com_port = self.com_port.get()
        com_connect = self.com_connect_var.get()
        if com_connect:
            try:
                self.inverter = TECO_E510_Inverter(port=com_port, slaveaddress=1)
                self.read_inverter_info()
                self.serial_status(connected=com_connect)
                self.read_motor_info()
                self.enable_controls()
                self.stop_motor()
                self.start_sampling()
            except Exception as e:
                self.message_var.set(value="ERROR on connection to VFD: {}.\nCheck device is powered and cables are connected. Click 'Refresh Ports' button and then choose another COM Port.".format(str(e)))
                self.serial_status(connected=FALSE)
                self.disable_controls()
        else:
            # Disconnect from drive
            if self.inverter:
                self.inverter.serial.close()
                self.inverter = None
            self.disable_controls()
            self.stop_sampling()
            # Clear VFD and Motor Nameplate fields 
            self.clear_inverter_info()
            self.clear_motor_info()
            self.serial_status(connected=com_connect)
            self.message_var.set(value="DISCONNECT from VFD. No active communication.")
    

    def serial_status(self, connected):
        if connected:
            self.online_canvas.itemconfig(self.online_indicator, fill="green")
            self.online_label.configure(text="ONLINE")
        else:
            self.online_canvas.itemconfig(self.online_indicator, fill="red")
            self.online_label.configure(text="OFFLINE")
    

    def read_inverter_info(self):
        """
        Read inverter parameters (model and version).
        """
        try:
            self.inverter.get_inverter_details()
            self.vfd_model_var.set(value=self.inverter.model)
            self.vfd_firmware_var.set(value=self.inverter.version)
        except Exception:
            self.message_var.set(value=f"ERROR : TECO E510 Inverter - cannot read VFD information.")


    def clear_inverter_info(self):
        try:
            self.vfd_model_var.set(value='---')
            self.vfd_firmware_var.set(value='---')
        except Exception as e:
            self.message_var.set(value="ERROR : clearing VFD information: {}".format(str(e)))
    

    def read_motor_info(self):
        """
        Read motor parameters.
        """
        try: 
            self.inverter.get_motor_parameters()
            self.motor_voltage_var.set(value=self.inverter.motor_voltage)   # Register for motor voltage
            self.motor_amps_var.set(value=self.inverter.motor_current)      # Register for motor amps 
            self.motor_freq_var.set(value=self.inverter.motor_frequency)    # Register for motor base frequency 
            self.motor_maxrpm_var.set(value=self.inverter.motor_speed)      # Register for motor base speed 
            
            vfd_maxfreq = float(self.motor_freq_var.get())      # Convert String to float
            #motor_maxrpm = int(self.motor_maxrpm_var.get())     # Convert String to integer
            motor_maxrpm = round(120*(vfd_maxfreq/int(self.inverter.motor_poles)))    # 750 rpm

            # Update the max. value of vfd_freq_ widgets to match base frequency
            self.vfd_freq_spinbox.configure(to=vfd_maxfreq)
            self.vfd_freq_meter.configure(amounttotal=vfd_maxfreq)

            # Update the max. value of motor_rpm_ widgets to match max. motor speed 
            self.shaft_rpm_spinbox.configure(to=motor_maxrpm)
            self.motor_rpm_meter.configure(amounttotal=motor_maxrpm)

            # Calculate the max. value of vessel_rpm_ widgets 
            mill_dia = self.mill_dia_var.get()
            shaft_dia = self.shaft_dia_var.get()
            vessel_maxrpm = round(motor_maxrpm*(shaft_dia/mill_dia))
            self.vessel_rpm_spinbox.configure(to=vessel_maxrpm)
            self.mill_rpm_meter.configure(amounttotal=vessel_maxrpm)
        except Exception:
            self.message_var.set(value=f"ERROR : TECO E510 Inverter - cannot read motor parameters.")
    

    def clear_motor_info(self):
        try:
            self.motor_voltage_var.set(value='---')
            self.motor_amps_var.set(value='---')
            self.motor_freq_var.set(value='---')
            self.motor_maxrpm_var.set(value='---') 
        except Exception as e:
            self.message_var.set(value="ERROR clearing motor parameters: {}".format(str(e)))
    

    def enable_controls(self):
        self.run_button.configure(state='normal')
        self.stop_button.configure(state='normal', bootstyle='danger-outline')
        self.hour_spinbox.configure(state='normal')
        self.minute_spinbox.configure(state='normal')
        self.vessel_rpm_radio.configure(state='normal')
        self.shaft_rpm_radio.configure(state='normal')
        self.vfd_freq_radio.configure(state='normal')
        self.vessel_rpm_var.set(value=74)
        self.vessel_rpm_radio.invoke()
        self.motor_forward.configure(state='normal')
        self.motor_reverse.configure(state='normal')
        self.mill_edit_toggle.configure(state='normal')
    

    def disable_controls(self):
        self.run_button.configure(state='disabled')
        self.stop_button.configure(state='disabled')
        self.hour_spinbox.configure(state='disabled')
        self.minute_spinbox.configure(state='disabled')
        self.vessel_rpm_radio.configure(state='disabled')
        self.vessel_rpm_spinbox.configure(state='disabled')
        self.shaft_rpm_radio.configure(state='disabled')
        self.shaft_rpm_spinbox.configure(state='disabled')
        self.vfd_freq_radio.configure(state='disabled')
        self.vfd_freq_spinbox.configure(state='disabled')
        self.motor_forward.configure(state='disabled')
        self.motor_reverse.configure(state='disabled')
        self.mill_edit_toggle.configure(state='disabled')
    

    def edit_mill_diameter(self):
        if self.mill_edit_var.get():
            try:
                self.vessel_dia_spinbox.configure(state='normal')
                self.shaft_dia_spinbox.configure(state='normal')
            except Exception as e:
                self.message_var.set(value="ERROR setting parameters for mill vessel and/or drive shaft diameters: {}".format(str(e)))
        else:
            self.vessel_dia_spinbox.configure(state='disabled')
            self.shaft_dia_spinbox.configure(state='disabled')
    

    def set_mill_time(self):
        try:
            mill_hour = int(self.hour_var.get())
            mill_minute = int(self.minute_var.get())  
            # Concatenate (join) strings representing the milling period (hh:mm)
            mill_total_time_str = ":".join(['{:02}'.format(mill_hour), '{:02}'.format(mill_minute)])
            self.mill_total_time_var.set(value=mill_total_time_str)
        except Exception as e:
            self.message_var.set(value="ERROR setting mill duration: {}".format(str(e)))


    def mill_time_spinbox_return(self, event):
        self.set_mill_time()
    

    def set_drive_frequency(self):
        if self.inverter:
            try:
                mill_ctrl = self.mill_radio_var.get() 
                vfd_maxfreq = float(self.motor_freq_var.get()) 
                #motor_maxrpm = int(self.motor_maxrpm_var.get()) 
                motor_maxrpm = round(120*(vfd_maxfreq/int(self.inverter.motor_poles)))     # 750 rpm

                mill_dia = self.mill_dia_var.get() 
                shaft_dia = self.shaft_dia_var.get() 
                match mill_ctrl: 
                    case 1:     # Set control by Mill Vessel RPM
                        self.vessel_rpm_spinbox.configure(state='normal')
                        self.shaft_rpm_spinbox.configure(state='disabled')
                        self.vfd_freq_spinbox.configure(state='disabled')
                        # VFD frequency is calculated by setting vessel speed 
                        vessel_rpm = self.vessel_rpm_var.get() 
                        # Update value of Shaft/Motor RPM
                        shaft_rpm = round(vessel_rpm*(mill_dia/shaft_dia))
                        self.shaft_rpm_var.set(value=shaft_rpm)
                        # Update value of VFD Frequency 
                        self.vfd_freq_var.set(value=round((shaft_rpm/motor_maxrpm)*vfd_maxfreq,ndigits=1))
                    case 2:     # Set control by Shaft/Motor RPM
                        self.vessel_rpm_spinbox.configure(state='disabled')
                        self.shaft_rpm_spinbox.configure(state='normal')
                        self.vfd_freq_spinbox.configure(state='disabled')
                        # VFD frequency is calculated by setting motor speed 
                        shaft_rpm = self.shaft_rpm_var.get() 
                        # Update value of Mill Vessel RPM
                        self.vessel_rpm_var.set(value=round(shaft_rpm*(shaft_dia/mill_dia)))
                        # Update value of VFD Frequency 
                        self.vfd_freq_var.set(value=round((shaft_rpm/motor_maxrpm)*vfd_maxfreq,ndigits=1))
                    case 3:     # Set control by AC frequency
                        self.vessel_rpm_spinbox.configure(state='disabled')
                        self.shaft_rpm_spinbox.configure(state='disabled')
                        self.vfd_freq_spinbox.configure(state='normal')
                        vfd_freq = self.vfd_freq_var.get() 
                        # Update value of Shaft/Motor RPM
                        shaft_rpm = round(motor_maxrpm*(vfd_freq/vfd_maxfreq))
                        self.shaft_rpm_var.set(value=shaft_rpm)
                        # Update value of Mill Vessel RPM
                        self.vessel_rpm_var.set(value=round(shaft_rpm*(shaft_dia/mill_dia)))
                    case _:
                        raise Exception("There is no corresponding radio button value.")
                
                # VFD frequency is set directly 
                frequency = self.vfd_freq_var.get()
                if 0.0 <= frequency <= vfd_maxfreq:
                    self.inverter.set_frequency(frequency=frequency)
                    self.message_var.set(value="VFD output frequency set to {} Hz".format(frequency)) 
                else:
                    self.message_var.set(value="VFD output frequency out of range. Output frequency must be between 0.0 (stop) and {} (max.) Hz".format(vfd_maxfreq))
            except Exception as e:
                self.message_var.set(value="ERROR setting VFD output frequency: {}".format(str(e)))
    

    def mill_ctrl_spinbox_return(self, event):
        self.set_drive_frequency()
    

    def mill_period(self, start_time: datetime, stop_time: timedelta, total_time: timedelta):
        try:
            time_now = datetime.now()
            # Calculate the elapsed milling time (timedelta object)
            elapsed_time = time_now - start_time
            # Calculate the remaining milling time (timedelta object)
            remaining_time = stop_time - time_now + timedelta(seconds=1)
            # Calculate the progress 
            total_seconds = total_time.total_seconds()
            progress_seconds = elapsed_time.total_seconds()
            progress_ratio = round(progress_seconds/total_seconds,3)
            
            # STOP motor after set milling time
            if progress_ratio >= 1:
                self.stop_motor()

                progress_ratio = 1
                self.mill_progressbar_var.set(value=progress_ratio)     # Update the Progressbar Widget
                progress_percent = '{:.1%}'.format(progress_ratio)
                self.mill_percent_var.set(value=progress_percent)

                # Convert timedelta object(s) to a presentable format (hh:mm:ss)
                elapsed_time = datetime.strftime(datetime.strptime(str(total_time), '%H:%M:%S'),'%H:%M:%S')
                self.time_elapsed_var.set(value=elapsed_time)
                remaining_time = datetime.strftime(datetime.strptime(str('0'), '%S'),'%H:%M:%S')
                self.time_remain_var.set(value=remaining_time)

                self.message_var.set(value="STOPPED. Milling process completed.")
            
            else:
                self.mill_progressbar_var.set(value=progress_ratio)     # Update the Progressbar Widget
                progress_percent = '{:.1%}'.format(progress_ratio)
                self.mill_percent_var.set(value=progress_percent)

                # Convert timedelta object(s) to a presentable format (hh:mm:ss)
                elapsed_time = datetime.strftime(datetime.strptime(str(elapsed_time), '%H:%M:%S.%f'),'%H:%M:%S')
                self.time_elapsed_var.set(value=elapsed_time)
                remaining_time = datetime.strftime(datetime.strptime(str(remaining_time), '%H:%M:%S.%f'),'%H:%M:%S')
                self.time_remain_var.set(value=remaining_time)

        except Exception as e:
                self.message_var.set(value="ERROR calculating mill progress: {}".format(str(e)))


    def run_motor(self):
        """
        Set the operation mode of the inverter.
        (0 = stop forward, 1 = run forward, 2 = stop reverse, 3 = run reverse)
        """
        global mill_start_time
        global mill_stop_time
        global mill_total_time

        if self.inverter:
            try:
                # Disable editing of mill duration while the motor is running
                self.hour_spinbox.configure(state='disabled')
                self.minute_spinbox.configure(state='disabled')

                # Disable direction radio buttons while the motor is running
                self.motor_forward.configure(state='disabled')
                self.motor_reverse.configure(state='disabled')

                # Disable editing of mill diameters while the motor is running
                self.mill_edit_var.set(value=FALSE)
                self.edit_mill_diameter()
                self.mill_edit_toggle.configure(state='disabled',)

                # Modify the appearance of the RUN / STOP buttons
                self.run_button.configure(bootstyle=(SUCCESS, OUTLINE))
                self.pause_button.configure(state='normal', bootstyle=(WARNING))
                self.stop_button.configure(bootstyle=(DANGER))

                # Initialise the mill duration
                mill_hour = int(self.hour_var.get()) 
                mill_minute = int(self.minute_var.get()) 
                mill_total_time = timedelta(hours=mill_hour, minutes=mill_minute)
                mill_start_time = datetime.now()
                mill_stop_time = mill_start_time + mill_total_time

                """
                Run the motor
                operation (int): Operation mode (0 = stop forward, 1 = run forward, 2 = stop reverse, 3 = run reverse).
                """
                motor_direction = self.motor_rotation_var.get()
                operation = (1 + motor_direction)
                self.inverter.set_operation(operation=operation)
                self.message_var.set(value="RUNNING. Rotation: {}".format("Forward (CCW)" if motor_direction == 0 else "Reverse (CW)"))

            except Exception as e:
                self.message_var.set(value="ERROR on motor RUN command: {}".format(str(e)))
    

    def stop_motor(self):
        """
        Set the operation mode of the inverter to 0 = stop forward; 2 = stop reverse. 
        """
        if self.inverter:
            try:
                # Stop the motor
                motor_direction = self.motor_rotation_var.get() 
                self.inverter.set_operation(operation=motor_direction)
                self.message_var.set(value="STOPPED. Motor stopped.")

                # Enable editing of mill duration after the motor stops
                self.hour_spinbox.configure(state='normal')
                self.minute_spinbox.configure(state='normal')

                # Enable direction radio buttons when the motor stops
                self.motor_forward.configure(state='normal')
                self.motor_reverse.configure(state='normal')

                # Enable editing of mill diameters while the motor is stopped.
                self.mill_edit_toggle.configure(state='normal')

                # Modify the appearance of the RUN / STOP buttons
                self.run_button.configure(bootstyle=(SUCCESS))
                self.pause_button.configure(state='disabled', bootstyle=(WARNING, OUTLINE))
                self.stop_button.configure(bootstyle=(DANGER, OUTLINE))
            except Exception as e:
                self.message_var.set(value="ERROR on motor STOP command: {}".format(str(e)))


    def pause_motor(self):
        global mill_start_time
        global mill_stop_time
        
        if self.inverter:
            try:
                motor_paused = self.pause_var.get()
                match motor_paused: 
                    case 'PAUSE':
                        # Stop the motor
                        motor_direction = self.motor_rotation_var.get()     # 0 = forward ; 2 = reverse 
                        self.inverter.set_operation(operation=motor_direction)
                        self.message_var.set(value="PAUSED. Motor stopped. Click 'RESTART' button to continue.")

                        # Modify the appearance of the RUN, PAUSE/RESTART, STOP buttons
                        self.pause_var.set(value='RESTART')
                        self.run_button.configure(bootstyle=(SUCCESS, OUTLINE))
                        self.pause_button.configure(bootstyle=(WARNING))
                        self.stop_button.configure(bootstyle=(DANGER, OUTLINE))
                    case 'RESTART':
                        # Modify the appearance of the RUN, PAUSE/RESTART, STOP buttons
                        self.pause_var.set(value='PAUSE')
                        self.run_button.configure(bootstyle=(SUCCESS, OUTLINE))
                        self.pause_button.configure(bootstyle=(WARNING, OUTLINE))
                        self.stop_button.configure(bootstyle=(DANGER))

                        # Reset the mill duration (for continued data sampling)
                        elapsed_time = self.time_elapsed_var.get().split(sep=':')   # Split string into a list of [hours, minutes, seconds]
                        remaining_time = self.time_remain_var.get().split(sep=':')  # Split string into a list of [hours, minutes, seconds]
                        mill_start_time = datetime.now() - timedelta(hours=int(elapsed_time[0]), minutes=int(elapsed_time[1]), seconds=int(elapsed_time[2]))
                        mill_stop_time = datetime.now() + timedelta(hours=int(remaining_time[0]), minutes=int(remaining_time[1]), seconds=int(remaining_time[2]))

                        # RUN the motor
                        motor_direction = self.motor_rotation_var.get()     # 0 = forward ; 2 = reverse
                        operation = (1 + motor_direction)
                        self.inverter.set_operation(operation=operation)
                        self.message_var.set(value="RUNNING. Motor restarted after pause event.")
                
            except Exception as e:
                self.message_var.set(value="ERROR on motor PAUSE command: {}".format(str(e)))


    def start_sampling(self):
        self.sample_data()
        self.update_task = self.after(200, self.start_sampling)  # Schedule the next update


    def stop_sampling(self):
        if self.update_task:
            self.after_cancel(self.update_task)
            self.update_task = None


    def sample_data(self):
        """
        Read inverter status.
        """
        if self.inverter:
            try:
                mill_dia = self.mill_dia_var.get()
                shaft_dia = self.shaft_dia_var.get()
                
                self.inverter.get_status()  # Read VFD status
                vfd_freq = self.inverter.output_frequency   # Register for output frequency (Hz)
                """
                The line speed display is linearly proportional to the output frequency 0 to 50Hz.
                Motor synchronous speed = 120 x Rated frequency/Number of poles.
                """
                motor_rpm = int(120*(float(vfd_freq)/int(self.inverter.motor_poles)))

                # Calculate Mill Vessel Speed (RPM)
                mill_rpm = round(motor_rpm*(shaft_dia/mill_dia))

                # Update status fields 
                self.mill_rpm_var.set(value=mill_rpm)
                self.mill_rpm_meter.configure(amountused=mill_rpm)  # Update radial meter

                self.motor_rpm_var.set(value=motor_rpm)
                self.motor_rpm_meter.configure(amountused=motor_rpm)  # Update radial meter

                self.drive_freq_var.set(value=f"{vfd_freq:.1f}")
                self.vfd_freq_meter.configure(amountused=round(float(vfd_freq),ndigits=1))  # Update radial meter
                
                # Update milling progress - check if motor is running
                if self.inverter.operation == 1: 
                    self.mill_period(start_time=mill_start_time, stop_time=mill_stop_time, total_time=mill_total_time)
                
            except Exception as e:
                self.message_var.set(value="ERROR sampling data: {}".format(str(e)))


##############################################################################
# MAIN LOOP
##############################################################################
if __name__ == '__main__':

    app = tkb.Window(
        title="Ball Mill Controller", 
        themename="superhero", 
        iconphoto=MEDIA/'icon_32.png',
        position=(20,20)
        )
    BallMillControl(master=app)
    app.mainloop()

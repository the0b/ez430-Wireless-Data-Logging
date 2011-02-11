"""
Adapted from Eli Bendersky's demo of how to draw a dynamic mpl (matplotlib)
plot in a wxPython application.

Name:        MSP430 ADC Value Viewer
Purpose:     Plots the voltages communicated through UART from ez430-RF2500
Author:      Theo Brower
Updated:     13 January 2011
Licence:     Public domain
"""
import os
import pprint
import random
import sys
import wx
import serial

# The recommended way to use wx with mpl is with the WXAgg
# backend.
#
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar
import numpy as np
import pylab

buffersize = 2000

class DataRead(object):
    """
    This class reads data from the MSP430 serial port
    """
    #Flag to tell if new data has been read
    drawIt = 0
    counter = 0
    readingData = 0

    #Variable arrays to keep track of data
    A0 = []
    A1 = []
    A2 = []
    A3 = []
    A4 = []
    A12 = []
    xValues = []

    #initialize arrays to have [buffersize] elements, will keep track of
    #the last [buffersize] data points
    for i in range(buffersize):
        A0.append(0)
        A1.append(0)
        A2.append(0)
        A3.append(0)
        A4.append(0)
        A12.append(0)
        xValues.append(i)

    #Global variable to keep track of serial port configuration
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.timeout = .1
    port = 0

    def __init__(self):
        print "\n------------------------------------------------------------------"
        print "MSP430 ADC value viewer\nPress Ctrl+C to exit at any time"
        print "------------------------------------------------------------------"
        print "Available serial ports:"
        for n,s in self.scan():
            print "(%d) %s" % (n,s)


        #Ask user which serial port to open
        while self.port == 0:
            self.port = raw_input("\nWhich serial port is connected to the MSP430?\n\
    (Enter the number in parentheses or 'skip' for no connection)")
            if self.port == "skip":
                break
            try:
                self.port = int(self.port)
            except ValueError:
                self.port = 0
                print "error: port selection must be an integer"

        #Got the serial port information, use it
        if self.port != "skip":
            self.ser.port = self.port

            try:
                self.ser.open()
            except:
                print "Could not connect to serial port %d, exiting" %self.port
                exit()

            if self.ser.isOpen() == True:
                print "Serial port " + self.ser.portstr + " successfully opened, listening for data"
                print "Press Ctrl+C to exit or select File->Exit from the menu bar\n"
        else:
            print "Skipping serial connection, no serial port will be opened"

    def startADC(self):
        #send the ADC start signal through the serial port
        if self.ser.isOpen() == True:
            #send the letter s
            print "Sending ADC start signal through serial port"
            self.ser.write("s")
        else:
                print "Could not send ADC start signal, serial port is not open"

    def read(self):
        #read data from the serial port
        if self.ser.isOpen() == True:
            #read a serial port until endline ("\n")
            serialmsg = self.ser.readline()


            #if we read something, parse and convert ADC values to volts
            if len(serialmsg) > 0:
                if len(serialmsg)> 4 and serialmsg[4] == ',':
                    #parse message data
                    serialmsg = serialmsg.strip()
                    ADCvals = serialmsg.split(', ')

                    #Check for complete data set notification
                    if ADCvals[0] == '0682' \
                        and ADCvals[1] == '0682' \
                        and ADCvals[2] == '0682' \
                        and ADCvals[3] == '0682' \
                        and ADCvals[4] == '0682' \
                        and ADCvals[5] == '0682':
                        self.drawIt = 1
                        print "End of data set"
                        self.readingData = 0
                    #Data set is not complete, convert data to volts and store
                    else:
                        try:
                            for i in range (len(ADCvals)):
                                ADCvals[i] = float(ADCvals[i])*2.5/1023
                            print ["%0.3f" %i for i in ADCvals]
                            self.update_data_arrays(ADCvals)
                        except ValueError:
                            pass
                else:
                    print serialmsg,
        else:
            if self.port != "skip":
                print "serial port not open"


    #Scan function from the pySerial example code
    def scan(self):
        """scan for available ports. return a list of tuples (num, name)"""
        available = []
        for i in range(256):
            try:
                s = serial.Serial(i)
                available.append( (i, s.portstr))
                s.close()   # explicit close 'cause of delayed GC in java
            except serial.SerialException:
                pass
        return available
    def closeSerial(self):
        """Closes serial port for clean exit"""
        print "Closing serial port"
        try:
            self.ser.close();
        except:
            pass

    #This function updates the data arrays when given the latest data
    def update_data_arrays(self, lastReading):
        if len(lastReading) >= 6:
            self.A0.append(lastReading[0])
            self.A1.append(lastReading[1])
            self.A2.append(lastReading[2])
            self.A3.append(lastReading[3])
            self.A4.append(lastReading[4])
            self.A12.append(lastReading[5])
            self.A0.pop(0)
            self.A1.pop(0)
            self.A2.pop(0)
            self.A3.pop(0)
            self.A4.pop(0)
            self.A12.pop(0)
            self.counter += 1
            if self.counter%50 == 0:
                self.drawIt = 1
        else:
            print "Could not append corrupted data"

    def save_data_arrays(self, path):
        #write the data to the file
        datastring = ""
        thefile = open(path, 'w')

        #Write data Labels to file
        datastring = "A0, A1, A2, A3, A4, A12\n"
        thefile.write(datastring)
        #Now write the data
        for i in range(buffersize):
            datastring =    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n" %(self.A0[i],
                    self.A1[i], self.A2[i], self.A3[i], self.A4[i], self.A12[i])
            thefile.write(datastring)
        thefile.close()
        print "save successful"


class GraphFrame(wx.Frame):
    """ The main frame of the application
    """
    title = 'MSP430 ADC Value Plotter'
    refreshCounter = 0

    def __init__(self):
        wx.Frame.__init__(self, None, -1, self.title)

        #Instantiate DataRead class to talk to serial port
        self.dataread = DataRead()

        #Create wx window
        self.create_menu()
        self.create_status_bar()
        self.create_main_panel()

        #Start recurring timer for updating plot
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)
        self.redraw_timer.Start(5)

    def create_menu(self):
        #Configure window menu bar
        self.menubar = wx.MenuBar()

        menu_file = wx.Menu()
        m_expt = menu_file.Append(-1, "&Save plot image\tCtrl-S", "Save plot image")
        self.Bind(wx.EVT_MENU, self.on_save_plot, m_expt)
        m_startadc = menu_file.Append(-1, "&Start ADC\tCtrl-G",\
          "Trigger ADC to start taking data")
        self.Bind(wx.EVT_MENU, self.on_startADC, m_startadc)
        menu_file.AppendSeparator()
        m_exit = menu_file.Append(-1, "&Exit\tCtrl-X", "Exit")
        self.Bind(wx.EVT_MENU, self.on_exit, m_exit)    #exit event handler
        self.Bind(wx.EVT_CLOSE, self.on_close)      #window close event handler

        self.menubar.Append(menu_file, "&File")
        self.SetMenuBar(self.menubar)

    def create_main_panel(self):
        #Configure main window panel
        self.panel = wx.Panel(self)

        self.init_plot()
        self.canvas = FigCanvas(self.panel, -1, self.fig)

        self.save_button = wx.Button(self.panel, -1, "Save plot data")
        self.Bind(wx.EVT_BUTTON, self.on_save_button, self.save_button)
        self.startADC_button= wx.Button(self.panel, -1, "    Start ADC    ")
        self.Bind(wx.EVT_BUTTON, self.on_startADC, self.startADC_button)

        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.Add(self.save_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL )
        self.hbox1.Add(self.startADC_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)

        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.vbox.Add(self.hbox1, 0, flag=wx.ALIGN_RIGHT)

        self.panel.SetSizer(self.vbox)
        self.vbox.Fit(self)

    def create_status_bar(self):
        self.statusbar = self.CreateStatusBar()

    def init_plot(self):
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_axis_bgcolor('black')
        self.axes.set_title('MSP430 ADC Values', size=12)
        self.axes.grid(True, color='gray')
        self.axes.axis([0,buffersize,0,2.7])   #Axes limits:([xmin,xmax,ymin,ymax])

        pylab.setp(self.axes.get_xticklabels(), fontsize=8)
        pylab.setp(self.axes.get_yticklabels(), fontsize=8)

        # plot the data as a line series, and save the reference
        # to the plotted line series
        self.line0, = self.axes.plot(DataRead.xValues, DataRead.A0, label="A0")
        self.line1, = self.axes.plot(DataRead.xValues, DataRead.A1, label="A1")
        self.line2, = self.axes.plot(DataRead.xValues, DataRead.A2, label="A2")
        self.line3, = self.axes.plot(DataRead.xValues, DataRead.A3, label="A3")
        self.line4, = self.axes.plot(DataRead.xValues, DataRead.A4, label="A4")
        self.line12, = self.axes.plot(DataRead.xValues, DataRead.A12, label="A5")
        self.axes.legend(loc='upper center',
            prop=matplotlib.font_manager.FontProperties(size='xx-small'),
            ncol=6)
        #
    def draw_plot(self):
        """ Redraws the plot
        """
        #Read serial data and update the plot
        self.dataread.read()

        self.line0.set_ydata(DataRead.A0)
        self.line1.set_ydata(DataRead.A1)
        self.line2.set_ydata(DataRead.A2)
        self.line3.set_ydata(DataRead.A3)
        self.line4.set_ydata(DataRead.A4)
        self.line12.set_ydata(DataRead.A12)

        #Only readraw canvas only if the data set is complete
        #Do this to minimize the effect of suspected memory leak in canvas.draw()
        if self.dataread.drawIt == 1:
            self.canvas.draw()
            self.dataread.drawIt = 0
            #if end of data set, re-enable "start ADC" button
            if self.dataread.readingData == 0:
                self.startADC_button.SetLabel("Start ADC")
                self.startADC_button.Enable()

    def on_startADC(self, event):
        if self.dataread.readingData == 0:
            print "start ADC was triggered"
            self.dataread.startADC()
            print "trigger was sent, waiting for data"
            #Disable the button and menu item until data acquisition is finished
            self.startADC_button.Disable()
            self.startADC_button.SetLabel("Reading data...")
            self.dataread.readingData = 1
        else:
            print "ADC operation already in progress"


    def on_save_button(self, event):
        print "save button was pressed"
        #Write the data to a file, launch a file dialog box
        file_choices = "CSV (*.csv)|*.csv"
        dlg = wx.FileDialog(
            self,
            message="Save plot data as...",
            defaultDir=os.getcwd(),
            defaultFile="MSP430 data.csv",
            wildcard=file_choices,
            style=wx.SAVE | wx.OVERWRITE_PROMPT)

        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            self.dataread.save_data_arrays(path)



    def on_save_plot(self, event):
        file_choices = "PNG (*.png)|*.png"

        dlg = wx.FileDialog(
            self,
            message="Save plot as...",
            defaultDir=os.getcwd(),
            defaultFile="plot.png",
            wildcard=file_choices,
            style=wx.SAVE | wx.OVERWRITE_PROMPT)

        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            self.canvas.print_figure(path, dpi=self.dpi)
            self.flash_status_message("Saved to %s" % path)

    def on_redraw_timer(self, event):
        self.draw_plot()
        self.refreshCounter += 1

    def on_exit(self, event):
        self.redraw_timer.Stop()
        self.dataread.closeSerial()
        self.Destroy()

    def on_close(self, event):
        self.redraw_timer.Stop()
        self.dataread.closeSerial()
        self.Destroy()

    def flash_status_message(self, msg, flash_len_ms=1500):
        self.statusbar.SetStatusText(msg)
        self.timeroff = wx.Timer(self)
        self.Bind(
            wx.EVT_TIMER,
            self.on_flash_status_off,
            self.timeroff)
        self.timeroff.Start(flash_len_ms, oneShot=True)

    def on_flash_status_off(self, event):
        self.statusbar.SetStatusText('')


if __name__ == '__main__':
    app = wx.PySimpleApp()
    app.frame = GraphFrame()
    app.frame.Show()
    app.MainLoop()
import numpy as np
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from gcs_ui import *

ACCEL_PATH = 'results/accel.txt'
GYRO_PATH = 'results/gyro.txt'
MAGN_PATH = 'results/magn.txt'

accel_calibration = 0
gyro_calibration = 0
magn_calibration = 0


class CalibrationGlobe(gl.GLViewWidget):
    def __init__(self, *args, **kwargs):
        super(CalibrationGlobe, self).__init__(*args, **kwargs)
        self.setCameraPosition(distance=30)
        g = gl.GLGridItem()
        self.addItem(g)

        self.data3d = np.array([[0,0,0]])
        self.view3d = gl.GLScatterPlotItem(pos=self.data3d, color=[1,0,0,1], size=0.5, pxMode=False)
        self.addItem(self.view3d)

        self.data3d = np.delete(self.data3d, 0, axis=0)
        print(self.data3d)

        isc_coord = gl.GLAxisItem()
        isc_coord.setSize(10, 10, 10)
        # isc_coord.translate(0, 0, 0)
        self.addItem(isc_coord)

    def updateView(self, data):
        if (data == [0,0,0]):
            pass
        self.data3d = np.concatenate((self.data3d, [data]))
        self.view3d.setData(pos=self.data3d, color=[1,0,0,1], size=0.5, pxMode=False)
        pass


class MyWin(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle('gcs')

        # ----------------------------------------------
        self.accel_file = open(ACCEL_PATH, 'w')
        self.gyro_file = open(GYRO_PATH, 'w')
        self.magn_file = open(MAGN_PATH, 'w')
        # ----------------------------------------------

        pg.setConfigOption('background', 'k')
        pg.setConfigOption('foreground', 'w')

        # Variables
        self.accel_x = []
        self.accel_y = []
        self.accel_z = []

        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []

        self.magn_x = []
        self.magn_y = []
        self.magn_z = []

        self.time = []

        self.state = 255

        self.length = 20   # number of samples total on screen
        self.cut = 1        # number of samples to be removed on iteration, 1 makes maximum smooth

        # GUI
        self.ui.resetButton.pressed.connect(self.reset_graphs)

        # GLV
        self.ui.glv = pg.GraphicsLayoutWidget(self.ui.centralwidget)
        self.ui.graph_layout.addWidget(self.ui.glv)

        self.plot_item_accel = pg.PlotItem(title='Accelerometer')
        self.ui.glv.ci.addItem(self.plot_item_accel)
        self.ui.glv.ci.nextRow()

        self.plot_item_gyro = pg.PlotItem(title='Gyroscope')
        self.ui.glv.ci.addItem(self.plot_item_gyro)
        self.ui.glv.ci.nextRow()

        self.plot_item_magn = pg.PlotItem(title='Magnetometer')
        self.ui.glv.ci.addItem(self.plot_item_magn)

        # Text and 3D
        self.ui.dockwid.setTitleBarWidget(QtWidgets.QWidget(self.ui.dockwid))

        self.globe = CalibrationGlobe(parent=self)
        self.ui.glwid = self.globe
        self.ui.dockwid.setWidget(self.ui.glwid)
        self.ui.glwid.show()

        # graphs
        self.accel_x_graph = self.plot_item_accel.plot()
        self.accel_y_graph = self.plot_item_accel.plot()
        self.accel_z_graph = self.plot_item_accel.plot()

        self.gyro_x_graph = self.plot_item_gyro.plot()
        self.gyro_y_graph = self.plot_item_gyro.plot()
        self.gyro_z_graph = self.plot_item_gyro.plot()

        self.magn_x_graph = self.plot_item_magn.plot()
        self.magn_y_graph = self.plot_item_magn.plot()
        self.magn_z_graph = self.plot_item_magn.plot()

    @QtCore.pyqtSlot(str, int)
    def set_status(self, message, time=0):
        self.ui.statusBar.showMessage(message, time)

    @QtCore.pyqtSlot()
    def blank_status(self):
        self.ui.statusBar.showMessage("")

    @QtCore.pyqtSlot(list)
    def serial_msg(self, msgs):
        for i in range(len(msgs)):
            tmp = msgs[i].state
            if tmp != self.state:
                self.state = tmp
                phrase = 'device state '
                if tmp == 0:
                    phrase += 'OK'
                else:
                    phrase += 'ERROR: {}'.format(tmp)
                self.ui.textBrowser.append(phrase)

            self.accel_x.append(msgs[i].accel[0])
            self.accel_y.append(msgs[i].accel[1])
            self.accel_z.append(msgs[i].accel[2])

            self.gyro_x.append(msgs[i].gyro[0])
            self.gyro_y.append(msgs[i].gyro[1])
            self.gyro_z.append(msgs[i].gyro[2])

            self.magn_x.append(msgs[i].magn[0])
            self.magn_y.append(msgs[i].magn[1])
            self.magn_z.append(msgs[i].magn[2])

            self.time.append(msgs[i].time)

            self.accel_file.write("%f\t%f\t%f\n" % (msgs[i].accel[0], msgs[i].accel[1], msgs[i].accel[2]))
            self.gyro_file.write("%f\t%f\t%f\n" % (msgs[i].gyro[0], msgs[i].gyro[1], msgs[i].gyro[2]))
            self.magn_file.write("%f\t%f\t%f\n" % (msgs[i].magn[0], msgs[i].magn[1], msgs[i].magn[2]))

            self.accel_x_graph.setData(x=self.time, y=self.accel_x, pen=('r'), width=0.5)
            self.accel_y_graph.setData(x=self.time, y=self.accel_y, pen=('g'), width=0.5)
            self.accel_z_graph.setData(x=self.time, y=self.accel_z, pen=('b'), width=0.5)

            self.gyro_x_graph.setData(x=self.time, y=self.gyro_x, pen=('r'), width=0.5)
            self.gyro_y_graph.setData(x=self.time, y=self.gyro_y, pen=('g'), width=0.5)
            self.gyro_z_graph.setData(x=self.time, y=self.gyro_z, pen=('b'), width=0.5)

            self.magn_x_graph.setData(x=self.time, y=self.magn_x, pen=('r'), width=0.5)
            self.magn_y_graph.setData(x=self.time, y=self.magn_y, pen=('g'), width=0.5)
            self.magn_z_graph.setData(x=self.time, y=self.magn_z, pen=('b'), width=0.5)

            # Update scatter plot.
            self.globe.updateView(msgs[i].accel)

            if len(self.time) > self.length:
                self.time = self.time[self.cut:(self.length - 1)]

                self.accel_x = self.accel_x[self.cut:(self.length - 1)]
                self.accel_y = self.accel_y[self.cut:(self.length - 1)]
                self.accel_z = self.accel_z[self.cut:(self.length - 1)]

                self.gyro_x = self.gyro_x[self.cut:(self.length - 1)]
                self.gyro_y = self.gyro_y[self.cut:(self.length - 1)]
                self.gyro_z = self.gyro_z[self.cut:(self.length - 1)]

                self.magn_x = self.magn_x[self.cut:(self.length - 1)]
                self.magn_y = self.magn_y[self.cut:(self.length - 1)]
                self.magn_z = self.magn_z[self.cut:(self.length - 1)]

    @QtCore.pyqtSlot()
    def reset_graphs(self):
        self.accel_x = []
        self.accel_y = []
        self.accel_z = []
        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []
        self.magn_x = []
        self.magn_y = []
        self.magn_z = []
        self.time = []

        self.accel_x_graph.setData(x=self.time, y=self.accel_x, pen=('r'), width=0.5)
        self.accel_y_graph.setData(x=self.time, y=self.accel_y, pen=('g'), width=0.5)
        self.accel_z_graph.setData(x=self.time, y=self.accel_z, pen=('b'), width=0.5)

        self.gyro_x_graph.setData(x=self.time, y=self.gyro_x, pen=('r'), width=0.5)
        self.gyro_y_graph.setData(x=self.time, y=self.gyro_y, pen=('g'), width=0.5)
        self.gyro_z_graph.setData(x=self.time, y=self.gyro_z, pen=('b'), width=0.5)

        self.magn_x_graph.setData(x=self.time, y=self.magn_x, pen=('r'), width=0.5)
        self.magn_y_graph.setData(x=self.time, y=self.magn_y, pen=('g'), width=0.5)
        self.magn_z_graph.setData(x=self.time, y=self.magn_z, pen=('b'), width=0.5)
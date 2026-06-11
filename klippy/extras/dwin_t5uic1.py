# Support for the DWIN T5UIC1 serial LCD - the stock display on the
# Creality Ender 3 V2 / Ender 3 V2 Neo (and similar printers using a
# Creality 4.2.x mainboard).
#
# The display connects to the mainboard mcu over a uart (USART3
# PB10/PB11 on the stock 10 pin EXP3 header).  The mcu implements the
# T5UIC1 frame protocol (see src/lcd_dwin.c) while this module renders
# the original Creality user interface by sending drawing commands
# (the icons themselves live in the display's onboard flash, so the
# screen looks just like it did with the stock firmware).
#
# Reliability/bandwidth notes:
#  - Every T5UIC1 frame is sent atomically by the mcu; if the mcu's
#    transmit buffer is full the whole frame is dropped (never split),
#    the drop is reported back, and this module schedules a full
#    redraw of the current screen - so a lost frame heals itself.
#  - Status updates are rate limited (update_interval) and only fields
#    whose values actually changed are redrawn, keeping the host->mcu
#    bandwidth usage low.
#  - A periodic full redraw (full_redraw_interval) recovers from any
#    bytes corrupted on the wire.
#
# UI layout/behavior ported from the stock Marlin DWIN implementation
# via https://github.com/GalvanicGlaze/DWIN_T5UIC1_LCD
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mcu
from . import bus

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

# Klipper mcu messages are limited to 64 bytes - keep frame payloads
# safely below that (the mcu adds the 5 framing bytes itself)
DWIN_MAX_PAYLOAD = 48


def _MAX(lhs, rhs):
    return lhs if lhs > rhs else rhs

def _MIN(lhs, rhs):
    return lhs if lhs < rhs else rhs


######################################################################
# Low-level T5UIC1 chip interface (drawing primitives)
######################################################################

class T5UIC1:
    DWIN_WIDTH = 272
    DWIN_HEIGHT = 480

    font6x12 = 0x00
    font8x16 = 0x01
    font10x20 = 0x02
    font12x24 = 0x03
    font14x28 = 0x04
    font16x32 = 0x05
    font20x40 = 0x06
    font24x48 = 0x07
    font28x56 = 0x08
    font32x64 = 0x09

    Color_White = 0xFFFF
    Color_Yellow = 0xFF0F
    Color_Bg_Window = 0x31E8
    Color_Bg_Blue = 0x1125
    Color_Bg_Black = 0x0841
    Color_Bg_Red = 0xF00F
    Popup_Text_Color = 0xD6BA
    Line_Color = 0x3A6A
    Rectangle_Color = 0xEE2F
    Percent_Color = 0xFE29
    BarFill_Color = 0x10E4
    Select_Color = 0x33BB

    DWIN_FONT_MENU = font8x16
    DWIN_FONT_STAT = font10x20
    DWIN_FONT_HEAD = font10x20

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu.get_printer_mcu(self.printer,
                                       config.get('display_mcu', 'mcu'))
        self.uart_bus = config.get('uart_bus', 'usart3')
        self.baud = config.getint('baud', 115200, minval=1200)
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
        self.send_dwin_cmd = None
        self.handshake_ok = False
        self.tx_drop_callback = None
    def build_config(self):
        bus_id = bus.resolve_bus_name(self.mcu, "uart_bus", self.uart_bus)
        self.mcu.add_config_cmd("config_dwin oid=%d uart_bus=%s baud=%d"
                                % (self.oid, bus_id, self.baud))
        cmd_queue = self.mcu.alloc_command_queue()
        self.send_dwin_cmd = self.mcu.lookup_command(
            "dwin_send oid=%c data=%*s", cq=cmd_queue)
        self.mcu.register_response(self._handle_rx, "dwin_rx", self.oid)
        self.mcu.register_response(self._handle_tx_drops,
                                   "dwin_tx_drops", self.oid)
    def _handle_rx(self, params):
        data = bytearray(params['data'])
        if data[:3] == b'\x00OK':
            self.handshake_ok = True
    def _handle_tx_drops(self, params):
        logging.info("dwin_t5uic1: mcu dropped %d display frame(s)",
                     params['count'])
        if self.tx_drop_callback is not None:
            self.tx_drop_callback()
    def send(self, payload):
        if self.send_dwin_cmd is None:
            return
        payload = bytes(payload)
        if len(payload) > DWIN_MAX_PAYLOAD:
            payload = payload[:DWIN_MAX_PAYLOAD]
        self.send_dwin_cmd.send([self.oid, payload],
                                reqclock=BACKGROUND_PRIORITY_CLOCK)
    # Helpers to build big-endian payloads
    def _b(self, out, val):
        out.append(val & 0xFF)
    def _w(self, out, val):
        val = int(val) & 0xFFFF
        out.extend([(val >> 8) & 0xFF, val & 0xFF])
    def _l(self, out, val):
        val = int(val) & 0xFFFFFFFF
        out.extend([(val >> 24) & 0xFF, (val >> 16) & 0xFF,
                    (val >> 8) & 0xFF, val & 0xFF])
    def _d64(self, out, val):
        self._l(out, 0)
        self._l(out, val)
    # Display handshake (returns True if the display answered)
    def Handshake(self, retries=10):
        for i in range(retries):
            self.handshake_ok = False
            self.send(b'\x00')
            self.reactor.pause(self.reactor.monotonic() + 0.25)
            if self.handshake_ok:
                return True
        return False
    def Backlight_SetLuminance(self, luminance):
        self.send([0x30, _MAX(luminance, 0x1F) & 0xFF])
    def Frame_SetDir(self, direction):
        self.send([0x34, 0x5A, 0xA5, direction & 0xFF])
    def UpdateLCD(self):
        self.send([0x3D])
    def Frame_Clear(self, color):
        out = bytearray([0x01])
        self._w(out, color)
        self.send(out)
    def Draw_Line(self, color, xStart, yStart, xEnd, yEnd):
        out = bytearray([0x03])
        self._w(out, color)
        for v in (xStart, yStart, xEnd, yEnd):
            self._w(out, v)
        self.send(out)
    def Draw_Rectangle(self, mode, color, xStart, yStart, xEnd, yEnd):
        out = bytearray([0x05, mode & 0xFF])
        self._w(out, color)
        for v in (xStart, yStart, xEnd, yEnd):
            self._w(out, v)
        self.send(out)
    def Frame_AreaMove(self, mode, direction, dis, color,
                       xStart, yStart, xEnd, yEnd):
        out = bytearray([0x09, ((mode << 7) | direction) & 0xFF])
        self._w(out, dis)
        self._w(out, color)
        for v in (xStart, yStart, xEnd, yEnd):
            self._w(out, v)
        self.send(out)
    def Draw_String(self, widthAdjust, bShow, size, color, bColor, x, y,
                    string):
        maxlen = DWIN_MAX_PAYLOAD - 11
        data = str(string).encode('ascii', 'replace')[:maxlen]
        out = bytearray([0x11,
                         ((0x80 if widthAdjust else 0)
                          | (0x40 if bShow else 0) | (size & 0x0F))])
        self._w(out, color)
        self._w(out, bColor)
        self._w(out, x)
        self._w(out, y)
        out.extend(data)
        self.send(out)
    def Draw_IntValue(self, bShow, zeroFill, zeroMode, size, color, bColor,
                      iNum, x, y, value):
        out = bytearray([0x14,
                         ((0x80 if bShow else 0) | (0x20 if zeroFill else 0)
                          | (0x10 if zeroMode else 0) | (size & 0x0F))])
        self._w(out, color)
        self._w(out, bColor)
        out.append(iNum & 0xFF)
        out.append(0)
        self._w(out, x)
        self._w(out, y)
        self._d64(out, int(value))
        self.send(out)
    def Draw_FloatValue(self, bShow, zeroFill, zeroMode, size, color, bColor,
                        iNum, fNum, x, y, value):
        out = bytearray([0x14,
                         ((0x80 if bShow else 0) | (0x20 if zeroFill else 0)
                          | (0x10 if zeroMode else 0) | (size & 0x0F))])
        self._w(out, color)
        self._w(out, bColor)
        out.append(iNum & 0xFF)
        out.append(fNum & 0xFF)
        self._w(out, x)
        self._w(out, y)
        self._l(out, int(round(value)))
        self.send(out)
    def Draw_Signed_Float(self, size, bColor, iNum, fNum, x, y, value):
        if value < 0:
            self.Draw_String(False, True, size, self.Color_White, bColor,
                             x - 6, y, "-")
            self.Draw_FloatValue(True, True, 0, size, self.Color_White,
                                 bColor, iNum, fNum, x, y, -value)
        else:
            self.Draw_String(False, True, size, self.Color_White, bColor,
                             x - 6, y, " ")
            self.Draw_FloatValue(True, True, 0, size, self.Color_White,
                                 bColor, iNum, fNum, x, y, value)
    def JPG_ShowAndCache(self, jpg_id):
        out = bytearray()
        self._w(out, 0x2200)
        out.append(jpg_id & 0xFF)
        self.send(out)
    def ICON_Show(self, libID, picID, x, y):
        x = _MIN(x, self.DWIN_WIDTH - 1)
        y = _MIN(y, self.DWIN_HEIGHT - 1)
        out = bytearray([0x23])
        self._w(out, x)
        self._w(out, y)
        out.append(0x80 | (libID & 0x7F))
        out.append(picID & 0xFF)
        self.send(out)
    def JPG_CacheToN(self, n, jpg_id):
        self.send([0x25, n & 0xFF, jpg_id & 0xFF])
    def JPG_CacheTo1(self, jpg_id):
        self.JPG_CacheToN(1, jpg_id)
    def Frame_AreaCopy(self, cacheID, xStart, yStart, xEnd, yEnd, x, y):
        out = bytearray([0x27, 0x80 | (cacheID & 0x7F)])
        for v in (xStart, yStart, xEnd, yEnd, x, y):
            self._w(out, v)
        self.send(out)
    def Frame_TitleCopy(self, cacheID, x1, y1, x2, y2):
        self.Frame_AreaCopy(cacheID, x1, y1, x2, y2, 14, 8)


######################################################################
# Printer state interface
######################################################################

class select_t:
    def __init__(self):
        self.now = self.last = 0
    def set(self, v):
        self.now = self.last = v
    def reset(self):
        self.set(0)
    def changed(self):
        c = (self.now != self.last)
        if c:
            self.last = self.now
        return c
    def dec(self):
        if self.now:
            self.now -= 1
        return self.changed()
    def inc(self, v):
        if self.now < (v - 1):
            self.now += 1
        else:
            self.now = v - 1
        return self.changed()


class xyze_t:
    def __init__(self):
        self.x = self.y = self.z = self.e = 0.0


class HMI_value_t:
    E_Temp = 0
    Bed_Temp = 0
    Fan_speed = 0
    print_speed = 100
    Move_X_scale = 0.0
    Move_Y_scale = 0.0
    Move_Z_scale = 0.0
    Move_E_scale = 0.0
    offset_value = 0.0
    show_mode = 0


class HMI_Flag_t:
    pause_flag = False
    print_finish = False
    done_confirm_flag = False
    select_flag = False
    home_flag = False
    leveling_flag = False
    ETempTooLow_flag = False


class material_preset_t:
    def __init__(self, name, hotend_temp, bed_temp, fan_speed=100):
        self.name = name
        self.hotend_temp = hotend_temp
        self.bed_temp = bed_temp
        self.fan_speed = fan_speed


class PrinterData:
    HAS_HOTEND = True
    HAS_HEATED_BED = True
    HAS_FAN = False
    HAS_ZOFFSET_ITEM = True
    HAS_ONESTEP_LEVELING = False
    HAS_PREHEAT = True

    HOTEND_OVERSHOOT = 15
    BED_OVERSHOOT = 10
    MAX_E_TEMP = 260
    MIN_E_TEMP = 0
    BED_MAX_TARGET = 120
    MIN_BED_TEMP = 0
    EXTRUDE_MAXLENGTH = 200

    X_MIN_POS = 0.0
    Y_MIN_POS = 0.0
    Z_MIN_POS = 0.0
    X_MAX_POS = 235.0
    Y_MAX_POS = 235.0
    Z_MAX_POS = 250.0

    MACHINE_SIZE = "?"
    SHORT_BUILD_VERSION = "klipper"
    CORP_WEBSITE_E = "www.klipper3d.org"
    absolute_moves = True

    def __init__(self, config, run_gcode):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.run_gcode = run_gcode
        self.status = 'standby'
        self.file_name = None
        self.feedrate_percentage = 100
        self.BABY_Z_VAR = 0.0
        self.HMI_ValueStruct = HMI_value_t()
        self.HMI_flag = HMI_Flag_t()
        self.current_position = xyze_t()
        self.thermalManager = {
            'temp_bed': {'celsius': 0, 'target': 0},
            'temp_hotend': [{'celsius': 0, 'target': 0}],
            'fan_speed': [0],
        }
        self.material_preset = [
            material_preset_t(
                'PLA', config.getint('preheat_pla_hotend_temp', 200),
                config.getint('preheat_pla_bed_temp', 60)),
            material_preset_t(
                'ABS', config.getint('preheat_abs_hotend_temp', 210),
                config.getint('preheat_abs_bed_temp', 100)),
        ]
        self.files = []
        self.objects = {}
        self.job_status = {}
    def handle_ready(self):
        lookup = self.printer.lookup_object
        self.objects = {
            name: lookup(name, None)
            for name in ['toolhead', 'gcode_move', 'extruder', 'heater_bed',
                         'fan', 'virtual_sdcard', 'print_stats']}
        self.HAS_HOTEND = self.objects['extruder'] is not None
        self.HAS_HEATED_BED = self.objects['heater_bed'] is not None
        self.HAS_FAN = self.objects['fan'] is not None
        probe = lookup('probe', None) or lookup('bltouch', None)
        bed_mesh = lookup('bed_mesh', None)
        self.HAS_ONESTEP_LEVELING = (probe is not None
                                     and bed_mesh is not None)
        if self.HAS_HOTEND:
            heater = self.objects['extruder'].get_heater()
            self.MAX_E_TEMP = int(heater.max_temp - self.HOTEND_OVERSHOOT)
            self.MIN_E_TEMP = int(_MAX(heater.min_temp, 0))
        if self.HAS_HEATED_BED:
            heater = self.objects['heater_bed'].get_heater() \
                if hasattr(self.objects['heater_bed'], 'get_heater') \
                else self.objects['heater_bed'].heater
            self.BED_MAX_TARGET = int(heater.max_temp - self.BED_OVERSHOOT)
            self.MIN_BED_TEMP = int(_MAX(heater.min_temp, 0))
        toolhead = self.objects['toolhead']
        if toolhead is not None:
            eventtime = self.reactor.monotonic()
            sts = toolhead.get_status(eventtime)
            amin = sts.get('axis_minimum', (0., 0., 0., 0.))
            amax = sts.get('axis_maximum', (235., 235., 250., 0.))
            self.X_MIN_POS, self.Y_MIN_POS = amin[0], amin[1]
            self.Z_MIN_POS = _MAX(amin[2], 0.)
            self.X_MAX_POS, self.Y_MAX_POS = amax[0], amax[1]
            self.Z_MAX_POS = amax[2]
            self.MACHINE_SIZE = "%dx%dx%d" % (
                amax[0] - amin[0], amax[1] - amin[1], amax[2])
        start_args = self.printer.get_start_args()
        version = start_args.get('software_version', 'klipper')
        self.SHORT_BUILD_VERSION = version.split('-g')[0]
    def _get_status(self, name):
        obj = self.objects.get(name)
        if obj is None:
            return {}
        return obj.get_status(self.reactor.monotonic())
    def update_variable(self):
        update = False
        tm = self.thermalManager
        if self.HAS_HOTEND:
            sts = self._get_status('extruder')
            if tm['temp_hotend'][0]['celsius'] != int(sts['temperature']):
                tm['temp_hotend'][0]['celsius'] = int(sts['temperature'])
                update = True
            if tm['temp_hotend'][0]['target'] != int(sts['target']):
                tm['temp_hotend'][0]['target'] = int(sts['target'])
                update = True
        if self.HAS_HEATED_BED:
            sts = self._get_status('heater_bed')
            if tm['temp_bed']['celsius'] != int(sts['temperature']):
                tm['temp_bed']['celsius'] = int(sts['temperature'])
                update = True
            if tm['temp_bed']['target'] != int(sts['target']):
                tm['temp_bed']['target'] = int(sts['target'])
                update = True
        if self.HAS_FAN:
            sts = self._get_status('fan')
            if tm['fan_speed'][0] != int(sts['speed'] * 100. + .5):
                tm['fan_speed'][0] = int(sts['speed'] * 100. + .5)
                update = True
        gcm = self._get_status('gcode_move')
        if gcm:
            pos = gcm['gcode_position']
            cp = self.current_position
            cp.x, cp.y, cp.z, cp.e = pos[0], pos[1], pos[2], pos[3]
            self.absolute_moves = gcm['absolute_coordinates']
            self.feedrate_percentage = int(gcm['speed_factor'] * 100. + .5)
            z_offset = gcm['homing_origin'][2]
            if self.BABY_Z_VAR != z_offset:
                self.BABY_Z_VAR = z_offset
                self.HMI_ValueStruct.offset_value = z_offset * 100
                update = True
        psts = self._get_status('print_stats')
        if psts:
            self.file_name = psts['filename']
            self.status = psts['state']
            self.job_status = psts
        return update
    def ishomed(self):
        sts = self._get_status('toolhead')
        homed = sts.get('homed_axes', '')
        return 'x' in homed and 'y' in homed and 'z' in homed
    def printingIsPaused(self):
        return self.status in ('paused', 'pausing')
    def getPercent(self):
        sts = self._get_status('virtual_sdcard')
        if sts.get('is_active') or self.status == 'paused':
            return sts.get('progress', 0.) * 100.
        return 0
    def duration(self):
        return self.job_status.get('print_duration', 0.)
    def remain(self):
        percent = self.getPercent()
        duration = self.duration()
        if percent > 0.01:
            return duration / (percent / 100.) - duration
        return 0
    def GetFiles(self, refresh=False):
        if not self.files or refresh:
            vsd = self.objects.get('virtual_sdcard')
            if vsd is None:
                self.files = []
            else:
                self.files = [fname for fname, fsize
                              in vsd.get_file_list(check_subdirs=True)]
        return self.files
    def openAndPrintFile(self, filenum):
        self.file_name = self.files[filenum]
        self.run_gcode('SDCARD_PRINT_FILE FILENAME="%s"'
                       % (self.file_name.replace('"', ''),))
    def pause_job(self):
        self.run_gcode('PAUSE')
    def resume_job(self):
        self.run_gcode('RESUME')
    def cancel_job(self):
        self.run_gcode('CANCEL_PRINT')
    def set_feedrate(self, fr):
        self.feedrate_percentage = fr
        self.run_gcode('M220 S%d' % (fr,))
    def setExtTemp(self, target):
        self.run_gcode('M104 S%d' % (target,))
    def setBedTemp(self, target):
        self.run_gcode('M140 S%d' % (target,))
    def setFanSpeed(self, percent):
        self.run_gcode('M106 S%d' % (int(percent * 255. / 100. + .5),))
    def preheat(self, profile):
        idx = 0 if profile == "PLA" else 1
        preset = self.material_preset[idx]
        self.setBedTemp(preset.bed_temp)
        self.setExtTemp(preset.hotend_temp)
    def disable_all_heaters(self):
        self.setExtTemp(0)
        self.setBedTemp(0)
    def zero_fan_speeds(self):
        if self.HAS_FAN:
            self.run_gcode('M106 S0')
    def moveAbsolute(self, axis, position, feedrate, callback=None):
        script = "G90\nG1 %s%.3f F%d" % (axis, position, feedrate)
        if not self.absolute_moves:
            script += "\nG91"
        self.run_gcode(script, callback)
    def offset_z(self, adjust):
        self.run_gcode('SET_GCODE_OFFSET Z_ADJUST=%.3f MOVE=1' % (adjust,))
    def home(self, callback=None):
        self.run_gcode('G28', callback)
    def level_bed(self, callback=None):
        self.run_gcode('G28\nBED_MESH_CALIBRATE', callback)
    def can_extrude(self):
        if not self.HAS_HOTEND:
            return False
        return self._get_status('extruder').get('can_extrude', False)
    def save_settings(self):
        return True


######################################################################
# User interface (ported from the stock Creality DWIN firmware)
######################################################################

class DWIN_LCD:
    TROWS = 6
    MROWS = TROWS - 1  # Total rows, and other-than-Back
    TITLE_HEIGHT = 30  # Title bar height
    MLINE = 53         # Menu line height
    LBLX = 60          # Menu item label X
    MENU_CHR_W = 8
    STAT_CHR_W = 10
    MENU_CHAR_LIMIT = 24
    STATUS_Y = 360

    MSG_STOP_PRINT = "Stop Print"
    MSG_PAUSE_PRINT = "Pausing..."

    DWIN_SCROLL_UP = 2
    DWIN_SCROLL_DOWN = 3

    # Process ("checkkey") IDs
    MainMenu = 0
    SelectFile = 1
    Prepare = 2
    Control = 3
    Leveling = 4
    PrintProcess = 5
    AxisMove = 6
    TemperatureID = 7
    Motion = 8
    Info = 9
    Tune = 10
    PLAPreheat = 11
    ABSPreheat = 12
    Last_Prepare = 21
    Move_X = 24
    Move_Y = 25
    Move_Z = 26
    Extruder = 27
    ETemp = 28
    Homeoffset = 29
    BedTemp = 30
    FanSpeed = 31
    PrintSpeed = 32
    Print_window = 33
    Popup_Window = 34

    MINUNITMULT = 10

    ENCODER_DIFF_NO = 0
    ENCODER_DIFF_CW = 1
    ENCODER_DIFF_CCW = 2
    ENCODER_DIFF_ENTER = 3

    # Picture IDs (jpgs in the display's flash)
    Start_Process = 0
    Language_English = 1
    Language_Chinese = 2

    # Icon library and icon IDs (in the display's flash)
    ICON = 0x09

    ICON_LOGO = 0
    ICON_Print_0 = 1
    ICON_Print_1 = 2
    ICON_Prepare_0 = 3
    ICON_Prepare_1 = 4
    ICON_Control_0 = 5
    ICON_Control_1 = 6
    ICON_Leveling_0 = 7
    ICON_Leveling_1 = 8
    ICON_HotendTemp = 9
    ICON_BedTemp = 10
    ICON_Speed = 11
    ICON_Zoffset = 12
    ICON_Back = 13
    ICON_File = 14
    ICON_PrintTime = 15
    ICON_RemainTime = 16
    ICON_Setup_0 = 17
    ICON_Setup_1 = 18
    ICON_Pause_0 = 19
    ICON_Pause_1 = 20
    ICON_Continue_0 = 21
    ICON_Continue_1 = 22
    ICON_Stop_0 = 23
    ICON_Stop_1 = 24
    ICON_Bar = 25
    ICON_More = 26

    ICON_Axis = 27
    ICON_CloseMotor = 28
    ICON_Homing = 29
    ICON_SetHome = 30
    ICON_PLAPreheat = 31
    ICON_ABSPreheat = 32
    ICON_Cool = 33
    ICON_Language = 34

    ICON_MoveX = 35
    ICON_MoveY = 36
    ICON_MoveZ = 37
    ICON_Extruder = 38

    ICON_Temperature = 40
    ICON_Motion = 41
    ICON_WriteEEPROM = 42
    ICON_Info = 45

    ICON_SetEndTemp = 46
    ICON_SetBedTemp = 47
    ICON_FanSpeed = 48

    ICON_MaxSpeed = 51
    ICON_MaxAccelerated = 52
    ICON_MaxJerk = 53
    ICON_Step = 54
    ICON_PrintSize = 55
    ICON_Version = 56
    ICON_Contact = 57

    ICON_Rectangle = 77
    ICON_BLTouch = 78
    ICON_TempTooLow = 79
    ICON_AutoLeveling = 80
    ICON_TempTooHigh = 81
    ICON_Cancel_E = 87
    ICON_Confirm_E = 89
    ICON_Info_0 = 90
    ICON_Info_1 = 91

    # Menu item layout
    PREPARE_CASE_MOVE = 1
    PREPARE_CASE_DISA = 2
    PREPARE_CASE_HOME = 3
    PREPARE_CASE_ZOFF = 4
    PREPARE_CASE_PLA = 5
    PREPARE_CASE_ABS = 6
    PREPARE_CASE_COOL = 7
    PREPARE_CASE_TOTAL = 7

    CONTROL_CASE_TEMP = 1
    CONTROL_CASE_MOVE = 2
    CONTROL_CASE_INFO = 3
    CONTROL_CASE_TOTAL = 3

    MOTION_CASE_RATE = 1
    MOTION_CASE_ACCEL = 2
    MOTION_CASE_STEPS = 3
    MOTION_CASE_TOTAL = 3

    TUNE_CASE_SPEED = 1
    TUNE_CASE_TEMP = 2
    TUNE_CASE_BED = 3
    TUNE_CASE_FAN = 4
    TUNE_CASE_ZOFF = 5
    TUNE_CASE_TOTAL = 5

    TEMP_CASE_TEMP = 1
    TEMP_CASE_BED = 2
    TEMP_CASE_FAN = 3
    TEMP_CASE_PLA = 4
    TEMP_CASE_ABS = 5
    TEMP_CASE_TOTAL = 5

    PREHEAT_CASE_TEMP = 1
    PREHEAT_CASE_BED = 2
    PREHEAT_CASE_FAN = 3
    PREHEAT_CASE_SAVE = 4
    PREHEAT_CASE_TOTAL = 4

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.lcd = T5UIC1(config)
        self.lcd.tx_drop_callback = self._handle_tx_drops
        self.pd = PrinterData(config, self.run_gcode)
        # Encoder / button input
        buttons = self.printer.load_object(config, "buttons")
        encoder_pins = config.get('encoder_pins')
        try:
            pin1, pin2 = [p.strip() for p in encoder_pins.split(',')]
        except:
            raise config.error("Unable to parse encoder_pins")
        steps_per_detent = config.getchoice('encoder_steps_per_detent',
                                            [2, 4], 4)
        if config.getboolean('reverse_encoder_direction', False):
            pin1, pin2 = pin2, pin1
        buttons.register_rotary_encoder(pin1, pin2,
                                        self._encoder_cw_callback,
                                        self._encoder_ccw_callback,
                                        steps_per_detent)
        buttons.register_buttons([config.get('click_pin')],
                                 self._click_callback)
        # Update timing
        self.update_interval = config.getfloat('update_interval', 2.,
                                               minval=.2, maxval=30.)
        self.full_redraw_interval = config.getfloat('full_redraw_interval',
                                                    30., minval=0.,
                                                    maxval=600.)
        # UI state
        self.checkkey = self.MainMenu
        self.select_page = select_t()
        self.select_file = select_t()
        self.select_print = select_t()
        self.select_prepare = select_t()
        self.select_control = select_t()
        self.select_axis = select_t()
        self.select_temp = select_t()
        self.select_motion = select_t()
        self.select_tune = select_t()
        self.select_PLA = select_t()
        self.select_ABS = select_t()
        self.index_file = self.MROWS
        self.index_prepare = self.MROWS
        self.index_control = self.MROWS
        self.index_tune = self.MROWS
        self.dwin_zoffset = 0.0
        self.last_status = 'standby'
        self.last_status_draw = {}
        self.last_progress_draw = {}
        self.need_full_redraw = False
        self.last_full_redraw = 0.
        self.init_done = False
        self.gcode_queue = []
        self.update_timer = self.reactor.register_timer(self._update_event)
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)

    ##################################################################
    # Setup / event plumbing
    ##################################################################

    def _handle_ready(self):
        self.pd.handle_ready()
        self.reactor.register_callback(self._init_display)
    def _init_display(self, eventtime):
        try:
            self._init_display_inner()
        except Exception:
            logging.exception("dwin_t5uic1: error initializing display")
    def _init_display_inner(self):
        if not self.lcd.Handshake():
            logging.warning("dwin_t5uic1: display did not respond to"
                            " handshake; continuing anyway")
        self.lcd.JPG_ShowAndCache(self.Start_Process)
        self.lcd.Frame_SetDir(1)
        self.lcd.UpdateLCD()
        # Cache the (english) text bitmap used by Frame_AreaCopy labels
        self.lcd.JPG_CacheTo1(self.Language_English)
        # Brief boot splash, then enter the regular UI
        self.reactor.pause(self.reactor.monotonic() + 1.5)
        self.pd.update_variable()
        self.last_status = self.pd.status
        self.init_done = True
        self.HMI_StartFrame(True)
        self.reactor.update_timer(self.update_timer, self.reactor.NOW)
    def _handle_shutdown(self):
        if not self.init_done:
            return
        self.init_done = False
        self.reactor.update_timer(self.update_timer, self.reactor.NEVER)
        try:
            self.Clear_Main_Window()
            self.Draw_Popup_Bkgd_60()
            self.lcd.Draw_String(
                False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
                self.lcd.Color_Bg_Window, (272 - 8 * 14) // 2, 210,
                "Printer halted")
            self.lcd.Draw_String(
                False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
                self.lcd.Color_Bg_Window, (272 - 8 * 16) // 2, 240,
                "See klippy log.")
            self.lcd.UpdateLCD()
        except Exception:
            logging.exception("dwin_t5uic1: error drawing shutdown screen")
    def _handle_tx_drops(self):
        # The mcu had to drop a display frame - the screen contents may
        # be incomplete, so schedule a full redraw of everything.
        self.need_full_redraw = True
    def run_gcode(self, script, callback=None):
        if not script:
            if callback is not None:
                callback()
            return
        if not self.gcode_queue:
            self.reactor.register_callback(self._dispatch_gcode)
        self.gcode_queue.append((script, callback))
    def _dispatch_gcode(self, eventtime):
        while self.gcode_queue:
            script, callback = self.gcode_queue[0]
            try:
                self.gcode.run_script(script)
            except Exception:
                logging.exception("dwin_t5uic1: error running script %r",
                                  script)
            self.gcode_queue.pop(0)
            if callback is not None:
                try:
                    callback()
                except Exception:
                    logging.exception("dwin_t5uic1: script callback error")

    ##################################################################
    # Encoder handling
    ##################################################################

    def _encoder_cw_callback(self, eventtime):
        self._encoder_event(self.ENCODER_DIFF_CW)
    def _encoder_ccw_callback(self, eventtime):
        self._encoder_event(self.ENCODER_DIFF_CCW)
    def _click_callback(self, eventtime, state):
        if state:
            self._encoder_event(self.ENCODER_DIFF_ENTER)
    def _encoder_event(self, encoder_diffState):
        if not self.init_done:
            return
        try:
            self.encoder_has_data(encoder_diffState)
        except Exception:
            logging.exception("dwin_t5uic1: error handling input event")
    def encoder_has_data(self, s):
        handlers = {
            self.MainMenu: self.HMI_MainMenu,
            self.SelectFile: self.HMI_SelectFile,
            self.Prepare: self.HMI_Prepare,
            self.Control: self.HMI_Control,
            self.Leveling: self.HMI_Leveling,
            self.PrintProcess: self.HMI_Printing,
            self.Print_window: self.HMI_PauseOrStop,
            self.AxisMove: self.HMI_AxisMove,
            self.TemperatureID: self.HMI_Temperature,
            self.Motion: self.HMI_Motion,
            self.Info: self.HMI_Info,
            self.Tune: self.HMI_Tune,
            self.PLAPreheat: self.HMI_PLAPreheatSetting,
            self.ABSPreheat: self.HMI_ABSPreheatSetting,
            self.Move_X: self.HMI_Move_X,
            self.Move_Y: self.HMI_Move_Y,
            self.Move_Z: self.HMI_Move_Z,
            self.Extruder: self.HMI_Move_E,
            self.ETemp: self.HMI_ETemp,
            self.Homeoffset: self.HMI_Zoffset,
            self.BedTemp: self.HMI_BedTemp,
            self.FanSpeed: self.HMI_FanSpeed,
            self.PrintSpeed: self.HMI_PrintSpeed,
            self.Last_Prepare: self.HMI_Waiting,
        }
        handler = handlers.get(self.checkkey)
        if handler is not None:
            handler(s)

    ##################################################################
    # Screen input handlers
    ##################################################################

    def HMI_MainMenu(self, s):
        if s == self.ENCODER_DIFF_CW:
            if self.select_page.inc(4):
                if self.select_page.now == 0:
                    self.ICON_Print()
                if self.select_page.now == 1:
                    self.ICON_Print()
                    self.ICON_Prepare()
                if self.select_page.now == 2:
                    self.ICON_Prepare()
                    self.ICON_Control()
                if self.select_page.now == 3:
                    self.ICON_Control()
                    if self.pd.HAS_ONESTEP_LEVELING:
                        self.ICON_Leveling(True)
                    else:
                        self.ICON_StartInfo(True)
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_page.dec():
                if self.select_page.now == 0:
                    self.ICON_Print()
                    self.ICON_Prepare()
                elif self.select_page.now == 1:
                    self.ICON_Prepare()
                    self.ICON_Control()
                elif self.select_page.now == 2:
                    self.ICON_Control()
                    if self.pd.HAS_ONESTEP_LEVELING:
                        self.ICON_Leveling(False)
                    else:
                        self.ICON_StartInfo(False)
                elif self.select_page.now == 3:
                    if self.pd.HAS_ONESTEP_LEVELING:
                        self.ICON_Leveling(True)
                    else:
                        self.ICON_StartInfo(True)
        elif s == self.ENCODER_DIFF_ENTER:
            if self.select_page.now == 0:  # Print file
                self.checkkey = self.SelectFile
                self.Draw_Print_File_Menu()
            elif self.select_page.now == 1:  # Prepare
                self.checkkey = self.Prepare
                self.select_prepare.reset()
                self.index_prepare = self.MROWS
                self.Draw_Prepare_Menu()
            elif self.select_page.now == 2:  # Control
                self.checkkey = self.Control
                self.select_control.reset()
                self.index_control = self.MROWS
                self.Draw_Control_Menu()
            elif self.select_page.now == 3:  # Leveling or Info
                if self.pd.HAS_ONESTEP_LEVELING:
                    self.checkkey = self.Leveling
                    self.HMI_StartLeveling()
                else:
                    self.checkkey = self.Info
                    self.Draw_Info_Menu()
        self.lcd.UpdateLCD()

    def HMI_SelectFile(self, s):
        fullCnt = len(self.pd.GetFiles())
        if s == self.ENCODER_DIFF_CW and fullCnt:
            if self.select_file.inc(1 + fullCnt):
                itemnum = self.select_file.now - 1
                if (self.select_file.now > self.MROWS
                        and self.select_file.now > self.index_file):
                    self.index_file = self.select_file.now
                    self.Scroll_Menu(self.DWIN_SCROLL_UP)
                    self.Draw_SDItem(itemnum, self.MROWS)
                else:
                    self.Move_Highlight(
                        1, self.select_file.now + self.MROWS - self.index_file)
        elif s == self.ENCODER_DIFF_CCW and fullCnt:
            if self.select_file.dec():
                itemnum = self.select_file.now - 1
                if self.select_file.now < self.index_file - self.MROWS:
                    self.index_file -= 1
                    self.Scroll_Menu(self.DWIN_SCROLL_DOWN)
                    if self.index_file == self.MROWS:
                        self.Draw_Back_First()
                    else:
                        self.Draw_SDItem(itemnum, 0)
                else:
                    self.Move_Highlight(
                        -1,
                        self.select_file.now + self.MROWS - self.index_file)
        elif s == self.ENCODER_DIFF_ENTER:
            if self.select_file.now == 0:  # Back
                self.select_page.set(0)
                self.Goto_MainMenu()
            else:
                filenum = self.select_file.now - 1
                self.select_print.reset()
                self.select_file.reset()
                self.pd.HMI_flag.print_finish = False
                self.pd.HMI_flag.done_confirm_flag = False
                self.pd.openAndPrintFile(filenum)
                self.Goto_PrintProcess()
        self.lcd.UpdateLCD()

    def HMI_Prepare(self, s):
        if s == self.ENCODER_DIFF_CW:
            if self.select_prepare.inc(1 + self.PREPARE_CASE_TOTAL):
                if (self.select_prepare.now > self.MROWS
                        and self.select_prepare.now > self.index_prepare):
                    self.index_prepare = self.select_prepare.now
                    self.Scroll_Menu(self.DWIN_SCROLL_UP)
                    self.Draw_Menu_Icon(
                        self.MROWS,
                        self.ICON_Axis + self.select_prepare.now - 1)
                    if self.index_prepare == self.PREPARE_CASE_ABS:
                        self.Item_Prepare_ABS(self.MROWS)
                    elif self.index_prepare == self.PREPARE_CASE_COOL:
                        self.Item_Prepare_Cool(self.MROWS)
                else:
                    self.Move_Highlight(
                        1,
                        self.select_prepare.now + self.MROWS
                        - self.index_prepare)
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_prepare.dec():
                if self.select_prepare.now < self.index_prepare - self.MROWS:
                    self.index_prepare -= 1
                    self.Scroll_Menu(self.DWIN_SCROLL_DOWN)
                    if self.index_prepare == self.MROWS:
                        self.Draw_Back_First()
                    else:
                        self.Draw_Menu_Line(
                            0, self.ICON_Axis + self.select_prepare.now - 1)
                    if self.index_prepare == 6:
                        self.Item_Prepare_Move(0)
                    elif self.index_prepare == 7:
                        self.Item_Prepare_Disable(0)
                    elif self.index_prepare == 8:
                        self.Item_Prepare_Home(0)
                else:
                    self.Move_Highlight(
                        -1,
                        self.select_prepare.now + self.MROWS
                        - self.index_prepare)
        elif s == self.ENCODER_DIFF_ENTER:
            now = self.select_prepare.now
            if now == 0:  # Back
                self.select_page.set(1)
                self.Goto_MainMenu()
            elif now == self.PREPARE_CASE_MOVE:  # Axis move
                self.checkkey = self.AxisMove
                self.select_axis.reset()
                self.pd.run_gcode("G92 E0")
                self.pd.current_position.e = 0
                self.pd.HMI_ValueStruct.Move_E_scale = 0
                self.Draw_Move_Menu()
                self.Draw_Move_Values()
            elif now == self.PREPARE_CASE_DISA:  # Disable steppers
                self.pd.run_gcode("M84")
            elif now == self.PREPARE_CASE_HOME:  # Homing
                self.checkkey = self.Last_Prepare
                self.index_prepare = self.MROWS
                self.pd.HMI_flag.home_flag = True
                self.Popup_Window_Home()
                self.lcd.UpdateLCD()
                self.pd.home(self.CompletedHoming)
            elif now == self.PREPARE_CASE_ZOFF:  # Z-offset (babystep)
                self.checkkey = self.Homeoffset
                self.pd.HMI_ValueStruct.show_mode = -4
                self.dwin_zoffset = self.pd.BABY_Z_VAR
                self.pd.HMI_ValueStruct.offset_value = int(
                    self.dwin_zoffset * 100)
                self.lcd.Draw_Signed_Float(
                    self.lcd.font8x16, self.lcd.Select_Color, 2, 2, 202,
                    self.MBASE(self.PREPARE_CASE_ZOFF + self.MROWS
                               - self.index_prepare),
                    self.pd.HMI_ValueStruct.offset_value)
            elif now == self.PREPARE_CASE_PLA:  # PLA preheat
                self.pd.preheat("PLA")
            elif now == self.PREPARE_CASE_ABS:  # ABS preheat
                self.pd.preheat("ABS")
            elif now == self.PREPARE_CASE_COOL:  # Cool
                self.pd.zero_fan_speeds()
                self.pd.disable_all_heaters()
        self.lcd.UpdateLCD()

    def HMI_Control(self, s):
        if s == self.ENCODER_DIFF_CW:
            if self.select_control.inc(1 + self.CONTROL_CASE_TOTAL):
                self.Move_Highlight(
                    1,
                    self.select_control.now + self.MROWS - self.index_control)
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_control.dec():
                self.Move_Highlight(
                    -1,
                    self.select_control.now + self.MROWS - self.index_control)
        elif s == self.ENCODER_DIFF_ENTER:
            now = self.select_control.now
            if now == 0:  # Back
                self.select_page.set(2)
                self.Goto_MainMenu()
            elif now == self.CONTROL_CASE_TEMP:  # Temperature
                self.checkkey = self.TemperatureID
                self.pd.HMI_ValueStruct.show_mode = -1
                self.select_temp.reset()
                self.Draw_Temperature_Menu()
            elif now == self.CONTROL_CASE_MOVE:  # Motion
                self.checkkey = self.Motion
                self.select_motion.reset()
                self.Draw_Motion_Menu()
            elif now == self.CONTROL_CASE_INFO:  # Info
                self.checkkey = self.Info
                self.Draw_Info_Menu()
        self.lcd.UpdateLCD()

    def HMI_Info(self, s):
        if s == self.ENCODER_DIFF_ENTER:
            if self.pd.HAS_ONESTEP_LEVELING:
                self.checkkey = self.Control
                self.select_control.set(self.CONTROL_CASE_INFO)
                self.Draw_Control_Menu()
            else:
                self.select_page.set(3)
                self.Goto_MainMenu()
        self.lcd.UpdateLCD()

    def HMI_Printing(self, s):
        if self.pd.HMI_flag.done_confirm_flag:
            if s == self.ENCODER_DIFF_ENTER:
                self.pd.HMI_flag.done_confirm_flag = False
                self.select_page.set(0)
                self.Goto_MainMenu()
            return
        if s == self.ENCODER_DIFF_CW:
            if self.select_print.inc(3):
                if self.select_print.now == 0:
                    self.ICON_Tune()
                elif self.select_print.now == 1:
                    self.ICON_Tune()
                    self.ICON_ResumeOrPause()
                elif self.select_print.now == 2:
                    self.ICON_ResumeOrPause()
                    self.ICON_Stop()
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_print.dec():
                if self.select_print.now == 0:
                    self.ICON_Tune()
                    self.ICON_ResumeOrPause()
                elif self.select_print.now == 1:
                    self.ICON_ResumeOrPause()
                    self.ICON_Stop()
                elif self.select_print.now == 2:
                    self.ICON_Stop()
        elif s == self.ENCODER_DIFF_ENTER:
            if self.select_print.now == 0:  # Tune
                self.checkkey = self.Tune
                self.pd.HMI_ValueStruct.show_mode = 0
                self.select_tune.reset()
                self.index_tune = self.MROWS
                self.Draw_Tune_Menu()
            elif self.select_print.now == 1:  # Pause / Resume
                if self.pd.HMI_flag.pause_flag:
                    self.ICON_Pause()
                    self.pd.resume_job()
                else:
                    self.pd.HMI_flag.select_flag = True
                    self.checkkey = self.Print_window
                    self.Popup_window_PauseOrStop()
            elif self.select_print.now == 2:  # Stop
                self.pd.HMI_flag.select_flag = True
                self.checkkey = self.Print_window
                self.Popup_window_PauseOrStop()
        self.lcd.UpdateLCD()

    def HMI_PauseOrStop(self, s):
        if s == self.ENCODER_DIFF_CW:
            self.Draw_Select_Highlight(False)
        elif s == self.ENCODER_DIFF_CCW:
            self.Draw_Select_Highlight(True)
        elif s == self.ENCODER_DIFF_ENTER:
            if self.select_print.now == 1:  # pause window
                if self.pd.HMI_flag.select_flag:
                    self.pd.HMI_flag.pause_flag = True
                    self.ICON_Continue()
                    self.pd.pause_job()
                self.Goto_PrintProcess()
            elif self.select_print.now == 2:  # stop window
                if self.pd.HMI_flag.select_flag:
                    self.pd.cancel_job()
                    self.select_page.set(0)
                    self.Goto_MainMenu()
                else:
                    self.Goto_PrintProcess()
        self.lcd.UpdateLCD()

    def HMI_Tune(self, s):
        if s == self.ENCODER_DIFF_CW:
            if self.select_tune.inc(1 + self.TUNE_CASE_TOTAL):
                if (self.select_tune.now > self.MROWS
                        and self.select_tune.now > self.index_tune):
                    self.index_tune = self.select_tune.now
                    self.Scroll_Menu(self.DWIN_SCROLL_UP)
                    if self.index_tune == self.TUNE_CASE_ZOFF:
                        self.Item_Tune_Zoffset(self.MROWS)
                else:
                    self.Move_Highlight(
                        1, self.select_tune.now + self.MROWS - self.index_tune)
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_tune.dec():
                if self.select_tune.now < self.index_tune - self.MROWS:
                    self.index_tune -= 1
                    self.Scroll_Menu(self.DWIN_SCROLL_DOWN)
                    if self.index_tune == self.MROWS:
                        self.Draw_Back_First()
                else:
                    self.Move_Highlight(
                        -1,
                        self.select_tune.now + self.MROWS - self.index_tune)
        elif s == self.ENCODER_DIFF_ENTER:
            now = self.select_tune.now
            if now == 0:  # Back
                self.select_print.set(0)
                self.Goto_PrintProcess()
            elif now == self.TUNE_CASE_SPEED:
                self.checkkey = self.PrintSpeed
                self.pd.HMI_ValueStruct.print_speed = \
                    self.pd.feedrate_percentage
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216,
                    self.MBASE(self.TUNE_CASE_SPEED + self.MROWS
                               - self.index_tune),
                    self.pd.feedrate_percentage)
            elif now == self.TUNE_CASE_TEMP:
                self.checkkey = self.ETemp
                self.pd.HMI_ValueStruct.E_Temp = \
                    self.pd.thermalManager['temp_hotend'][0]['target']
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216,
                    self.MBASE(self.TUNE_CASE_TEMP + self.MROWS
                               - self.index_tune),
                    self.pd.HMI_ValueStruct.E_Temp)
            elif now == self.TUNE_CASE_BED:
                self.checkkey = self.BedTemp
                self.pd.HMI_ValueStruct.Bed_Temp = \
                    self.pd.thermalManager['temp_bed']['target']
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216,
                    self.MBASE(self.TUNE_CASE_BED + self.MROWS
                               - self.index_tune),
                    self.pd.HMI_ValueStruct.Bed_Temp)
            elif now == self.TUNE_CASE_FAN:
                self.checkkey = self.FanSpeed
                self.pd.HMI_ValueStruct.Fan_speed = \
                    self.pd.thermalManager['fan_speed'][0]
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216,
                    self.MBASE(self.TUNE_CASE_FAN + self.MROWS
                               - self.index_tune),
                    self.pd.HMI_ValueStruct.Fan_speed)
            elif now == self.TUNE_CASE_ZOFF:
                self.checkkey = self.Homeoffset
                self.pd.HMI_ValueStruct.show_mode = 0
                self.dwin_zoffset = self.pd.BABY_Z_VAR
                self.pd.HMI_ValueStruct.offset_value = int(
                    self.dwin_zoffset * 100)
                self.lcd.Draw_Signed_Float(
                    self.lcd.font8x16, self.lcd.Select_Color, 2, 2, 202,
                    self.MBASE(self.TUNE_CASE_ZOFF + self.MROWS
                               - self.index_tune),
                    self.pd.HMI_ValueStruct.offset_value)
        self.lcd.UpdateLCD()

    def HMI_PrintSpeed(self, s):
        hv = self.pd.HMI_ValueStruct
        if s == self.ENCODER_DIFF_CW:
            hv.print_speed += 1
        elif s == self.ENCODER_DIFF_CCW:
            hv.print_speed -= 1
        elif s == self.ENCODER_DIFF_ENTER:
            self.checkkey = self.Tune
            self.pd.set_feedrate(hv.print_speed)
            self.lcd.Draw_IntValue(
                True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                self.lcd.Color_Bg_Black, 3, 216,
                self.MBASE(self.TUNE_CASE_SPEED + self.MROWS
                           - self.index_tune),
                hv.print_speed)
            self.lcd.UpdateLCD()
            return
        hv.print_speed = max(10, min(500, hv.print_speed))
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Select_Color, 3, 216,
            self.MBASE(self.TUNE_CASE_SPEED + self.MROWS - self.index_tune),
            hv.print_speed)
        self.lcd.UpdateLCD()

    def HMI_AxisMove(self, s):
        if self.pd.HMI_flag.ETempTooLow_flag:
            if s == self.ENCODER_DIFF_ENTER:
                self.pd.HMI_flag.ETempTooLow_flag = False
                self.pd.current_position.e = 0
                self.pd.HMI_ValueStruct.Move_E_scale = 0
                self.Draw_Move_Menu()
                self.Draw_Move_Values()
                self.lcd.UpdateLCD()
            return
        if s == self.ENCODER_DIFF_CW:
            if self.select_axis.inc(1 + 4):
                self.Move_Highlight(1, self.select_axis.now)
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_axis.dec():
                self.Move_Highlight(-1, self.select_axis.now)
        elif s == self.ENCODER_DIFF_ENTER:
            hv = self.pd.HMI_ValueStruct
            now = self.select_axis.now
            if now == 0:  # Back
                self.checkkey = self.Prepare
                self.select_prepare.set(1)
                self.index_prepare = self.MROWS
                self.Draw_Prepare_Menu()
            elif now == 1:  # X
                self.checkkey = self.Move_X
                hv.Move_X_scale = (self.pd.current_position.x
                                   * self.MINUNITMULT)
                self.lcd.Draw_FloatValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 1, 216, self.MBASE(1),
                    hv.Move_X_scale)
            elif now == 2:  # Y
                self.checkkey = self.Move_Y
                hv.Move_Y_scale = (self.pd.current_position.y
                                   * self.MINUNITMULT)
                self.lcd.Draw_FloatValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 1, 216, self.MBASE(2),
                    hv.Move_Y_scale)
            elif now == 3:  # Z
                self.checkkey = self.Move_Z
                hv.Move_Z_scale = (self.pd.current_position.z
                                   * self.MINUNITMULT)
                self.lcd.Draw_FloatValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 1, 216, self.MBASE(3),
                    hv.Move_Z_scale)
            elif now == 4:  # Extruder
                if not self.pd.can_extrude():
                    self.pd.HMI_flag.ETempTooLow_flag = True
                    self.Popup_Window_ETempTooLow()
                    self.lcd.UpdateLCD()
                    return
                self.checkkey = self.Extruder
                hv.Move_E_scale = (self.pd.current_position.e
                                   * self.MINUNITMULT)
                self.lcd.Draw_Signed_Float(
                    self.lcd.font8x16, self.lcd.Select_Color, 3, 1, 216,
                    self.MBASE(4), hv.Move_E_scale)
        self.lcd.UpdateLCD()

    def _hmi_move_axis(self, s, axis, line, attr, minval, maxval, feedrate):
        hv = self.pd.HMI_ValueStruct
        if s == self.ENCODER_DIFF_ENTER:
            self.checkkey = self.AxisMove
            value = getattr(hv, attr)
            if axis == 'E':
                self.lcd.Draw_Signed_Float(
                    self.lcd.font8x16, self.lcd.Color_Bg_Black, 3, 1, 216,
                    self.MBASE(line), value)
            else:
                self.lcd.Draw_FloatValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Color_Bg_Black, 3, 1, 216, self.MBASE(line),
                    value)
            self.pd.moveAbsolute(axis, value / float(self.MINUNITMULT),
                                 feedrate)
            self.lcd.UpdateLCD()
            return
        elif s == self.ENCODER_DIFF_CW:
            setattr(hv, attr, getattr(hv, attr) + 1)
        elif s == self.ENCODER_DIFF_CCW:
            setattr(hv, attr, getattr(hv, attr) - 1)
        value = getattr(hv, attr)
        value = max(minval * self.MINUNITMULT,
                    min(maxval * self.MINUNITMULT, value))
        setattr(hv, attr, value)
        if axis == 'E':
            self.lcd.Draw_Signed_Float(
                self.lcd.font8x16, self.lcd.Select_Color, 3, 1, 216,
                self.MBASE(line), value)
        else:
            self.lcd.Draw_FloatValue(
                True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                self.lcd.Select_Color, 3, 1, 216, self.MBASE(line), value)
        self.lcd.UpdateLCD()

    def HMI_Move_X(self, s):
        self._hmi_move_axis(s, 'X', 1, 'Move_X_scale',
                            self.pd.X_MIN_POS, self.pd.X_MAX_POS, 5000)
    def HMI_Move_Y(self, s):
        self._hmi_move_axis(s, 'Y', 2, 'Move_Y_scale',
                            self.pd.Y_MIN_POS, self.pd.Y_MAX_POS, 5000)
    def HMI_Move_Z(self, s):
        self._hmi_move_axis(s, 'Z', 3, 'Move_Z_scale',
                            self.pd.Z_MIN_POS, self.pd.Z_MAX_POS, 600)
    def HMI_Move_E(self, s):
        self._hmi_move_axis(s, 'E', 4, 'Move_E_scale',
                            -self.pd.EXTRUDE_MAXLENGTH,
                            self.pd.EXTRUDE_MAXLENGTH, 300)

    def HMI_Temperature(self, s):
        if s == self.ENCODER_DIFF_CW:
            if self.select_temp.inc(1 + self.TEMP_CASE_TOTAL):
                self.Move_Highlight(1, self.select_temp.now)
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_temp.dec():
                self.Move_Highlight(-1, self.select_temp.now)
        elif s == self.ENCODER_DIFF_ENTER:
            now = self.select_temp.now
            hv = self.pd.HMI_ValueStruct
            if now == 0:  # Back
                self.checkkey = self.Control
                self.select_control.set(1)
                self.index_control = self.MROWS
                self.Draw_Control_Menu()
            elif now == self.TEMP_CASE_TEMP:
                self.checkkey = self.ETemp
                hv.E_Temp = self.pd.thermalManager['temp_hotend'][0]['target']
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216, self.MBASE(1), hv.E_Temp)
            elif now == self.TEMP_CASE_BED:
                self.checkkey = self.BedTemp
                hv.Bed_Temp = self.pd.thermalManager['temp_bed']['target']
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216, self.MBASE(2), hv.Bed_Temp)
            elif now == self.TEMP_CASE_FAN:
                self.checkkey = self.FanSpeed
                hv.Fan_speed = self.pd.thermalManager['fan_speed'][0]
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216, self.MBASE(3), hv.Fan_speed)
            elif now == self.TEMP_CASE_PLA:
                self.checkkey = self.PLAPreheat
                self.select_PLA.reset()
                hv.show_mode = -2
                self.Draw_Preheat_Menu(0)
            elif now == self.TEMP_CASE_ABS:
                self.checkkey = self.ABSPreheat
                self.select_ABS.reset()
                hv.show_mode = -3
                self.Draw_Preheat_Menu(1)
        self.lcd.UpdateLCD()

    def _hmi_preheat_setting(self, s, preset_idx, select):
        preset = self.pd.material_preset[preset_idx]
        hv = self.pd.HMI_ValueStruct
        if s == self.ENCODER_DIFF_CW:
            if select.inc(1 + self.PREHEAT_CASE_TOTAL):
                self.Move_Highlight(1, select.now)
        elif s == self.ENCODER_DIFF_CCW:
            if select.dec():
                self.Move_Highlight(-1, select.now)
        elif s == self.ENCODER_DIFF_ENTER:
            now = select.now
            if now == 0:  # Back
                self.checkkey = self.TemperatureID
                self.select_temp.now = self.TEMP_CASE_PLA + preset_idx
                hv.show_mode = -1
                self.Draw_Temperature_Menu()
            elif now == self.PREHEAT_CASE_TEMP:
                self.checkkey = self.ETemp
                hv.E_Temp = preset.hotend_temp
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216,
                    self.MBASE(self.PREHEAT_CASE_TEMP), preset.hotend_temp)
            elif now == self.PREHEAT_CASE_BED:
                self.checkkey = self.BedTemp
                hv.Bed_Temp = preset.bed_temp
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216,
                    self.MBASE(self.PREHEAT_CASE_BED), preset.bed_temp)
            elif now == self.PREHEAT_CASE_FAN:
                self.checkkey = self.FanSpeed
                hv.Fan_speed = preset.fan_speed
                self.lcd.Draw_IntValue(
                    True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                    self.lcd.Select_Color, 3, 216,
                    self.MBASE(self.PREHEAT_CASE_FAN), preset.fan_speed)
            elif now == self.PREHEAT_CASE_SAVE:
                self.pd.save_settings()
        self.lcd.UpdateLCD()

    def HMI_PLAPreheatSetting(self, s):
        self._hmi_preheat_setting(s, 0, self.select_PLA)
    def HMI_ABSPreheatSetting(self, s):
        self._hmi_preheat_setting(s, 1, self.select_ABS)

    def _temp_edit_line(self, case_temp, case_preheat, case_tune):
        show_mode = self.pd.HMI_ValueStruct.show_mode
        if show_mode == -1:
            return case_temp
        if show_mode in (-2, -3):
            return case_preheat
        return case_tune + self.MROWS - self.index_tune

    def HMI_ETemp(self, s):
        hv = self.pd.HMI_ValueStruct
        temp_line = self._temp_edit_line(self.TEMP_CASE_TEMP,
                                         self.PREHEAT_CASE_TEMP,
                                         self.TUNE_CASE_TEMP)
        if s == self.ENCODER_DIFF_ENTER:
            if hv.show_mode == -1:
                self.checkkey = self.TemperatureID
                self.pd.setExtTemp(hv.E_Temp)
            elif hv.show_mode == -2:
                self.checkkey = self.PLAPreheat
                self.pd.material_preset[0].hotend_temp = hv.E_Temp
            elif hv.show_mode == -3:
                self.checkkey = self.ABSPreheat
                self.pd.material_preset[1].hotend_temp = hv.E_Temp
            else:
                self.checkkey = self.Tune
                self.pd.setExtTemp(hv.E_Temp)
            self.lcd.Draw_IntValue(
                True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                self.lcd.Color_Bg_Black, 3, 216, self.MBASE(temp_line),
                hv.E_Temp)
            self.lcd.UpdateLCD()
            return
        elif s == self.ENCODER_DIFF_CW:
            hv.E_Temp += 1
        elif s == self.ENCODER_DIFF_CCW:
            hv.E_Temp -= 1
        hv.E_Temp = max(self.pd.MIN_E_TEMP,
                        min(self.pd.MAX_E_TEMP, hv.E_Temp))
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Select_Color, 3, 216, self.MBASE(temp_line), hv.E_Temp)
        self.lcd.UpdateLCD()

    def HMI_BedTemp(self, s):
        hv = self.pd.HMI_ValueStruct
        bed_line = self._temp_edit_line(self.TEMP_CASE_BED,
                                        self.PREHEAT_CASE_BED,
                                        self.TUNE_CASE_BED)
        if s == self.ENCODER_DIFF_ENTER:
            if hv.show_mode == -1:
                self.checkkey = self.TemperatureID
                self.pd.setBedTemp(hv.Bed_Temp)
            elif hv.show_mode == -2:
                self.checkkey = self.PLAPreheat
                self.pd.material_preset[0].bed_temp = hv.Bed_Temp
            elif hv.show_mode == -3:
                self.checkkey = self.ABSPreheat
                self.pd.material_preset[1].bed_temp = hv.Bed_Temp
            else:
                self.checkkey = self.Tune
                self.pd.setBedTemp(hv.Bed_Temp)
            self.lcd.Draw_IntValue(
                True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                self.lcd.Color_Bg_Black, 3, 216, self.MBASE(bed_line),
                hv.Bed_Temp)
            self.lcd.UpdateLCD()
            return
        elif s == self.ENCODER_DIFF_CW:
            hv.Bed_Temp += 1
        elif s == self.ENCODER_DIFF_CCW:
            hv.Bed_Temp -= 1
        hv.Bed_Temp = max(self.pd.MIN_BED_TEMP,
                          min(self.pd.BED_MAX_TARGET, hv.Bed_Temp))
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Select_Color, 3, 216, self.MBASE(bed_line), hv.Bed_Temp)
        self.lcd.UpdateLCD()

    def HMI_FanSpeed(self, s):
        hv = self.pd.HMI_ValueStruct
        fan_line = self._temp_edit_line(self.TEMP_CASE_FAN,
                                        self.PREHEAT_CASE_FAN,
                                        self.TUNE_CASE_FAN)
        if s == self.ENCODER_DIFF_ENTER:
            if hv.show_mode == -1:
                self.checkkey = self.TemperatureID
                self.pd.setFanSpeed(hv.Fan_speed)
            elif hv.show_mode == -2:
                self.checkkey = self.PLAPreheat
                self.pd.material_preset[0].fan_speed = hv.Fan_speed
            elif hv.show_mode == -3:
                self.checkkey = self.ABSPreheat
                self.pd.material_preset[1].fan_speed = hv.Fan_speed
            else:
                self.checkkey = self.Tune
                self.pd.setFanSpeed(hv.Fan_speed)
            self.lcd.Draw_IntValue(
                True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
                self.lcd.Color_Bg_Black, 3, 216, self.MBASE(fan_line),
                hv.Fan_speed)
            self.lcd.UpdateLCD()
            return
        elif s == self.ENCODER_DIFF_CW:
            hv.Fan_speed += 1
        elif s == self.ENCODER_DIFF_CCW:
            hv.Fan_speed -= 1
        hv.Fan_speed = max(0, min(100, hv.Fan_speed))
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Select_Color, 3, 216, self.MBASE(fan_line), hv.Fan_speed)
        self.lcd.UpdateLCD()

    def HMI_Zoffset(self, s):
        hv = self.pd.HMI_ValueStruct
        if hv.show_mode == -4:
            zoff_line = (self.PREPARE_CASE_ZOFF + self.MROWS
                         - self.index_prepare)
        else:
            zoff_line = self.TUNE_CASE_ZOFF + self.MROWS - self.index_tune
        if s == self.ENCODER_DIFF_ENTER:
            self.checkkey = (self.Prepare if hv.show_mode == -4
                             else self.Tune)
            self.lcd.Draw_Signed_Float(
                self.lcd.font8x16, self.lcd.Color_Bg_Black, 2, 2, 202,
                self.MBASE(zoff_line), hv.offset_value)
            self.lcd.UpdateLCD()
            return
        elif s == self.ENCODER_DIFF_CW:
            hv.offset_value += 1
        elif s == self.ENCODER_DIFF_CCW:
            hv.offset_value -= 1
        hv.offset_value = max(-500, min(500, hv.offset_value))
        last_zoffset = self.dwin_zoffset
        self.dwin_zoffset = hv.offset_value / 100.0
        # Apply as a live babystep
        self.pd.offset_z(self.dwin_zoffset - last_zoffset)
        self.lcd.Draw_Signed_Float(
            self.lcd.font8x16, self.lcd.Select_Color, 2, 2, 202,
            self.MBASE(zoff_line), hv.offset_value)
        self.lcd.UpdateLCD()

    def HMI_Motion(self, s):
        if s == self.ENCODER_DIFF_CW:
            if self.select_motion.inc(1 + self.MOTION_CASE_TOTAL):
                self.Move_Highlight(1, self.select_motion.now)
        elif s == self.ENCODER_DIFF_CCW:
            if self.select_motion.dec():
                self.Move_Highlight(-1, self.select_motion.now)
        elif s == self.ENCODER_DIFF_ENTER:
            if self.select_motion.now == 0:  # Back
                self.checkkey = self.Control
                self.select_control.set(self.CONTROL_CASE_MOVE)
                self.index_control = self.MROWS
                self.Draw_Control_Menu()
        self.lcd.UpdateLCD()

    def HMI_Leveling(self, s):
        # Wait for the leveling sequence to complete
        pass

    def HMI_Waiting(self, s):
        # Waiting for homing to complete
        pass

    def HMI_StartLeveling(self):
        self.pd.HMI_flag.leveling_flag = True
        self.Popup_Window_Leveling()
        self.lcd.UpdateLCD()
        self.pd.level_bed(self.CompletedLeveling)

    def CompletedHoming(self):
        self.pd.HMI_flag.home_flag = False
        if self.checkkey == self.Last_Prepare:
            self.checkkey = self.Prepare
            self.select_prepare.now = self.PREPARE_CASE_HOME
            self.index_prepare = self.MROWS
            self.Draw_Prepare_Menu()
            self.lcd.UpdateLCD()

    def CompletedLeveling(self):
        self.pd.HMI_flag.leveling_flag = False
        if self.checkkey == self.Leveling:
            self.select_page.set(3)
            self.Goto_MainMenu()
            self.lcd.UpdateLCD()

    ##################################################################
    # Menu / screen drawing
    ##################################################################

    def MBASE(self, line):
        return 49 + self.MLINE * line

    def Draw_Title(self, title):
        self.lcd.Draw_String(False, False, self.lcd.DWIN_FONT_HEAD,
                             self.lcd.Color_White, self.lcd.Color_Bg_Blue,
                             14, 4, title)

    def Clear_Title_Bar(self):
        self.lcd.Draw_Rectangle(1, self.lcd.Color_Bg_Blue, 0, 0,
                                self.lcd.DWIN_WIDTH, 30)
    def Clear_Menu_Area(self):
        self.lcd.Draw_Rectangle(1, self.lcd.Color_Bg_Black, 0, 31,
                                self.lcd.DWIN_WIDTH, self.STATUS_Y)
    def Clear_Main_Window(self):
        self.Clear_Title_Bar()
        self.Clear_Menu_Area()

    def Draw_Popup_Bkgd_60(self):
        self.lcd.Draw_Rectangle(1, self.lcd.Color_Bg_Window, 14, 60, 258, 330)

    def Draw_More_Icon(self, line):
        self.lcd.ICON_Show(self.ICON, self.ICON_More, 226,
                           self.MBASE(line) - 3)
    def Draw_Menu_Cursor(self, line):
        self.lcd.Draw_Rectangle(1, self.lcd.Rectangle_Color, 0,
                                self.MBASE(line) - 18, 14,
                                self.MBASE(line + 1) - 20)
    def Erase_Menu_Cursor(self, line):
        self.lcd.Draw_Rectangle(1, self.lcd.Color_Bg_Black, 0,
                                self.MBASE(line) - 18, 14,
                                self.MBASE(line + 1) - 20)
    def Move_Highlight(self, ffrom, newline):
        self.Erase_Menu_Cursor(newline - ffrom)
        self.Draw_Menu_Cursor(newline)
    def Add_Menu_Line(self):
        self.Move_Highlight(1, self.MROWS)
        self.lcd.Draw_Line(self.lcd.Line_Color, 16,
                           self.MBASE(self.MROWS + 1) - 20, 256,
                           self.MBASE(self.MROWS + 1) - 19)
    def Scroll_Menu(self, direction):
        self.lcd.Frame_AreaMove(1, direction, self.MLINE,
                                self.lcd.Color_Bg_Black, 0, 31,
                                self.lcd.DWIN_WIDTH, 349)
        if direction == self.DWIN_SCROLL_DOWN:
            self.Move_Highlight(-1, 0)
        elif direction == self.DWIN_SCROLL_UP:
            self.Add_Menu_Line()

    def Draw_Menu_Icon(self, line, icon):
        self.lcd.ICON_Show(self.ICON, icon, 26, self.MBASE(line) - 3)
    def Draw_Menu_Line(self, line, icon=None, label=None):
        if label:
            self.lcd.Draw_String(False, False, self.lcd.font8x16,
                                 self.lcd.Color_White,
                                 self.lcd.Color_Bg_Black, self.LBLX,
                                 self.MBASE(line) - 1, label)
        if icon:
            self.Draw_Menu_Icon(line, icon)
        self.lcd.Draw_Line(self.lcd.Line_Color, 16, self.MBASE(line) + 33,
                           256, self.MBASE(line) + 34)

    def Draw_Back_Label(self):
        self.lcd.Frame_AreaCopy(1, 226, 179, 256, 189, self.LBLX,
                                self.MBASE(0))
    def Draw_Back_First(self, is_sel=True):
        self.Draw_Menu_Line(0, self.ICON_Back)
        self.Draw_Back_Label()
        if is_sel:
            self.Draw_Menu_Cursor(0)

    # Label fragments copied from the cached language bitmap
    def draw_move_en(self, line):
        self.lcd.Frame_AreaCopy(1, 69, 61, 102, 71, self.LBLX, line)
    def draw_max_en(self, line):
        self.lcd.Frame_AreaCopy(1, 245, 119, 269, 129, self.LBLX, line)
    def draw_max_accel_en(self, line):
        self.draw_max_en(line)
        self.lcd.Frame_AreaCopy(1, 1, 135, 79, 145, self.LBLX + 27, line)
    def draw_speed_en(self, inset, line):
        self.lcd.Frame_AreaCopy(1, 184, 119, 224, 132, self.LBLX + inset,
                                line)
    def draw_steps_per_mm(self, line):
        self.lcd.Frame_AreaCopy(1, 1, 151, 101, 161, self.LBLX, line)
    def say_x(self, inset, line):
        self.lcd.Frame_AreaCopy(1, 95, 104, 102, 114, self.LBLX + inset, line)
    def say_y(self, inset, line):
        self.lcd.Frame_AreaCopy(1, 104, 104, 110, 114, self.LBLX + inset,
                                line)
    def say_z(self, inset, line):
        self.lcd.Frame_AreaCopy(1, 112, 104, 120, 114, self.LBLX + inset,
                                line)

    def Draw_SDItem(self, item, row=0):
        files = self.pd.GetFiles()
        if item >= len(files):
            return
        fname = files[item][:self.MENU_CHAR_LIMIT]
        self.Draw_Menu_Line(row, self.ICON_File, fname)

    def Redraw_SD_List(self):
        self.select_file.reset()
        self.index_file = self.MROWS
        self.Clear_Menu_Area()
        self.Draw_Back_First()
        files = self.pd.GetFiles(refresh=True)
        ed = _MIN(len(files), self.MROWS)
        if len(files) > 0:
            for i in range(ed):
                self.Draw_SDItem(i, i + 1)
        else:
            self.lcd.Draw_Rectangle(
                1, self.lcd.Color_Bg_Red, 10, self.MBASE(3) - 10,
                self.lcd.DWIN_WIDTH - 10, self.MBASE(4))
            self.lcd.Draw_String(
                False, False, self.lcd.font16x32, self.lcd.Color_Yellow,
                self.lcd.Color_Bg_Red, (self.lcd.DWIN_WIDTH - 8 * 16) // 2,
                self.MBASE(3), "No Media")

    def Draw_Print_File_Menu(self):
        self.Clear_Title_Bar()
        self.lcd.Frame_TitleCopy(1, 52, 31, 137, 41)  # "Print file"
        self.Redraw_SD_List()

    def Draw_Prepare_Menu(self):
        self.Clear_Main_Window()
        scroll = self.MROWS - self.index_prepare
        self.lcd.Frame_TitleCopy(1, 178, 2, 229, 14)  # "Prepare"
        if scroll == 0:
            self.Draw_Back_First(self.select_prepare.now == 0)
        if 0 < scroll + self.PREPARE_CASE_MOVE <= self.MROWS:
            self.Item_Prepare_Move(scroll + self.PREPARE_CASE_MOVE)
        if 0 < scroll + self.PREPARE_CASE_DISA <= self.MROWS:
            self.Item_Prepare_Disable(scroll + self.PREPARE_CASE_DISA)
        if 0 < scroll + self.PREPARE_CASE_HOME <= self.MROWS:
            self.Item_Prepare_Home(scroll + self.PREPARE_CASE_HOME)
        if 0 < scroll + self.PREPARE_CASE_ZOFF <= self.MROWS:
            self.Item_Prepare_Offset(scroll + self.PREPARE_CASE_ZOFF)
        if 0 < scroll + self.PREPARE_CASE_PLA <= self.MROWS:
            self.Item_Prepare_PLA(scroll + self.PREPARE_CASE_PLA)
        if 0 < scroll + self.PREPARE_CASE_ABS <= self.MROWS:
            self.Item_Prepare_ABS(scroll + self.PREPARE_CASE_ABS)
        if 0 < scroll + self.PREPARE_CASE_COOL <= self.MROWS:
            self.Item_Prepare_Cool(scroll + self.PREPARE_CASE_COOL)
        row = scroll + self.select_prepare.now
        if self.select_prepare.now and 0 < row <= self.MROWS:
            self.Draw_Menu_Cursor(row)

    def Item_Prepare_Move(self, row):
        self.draw_move_en(self.MBASE(row))  # "Move >"
        self.Draw_Menu_Line(row, self.ICON_Axis)
        self.Draw_More_Icon(row)
    def Item_Prepare_Disable(self, row):
        self.lcd.Frame_AreaCopy(1, 103, 59, 200, 74, self.LBLX,
                                self.MBASE(row))  # "Disable Stepper"
        self.Draw_Menu_Line(row, self.ICON_CloseMotor)
    def Item_Prepare_Home(self, row):
        self.lcd.Frame_AreaCopy(1, 202, 61, 271, 71, self.LBLX,
                                self.MBASE(row))  # "Auto Home"
        self.Draw_Menu_Line(row, self.ICON_Homing)
    def Item_Prepare_Offset(self, row):
        self.lcd.Frame_AreaCopy(1, 93, 179, 141, 189, self.LBLX,
                                self.MBASE(row))  # "Z-Offset"
        self.lcd.Draw_Signed_Float(self.lcd.font8x16,
                                   self.lcd.Color_Bg_Black, 2, 2, 202,
                                   self.MBASE(row),
                                   self.pd.BABY_Z_VAR * 100)
        self.Draw_Menu_Line(row, self.ICON_SetHome)
    def Item_Prepare_PLA(self, row):
        self.lcd.Frame_AreaCopy(1, 107, 76, 156, 86, self.LBLX,
                                self.MBASE(row))  # "Preheat"
        self.lcd.Frame_AreaCopy(1, 157, 76, 181, 86, self.LBLX + 52,
                                self.MBASE(row))  # "PLA"
        self.Draw_Menu_Line(row, self.ICON_PLAPreheat)
    def Item_Prepare_ABS(self, row):
        self.lcd.Frame_AreaCopy(1, 107, 76, 156, 86, self.LBLX,
                                self.MBASE(row))  # "Preheat"
        self.lcd.Frame_AreaCopy(1, 172, 76, 198, 86, self.LBLX + 52,
                                self.MBASE(row))  # "ABS"
        self.Draw_Menu_Line(row, self.ICON_ABSPreheat)
    def Item_Prepare_Cool(self, row):
        self.lcd.Frame_AreaCopy(1, 200, 76, 264, 86, self.LBLX,
                                self.MBASE(row))  # "Cooldown"
        self.Draw_Menu_Line(row, self.ICON_Cool)
    def Item_Tune_Zoffset(self, row):
        self.lcd.Frame_AreaCopy(1, 93, 179, 141, 189, self.LBLX,
                                self.MBASE(row))  # "Z-Offset"
        self.lcd.Draw_Signed_Float(self.lcd.font8x16,
                                   self.lcd.Color_Bg_Black, 2, 2, 202,
                                   self.MBASE(row),
                                   self.pd.BABY_Z_VAR * 100)
        self.Draw_Menu_Line(row, self.ICON_Zoffset)

    def Draw_Control_Menu(self):
        self.Clear_Main_Window()
        self.Draw_Back_First(self.select_control.now == 0)
        self.lcd.Frame_TitleCopy(1, 128, 2, 176, 12)  # "Control"
        self.lcd.Frame_AreaCopy(1, 1, 89, 83, 101, self.LBLX,
                                self.MBASE(self.CONTROL_CASE_TEMP))
        self.lcd.Frame_AreaCopy(1, 84, 89, 128, 99, self.LBLX,
                                self.MBASE(self.CONTROL_CASE_MOVE))
        self.lcd.Frame_AreaCopy(1, 0, 104, 25, 115, self.LBLX,
                                self.MBASE(self.CONTROL_CASE_INFO))
        if self.select_control.now and self.select_control.now < self.MROWS:
            self.Draw_Menu_Cursor(self.select_control.now)
        self.Draw_Menu_Line(1, self.ICON_Temperature)
        self.Draw_More_Icon(1)
        self.Draw_Menu_Line(2, self.ICON_Motion)
        self.Draw_More_Icon(2)
        self.Draw_Menu_Line(3, self.ICON_Info)
        self.Draw_More_Icon(3)

    def Draw_Info_Menu(self):
        self.Clear_Main_Window()
        self.lcd.Frame_TitleCopy(1, 190, 16, 215, 26)  # "Info"
        self.lcd.Draw_String(
            False, False, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black,
            (self.lcd.DWIN_WIDTH
             - len(self.pd.MACHINE_SIZE) * self.MENU_CHR_W) // 2, 122,
            self.pd.MACHINE_SIZE)
        self.lcd.Draw_String(
            False, False, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black,
            (self.lcd.DWIN_WIDTH
             - len(self.pd.SHORT_BUILD_VERSION) * self.MENU_CHR_W) // 2, 195,
            self.pd.SHORT_BUILD_VERSION)
        self.lcd.Frame_AreaCopy(1, 120, 150, 146, 161, 124, 102)  # "Size"
        self.lcd.Frame_AreaCopy(1, 146, 151, 254, 161, 82, 175)  # "Version"
        self.lcd.Frame_AreaCopy(1, 0, 165, 94, 175, 89, 248)  # "Contact"
        self.lcd.Draw_String(
            False, False, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black,
            (self.lcd.DWIN_WIDTH
             - len(self.pd.CORP_WEBSITE_E) * self.MENU_CHR_W) // 2, 268,
            self.pd.CORP_WEBSITE_E)
        self.Draw_Back_First()
        for i in range(3):
            self.lcd.ICON_Show(self.ICON, self.ICON_PrintSize + i, 26,
                               99 + i * 73)
            self.lcd.Draw_Line(self.lcd.Line_Color, 16,
                               self.MBASE(2) + i * 73, 256, 156 + i * 73)

    def Draw_Tune_Menu(self):
        self.Clear_Main_Window()
        self.lcd.Frame_AreaCopy(1, 94, 2, 126, 12, 14, 9)  # "Tune"
        self.lcd.Frame_AreaCopy(1, 1, 179, 92, 190, self.LBLX,
                                self.MBASE(self.TUNE_CASE_SPEED))
        self.lcd.Frame_AreaCopy(1, 197, 104, 238, 114, self.LBLX,
                                self.MBASE(self.TUNE_CASE_TEMP))
        self.lcd.Frame_AreaCopy(1, 1, 89, 83, 101, self.LBLX + 44,
                                self.MBASE(self.TUNE_CASE_TEMP))
        self.lcd.Frame_AreaCopy(1, 240, 104, 264, 114, self.LBLX,
                                self.MBASE(self.TUNE_CASE_BED))
        self.lcd.Frame_AreaCopy(1, 1, 89, 83, 101, self.LBLX + 27,
                                self.MBASE(self.TUNE_CASE_BED))
        self.lcd.Frame_AreaCopy(1, 0, 119, 64, 132, self.LBLX,
                                self.MBASE(self.TUNE_CASE_FAN))
        if self.pd.HAS_ZOFFSET_ITEM:
            self.lcd.Frame_AreaCopy(1, 93, 179, 141, 189, self.LBLX,
                                    self.MBASE(self.TUNE_CASE_ZOFF))
        self.Draw_Back_First(self.select_tune.now == 0)
        if self.select_tune.now:
            self.Draw_Menu_Cursor(self.select_tune.now)
        self.Draw_Menu_Line(self.TUNE_CASE_SPEED, self.ICON_Speed)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216,
            self.MBASE(self.TUNE_CASE_SPEED), self.pd.feedrate_percentage)
        self.Draw_Menu_Line(self.TUNE_CASE_TEMP, self.ICON_HotendTemp)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216, self.MBASE(self.TUNE_CASE_TEMP),
            self.pd.thermalManager['temp_hotend'][0]['target'])
        self.Draw_Menu_Line(self.TUNE_CASE_BED, self.ICON_BedTemp)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216, self.MBASE(self.TUNE_CASE_BED),
            self.pd.thermalManager['temp_bed']['target'])
        self.Draw_Menu_Line(self.TUNE_CASE_FAN, self.ICON_FanSpeed)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216, self.MBASE(self.TUNE_CASE_FAN),
            self.pd.thermalManager['fan_speed'][0])
        if self.pd.HAS_ZOFFSET_ITEM:
            self.Draw_Menu_Line(self.TUNE_CASE_ZOFF, self.ICON_Zoffset)
            self.lcd.Draw_Signed_Float(
                self.lcd.font8x16, self.lcd.Color_Bg_Black, 2, 2, 202,
                self.MBASE(self.TUNE_CASE_ZOFF), self.pd.BABY_Z_VAR * 100)

    def Draw_Temperature_Menu(self):
        self.Clear_Main_Window()
        self.lcd.Frame_TitleCopy(1, 56, 16, 141, 28)  # "Temperature"
        self.lcd.Frame_AreaCopy(1, 197, 104, 238, 114, self.LBLX,
                                self.MBASE(self.TEMP_CASE_TEMP))  # Nozzle...
        self.lcd.Frame_AreaCopy(1, 1, 89, 83, 101, self.LBLX + 44,
                                self.MBASE(self.TEMP_CASE_TEMP))  # ...Temp
        self.lcd.Frame_AreaCopy(1, 240, 104, 264, 114, self.LBLX,
                                self.MBASE(self.TEMP_CASE_BED))  # Bed...
        self.lcd.Frame_AreaCopy(1, 1, 89, 83, 101, self.LBLX + 27,
                                self.MBASE(self.TEMP_CASE_BED))  # ...Temp
        self.lcd.Frame_AreaCopy(1, 0, 119, 64, 132, self.LBLX,
                                self.MBASE(self.TEMP_CASE_FAN))  # Fan speed
        self.lcd.Frame_AreaCopy(1, 107, 76, 156, 86, self.LBLX,
                                self.MBASE(self.TEMP_CASE_PLA))  # Preheat...
        self.lcd.Frame_AreaCopy(1, 157, 76, 181, 86, self.LBLX + 52,
                                self.MBASE(self.TEMP_CASE_PLA))  # ...PLA
        self.lcd.Frame_AreaCopy(1, 131, 119, 182, 132, self.LBLX + 79,
                                self.MBASE(self.TEMP_CASE_PLA))  # setting
        self.lcd.Frame_AreaCopy(1, 107, 76, 156, 86, self.LBLX,
                                self.MBASE(self.TEMP_CASE_ABS))  # Preheat...
        self.lcd.Frame_AreaCopy(1, 172, 76, 198, 86, self.LBLX + 52,
                                self.MBASE(self.TEMP_CASE_ABS))  # ...ABS
        self.lcd.Frame_AreaCopy(1, 131, 119, 182, 132, self.LBLX + 81,
                                self.MBASE(self.TEMP_CASE_ABS))  # setting
        self.Draw_Back_First(self.select_temp.now == 0)
        if self.select_temp.now:
            self.Draw_Menu_Cursor(self.select_temp.now)
        self.Draw_Menu_Line(self.TEMP_CASE_TEMP, self.ICON_SetEndTemp)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216, self.MBASE(self.TEMP_CASE_TEMP),
            self.pd.thermalManager['temp_hotend'][0]['target'])
        self.Draw_Menu_Line(self.TEMP_CASE_BED, self.ICON_SetBedTemp)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216, self.MBASE(self.TEMP_CASE_BED),
            self.pd.thermalManager['temp_bed']['target'])
        self.Draw_Menu_Line(self.TEMP_CASE_FAN, self.ICON_FanSpeed)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216, self.MBASE(self.TEMP_CASE_FAN),
            self.pd.thermalManager['fan_speed'][0])
        self.Draw_Menu_Line(self.TEMP_CASE_PLA, self.ICON_PLAPreheat)
        self.Draw_More_Icon(self.TEMP_CASE_PLA)
        self.Draw_Menu_Line(self.TEMP_CASE_ABS, self.ICON_ABSPreheat)
        self.Draw_More_Icon(self.TEMP_CASE_ABS)

    def Draw_Preheat_Menu(self, preset_idx):
        # Shared drawing for the PLA / ABS preheat settings menus
        preset = self.pd.material_preset[preset_idx]
        self.Clear_Main_Window()
        if preset_idx == 0:
            self.lcd.Frame_TitleCopy(1, 56, 16, 141, 28)  # "PLA Settings"
            mat_x1, mat_x2 = 157, 181
        else:
            self.lcd.Frame_TitleCopy(1, 56, 16, 141, 28)  # "ABS Settings"
            mat_x1, mat_x2 = 172, 198
        self.lcd.Frame_AreaCopy(1, mat_x1, 76, mat_x2, 86, self.LBLX,
                                self.MBASE(self.PREHEAT_CASE_TEMP))
        self.lcd.Frame_AreaCopy(1, 197, 104, 238, 114, self.LBLX + 27,
                                self.MBASE(self.PREHEAT_CASE_TEMP))
        self.lcd.Frame_AreaCopy(1, 1, 89, 83, 101, self.LBLX + 71,
                                self.MBASE(self.PREHEAT_CASE_TEMP))
        self.lcd.Frame_AreaCopy(1, mat_x1, 76, mat_x2, 86, self.LBLX,
                                self.MBASE(self.PREHEAT_CASE_BED) + 3)
        self.lcd.Frame_AreaCopy(1, 240, 104, 264, 114, self.LBLX + 27,
                                self.MBASE(self.PREHEAT_CASE_BED) + 3)
        self.lcd.Frame_AreaCopy(1, 1, 89, 83, 101, self.LBLX + 54,
                                self.MBASE(self.PREHEAT_CASE_BED) + 3)
        self.lcd.Frame_AreaCopy(1, mat_x1, 76, mat_x2, 86, self.LBLX,
                                self.MBASE(self.PREHEAT_CASE_FAN))
        self.lcd.Frame_AreaCopy(1, 0, 119, 64, 132, self.LBLX + 27,
                                self.MBASE(self.PREHEAT_CASE_FAN))
        self.lcd.Frame_AreaCopy(1, 97, 165, 229, 177, self.LBLX,
                                self.MBASE(self.PREHEAT_CASE_SAVE))  # Save
        self.Draw_Back_First()
        self.Draw_Menu_Line(self.PREHEAT_CASE_TEMP, self.ICON_SetEndTemp)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216,
            self.MBASE(self.PREHEAT_CASE_TEMP), preset.hotend_temp)
        self.Draw_Menu_Line(self.PREHEAT_CASE_BED, self.ICON_SetBedTemp)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216,
            self.MBASE(self.PREHEAT_CASE_BED), preset.bed_temp)
        self.Draw_Menu_Line(self.PREHEAT_CASE_FAN, self.ICON_FanSpeed)
        self.lcd.Draw_IntValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 216,
            self.MBASE(self.PREHEAT_CASE_FAN), preset.fan_speed)
        self.Draw_Menu_Line(self.PREHEAT_CASE_SAVE, self.ICON_WriteEEPROM)

    def Draw_Motion_Menu(self):
        self.Clear_Main_Window()
        self.lcd.Frame_TitleCopy(1, 144, 16, 189, 26)  # "Motion"
        self.draw_max_en(self.MBASE(self.MOTION_CASE_RATE))
        self.draw_speed_en(27, self.MBASE(self.MOTION_CASE_RATE))
        self.draw_max_accel_en(self.MBASE(self.MOTION_CASE_ACCEL))
        self.draw_steps_per_mm(self.MBASE(self.MOTION_CASE_STEPS))
        self.Draw_Back_First(self.select_motion.now == 0)
        if self.select_motion.now:
            self.Draw_Menu_Cursor(self.select_motion.now)
        self.Draw_Menu_Line(self.MOTION_CASE_RATE, self.ICON_MaxSpeed)
        self.Draw_Menu_Line(self.MOTION_CASE_ACCEL,
                            self.ICON_MaxAccelerated)
        self.Draw_Menu_Line(self.MOTION_CASE_STEPS, self.ICON_Step)

    def Draw_Move_Menu(self):
        self.Clear_Main_Window()
        self.lcd.Frame_TitleCopy(1, 231, 2, 265, 12)  # "Move"
        self.draw_move_en(self.MBASE(1))
        self.say_x(36, self.MBASE(1))
        self.draw_move_en(self.MBASE(2))
        self.say_y(36, self.MBASE(2))
        self.draw_move_en(self.MBASE(3))
        self.say_z(36, self.MBASE(3))
        self.lcd.Frame_AreaCopy(1, 123, 192, 176, 202, self.LBLX,
                                self.MBASE(4))  # "Extruder"
        self.Draw_Back_First(self.select_axis.now == 0)
        if self.select_axis.now:
            self.Draw_Menu_Cursor(self.select_axis.now)
        for i in range(4):
            self.Draw_Menu_Line(i + 1, self.ICON_MoveX + i)

    def Draw_Move_Values(self):
        cp = self.pd.current_position
        self.lcd.Draw_FloatValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 1, 216, self.MBASE(1),
            cp.x * self.MINUNITMULT)
        self.lcd.Draw_FloatValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 1, 216, self.MBASE(2),
            cp.y * self.MINUNITMULT)
        self.lcd.Draw_FloatValue(
            True, True, 0, self.lcd.font8x16, self.lcd.Color_White,
            self.lcd.Color_Bg_Black, 3, 1, 216, self.MBASE(3),
            cp.z * self.MINUNITMULT)
        self.lcd.Draw_Signed_Float(self.lcd.font8x16,
                                   self.lcd.Color_Bg_Black, 3, 1, 216,
                                   self.MBASE(4), cp.e * self.MINUNITMULT)

    ##################################################################
    # Main screens
    ##################################################################

    def Goto_MainMenu(self):
        self.checkkey = self.MainMenu
        self.Clear_Main_Window()
        self.lcd.Frame_AreaCopy(1, 0, 2, 39, 12, 14, 9)  # "Home"
        self.lcd.ICON_Show(self.ICON, self.ICON_LOGO, 71, 52)
        self.ICON_Print()
        self.ICON_Prepare()
        self.ICON_Control()
        if self.pd.HAS_ONESTEP_LEVELING:
            self.ICON_Leveling(self.select_page.now == 3)
        else:
            self.ICON_StartInfo(self.select_page.now == 3)
        self.lcd.UpdateLCD()

    def Goto_PrintProcess(self):
        self.checkkey = self.PrintProcess
        self.Clear_Main_Window()
        self.Draw_Printing_Screen()
        self.ICON_Tune()
        self.ICON_ResumeOrPause()
        self.ICON_Stop()
        name = self.pd.file_name
        if name:
            name = name[:30]
            npos = _MAX(0, self.lcd.DWIN_WIDTH
                        - len(name) * self.MENU_CHR_W) // 2
            self.lcd.Draw_String(False, False, self.lcd.font8x16,
                                 self.lcd.Color_White,
                                 self.lcd.Color_Bg_Black, npos, 60, name)
        self.lcd.ICON_Show(self.ICON, self.ICON_PrintTime, 17, 193)
        self.lcd.ICON_Show(self.ICON, self.ICON_RemainTime, 150, 191)
        self.last_progress_draw = {}
        self.Draw_Print_ProgressBar()
        self.Draw_Print_ProgressElapsed()
        self.Draw_Print_ProgressRemain()
        self.lcd.UpdateLCD()

    def Draw_Printing_Screen(self):
        self.lcd.Frame_AreaCopy(1, 40, 2, 92, 14, 14, 9)  # "Printing"
        self.lcd.Frame_AreaCopy(1, 0, 44, 96, 58, 41, 188)  # "Printing time"
        self.lcd.Frame_AreaCopy(1, 98, 44, 152, 58, 176, 188)  # "Remain"

    def Draw_Print_ProgressBar(self, percent=None):
        if percent is None:
            percent = int(self.pd.getPercent())
        self.lcd.ICON_Show(self.ICON, self.ICON_Bar, 15, 93)
        self.lcd.Draw_Rectangle(1, self.lcd.BarFill_Color,
                                16 + percent * 240 // 100, 93, 256, 113)
        self.lcd.Draw_IntValue(True, True, 0, self.lcd.font8x16,
                               self.lcd.Percent_Color,
                               self.lcd.Color_Bg_Black, 2, 117, 133, percent)
        self.lcd.Draw_String(False, False, self.lcd.font8x16,
                             self.lcd.Percent_Color,
                             self.lcd.Color_Bg_Black, 133, 133, "%")

    def Draw_Print_ProgressElapsed(self):
        elapsed = int(self.pd.duration())
        self.lcd.Draw_IntValue(True, True, 1, self.lcd.font8x16,
                               self.lcd.Color_White, self.lcd.Color_Bg_Black,
                               2, 42, 212, elapsed // 3600)
        self.lcd.Draw_String(False, False, self.lcd.font8x16,
                             self.lcd.Color_White, self.lcd.Color_Bg_Black,
                             58, 212, ":")
        self.lcd.Draw_IntValue(True, True, 1, self.lcd.font8x16,
                               self.lcd.Color_White, self.lcd.Color_Bg_Black,
                               2, 66, 212, (elapsed % 3600) // 60)

    def Draw_Print_ProgressRemain(self):
        remain = int(self.pd.remain())
        self.lcd.Draw_IntValue(True, True, 1, self.lcd.font8x16,
                               self.lcd.Color_White, self.lcd.Color_Bg_Black,
                               2, 176, 212, remain // 3600)
        self.lcd.Draw_String(False, False, self.lcd.font8x16,
                             self.lcd.Color_White, self.lcd.Color_Bg_Black,
                             192, 212, ":")
        self.lcd.Draw_IntValue(True, True, 1, self.lcd.font8x16,
                               self.lcd.Color_White, self.lcd.Color_Bg_Black,
                               2, 200, 212, (remain % 3600) // 60)

    ##################################################################
    # Popup windows
    ##################################################################

    def Draw_Select_Highlight(self, sel):
        self.pd.HMI_flag.select_flag = sel
        if sel:
            c1 = self.lcd.Select_Color
            c2 = self.lcd.Color_Bg_Window
        else:
            c1 = self.lcd.Color_Bg_Window
            c2 = self.lcd.Select_Color
        self.lcd.Draw_Rectangle(0, c1, 25, 279, 126, 318)
        self.lcd.Draw_Rectangle(0, c1, 24, 278, 127, 319)
        self.lcd.Draw_Rectangle(0, c2, 145, 279, 246, 318)
        self.lcd.Draw_Rectangle(0, c2, 144, 278, 247, 319)

    def Popup_window_PauseOrStop(self):
        self.Clear_Main_Window()
        self.Draw_Popup_Bkgd_60()
        if self.select_print.now == 1:
            self.lcd.Draw_String(
                False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
                self.lcd.Color_Bg_Window, (272 - 8 * 11) // 2, 150,
                self.MSG_PAUSE_PRINT)
        elif self.select_print.now == 2:
            self.lcd.Draw_String(
                False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
                self.lcd.Color_Bg_Window, (272 - 8 * 10) // 2, 150,
                self.MSG_STOP_PRINT)
        self.lcd.ICON_Show(self.ICON, self.ICON_Confirm_E, 26, 280)
        self.lcd.ICON_Show(self.ICON, self.ICON_Cancel_E, 146, 280)
        self.Draw_Select_Highlight(True)

    def Popup_Window_Home(self):
        self.Clear_Main_Window()
        self.Draw_Popup_Bkgd_60()
        self.lcd.ICON_Show(self.ICON, self.ICON_BLTouch, 101, 105)
        self.lcd.Draw_String(
            False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
            self.lcd.Color_Bg_Window, (272 - 8 * 10) // 2, 230, "Homing XYZ")
        self.lcd.Draw_String(
            False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
            self.lcd.Color_Bg_Window, (272 - 8 * 23) // 2, 260,
            "Please wait until done.")

    def Popup_Window_Leveling(self):
        self.Clear_Main_Window()
        self.Draw_Popup_Bkgd_60()
        self.lcd.ICON_Show(self.ICON, self.ICON_AutoLeveling, 101, 105)
        self.lcd.Draw_String(
            False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
            self.lcd.Color_Bg_Window, (272 - 8 * 13) // 2, 230,
            "Auto leveling")
        self.lcd.Draw_String(
            False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
            self.lcd.Color_Bg_Window, (272 - 8 * 23) // 2, 260,
            "Please wait until done.")

    def Popup_Window_ETempTooLow(self):
        self.Clear_Main_Window()
        self.Draw_Popup_Bkgd_60()
        self.lcd.ICON_Show(self.ICON, self.ICON_TempTooLow, 102, 105)
        self.lcd.Draw_String(
            False, True, self.lcd.font8x16, self.lcd.Popup_Text_Color,
            self.lcd.Color_Bg_Window, 20, 235, "Nozzle is too cold")
        self.lcd.ICON_Show(self.ICON, self.ICON_Confirm_E, 86, 280)

    ##################################################################
    # Main menu / print screen icons
    ##################################################################

    def ICON_Print(self):
        if self.select_page.now == 0:
            self.lcd.ICON_Show(self.ICON, self.ICON_Print_1, 17, 130)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 17, 130, 126,
                                    229)
            self.lcd.Frame_AreaCopy(1, 1, 451, 31, 463, 57, 201)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Print_0, 17, 130)
            self.lcd.Frame_AreaCopy(1, 1, 423, 31, 435, 57, 201)

    def ICON_Prepare(self):
        if self.select_page.now == 1:
            self.lcd.ICON_Show(self.ICON, self.ICON_Prepare_1, 145, 130)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 145, 130, 254,
                                    229)
            self.lcd.Frame_AreaCopy(1, 33, 451, 82, 466, 175, 201)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Prepare_0, 145, 130)
            self.lcd.Frame_AreaCopy(1, 33, 423, 82, 438, 175, 201)

    def ICON_Control(self):
        if self.select_page.now == 2:
            self.lcd.ICON_Show(self.ICON, self.ICON_Control_1, 17, 246)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 17, 246, 126,
                                    345)
            self.lcd.Frame_AreaCopy(1, 85, 451, 132, 463, 48, 318)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Control_0, 17, 246)
            self.lcd.Frame_AreaCopy(1, 85, 423, 132, 434, 48, 318)

    def ICON_Leveling(self, show):
        if show:
            self.lcd.ICON_Show(self.ICON, self.ICON_Leveling_1, 145, 246)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 145, 246, 254,
                                    345)
            self.lcd.Frame_AreaCopy(1, 84, 437, 120, 449, 182, 318)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Leveling_0, 145, 246)
            self.lcd.Frame_AreaCopy(1, 84, 465, 120, 478, 182, 318)

    def ICON_StartInfo(self, show):
        if show:
            self.lcd.ICON_Show(self.ICON, self.ICON_Info_1, 145, 246)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 145, 246, 254,
                                    345)
            self.lcd.Frame_AreaCopy(1, 132, 451, 159, 466, 186, 318)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Info_0, 145, 246)
            self.lcd.Frame_AreaCopy(1, 132, 423, 159, 435, 186, 318)

    def ICON_Tune(self):
        if self.select_print.now == 0:
            self.lcd.ICON_Show(self.ICON, self.ICON_Setup_1, 8, 252)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 8, 252, 87, 351)
            self.lcd.Frame_AreaCopy(1, 0, 466, 34, 476, 31, 325)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Setup_0, 8, 252)
            self.lcd.Frame_AreaCopy(1, 0, 438, 32, 448, 31, 325)

    def ICON_Continue(self):
        if self.select_print.now == 1:
            self.lcd.ICON_Show(self.ICON, self.ICON_Continue_1, 96, 252)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 96, 252, 175,
                                    351)
            self.lcd.Frame_AreaCopy(1, 1, 452, 32, 464, 121, 325)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Continue_0, 96, 252)
            self.lcd.Frame_AreaCopy(1, 1, 424, 31, 434, 121, 325)

    def ICON_Pause(self):
        if self.select_print.now == 1:
            self.lcd.ICON_Show(self.ICON, self.ICON_Pause_1, 96, 252)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 96, 252, 175,
                                    351)
            self.lcd.Frame_AreaCopy(1, 177, 451, 216, 462, 116, 325)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Pause_0, 96, 252)
            self.lcd.Frame_AreaCopy(1, 177, 423, 215, 433, 116, 325)

    def ICON_ResumeOrPause(self):
        if self.pd.printingIsPaused() or self.pd.HMI_flag.pause_flag:
            self.ICON_Continue()
        else:
            self.ICON_Pause()

    def ICON_Stop(self):
        if self.select_print.now == 2:
            self.lcd.ICON_Show(self.ICON, self.ICON_Stop_1, 184, 252)
            self.lcd.Draw_Rectangle(0, self.lcd.Color_White, 184, 252, 263,
                                    351)
            self.lcd.Frame_AreaCopy(1, 218, 452, 249, 466, 209, 325)
        else:
            self.lcd.ICON_Show(self.ICON, self.ICON_Stop_0, 184, 252)
            self.lcd.Frame_AreaCopy(1, 218, 423, 247, 436, 209, 325)

    ##################################################################
    # Status area
    ##################################################################

    def Draw_Status_Area(self):
        # Full draw of the bottom status area
        self.lcd.Draw_Rectangle(1, self.lcd.Color_Bg_Black, 0, self.STATUS_Y,
                                self.lcd.DWIN_WIDTH, self.lcd.DWIN_HEIGHT - 1)
        self.lcd.ICON_Show(self.ICON, self.ICON_HotendTemp, 13, 381)
        self.lcd.Draw_String(False, False, self.lcd.DWIN_FONT_STAT,
                             self.lcd.Color_White, self.lcd.Color_Bg_Black,
                             33 + 3 * self.STAT_CHR_W + 5, 383, "/")
        self.lcd.ICON_Show(self.ICON, self.ICON_BedTemp, 158, 381)
        self.lcd.Draw_String(False, False, self.lcd.DWIN_FONT_STAT,
                             self.lcd.Color_White, self.lcd.Color_Bg_Black,
                             178 + 3 * self.STAT_CHR_W + 5, 383, "/")
        self.lcd.ICON_Show(self.ICON, self.ICON_Speed, 13, 429)
        self.lcd.Draw_String(False, False, self.lcd.DWIN_FONT_STAT,
                             self.lcd.Color_White, self.lcd.Color_Bg_Black,
                             33 + 5 * self.STAT_CHR_W + 2, 429, "%")
        self.lcd.ICON_Show(self.ICON, self.ICON_Zoffset, 158, 428)
        self.last_status_draw = {}
        self._update_status_area()

    def _update_status_area(self):
        # Redraw only the status area values that changed
        tm = self.pd.thermalManager
        last = self.last_status_draw
        fields = {
            'he_cur': tm['temp_hotend'][0]['celsius'],
            'he_tgt': tm['temp_hotend'][0]['target'],
            'bed_cur': tm['temp_bed']['celsius'],
            'bed_tgt': tm['temp_bed']['target'],
            'speed': self.pd.feedrate_percentage,
            'zoff': round(self.pd.BABY_Z_VAR * 100),
        }
        stat = self.lcd.DWIN_FONT_STAT
        white = self.lcd.Color_White
        bg = self.lcd.Color_Bg_Black
        if fields['he_cur'] != last.get('he_cur'):
            self.lcd.Draw_IntValue(True, True, 0, stat, white, bg, 3, 33,
                                   382, fields['he_cur'])
        if fields['he_tgt'] != last.get('he_tgt'):
            self.lcd.Draw_IntValue(True, True, 0, stat, white, bg, 3,
                                   33 + 4 * self.STAT_CHR_W + 6, 382,
                                   fields['he_tgt'])
        if fields['bed_cur'] != last.get('bed_cur'):
            self.lcd.Draw_IntValue(True, True, 0, stat, white, bg, 3, 178,
                                   382, fields['bed_cur'])
        if fields['bed_tgt'] != last.get('bed_tgt'):
            self.lcd.Draw_IntValue(True, True, 0, stat, white, bg, 3,
                                   178 + 4 * self.STAT_CHR_W + 6, 382,
                                   fields['bed_tgt'])
        if fields['speed'] != last.get('speed'):
            self.lcd.Draw_IntValue(True, True, 0, stat, white, bg, 3,
                                   33 + 2 * self.STAT_CHR_W, 429,
                                   fields['speed'])
        if fields['zoff'] != last.get('zoff'):
            self.lcd.Draw_Signed_Float(stat, bg, 2, 2, 178, 429,
                                       fields['zoff'])
        self.last_status_draw = fields

    ##################################################################
    # Periodic update
    ##################################################################

    def HMI_StartFrame(self, with_update):
        if self.pd.status == 'printing':
            self.Goto_PrintProcess()
        elif self.pd.status in ('paused', 'pausing'):
            self.pd.HMI_flag.pause_flag = True
            self.Goto_PrintProcess()
        else:
            self.Goto_MainMenu()
        self.Draw_Status_Area()
        if with_update:
            self.lcd.UpdateLCD()

    def Redraw_Current_Screen(self):
        # Repaint the whole current screen (recovery from lost frames)
        redraw = {
            self.MainMenu: self.Goto_MainMenu,
            self.PrintProcess: self.Goto_PrintProcess,
            self.SelectFile: self.Draw_Print_File_Menu,
            self.Prepare: self.Draw_Prepare_Menu,
            self.Control: self.Draw_Control_Menu,
            self.Info: self.Draw_Info_Menu,
            self.Tune: self.Draw_Tune_Menu,
            self.TemperatureID: self.Draw_Temperature_Menu,
            self.Motion: self.Draw_Motion_Menu,
        }
        handler = redraw.get(self.checkkey)
        if handler is not None:
            handler()
        self.Draw_Status_Area()
        self.lcd.UpdateLCD()

    def _update_event(self, eventtime):
        try:
            self.EachMomentUpdate(eventtime)
        except Exception:
            logging.exception("dwin_t5uic1: error in update timer")
        return eventtime + self.update_interval

    def EachMomentUpdate(self, eventtime):
        if not self.init_done:
            return
        update = self.pd.update_variable()
        # Track print state transitions
        if self.last_status != self.pd.status:
            self.last_status = self.pd.status
            if self.pd.status == 'printing':
                if self.checkkey not in (self.Tune, self.PrintProcess,
                                         self.Print_window):
                    self.select_print.reset()
                    self.Goto_PrintProcess()
            elif self.pd.status == 'complete':
                if self.checkkey in (self.PrintProcess, self.Tune,
                                     self.Print_window):
                    self.pd.HMI_flag.done_confirm_flag = True
                    self.checkkey = self.PrintProcess
                    self.Goto_PrintProcess()
                    self.Draw_Print_ProgressBar(100)
                    self.lcd.Draw_Rectangle(
                        1, self.lcd.Color_Bg_Black, 0, 250,
                        self.lcd.DWIN_WIDTH - 1, self.STATUS_Y)
                    self.lcd.ICON_Show(self.ICON, self.ICON_Confirm_E, 86,
                                       283)
            elif self.pd.status in ('standby', 'cancelled', 'error'):
                if self.checkkey in (self.PrintProcess, self.Tune,
                                     self.Print_window):
                    self.pd.HMI_flag.pause_flag = False
                    self.select_page.set(0)
                    self.Goto_MainMenu()
        # Update pause/resume icon while printing
        if self.checkkey == self.PrintProcess:
            if (not self.pd.HMI_flag.done_confirm_flag
                    and self.pd.HMI_flag.pause_flag
                    != self.pd.printingIsPaused()):
                self.pd.HMI_flag.pause_flag = self.pd.printingIsPaused()
                self.ICON_ResumeOrPause()
            self._update_progress()
        # Periodic full redraw / recovery from dropped frames
        if (self.need_full_redraw
                or (self.full_redraw_interval
                    and eventtime > (self.last_full_redraw
                                     + self.full_redraw_interval))):
            self.need_full_redraw = False
            self.last_full_redraw = eventtime
            self.Redraw_Current_Screen()
        elif update:
            self._update_status_area()
        self.lcd.UpdateLCD()

    def _update_progress(self):
        if self.pd.HMI_flag.done_confirm_flag:
            return
        last = self.last_progress_draw
        percent = int(self.pd.getPercent())
        elapsed = int(self.pd.duration()) // 60
        remain = int(self.pd.remain()) // 60
        if percent != last.get('percent'):
            self.Draw_Print_ProgressBar(percent)
        if elapsed != last.get('elapsed'):
            self.Draw_Print_ProgressElapsed()
        if remain != last.get('remain'):
            self.Draw_Print_ProgressRemain()
        self.last_progress_draw = {'percent': percent, 'elapsed': elapsed,
                                   'remain': remain}


def load_config(config):
    return DWIN_LCD(config)

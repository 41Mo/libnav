from ctypes import *
import faulthandler
import os.path
import sys
from typing import List
from functools import singledispatch

faulthandler.enable()

source_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
files_list = []
for root, dirs, files in os.walk(source_dir):
    for name in files:
        if name.endswith((".so")):
            files_list.append(os.path.join(root, name))

if len(files_list) == 0:
    print("Error: nav library not found in " + source_dir)
    sys.exit(-1)

if len(files_list) > 1:
    print("Error: Found multiple libraries in " +source_dir)
    sys.exit(-1)

iface_lib = CDLL(files_list.pop())

Tarr3f = c_float*3
Tarr2f = c_float*2
Tarr3d = c_double*3
Tarr2d = c_double*2
class Vector3f(Structure):
    _fields_ = [("data", Tarr3f)]

class Vector2f(Structure):
    _fields_ = [("data", Tarr2f)]

class SENS(Structure):
    _fields_ = [("size", c_int),
                ("acc", POINTER(Vector3f)),
                ("gyr", POINTER(Vector3f)),
    ]

class NAV_OUT(Structure):
    _feilds_ = [("size", c_int),
                ("pry", POINTER(Vector3f)),
                ("coord", POINTER(Vector2f)),
                ("vel", POINTER(Vector2f)),
                ]

iface_lib.NavIface_new.restype  = c_void_p
iface_lib.NavIface_new.argtypes = [
    c_float,
    c_float,
    c_int
]

iface_lib.n_alignment_rph.restype = None
iface_lib.n_alignment_rph.argtypes = [
    c_void_p,
    c_float, c_float, c_float
]

iface_lib.n_alignment_acc.restype = None
iface_lib.n_alignment_acc.argtypes = [
    c_void_p,
    c_float, c_float, c_float, c_float
]

iface_lib.n_alignment_cos.restype = None
iface_lib.n_alignment_cos.argtypes = [
    c_void_p,
    c_float, c_float, c_float, c_float, c_float, c_float
]

iface_lib.i_get_g.restype = c_float
iface_lib.i_get_g.argtypes = None

iface_lib.i_get_u.restype = c_float
iface_lib.i_get_u.argtypes = None

iface_lib.i_nav.restype = c_void_p
iface_lib.i_nav.argtypes = [c_void_p]

iface_lib.n_iter.restype = c_void_p
iface_lib.n_iter.argtypes = [c_void_p, Tarr3d, Tarr3d]

iface_lib.n_iter_gnss.restype = c_void_p
iface_lib.n_iter_gnss.argtypes = [c_void_p, Tarr3d, Tarr3d, Tarr2d]

iface_lib.n_pry.restype = c_void_p
iface_lib.n_pry.argtypes = [c_void_p, Tarr3d]

iface_lib.n_vel.restype = c_void_p
iface_lib.n_vel.argtypes = [c_void_p, Tarr3d]

iface_lib.n_pos.restype = c_void_p
iface_lib.n_pos.argtypes = [c_void_p, Tarr2d]

iface_lib.n_align_prh.restype = c_void_p
iface_lib.n_align_prh.argtypes = [c_void_p, Tarr3f]

iface_lib.n_set_pos.restype = c_void_p
iface_lib.n_set_pos.argtypes = [c_void_p, c_float, c_float]

iface_lib.n_time_corr.restype = c_void_p
iface_lib.n_time_corr.argtypes = [c_void_p, c_float]

iface_lib.n_mode_corr.restype = c_void_p
iface_lib.n_mode_corr.argtypes = [c_void_p, c_bool]

iface_lib.n_corr_k.restype = c_float
iface_lib.n_corr_k.argtypes = [c_void_p, c_int]

iface_lib.toggle_rad_c.restype = c_void_p
iface_lib.toggle_rad_c.argtypes = [c_void_p, c_bool]

iface_lib.rad_set_k.restype = c_void_p
iface_lib.rad_set_k.argtypes = [c_void_p, c_float]

iface_lib.toggle_integ_rad_c.restype = c_void_p
iface_lib.toggle_integ_rad_c.argtypes = [c_void_p, c_bool]

iface_lib.integ_rad_set_k.restype = c_void_p
iface_lib.integ_rad_set_k.argtypes = [c_void_p, c_float]

class Nav(object):
    def __init__(self, nav_iface_ptr:c_void_p) -> None:
        self.obj = iface_lib.i_nav(nav_iface_ptr)

    def alignment_rph(self, roll, pitch, yaw):
        iface_lib.n_alignment_rph(self.obj, roll, pitch, yaw)
    def alignment_acc(self, ax_mean, ay_mean, az_mean, yaw):
        iface_lib.n_alignment_acc(self.obj, ax_mean, ay_mean, az_mean, yaw)
    def alignment_cos(self, st, ct, sg, cg, sp, cp):
        iface_lib.n_alignment_cos(self.obj, st, ct, sg, cg, sp, cp)

    def iter(val):
        raise NotImplementedError

    def iter(self, a_x, a_y, a_z, g_x, g_y, g_z):
        iface_lib.n_iter(self.obj,
            Tarr3d(a_x, a_y, a_z),
            Tarr3d(g_x, g_y, g_z)
        )

    def iter_gnss(self, a, g, gnss_p):
        iface_lib.n_iter_gnss(self.obj,
            Tarr3d(a[0], a[1], a[2]),
            Tarr3d(g[0], g[1], g[2]),
            Tarr2d(gnss_p[0], gnss_p[1]),
        )
    def gnss_T(self, T):
        iface_lib.n_time_corr(self.obj, T)
    def corr_mode(self, mode):
        iface_lib.n_mode_corr(self.obj, mode)
    def pry(self, rot:Tarr3f):
        iface_lib.n_pry(self.obj, rot)
    def get_pry(self):
        rot = Tarr3d()
        iface_lib.n_pry(self.obj, rot)
        return rot[:]

    def vel(self, vel:Tarr3f):
        iface_lib.n_vel(self.obj, vel)
    def get_vel(self):
        vel = Tarr3d()
        iface_lib.n_vel(self.obj, vel)
        return vel[:]

    def pos(self, pos:Tarr2f):
        iface_lib.n_pos(self.obj, pos)
    def get_pos(self):
        pos = Tarr2d()
        iface_lib.n_pos(self.obj, pos)
        return pos[:]

    def align_prh(self) -> List:
        res = Tarr3f()
        iface_lib.n_align_prh(self.obj, res)
        return res[:3]

    def set_pos(self, lat, lon):
        iface_lib.n_set_pos(self.obj, lat,lon)
    
    def get_k(self, number):
        return iface_lib.n_corr_k(self.obj, number)
    
    def toggle_rad_c(self, mode):
        iface_lib.toggle_rad_c(self.obj, mode)

    def rad_set_k(self, k):
        iface_lib.rad_set_k(self.obj, k)

    def toggle_integ_rad_c(self, mode):
        iface_lib.toggle_integ_rad_c(self.obj, mode)

    def integ_rad_set_k(self, k):
        iface_lib.integ_rad_set_k(self.obj, k)

class NavIface(object):
    def __init__(self, lat,lon, frequency):
        self.obj = iface_lib.NavIface_new(lat, lon, frequency)
        self.nav_obj = Nav(self.obj)

    def U(self):
        return iface_lib.i_get_u()

    def G(self):
        return iface_lib.i_get_g()

    def solution(self, a_x, a_y, a_z, g_x, g_y, g_z) -> NAV_OUT:
        '''
        a - accelerometer data
        g - gyroscope data
        '''
        # ctypes magic
        p = self.points
        TypeVec3fArr = Vector3f * p
        a = TypeVec3fArr()
        g = TypeVec3fArr()

        if (type(a_x) and type(a_y) and type(a_z)) == list:
            for i, ax_i, ay_i, az_i in zip(range(0, p), a_x, a_y, a_z):
                v = Vector3f([ax_i, ay_i, az_i])
                a[i] = v
        elif ((type(a_x) and type(a_y) and type(a_z)) == float):
            for i in range(0, p):
                v = Vector3f([a_x, a_y, a_z])
                a[i] = v
        else:
            print("Wrong set sens data type. Use float or list instead.")
            sys.exit()

        if (type(g_x) and type(g_y) and type(g_z)) == list:
            for i, gx_i, gy_i, gz_i in zip(range(0, p), g_x, g_y, g_z):
                v = Vector3f([gx_i, gy_i, gz_i])
                g[i] = v
        elif ((type(g_x) and type(g_y) and type(g_z)) == float):
            for i in range(0, p):
                v = Vector3f([g_x, g_y, g_z])
                g[i] = v
        else:
            print("Wrong set sens data type. Use float or list instead.")
            sys.exit()

        return iface_lib.solution(self.obj, SENS(p, a, g))
    
    def nav(self):
        return self.nav_obj
#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
from fibre import Logger, Event
from odrive.utils import OperationAbortedException
from fibre.protocol import ChannelBrokenException
import sys

#-------------参数配置-------------
reset_all= 1 #擦除参数
en_index= False #使用索引
fast_per_cal = True #快速上电标定 跳过电机FOC环系数标定
en_brake_resistance= 0#使用功率电阻

gain_all = 0.15 #无命令下的位置闭环

gear_ratio = 1 #位置控制的减速比

pole = 14  #极对数
max_a= 20  #最大电流
r_max_a= 30 #最大电流采样
max_v= 36  #保护电压

cpr_all= 16384 #编码器CPR
calib_range = 0.15#标定误差冗余0.019999999552965164*10

vel_limit = 50000#turn/s 转速保护

def set_gains(odrv,axis):
    """
    Sets the nested PID gains to a good default
    """

    axis.controller.config.pos_gain = 100.0* gain_all#f [(counts/s) / counts] <<-------------无命令下的参数 CPR模式
    axis.controller.config.pos_gain = 0.01 * gain_all#f [(counts/s) / counts]
    axis.controller.config.vel_gain = 5.0 / 10000.0 * gain_all#[A/(counts/s)]
    axis.controller.config.vel_limit = vel_limit
    axis.controller.config.vel_integrator_gain = 0 * gain_all#[A/((counts/s) * s)]

    axis.controller.config.kp_theta = 0.04 *  6000 / (2 * math.pi)* gain_all#<<-------------考虑减速比的 360°位置闭环模式
    axis.controller.config.kd_theta = 5.0 / 10000.0 *  6000 / (2 * math.pi)* gain_all
    axis.controller.config.kp_gamma = 0.0 *  6000 / (2 * math.pi)* gain_all
    axis.controller.config.kd_gamma = 5.0 / 10000.0 *  6000 / (2 * math.pi)* gain_all

def calibrate_motor(odrv, axis):
    # time.sleep(0.5)
    print('Calibrating motor...',end='',flush=True)
    run_state(axis, AXIS_STATE_MOTOR_CALIBRATION, True)
    axis.motor.config.pre_calibrated = True# then set motor pre calibrated to true
    print('Done');

def calibrate_motor_and_zencoder(odrv,axis):
    """
    Runs motor and encoder calibration (with z index) and saves
    Enables closed loop control on startup and encoder search on startup
    """

    calibrate_motor(odrv, axis)

    print('Calibrating encoder...',end='',flush=True)
    axis.encoder.config.use_index = True
    run_state(axis, AXIS_STATE_ENCODER_INDEX_SEARCH, True)
    run_state(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, True)
    axis.encoder.config.pre_calibrated = True
    print('Done');

    print('Setting startup flags...',end='',flush=True)
    # axis.config.startup_encoder_index_search = True
    axis.config.startup_encoder_index_search = True
    axis.config.startup_closed_loop_control = True
    print('Done with axis.')

def run_state(axis, requested_state, wait):
    """
    Sets the requested state on the given axis. If wait is True, the method
    will block until the state goes back to idle.

    Returns whether the odrive went back to idle in the given timeout period,
    which is 10s by default.
    """
    axis.requested_state = requested_state
    timeout_ctr = 100; # 20s timeout for encoder calibration to finish
    if wait:
        while(timeout_ctr > 0): # waits until state goes back to idle to continue
            time.sleep(0.2)
            timeout_ctr -= 1
            if axis.current_state == AXIS_STATE_IDLE:
                break
    return timeout_ctr > 0

def reboot_odrive(odrv):
    """
    Reboot odrive
    """
    try:
        odrv.reboot()
    except ChannelBrokenException:
        print('Lost connection because of reboot...')

def reset_odrive(odrv):
    """
    Erase config
    """
    print('Erasing config and rebooting...')
    odrv.erase_configuration()
    reboot_odrive(odrv)

def init_odrive(odrv):
    """
    NOTE: does not save
    """
    print('Initializing ODrive...')
    odrv.config.brake_resistance = 2.0*en_brake_resistance# 配置功率耗散电阻阻值，我们使用的的功率耗散电阻阻值为 2 Ohm

    odrv.axis0.motor.config.pole_pairs = pole
    odrv.axis1.motor.config.pole_pairs = pole

    odrv.axis0.motor.config.resistance_calib_max_voltage = 4.0#配置电机参数校准时的电压，当电机的相电阻越高此值应该越高，但是如果此值过高会造成电流过大，产生过流保护错误
    odrv.axis1.motor.config.resistance_calib_max_voltage = 4.0#配置电机参数校准时的电压，当电机的相电阻越高此值应该越高，但是如果此值过高会造成电流过大，产生过流保护错误

    odrv.axis0.encoder.config.cpr = cpr_all#16384
    odrv.axis1.encoder.config.cpr = cpr_all#16384

    #odrv.axis0.encoder.config.bandwidth = 3000#设置编码器 PLL 带宽，一般对于高分辨率编码器 (> 4000个计数/转) 此值应该越高，这样有助于减少电机振动
    #odrv.axis1.encoder.config.bandwidth = 3000#设置编码器 PLL 带宽，一般对于高分辨率编码器 (> 4000个计数/转) 此值应该越高，这样有助于减少电机振动
    #[RW] odrv0.axis0.encoder.config.calib_range calib_range = 0.019999999552965164 (float)
#类型为 [float]，通过编码器cpr检查所需的精度，用于在执行编码器偏移校准时编码器的实际输出计算的移动距离和电机开环步进移动距离之间允许的最大误差，超过此误差将报错 ERROR_CPR_OUT_OF_RANGE。

#[RW] odrv0.axis0.encoder.config.calib_scan_distance
#类型为 [float]，单位为 [rad]，编码器偏移校准时转动的距离，此值越大校准精度相对越高，但是同时校准所需的时间越长。
    #odrv.axis0.config.watchdog_timeout = 3
    #odrv.axis1.config.watchdog_timeout = 3
    odrv.config.dc_bus_overvoltage_trip_level = max_v

    odrv.axis0.motor.config.current_lim = max_a #40
    odrv.axis1.motor.config.current_lim = max_a #40
    #odrv.config.dc_max_positive_curren  = max_a
    odrv.axis0.motor.config.requested_current_range = r_max_a
    odrv.axis1.motor.config.requested_current_range = r_max_a
    
    odrv.axis0.controller.config.vel_limit = vel_limit
    odrv.axis1.controller.config.vel_limit = vel_limit

   # odrv.axis0.motor.current_control_.max_allowed_current
    # odrv.axis0.config.calibration_lockin.current = 10
    # odrv.axis0.config.calibration_lockin.ramp_time = 0.4
    # odrv.axis0.config.calibration_lockin.ramp_distance = 3.1415927410125732
    # odrv.axis0.config.calibration_lockin.accel = 20
    # odrv.axis0.config.calibration_lockin.vel = 40
    # odrv.axis1.config.calibration_lockin.current = 10
    # odrv.axis1.config.calibration_lockin.ramp_time = 0.4
    # odrv.axis1.config.calibration_lockin.ramp_distance = 3.1415927410125732
    # odrv.axis1.config.calibration_lockin.accel = 20
    # odrv.axis1.config.calibration_lockin.vel = 40

    odrv.axis0.controller.config.gear_ratio = gear_ratio
    odrv.axis1.controller.config.gear_ratio = gear_ratio
    
    odrv.axis0.motor.config.calibration_current = 10
    odrv.axis1.motor.config.calibration_current = 10

    odrv.axis0.motor.config.pre_calibrated = fast_per_cal
    odrv.axis1.motor.config.pre_calibrated = fast_per_cal

    odrv.axis0.encoder.config.pre_calibrated = True #HALL 传感器下直接跳过标定 使用存储偏差 
    odrv.axis1.encoder.config.pre_calibrated = True #HALL 传感器下直接跳过标定 使用存储偏差   
    print('Done!!!!!!!!!!!!!!!!!!!!')

def set_odrive_limits(odrv,axis,cur_lim):
    """
    Set motor current limits
    """
    axis.motor.config.current_lim = cur_lim

def calibrate_odrive(odrv):
    """
    Calibrate motors and encoders (including index) on both axes
    """
    print('Calibrating axes...')

    calibrate_motor_and_zencoder(odrv,odrv.axis0)
    calibrate_motor_and_zencoder(odrv,odrv.axis1)
    print('Done.')

def set_odrive_gains(odrv):
    """
    Set position control gains for both motors
    """
    print('Setting gains...')
    set_gains(odrv,odrv.axis0)
    set_gains(odrv,odrv.axis1)
    print('Done.')

def test_odrive_motors(odrv):
    """
    Give two position commands to both axis
    """
    print('Testing motors...')
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    time.sleep(0.5)
    odrv.axis0.controller.set_pos_setpoint(1000.0, 0.0, 0.0)
    odrv.axis1.controller.set_pos_setpoint(-1000.0, 0.0, 0.0)
    time.sleep(1)
    odrv.axis0.controller.set_pos_setpoint(0.0, 0.0, 0.0)
    odrv.axis1.controller.set_pos_setpoint(0.0, 0.0, 0.0)
    print('Done.')

def get_odrive(shutdown_token):
    """
    Look for and return an odrive connected via usb
    """

    print('Looking for ODrive...')
    odrv = odrive.find_any(search_cancellation_token=app_shutdown_token, channel_termination_token=app_shutdown_token)
    print('Found.')
    return odrv

def print_configs(odrv0):
    """
    Print configuration info
    """

    print('\n\nMOTOR CONFIG')
    print(odrv0.axis0.motor.config)
    print('\n')
    print(odrv0.axis1.motor.config)
    print('\n\n ENC CONFIG')
    print(odrv0.axis0.encoder.config)
    print('\n')
    print(odrv0.axis1.encoder.config)
    print('\n\n AXIS CONFIG')
    print(odrv0.axis0.config)
    print('\n')
    print(odrv0.axis1.config)

def full_calibration(app_shutdown_token):
    """
    Reset the odrive and calibrate everything:
    1) Gains and limits
    2) Motor
    3) Encoder index
    4) Encoder offset
    5) Set startup
    """

    # Find a connected ODrive (this will block until you connect one)
    odrv0 = get_odrive(app_shutdown_token)
    reset_odrive(odrv0)
    odrv0 = get_odrive(app_shutdown_token)

    init_odrive(odrv0)
    set_odrive_gains(odrv0)
    calibrate_odrive(odrv0)

    odrv0.save_configuration()

    #run_state(axis=odrv0.axis0, requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL, wait=False);
    #run_state(axis=odrv0.axis1, requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL, wait=False);
    #odrv0.axis0.controller.pos_setpoint = 0;
    #odrv0.axis1.controller.pos_setpoint = 0;

    print_configs(odrv0)

def bare_bones_calibration(app_shutdown_token, reset=True):
    """
    Just calibrate motors and basic gains
    """
    print("Run doghome!!!!!!!!!!!!!!!!!!!\n ")# odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH   odrv0.axis0.encoder.index_found
    odrv0 = get_odrive(app_shutdown_token)
    if reset_all :
        reset_odrive(odrv0)
        odrv0 = get_odrive(app_shutdown_token)

    init_odrive(odrv0)
    set_odrive_gains(odrv0)

    calibrate_motor(odrv0, odrv0.axis0)
    calibrate_motor(odrv0, odrv0.axis1)

    odrv0.axis0.config.startup_encoder_offset_calibration = True
    odrv0.axis1.config.startup_encoder_offset_calibration = True
    odrv0.axis0.config.startup_closed_loop_control = True
    odrv0.axis1.config.startup_closed_loop_control = True

    odrv0.axis0.encoder.config.use_index = en_index
    odrv0.axis1.encoder.config.use_index = en_index
    odrv0.axis0.encoder.config.calib_range =  calib_range
    odrv0.axis1.encoder.config.calib_range =  calib_range
    odrv0.axis0.config.startup_encoder_index_search =en_index
    odrv0.axis1.config.startup_encoder_index_search =en_index
    
    odrv0.save_configuration()

    print_configs(odrv0)

def main(app_shutdown_token):
    """
    !! Main program !!
    Looks for odrive, then calibrates, then sets gains, then tests motors


    WARNING: Saving more than twice per boot will cause a reversion of all changes
    """
    bare_bones_calibration(app_shutdown_token, reset=True)
    #bare_bones_calibration(app_shutdown_token, reset=False)

import odrive
from fibre import Logger, Event
from odrive.utils import OperationAbortedException
from fibre.protocol import ChannelBrokenException

app_shutdown_token = Event()
try:
    main(app_shutdown_token)
    # init_odrive(odrv0)
except OperationAbortedException:
    logger.info("Operation aborted.")
finally:
    app_shutdown_token.set()

# encoder calibration
# set use_index ot true
# request encoder_index_search
# pre_calibrated to True
# then change startup to encoder_index_search true

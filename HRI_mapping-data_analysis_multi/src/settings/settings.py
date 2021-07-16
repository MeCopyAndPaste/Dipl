#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 15 16:56:28 2020

@author: lis
"""
#####################
###  HRI MAPPING  ###
#####################

#####################################################

import context

from sklearn.linear_model import LinearRegression
from sklearn.multioutput import MultiOutputRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF)
from sklearn.pipeline import make_pipeline
from sklearn.svm import SVR
from sklearn.svm import NuSVR

import utilities.HRI as HRI

import numpy as np
import os
import datetime
import configparser
import logging


def get_settings(preset = None):
    config = configparser.ConfigParser()
    config.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'settings.ini'))

    def parse_ini(s):
        if s == 'True':
            return True
        if s == 'False':
            return False
        try:
            out = float(s)
            if out == int(out):
                return int(out)
            else:
                return out
        except:
            return s
    
    ####################################################

    settings = {}
    for i in config['basic']:
        settings[i] = parse_ini(config['basic'][i])
    for i in config['mapping']:
        settings[i] = parse_ini(config['mapping'][i])
    for i in config['communication']:
        settings[i] = parse_ini(config['communication'][i])
    for i in config['debug']:
        settings[i] = parse_ini(config['debug'][i])
    
    ####################################################

    for i in config['input_motive']:
        settings[i] = parse_ini(config['input_motive'][i])
    for i in config['input_IMUs']:
        settings[i] = parse_ini(config['input_IMUs'][i])
    
    ####################################################

    settings['imu_ID'] = {}
    for i in config['IMU_ID']:
        settings['imu_ID'][i] = parse_ini(config['IMU_ID'][i])

    ####################################################
    ####################################################
    # presets (used to force settings independently from .ini file)

    if preset is not None:
        for i in preset:
            settings[i] = preset[i]

    ####################################################
    ####################################################

    if settings['robot'] == 'fw':
        settings['robot'] = 'Fixed-wing (fixed speed)'
    if settings['robot'] == 'fws':
        settings['robot'] = 'Fixed-wing'
    if settings['robot'] == 'quad':
        settings['robot'] = 'Quadrotor'
    if settings['robot'] == 'manip':
        settings['robot'] = 'Manipulator'
    if settings['robot'] == 'quadmanip':
        settings['robot'] = 'QuadManip'
    
    ####################################################
    
    # if [n_readings] not defined, acquire indefintely
    settings['maneuvers_dict'] = {  18 : 'roll',
                                    19 : 'roll',
                                    20 : 'pitch',
                                    21 : 'pitch',
                                    26 : 'speed',
                                    27 : 'speed',
                                    17 : 'no',
                                    28 : 'no', 
                                    0 : 'vel_z',
                                    1 : 'vel_z',
                                    2 : 'pitch_rate',
                                    3 : 'pitch_rate',
                                    4 : 'vel_y',
                                    5 : 'vel_y',
                                    6 : 'vel_x',
                                    7 : 'vel_x',
                                    8 : 'no',
                                    9 : 'vel_z',
                                    10 : 'vel_z',
                                    11 : 'roll_rate',
                                    12 : 'roll_rate',
                                    13 : 'yaw_rate',
                                    14 : 'yaw_rate',
                                    15 : 'pitch_rate',
                                    16 : 'pitch_rate'
                                    }

    if settings['robot'] == 'Manipulator':
          settings['maneuvers_dict'][0] = 'vel_x'
          settings['maneuvers_dict'][1] = 'vel_x'
    
    # if [n_readings] not defined, acquire indefintely
    if settings['n_readings'] == 0:
        settings['n_readings'] = np.inf

    ###

    motive_used_body_parts = {'upper_body':[3, 6, 7, 8, 9, 10, 11, 12, 13]}
    motive_kinematic_chain = {'upper_body':[0, 3, 6, 7, 8, 3, 10, 11, 12]}
    body_parts_code = {'upper_body':{3 : 'torso',
                                    6 : 'left shoulder',
                                    7 : 'left arm',
                                    8 : 'left forearm',
                                    9 : 'left hand',
                                    10 : 'right shoulder',
                                    11 : 'right arm',
                                    12 : 'right forearm',
                                    13 : 'right hand'
                                    }
                        }

    if settings['input_device'] == 'motive':
        settings['used_body_parts'] = motive_used_body_parts[settings['motive_body_representation']]
        settings['kinematic_chain'] = motive_kinematic_chain[settings['motive_body_representation']]
        settings['body_parts_code'] = body_parts_code[settings['motive_body_representation']]

        settings['n_rigid_bodies_in_skeleton'] = 13 #len(settings['used_body_parts'])

    imus_kinematic_chain = {'upper_body':[0, 1, 2, 3, 4, 0, 6, 7, 8],
                            'full_body_7':[0, 1, 2, 3, 0, 5, 6],
                            'torso_arm':[0, 1, 2, 3, 4],
                            'arm':[0, 1, 2, 3],
                            'arm_no_shoulder':[0, 1, 2],
                            'forearm':[0, 1],
                            'two_hands':[0, 0],
                            'single':[0],
                            'upper_body_no_hands':[1, 2, 3, 4, 6, 7, 8]}
                            
    if settings['input_device'] == 'imus':

        settings['kinematic_chain'] = imus_kinematic_chain[settings['imu_body_representation']]

        settings['used_body_parts'] = list(range(1,len(settings['kinematic_chain'])+1))

    ###

    # options : 'roll', 'pitch', both (list)
    settings['outputs_no_pll'] = ['roll', 'pitch']
    if settings['robot'] == 'Fixed-wing':
        settings['outputs_no_pll'] = ['roll', 'pitch', 'speed']
    if settings['robot'] == 'Quadrotor':
        settings['outputs_no_pll'] = ['vel_x', 'vel_y', 'vel_z', 'pitch_rate']
    if settings['robot'] == 'Manipulator':
        settings['outputs_no_pll'] = ['vel_x', 'vel_z', 'vel_y', 'roll_rate',  'yaw_rate', 'pitch_rate']
    if settings['robot'] == 'QuadManip':
        settings['outputs_no_pll'] = ['vel_x', 'vel_y', 'vel_z', 'pitch_rate', 'man_vel_x', 'man_vel_z', 'man_vel_y', 'man_roll_rate',  'man_yaw_rate', 'man_pitch_rate']
    # set pll outputs
    settings['outputs'] = [x + '_pll' for x in settings['outputs_no_pll']] ### TOFIX

    ###

    # options : 'roll', 'pitch', both (list)
    if settings['robot'] == 'Fixed-wing':
        settings['maneuver_output'] = ['none', 'roll', 'roll', 'pitch', 'pitch', 'speed', 'speed']
    if settings['robot'] == 'Quadrotor':
        settings['maneuver_output'] = ['none', 'vel_x', 'vel_x', 'vel_y', 'vel_y', 'vel_z', 'vel_z', 'pitch_rate', 'pitch_rate']
    if settings['robot'] == 'Manipulator':
        settings['maneuver_output'] = ['none', 'pos_x', 'pos_x', 'pos_y', 'pos_y', 'pos_z', 'pos_z', 'roll', 'roll', 'pitch', 'pitch',  'yaw', 'yaw']
    if settings['robot'] == 'QuadManip':
        settings['maneuver_output'] = ['none', 'vel_x', 'vel_x', 'vel_y', 'vel_y', 'vel_z', 'vel_z', 'pitch_rate', 'pitch_rate', 'man_pos_x', 'man_pos_x', 'man_pos_y', 'man_pos_y', 'man_pos_z', 'man_pos_z', 'man_roll', 'man_roll', 'man_pitch', 'man_pitch',  'man_yaw', 'man_yaw']

    ###

    if settings['separate_regressors']:
        settings['regression_outputs'] = settings['outputs']
    else:
        settings['regression_outputs_list'] = settings['outputs']
        settings['regression_outputs'] = ['all']

    ###
        
    # working folder depends on pc
        # yasser's folder
    if settings['location'] == 'mocap_yasser':
        settings['data_folder'] = 'G:\\My Drive\\Online learning - Yasser\\DATA\\acquired_data'
    if settings['location'] == 'windows':
        # Windows folder
        settings['data_folder'] = 'G:\\My Drive\\Matteo\\EPFL\\LIS\\PhD\\__Natural_Mapping\\DATA\\acquired_data'
    if settings['location'] == 'dronedome':
        # DroneDome folder
        settings['data_folder'] = 'D:\\LIS\\Matteo\\__Natural_Mapping\\DATA\\acquired_data'
    if settings['location'] == 'mac':
        # mac folder
        settings['data_folder'] = '/Volumes/GoogleDrive/My Drive/Matteo/EPFL/LIS/PhD/__Natural_Mapping/DATA/acquired_data'
    if settings['location'] == 'behav':
        settings['data_folder'] = '/Volumes/GoogleDrive/My Drive/Matteo/EPFL/LIS/Semester projects/Body robot control - continuation/DATA/Manana copy/motion_data/08.05.2019 Experiments'
    if settings['location'] == 'multi':
        settings['data_folder'] = '/Volumes/GoogleDrive/My Drive/Matteo/EPFL/LIS/PhD/__Natural_Mapping/DATA/acquired_data/multi_robot_pilots'
    if settings['location'] == 'multi_win':
        settings['data_folder'] = 'C:\\Users\\SymbioticDrone\\Desktop\\_UnityData\\multi_fws_motion'
        # yasser's folder
    if settings['location'] == 'yasser':
        settings['data_folder'] = 'G:\\My Drive\\Online learning - Yasser\\DATA\\acquired_data'
    if settings['location'] == 'filip':
        settings['data_folder'] = '/home/fk-rog/Dipl/EPFL/W_Fold'
    if settings['fake_input']:
        settings['data_folder'] = os.path.join(settings['data_folder'], 'fake_inputs')

    if preset is not None:
        if 'data_folder' in list(preset):
            settings['data_folder'] = preset['data_folder']
    

    settings['subject_folder'] = os.path.normpath(os.path.join(settings['data_folder'], HRI.file_name(settings)))

    settings['control_folder'] = os.path.normpath(os.path.join(settings['subject_folder'], 'control_data'))

    settings['home_folder'] = os.path.dirname(os.path.realpath(__file__))

    settings['plot_folder'] = os.path.normpath(os.path.join(settings['subject_folder'], 'plots'))

    print('working in folder {}'.format(settings['subject_folder']))

    HRI.create_dir_safe(settings['subject_folder'])
    HRI.create_dir_safe(settings['control_folder'])
    HRI.create_dir_safe(settings['home_folder'])
    HRI.create_dir_safe(settings['plot_folder'])

    ###

    settings['filename'] = datetime.datetime.now().strftime("%Y_%b_%d_%I_%M_%S%p")

    ###

    regressors = get_regressors()
    settings['regressor'] = regressors[settings['regressor_settings']]

    ### headers

    control_history_header = np.array(['input_{}'.format(x) for x in range(1,11)])

    motive_header = np.array([])

    motive_header_base = np.char.array([ 'ID', 'pos_x', 'pos_y', 'pos_z', 'quat_x', 'quat_y', 'quat_z', 'quat_w'])

    motive_regression_header = np.array([])

    motive_regression_header_base = np.char.array([ 'ID', 'pos_x', 'pos_y', 'pos_z', 'quat_x', 'quat_y', 'quat_z', 'quat_w', 'roll', 'pitch', 'yaw'])

    if settings['input_device'] == 'motive':

        for i in range(settings['n_rigid_bodies_in_skeleton']):

            n = np.char.array([('_' + str(i+1))])

            if i==0:
                motive_header = motive_header_base + (n)
                if i+1 in settings['used_body_parts']:
                    motive_regression_header = motive_regression_header_base + (n)
            else:
                motive_header = np.r_[motive_header, motive_header_base + (n)]
                if i+1 in settings['used_body_parts']:
                    motive_regression_header = np.r_[motive_regression_header, motive_regression_header_base + (n)]

    settings['headers'] = {}

    if settings['robot'] in ['Fixed-wing (fixed speed)', 'Fixed-wing']:
        unity_header_calib = np.char.array(['input1', 'input2', 'input3', 'input4', 'input5', 'input6', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'speed', 'quat_x', 'quat_y', 'quat_z',
                                            'quat_w', 'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'timestamp', 'state', 'maneuver', 'maneuver duration', 'maneuver max amplitude', 'instance', 'loop counter'])

    # store all data in the same variable (inputs, position, velocity, quaternion, euler, angular velocity, timestamp, inputs)
    #                                      6 +     3 +       3 +       4 +         3 +    3 +               1 +        6 =         29   
    elif settings['robot'] in ['Quadrotor', 'Manipulator']:
        unity_header_calib = np.char.array(['input1', 'input2', 'input3', 'input4', 'input5', 'input6', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'speed', 'quat_x', 'quat_y', 'quat_z',
                                            'quat_w', 'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'timestamp', 'state', 'maneuver', 'maneuver duration', 'maneuver max amplitude', 'instance', 'loop counter'])
    
    elif settings['robot'] in ['QuadManip']:
        unity_header_calib = np.char.array(['input1', 'input2', 'input3', 'input4', 'input5', 'input6', 'input7', 'input8', 'input9', 'input10', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'speed', 'quat_x', 'quat_y', 'quat_z', 'quat_w', 'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'man_vel_x', 'man_vel_y', 'man_vel_z', 'man_roll_rate',  'man_yaw_rate', 'man_pitch_rate','timestamp', 'state', 'maneuver', 'maneuver duration', 'maneuver max amplitude', 'instance', 'loop counter'])

    settings['headers']['log_pad'] = np.char.array(['input1', 'input2', 'input3', 'input4', 'input5', 'input6', 'input7', 'input8', 'input9', 'input10'])
    settings['headers']['remote'] = np.char.array([ 'remote1', 'remote2', 'remote3', 'remote4' ])
    settings['headers']['imu'] = np.char.array([ 'roll_imu', 'pitch_imu', 'yaw_imu' ])
    settings['headers']['imus_base'] =  np.char.array([ 'ID', 'roll', 'pitch', 'yaw', 'quat_x', 'quat_y', 'quat_z', 'quat_w']) # it's built dinamically
    settings['headers']['motive'] = np.array(motive_header)
    

    settings['headers']['unity'] = np.array(unity_header_calib)

    settings['headers']['motive_calib'] = np.r_[settings['headers']['motive'], settings['headers']['unity']]

    settings['headers']['remote_calib'] = np.r_[settings['headers']['remote'], settings['headers']['unity']]

    settings['headers']['imu_calib'] = np.r_[settings['headers']['imu'], settings['headers']['unity']]

    settings['headers']['control_history'] = control_history_header

    settings['headers']['log_pad_calib'] = np.r_[settings['headers']['log_pad'], settings['headers']['unity']]

    ###
    
    settings['headers']['imus'] = settings['headers']['imus_base'] + '_1'

    if settings['input_device'] == 'motive':
        settings['headers']['calib'] = settings['headers']['motive_calib']
    elif settings['input_device'] == 'remote':
        settings['headers']['calib'] = settings['headers']['remote_calib']
    elif settings['input_device'] == 'imu':
        settings['headers']['calib'] = settings['headers']['imu_calib']
    elif settings['input_device'] == 'imus':
        for i in settings['used_body_parts'][1:]:                
            settings['headers']['imus'] = np.r_[settings['headers']['imus'], settings['headers']['imus_base'] + '_' + (str(i))]

        settings['headers']['calib'] = np.r_[settings['headers']['imus'], settings['headers']['unity']]

    elif settings['input_device'] == 'log_pad':
        settings['headers']['calib'] = settings['headers']['log_pad_calib']

    settings['headers']['calib'] = settings['headers']['calib'].reshape(1, settings['headers']['calib'].size)

    ###

    if settings['input_device'] == 'motive':
        settings['headers']['history'] = settings['headers']['motive']
    elif settings['input_device'] == 'remote':
        settings['headers']['history'] = settings['headers']['remote']
    elif settings['input_device'] == 'imu':
        settings['headers']['history'] = settings['headers']['imu']
    elif settings['input_device'] == 'log_pad':
        settings['headers']['history'] = settings['headers']['log_pad']

    ### debug

    if settings['logging_level'] == 'DEBUG':
        settings['logging_level'] = logging.DEBUG
    elif settings['logging_level'] == 'INFO':
        settings['logging_level'] = logging.INFO
    elif settings['logging_level'] == 'WARNING':
        settings['logging_level'] = logging.WARNING
    elif settings['logging_level'] == 'ERROR':
        settings['logging_level'] = logging.ERROR
    elif settings['logging_level'] == 'CRITICAL':
        settings['logging_level'] = logging.CRITICAL

    return settings


def get_sockets():
    
    config = configparser.ConfigParser()
    config.read(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'settings.ini'))

    def parse_ini(s):
        if s == 'True':
            return True
        if s == 'False':
            return False
        try:
            out = float(s)
            if out == int(out):
                return int(out)
            else:
                return out
        except:
            return s

    IPs = {}
    for i in config['IPs']:
        IPs[i] = parse_ini(config['IPs'][i])
    ports = {}
    for i in config['ports']:
        ports[i] = parse_ini(config['ports'][i])

    ###

    sockets = {}

    ## open

    sockets['read_unity_flag'] = None
    sockets['write_unity_sk'] = None
    sockets['read_unity_control'] = None
    sockets['read_unity_info'] = None
    sockets['read_motive_sk'] = None
    sockets['read_imu'] = None
    sockets['read_imus'] = None

    sockets['read_log_pad'] = {'IP' : IPs['logitech_gamepad'], 'PORT' : ports['logitech_gamepad']}

    sockets['motive'] = {'IP' : IPs['motive'], 'PORT' : ports['motive']}  # Local MOTIVE client, arbitrary non-privileged port

    sockets['imu'] = {'IP' : IPs['imu'], 'PORT' : ports['imu']}  # Local MOTIVE client, arbitrary non-privileged port
    sockets['imus'] = {'IP' : IPs['imus'], 'PORT' : ports['imus']}  # Local MOTIVE client, arbitrary non-privileged port

    ## unity

    sockets['unity_flag'] = {'IP' : IPs['unity_flag'], 'PORT' : ports['unity_flag']}  # Local UNITY client, arbitrary non-privileged port
    sockets['unity_calib'] = {'IP' : IPs['unity_calib'], 'PORT' : ports['unity_calib']}  # Local UNITY client, arbitrary non-privileged port
    sockets['unity_info'] = {'IP' : IPs['unity_info'], 'PORT' : ports['unity_info']}  # Local UNITY client, arbitrary non-privileged port
    sockets['unity_write_sk'] = {'IP' : IPs['unity_write_sk'], 'PORT' : ports['unity_write_sk']}  # Local UNITY client, arbitrary non-privileged port
    sockets['unity_write_sk_client'] = {'IP' : IPs['unity_write_sk_client'], 'PORT' : ports['unity_write_sk_client']}  # Local UNITY client, arbitrary non-privileged port
    #        sockets['unity_write_sk_client'] = {'IP' : "192.168.1.167", 'PORT' : 5000}  # hardware


    sockets['unity_sk_client'] = {'IP' : IPs['unity_sk_client'], 'PORT' : ports['unity_sk_client']}  # Local UNITY client, arbitrary non-privileged port

    return sockets


def get_feat_names():
    
    ###

    feat_names = {}
    feat_names['positions'] = ['pos_x', 'pos_y', 'pos_z']
    feat_names['quaternions'] = ['quat_x', 'quat_y', 'quat_z', 'quat_w']
    feat_names['euler'] = ['roll', 'pitch', 'yaw']

    feat_names['remote'] = ['remote1', 'remote2', 'remote3', 'remote4']  # can think of adding some buttons
    feat_names['imu'] = ['roll_imu', 'pitch_imu', 'yaw_imu']

    return feat_names

def get_regressors():

    regressors = {}
    regressors['one'] = [make_pipeline(MultiOutputRegressor(LinearRegression())), # linear
            make_pipeline(MLPRegressor(128, activation = 'relu', solver = 'adam', max_iter = 1000))] # MLP

    regressors['reduced'] = [make_pipeline(MultiOutputRegressor(LinearRegression())), # linear
            make_pipeline(MLPRegressor(8, activation = 'relu', solver = 'sgd', max_iter = 1000, early_stopping = True)),
            make_pipeline(MLPRegressor(32, activation = 'relu', solver = 'sgd', max_iter = 1000, early_stopping = True)),
            make_pipeline(MLPRegressor(64, activation = 'relu', solver = 'sgd', max_iter = 1000, early_stopping = True)),
            make_pipeline(MLPRegressor(128, activation = 'relu', solver = 'sgd', max_iter = 1000, early_stopping = True))] # MLP

    grid_hl = [8, 32, 64, 128, (8,8), (32,8), (64,8), (128,8), (8, 8, 8), (32, 8, 8), (64, 8, 8), (128, 8, 8)]
    grid_solver = ['lbfgs', 'sgd', 'adam']
    grid_alpha = [0.0001, 0.001, 0.01, 0.1, 1, 10]
    grid_lr = [0.0001, 0.001, 0.01]

    regs_search = [make_pipeline(MultiOutputRegressor(LinearRegression()))] # linear

    for i in grid_hl:
        for ii in grid_solver:
            for iii in grid_alpha:
                for iiii in grid_lr:
                    regs_search.append(make_pipeline(MLPRegressor(hidden_layer_sizes = i, solver = ii, alpha = iii, learning_rate_init = iiii)))

    regressors['full'] = [make_pipeline(MultiOutputRegressor(LinearRegression())), # linear
            make_pipeline(MLPRegressor(10, activation = 'relu', solver = 'lbfgs')),
            make_pipeline(MLPRegressor(25, activation = 'relu', solver = 'lbfgs')),
            make_pipeline(MLPRegressor(50, activation = 'relu', solver = 'lbfgs')),
            make_pipeline(MLPRegressor(100, activation = 'relu', solver = 'lbfgs')),
            make_pipeline(MLPRegressor(10, activation = 'relu', solver = 'adam')),
            make_pipeline(MLPRegressor(25, activation = 'relu', solver = 'adam')),
            make_pipeline(MLPRegressor(50, activation = 'relu', solver = 'adam')),
            make_pipeline(MLPRegressor(100, activation = 'relu', solver = 'adam')), # MLP
            make_pipeline(MultiOutputRegressor(SVR(kernel='rbf', C=1))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='linear', C=1))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='poly', C=1))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='rbf', C=10))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='linear', C=10))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='poly', C=10))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='rbf', C=100))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='linear', C=100))),
            make_pipeline(MultiOutputRegressor(SVR(kernel='poly', C=100)))] # SVM

    regressors['GP_one'] = [make_pipeline(MultiOutputRegressor(LinearRegression())), # linear
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-10,
                                            optimizer='fmin_l_bfgs_b',
                                            n_restarts_optimizer=0,
                                            normalize_y=False,
                                            copy_X_train=True,
                                            random_state=None)))]

    regressors['GP'] = [make_pipeline(MultiOutputRegressor(LinearRegression())), # linear
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-10,
                                            optimizer='fmin_l_bfgs_b',
                                            n_restarts_optimizer=0,
                                            normalize_y=False,
                                            copy_X_train=True,
                                            random_state=None))),
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-5,
                                            optimizer='fmin_l_bfgs_b',
                                            n_restarts_optimizer=0,
                                            normalize_y=False,
                                            copy_X_train=True,
                                            random_state=None))),
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-2,
                                            optimizer='fmin_l_bfgs_b',
                                            n_restarts_optimizer=0,
                                            normalize_y=False,
                                            copy_X_train=True,
                                            random_state=None))),
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-10,
                                            optimizer='fmin_l_bfgs_b',
                                            n_restarts_optimizer=1,
                                            normalize_y=False,
                                            copy_X_train=True,
                                            random_state=None))),
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-10,
                                            optimizer='fmin_l_bfgs_b',
                                            n_restarts_optimizer=0,
                                            normalize_y=True,
                                            copy_X_train=True,
                                            random_state=None))),
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-10,
                                            n_restarts_optimizer=0,
                                            normalize_y=False,
                                            copy_X_train=True,
                                            random_state=None))),
                    make_pipeline(MultiOutputRegressor(GaussianProcessRegressor(kernel=None,
                                            alpha=1e-10,
                                            optimizer='fmin_l_bfgs_b',
                                            n_restarts_optimizer=2,
                                            normalize_y=False,
                                            copy_X_train=True,
                                            random_state=None)))]

    regressors['SVR'] = [#make_pipeline(MultiOutputRegressor(LinearRegression())), # linear
                    make_pipeline(MultiOutputRegressor(SVR(kernel='rbf', C=10)))]

    regressors['LIN'] = [make_pipeline(MultiOutputRegressor(LinearRegression()))] # linear

    return regressors

######

if __name__ == '__main__':
    get_settings()
    get_sockets()
    get_feat_names()
    get_regressors()

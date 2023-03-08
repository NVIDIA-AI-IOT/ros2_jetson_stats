#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
from copy import deepcopy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def size_min(num, divider=1.0, n=0, start=''):
    if num >= divider * 1000.0:
        n += 1
        divider *= 1000.0
        return size_min(num, divider, n, start)
    else:
        vect = ['', 'k', 'M', 'G', 'T']
        idx = vect.index(start)
        return round(num / divider, 1), divider, vect[n + idx]


def strfdelta(tdelta, fmt):
    """ Print delta time
        - https://stackoverflow.com/questions/8906926/formatting-python-timedelta-objects
    """
    d = {"days": tdelta.days}
    d["hours"], rem = divmod(tdelta.seconds, 3600)
    d["minutes"], d["seconds"] = divmod(rem, 60)
    return fmt.format(**d)


def other_status(hardware, jetson, version):
    """
    Generic information about jetson_clock and nvpmodel
    """
    values = []
    nvpmodel = jetson.nvpmodel
    text = ""
    if nvpmodel is not None:
        nvp_name = nvpmodel.name.replace('MODE_', '').replace('_', ' ')
        values += [KeyValue(key="NV Power-ID" , value=str(nvpmodel.id)), KeyValue(key="NV Power-Mode" , value=str(nvp_name))]
        text += "NV Power[{id}] {name}".format(id=nvpmodel.id, name=nvp_name)
    jc = jetson.jetson_clocks
    if jetson.jetson_clocks is not None:
        if jetson.jetson_clocks.status in ["running", "inactive"]:
            level = DiagnosticStatus.OK
        elif "ing" in jc.status:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        # Show if JetsonClock is enabled or not
        values += [KeyValue(key="jetson_clocks" , value=str(jc.status))]
        values += [KeyValue(key="jetson_clocks on boot" , value=str(jc.boot))]
        text += " - JC {status}".format(status=jc.status)
    # Uptime
    uptime_string = strfdelta(jetson.uptime, "{days} days {hours}:{minutes}:{seconds}")
    values += [KeyValue(key="Up Time" , value=str(uptime_string))]
    # Jtop version
    values += [KeyValue(key="interval" , value=str(jetson.interval))]
    values += [KeyValue(key="jtop" , value=str(version))]
    # Make board diagnostic status
    status = DiagnosticStatus(
        level=level,
        name='jetson_stats board status',
        message=text,
        hardware_id=hardware,
        values=values)
    return status


def board_status(hardware, board, dgtype):
    """
    Board information and libraries installed
    """
    values = []
    for key, value in board['hardware'].items():
        values += [KeyValue(key=key, value=str(value))]
    for key, value in board['libraries'].items():
        values += [KeyValue(key="lib " + key, value=str(value))]
    # Make board diagnostic status
    d_board = DiagnosticStatus(
        name='jetson_stats {type} config'.format(type=dgtype),
        message='Jetpack {jetpack}'.format(jetpack=board['hardware']['Jetpack']),
        hardware_id=hardware,
        values=values)
    return d_board


def disk_status(hardware, disk, dgtype):
    """
    Status disk
    """
    value = int(float(disk['used']) / float(disk['total']) * 100.0)
    if value >= 90:
        level = DiagnosticStatus.ERROR
    elif value >= 70:
        level = DiagnosticStatus.WARN
    else:
        level = DiagnosticStatus.OK
    # Make board diagnostic status
    d_board = DiagnosticStatus(
        level=level,
        name='jetson_stats {type} disk'.format(type=dgtype),
        message="{0:2.1f}GB/{1:2.1f}GB".format(disk['used'], disk['total']),
        hardware_id=hardware,
        values=[
            KeyValue(key="Used", value=str(disk['used'])),
            KeyValue(key="Total", value=str(disk['total'])),
            KeyValue(key="Unit", value="GB")])
    return d_board


def cpu_status(hardware, name, cpu):
    """
    Decode a cpu stats

    Fields:
    * min_freq - Minimum frequency in kHz
    * max_freq - Maximum frequency in kHz
    * frq - Running frequency in kHz
    * governor - Governor selected
    * val - Status CPU, value between [0, 100]
    * model - Model Architecture
    * IdleStates
    """
    message = 'OFF'
    values = []
    if cpu:
        if 'val' in cpu:
            # read value
            val = cpu['val']
            message = '{val}%'.format(val=val)
            # Make Diagnostic Status message with cpu info
            values = [
                KeyValue(key="Val", value=str(val)),
                KeyValue(key="Freq", value=str(cpu['frq'])),
                KeyValue(key="Unit", value="khz")]
            if 'governor' in cpu:
                values += [KeyValue(key="Governor", value=str(cpu['governor']))]
            if 'model' in cpu:
                values += [KeyValue(key="Model", value=str(cpu['model']))]
    # Make Diagnostic message
    d_cpu = DiagnosticStatus(
        name='jetson_stats cpu {name}'.format(name=name),
        message=message,
        hardware_id=hardware,
        values=values)
    return d_cpu

def gpu_status(hardware, gpu):
    """
    Decode and build a diagnostic status message
    Fields:
    * min_freq - Minimum frequency in kHz
    * max_freq - Maximum frequency in kHz
    * frq - Running frequency in kHz
    * val - Status GPU, value between [0, 100]
    """
    d_gpu = DiagnosticStatus(
        name='jetson_stats gpu',
        message='{val}%'.format(val=gpu['val']),
        hardware_id=hardware,
        #values=[KeyValue('Val', '{val}%'.format(val=gpu['val'])), KeyValue("Freq", "{frq}".format(frq=gpu['frq'])), KeyValue("Unit", "khz")])
        values=[KeyValue(key = 'val', value=str(gpu['val'])),
            KeyValue(key='Freq', value=str(gpu['frq'])),
            KeyValue(key='Unit', value="khz")])
    return d_gpu


def fan_status(hardware, fan, dgtype):
    """
    Fan speed and type of control

    Fields:
    * auto - boolean with fan control.
        * True = Automatic speed control enabled
        * False = Automatic speed control disabled
    * speed - Speed set. Value between [0, 100] (float)
    * mode - Mode selected for your fan
    """
    # Make fan diagnostic status
    d_fan = DiagnosticStatus(
        name='jetson_stats {type} fan'.format(type=dgtype),
        message='speed={speed}%'.format(speed=fan['speed']),
        hardware_id=hardware,
        values=[
            KeyValue(key='Mode', value=str(fan['mode'])),
            KeyValue(key="Speed", value=str(fan['speed'])),
            KeyValue(key="Automatic",value=str(fan['auto'])),
            ])
    return d_fan


def ram_status(hardware, ram, dgtype):
    """
    Make a RAM diagnostic status message

    Fields:
    * use - status ram used
    * shared - status of shared memory used from GPU
    * tot - Total size RAM
    * unit - Unit size RAM, usually in kB
    * lfb - Largest Free Block (lfb) is a statistic about the memory allocator
        * nblock - Number of block used
        * size - Size of the largest free block
        * unit - Unit size lfb
    """
    lfb_status = ram['lfb']
    tot_ram, divider, unit_name = size_min(ram.get('tot', 0), start=ram.get('unit', 'M'))
    # Make ram diagnostic status
    d_ram = DiagnosticStatus(
        name='jetson_stats {type} ram'.format(type=dgtype),
        message='{use:2.1f}{unit_ram}B/{tot:2.1f}{unit_ram}B (lfb {nblock}x{size}{unit}B)'.format(
            use=ram['use'] / divider,
            unit_ram=unit_name,
            tot=tot_ram,
            nblock=lfb_status['nblock'],
            size=lfb_status['size'],
            unit=lfb_status['unit']),
        hardware_id=hardware,
        values=[
            KeyValue(key="Use", value=str(ram.get('use', 0))),
            KeyValue(key="Shared", value=str(ram.get('shared', 0))),
            KeyValue(key="Total", value=str(ram.get('tot', 0))),
            KeyValue(key="Unit", value=str(ram.get('unit', 'M'))),
            KeyValue(key="lfb-nblock", value=str(lfb_status['nblock'])),
            KeyValue(key="lfb-size", value=str(lfb_status['size'])),
            KeyValue(key="lfb-unit", value=str(lfb_status['unit']))])
    return d_ram


def swap_status(hardware, swap, dgtype):
    """
    Make a swap diagnostic message

    Fields:
    * use - Amount of SWAP in use
    * tot - Total amount of SWAP available for applications
    * unit - Unit SWAP, usually in MB
    * cached
        * size - Cache size
        * unit - Unit cache size
    """
    swap_cached = swap.get('cached', {})
    tot_swap, divider, unit = size_min(swap.get('tot', 0), start=swap.get('unit', 'M'))
    message = '{use}{unit_swap}B/{tot}{unit_swap}B (cached {size}{unit}B)'.format(
        use=swap.get('use', 0) / divider,
        tot=tot_swap,
        unit_swap=unit,
        size=swap_cached.get('size', '0'),
        unit=swap_cached.get('unit', ''))
    # Make swap diagnostic status
    d_swap = DiagnosticStatus(
        name='jetson_stats {type} swap'.format(type=dgtype),
        message=message,
        hardware_id=hardware,
        values=[
            KeyValue(key="Use", value=str(swap.get('use', 0))),
            KeyValue(key="Total", value=str(swap.get('tot', 0))),
            KeyValue(key="Unit", value=str(swap.get('unit', 'M'))),
            KeyValue(key="Cached-Size", value=str(swap_cached.get('size', '0'))),
            KeyValue(key="Cached-Unit", value=str(swap_cached.get('unit', '')))])
    return d_swap


def power_status(hardware, total, power):
    """
    Make a Power diagnostic message

    Fields:
    * Two power dictionaries:
    * total - The total power estimated is not available of the NVIDIA Jetson power comsumption
    * power - A dictionary with all power comsumption

    For each power comsumption there are two fields:
    * avg - Average power consumption in milliwatts
    * cur - Current power consumption in milliwatts
    """
    values = []
    # Make list power
    for watt in sorted(power):
        value = power[watt]
        watt_name = watt.replace("VDD_", "").replace("POM_", "").replace("_", " ")
        values += [KeyValue(key="Current Power", value=str(int(value['cur']))), KeyValue(key="Average Power", value=str(int(value['avg'])))]
    # Make voltage diagnostic status
    d_volt = DiagnosticStatus(
        name='jetson_stats power',
        message='curr={curr}mW avg={avg}mW'.format(curr=int(total['cur']), avg=int(total['avg'])),
        hardware_id=hardware,
        values=values)
    return d_volt


def temp_status(hardware, temp, level_options):
    """
    Make a temperature diagnostic message

    Fields:
    * All temperatures are in Celsius
    """
    values = []
    level = DiagnosticStatus.OK
    list_options = sorted(level_options.keys(), reverse=True)
    max_temp = 20
    # List all temperatures
    for key, value in temp.items():
        values += [KeyValue(key=key, value=str(value))]
        if value > max_temp:
            # Add last high temperature
            max_temp = value
    # Make status message
    for th in list_options:
        if max_temp >= th:
            level = level_options[th]
            break

    if level is not DiagnosticStatus.OK:
        max_temp_names = []
        # List off names
        for key, value in temp.items():
            if value >= th:
                # Store name
                max_temp_names += [key]
        # Write a message
        message = '[' + ', '.join(max_temp_names) + '] are more than {temp} C'.format(temp=th)
    else:
        message = '{n_temp} temperatures reads'.format(n_temp=len(temp))
    # Make temperature diagnostic status
    d_temp = DiagnosticStatus(
        level=level,
        name='jetson_stats temp',
        message=message,
        hardware_id=hardware,
        values=values)
    return d_temp


def emc_status(hardware, emc, dgtype):
    """
    Make a EMC diagnostic message

    Fields:
    * min_freq - Minimum frequency in kHz
    * max_freq - Maximum frequency in kHz
    * frq - Running frequency in kHz
    * val - Status EMC, value between [0, 100]
    * FreqOverride - Status override
    """
    # Make EMC diagnostic status
    d_emc = DiagnosticStatus(
        name='jetson_stats {type} emc'.format(type=dgtype),
        message='{val}%'.format(val=emc['val']),
        hardware_id=hardware,
        values=[
            KeyValue(key='Val', value=str(emc['val'])),
            KeyValue(key="Freq", value=str(emc['frq'])),
            KeyValue(key="Unit", value="khz")])
    return d_emc
# EOF


#!/usr/bin/env python

#import sys
#sys.path.append("..")
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

inregs=None
con=None
state=None

def connect(host,port,xml,uplink=False):
  global con,inregs
  conf = rtde_config.ConfigFile(xml)
  state_names, state_types = conf.get_recipe("state")
  if uplink: inregs_names, inregs_types = conf.get_recipe("input")

  try:
    con = rtde.RTDE(host,port)
    con.connect()
    print("connect...pass")

    con.get_controller_version()
    print("get version...pass")

    con.send_output_setup(state_names, state_types)
    if uplink: inregs = con.send_input_setup(inregs_names, inregs_types)
    print("setup...pass")
  except Exception as e:
    print(e)
    return False
  else:
    return True

def receive():
  global state
  state = con.receive()
  return False if state is None else True

def start():   # start data synchronization
  global state
  if not con.send_start():
    print('send_start...failed')
    return False
  else:
    return receive()

def send():
  if inregs is not None: con.send(inregs)

def pause():
  con.send_pause()

def disconnect():
  con.disconnect()

# ESP32 RMT IR for MicroPython
# MIT license; Copyright (c) 2022-2024 WEMOS.CC

import esp32
from machine import Pin
import struct 
import time
import gc


IR_TYPE=[{'tag':'NEC','start':(9000, 4500),'high':( 560,  1690 ),'low':( 560, 560 ),'end':( 560,0 ) },
		 {'tag':'SAMSUNG','start':(4500, 4450),'high':( 560, 1600 ),'low':( 560, 560 ),'end':( 8950,0 )},
		 {'tag':'LG32','start':(4500, 4500),'high':( 500, 1750 ),'low':( 500, 560 ),'end':( 8950,0 )}]



class ESP32_RMT_IR:

	def __init__(self,pin):
		try:
			self.ir = esp32.RMT(0, pin=Pin(pin), clock_div=80, tx_carrier=(38000, 33, 1))#1us,  38khz
		except:
			print("Error")
			self.available=0
		else:
			self.available=1
	
	def find_type(self,type):
		for i, dic in enumerate(IR_TYPE):
			if(dic['tag']==type):
				return i
		return -1
	
	def build_data(self,cmd_data,type):

		i=self.find_type(type)
		
		if(i!=-1):
			data_START=IR_TYPE[i]['start']
			data_HIGH=IR_TYPE[i]['high']
			data_LOW=IR_TYPE[i]['low']
			data_END=IR_TYPE[i]['end']
			
			tmp=data_START

			for b in ('{:032b}'.format(cmd_data)):
				if (b=='1'):
					tmp+=data_HIGH

				else:
					tmp+=data_LOW
			
			tmp+=data_END
			
			return tmp
			
		else:
			return -1
	
	def send_INT32(self,data,type):
		if(self.available):

			tmp=self.build_data(data,type)
			
			if(tmp!=-1):
				if(self.ir.wait_done()):
					self.ir.write_pulses(tmp, 1)
		
	
	def send_NEC(self,address,cmd):

		if(self.available):
			tmp=struct.pack(">4b",address,~address,cmd,~cmd)
			data=struct.unpack('>i',tmp)[0]
			rmt_data=self.send_INT32(data,'NEC')
			return rmt_data

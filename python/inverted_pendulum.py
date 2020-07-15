#!/usr/bin/env python3

import argparse
import json
import sys
import time
import traceback

import numpy as np

from arduino_io import ArduinoIO
from cart import Cart
from ultra96_io import Ultra96IO

class Controller:
	def __init__(self, params, sensors, motor):
		self.dt = params['dt']['val']
		self.Khat = np.array(params['Khat']['val'])

		self.cart = Cart(params, sensors, motor)

		self.r = 0.0
		self.zeta = 0.0

	def _waitForNextStep(self):
		time.sleep(max(0, self.dt - (time.time() - self.last_time)))
		self.last_time += self.dt

	def _step(self):
		# Get the current state of the cart
		[state, y] = self.cart.nextState(self.dt)

		# Zeta is the integral of the position error
		self.zeta += (self.r - y) * self.dt

		# The control equation
		u = np.dot(-self.Khat, np.append(state, self.zeta))

		self.cart.setForce(u)

	def setup(self):
		# Initialize encoder positions
		print("Initializing controller...")
		self.cart.zeroTheta()
		self.cart.findLimits()
		print("Done initializing controller.")

	def run(self):
		print("Running controller...")
		print("Centering cart...")
		# Center the cart
		self.cart.goTo(0.0)

		# Wait for the pendulum to be moved into position by hand
		self.cart.waitForPendulum()

		# Control loop
		print("Beginning control loop...")
		self.last_time = time.time()
		self.zeta = 0.0

		self.cart.resetState()

		while True:
			self._waitForNextStep()
			self._step()

			# Make sure the cart doesn't crash into the ends
			if not self.cart.checkLimits():
				break

def main(argv):
	print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! THIS IS THE PYTHON SCRIPT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
	parser = argparse.ArgumentParser()
	subparsers = parser.add_subparsers(help='Sub-command help', dest='sensor_type')

	parser_ard = subparsers.add_parser('arduino', help='Arduino configuration')
	parser_ard.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
	parser_ard.add_argument('--baud', default=115200, help='Baud rate')
	parser_ard.add_argument('--params', default='params.json', help='Parameter file')

	parser_u96 = subparsers.add_parser('ultra96', help='Ultra96 configuration')
	parser_u96.add_argument('--encoder1', default=1, help="First encoder's pin number")
	parser_u96.add_argument('--encoder2', default=0, help="Second encoder's pin number")
	parser_u96.add_argument('--motor1', default=508, help='First motor IO pin number')
	parser_u96.add_argument('--motor2', default=509, help='Second motor IO pin number')
	parser_u96.add_argument('--limit1', default=511, help='First limit switch IO pin number')
	parser_u96.add_argument('--limit2', default=510, help='Second limit switch IO pin number')
	parser_u96.add_argument('--params', default='params.json', help='Parameter file')

	args = parser.parse_args(argv[1:])
	if args.sensor_type is None:
		print("Please specify a sensor/motor interface!")
		exit(1)
	
	with open(args.params) as file:
		params = json.load(file)

	if args.sensor_type == 'arduino':
		sensor_int = ArduinoIO(params, port=args.port, baud=args.baud)
	else:
		sensor_int = Ultra96IO(params, [args.encoder1, args.encoder2],
				       [args.motor1, args.motor2], [args.limit1, args.limit2]) 

	controller = Controller(params, sensors=sensor_int, motor=sensor_int)

	try:
		controller.setup()
		while True:
			controller.run()
	except:
		print(traceback.format_exc())

	# Make sure to disable the motor before exiting
	sensor_int.setMotorV(0)

	return 0

if __name__ == '__main__':
	sys.exit(main(sys.argv))

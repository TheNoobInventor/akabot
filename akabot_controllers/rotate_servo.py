#!/usr/bin/python

from akabot_controllers.PCA9685 import PCA9685

from time import sleep
import argparse

# Setup arguments
parser = argparse.ArgumentParser()
parser.add_argument('-c', '--channel', type=int,
                    help='choose a servo channel from 1 to 6 for rotation')
parser.add_argument('-a', '--all', action='store_true', help='rotate all servos')
args = parser.parse_args()

# Initialize PWM driver
pwm = PCA9685()
pwm.setPWMFreq(50)
        
def rotate_servo(channel, motor_type):
    """ Rotate servo from 0 to 180 degrees or 180 to 0 degrees, depending on the servo motor type, 
        on the respective servo channel.
    """ 
    print('Servo', channel, 'is rotating ...')

    if motor_type == 'YF6125MG':
        for i in range(0, 181, 5): 
            pwm.setRotationAngle(channel, i)   
            sleep(0.5)
           
            # Pause at 90 degrees for 1 second
            if i == 90:
                print('Pausing rotation at 90 degrees')
                sleep(1)
                print('Rotation continues ...')

    if motor_type == 'M996R':
        for i in range(180, 4, -5): 
            pwm.setRotationAngle(channel, i)   
            sleep(0.5)
           
            # Pause at 90 degrees for 1 second
            if i == 90:
                print('Pausing rotation at 90 degrees')
                sleep(1)
                print('Rotation continues ...')
            
    print('Rotation completed\n')

def servo_type_check(channel):
    """ Confirm the servo motor type for the respective channel. 

        Channel - motor type

        1 - YF6125MG 
        2 - YF6125MG
        3 - M996R
        4 - M996R
        5 - M996R
        6 - M996R
    """

    if channel in [1, 2]:
        motor_type = 'YF6125MG'

    if channel in [3, 4, 5, 6]:
        motor_type = 'M996R'

    return motor_type

def main():
    """ Main function. """

    try:
       # Rotate one servo
        if args.channel:
            # Servo channel choice check
            if args.channel < 1 or args.channel > 6:
                raise ValueError('Choose a channel number from 1 to 6.')
            else:
                # Servo motor type check
                servo_motor_type = servo_type_check(args.channel)

                # Rotate servo 
                rotate_servo(args.channel, servo_motor_type)
                sleep(1)
        
        # Rotate all servos
        if args.all:
            for i in range(1, 7):
                # Servo motor type check
                servo_motor_type = servo_type_check(i)

                # Rotate servo
                rotate_servo(i, servo_motor_type)
                sleep(1)

    except KeyboardInterrupt:
        # Close PCA9685 connection
        pwm.exit_PCA9685()

    finally:
        # Close PCA9685 connection
        pwm.exit_PCA9685()

if __name__ == '__main__':
    main()
